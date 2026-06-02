#include "bvh8.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>

#include "bvh8_node.hpp"
#include "chrome_trace.hpp"

namespace rtc
{
namespace
{
constexpr std::size_t leaf_triangle_limit{4};
constexpr std::size_t bin_count{12};

struct bounds_accumulator
{
  rtc::math_point min{
      std::numeric_limits<rtc_float>::max(),
      std::numeric_limits<rtc_float>::max(),
      std::numeric_limits<rtc_float>::max(),
  };
  rtc::math_point max{
      std::numeric_limits<rtc_float>::lowest(),
      std::numeric_limits<rtc_float>::lowest(),
      std::numeric_limits<rtc_float>::lowest(),
  };
  std::uint32_t count{};

  auto extend(const rtc::bounding_box& box) noexcept -> void
  {
    for (const auto axis : {rtc::axis::x, rtc::axis::y, rtc::axis::z})
    {
      min.axis(axis) = std::min(min.axis(axis), box.min_boundary().axis(axis));
      max.axis(axis) = std::max(max.axis(axis), box.max_boundary().axis(axis));
    }
    ++count;
  }

  auto extend(const bounds_accumulator& bounds) noexcept -> void
  {
    if (!bounds.count)
      return;

    for (const auto axis : {rtc::axis::x, rtc::axis::y, rtc::axis::z})
    {
      min.axis(axis) = std::min(min.axis(axis), bounds.min.axis(axis));
      max.axis(axis) = std::max(max.axis(axis), bounds.max.axis(axis));
    }
    count += bounds.count;
  }

  [[nodiscard]] auto surface_area() const noexcept -> rtc_float
  {
    if (!count)
      return {};

    const auto v = max - min;
    return 2.0F * (v.x() * v.y() + v.x() * v.z() + v.y() * v.z());
  }
};

struct sah_split
{
  rtc::axis axis{};
  rtc_float position{};
  rtc_float cost{std::numeric_limits<rtc_float>::max()};
};

auto centroid_axis(const rtc::bounding_box& box, const rtc::axis axis) noexcept -> rtc_float
{
  return 0.5F * (box.min_boundary().axis(axis) + box.max_boundary().axis(axis));
}

auto make_bbox_from_bounds(const rtc::math_point& min, const rtc::math_point& max) noexcept -> rtc::bounding_box
{
  return rtc::bounding_box::from_boundaries(min, max);
}

auto accumulate_bounds(const std::vector<rtc::bounding_box>& primitive_bboxes,
                       const std::vector<std::uint32_t>& triangles) noexcept -> bounds_accumulator
{
  bounds_accumulator bounds{};
  for (const auto triangle_index : triangles)
    bounds.extend(primitive_bboxes[triangle_index]);

  return bounds;
}

auto find_sah_split(const std::vector<rtc::bounding_box>& primitive_bboxes,
                    const std::vector<std::uint32_t>& triangles) noexcept -> std::optional<sah_split>
{
  std::optional<sah_split> best;

  for (const auto axis : {rtc::axis::x, rtc::axis::y, rtc::axis::z})
  {
    auto centroid_min = std::numeric_limits<rtc_float>::max();
    auto centroid_max = std::numeric_limits<rtc_float>::lowest();

    for (const auto triangle_index : triangles)
    {
      const auto c = centroid_axis(primitive_bboxes[triangle_index], axis);
      centroid_min = std::min(centroid_min, c);
      centroid_max = std::max(centroid_max, c);
    }

    const auto extent = centroid_max - centroid_min;
    if (extent <= std::numeric_limits<rtc_float>::epsilon())
      continue;

    std::array<bounds_accumulator, bin_count> bins{};
    for (const auto triangle_index : triangles)
    {
      const auto c = centroid_axis(primitive_bboxes[triangle_index], axis);
      const auto relative = std::clamp((c - centroid_min) / extent, 0.0F, 0.999999F);
      const auto bin = static_cast<std::size_t>(relative * bin_count);
      bins[bin].extend(primitive_bboxes[triangle_index]);
    }

    std::array<bounds_accumulator, bin_count - 1> left{};
    std::array<bounds_accumulator, bin_count - 1> right{};

    bounds_accumulator accumulated_left{};
    for (std::size_t i{}; i + 1 < bin_count; ++i)
    {
      accumulated_left.extend(bins[i]);
      left[i] = accumulated_left;
    }

    bounds_accumulator accumulated_right{};
    for (std::size_t i = bin_count - 1; i > 0; --i)
    {
      accumulated_right.extend(bins[i]);
      right[i - 1] = accumulated_right;
    }

    for (std::size_t i{}; i + 1 < bin_count; ++i)
    {
      if (!left[i].count || !right[i].count)
        continue;

      const auto cost = left[i].surface_area() * left[i].count + right[i].surface_area() * right[i].count;
      if (!best || cost < best->cost)
      {
        best = sah_split{
            axis,
            centroid_min + extent * (static_cast<rtc_float>(i + 1) / static_cast<rtc_float>(bin_count)),
            cost,
        };
      }
    }
  }

  return best;
}

auto split_group(const std::vector<rtc::bounding_box>& primitive_bboxes,
                 std::vector<std::uint32_t> triangles,
                 const rtc::bounding_box& group_bbox) -> std::array<std::vector<std::uint32_t>, 2>
{
  std::vector<std::uint32_t> left{}, right{};
  if (const auto split = find_sah_split(primitive_bboxes, triangles))
  {
    const auto middle = std::partition(triangles.begin(), triangles.end(), [&](const auto triangle_index) {
      return centroid_axis(primitive_bboxes[triangle_index], split->axis) < split->position;
    });

    left = {triangles.begin(), middle};
    right = {middle, triangles.end()};
  }

  if (left.empty() || right.empty())
  {
    const auto split_axis = group_bbox.maximum_extent();
    const auto mid = triangles.begin() + static_cast<std::ptrdiff_t>(triangles.size() / 2);

    std::nth_element(triangles.begin(), mid, triangles.end(), [&](const auto lhs, const auto rhs) {
      return centroid_axis(primitive_bboxes[lhs], split_axis) < centroid_axis(primitive_bboxes[rhs], split_axis);
    });

    left = {triangles.begin(), mid};
    right = {mid, triangles.end()};
  }

  return {std::move(left), std::move(right)};
}
}  // namespace

bvh8::bvh8(const rtc::scene_model& scene)
{
  RTC_TRACE_SCOPE_CAT("bvh8::bvh8", "bvh8");

  std::vector<rtc::bounding_box> primitive_bboxes;
  primitive_bboxes.reserve(scene.triangles.size());

  for (std::uint32_t i{}; i < scene.triangles.size(); ++i)
    primitive_bboxes.emplace_back(make_triangle_bbox(scene, i));

  value_type triangles(scene.triangles.size());
  std::iota(triangles.begin(), triangles.end(), 0U);

  const auto max_depth = static_cast<std::uint32_t>(1.3F * std::log2(std::max<std::size_t>(1, triangles.size())) + 8);
  root = build_tree(primitive_bboxes, std::move(triangles), max_depth);
}

auto bvh8::cbegin(const rtc::math_ray& ray) const noexcept -> bvh8::const_iterator
{
  return root ? const_iterator{ray, root.get()} : const_iterator{};
}

auto bvh8::cend(const rtc::math_ray&) const noexcept -> bvh8::const_iterator { return const_iterator{}; }

bvh8::~bvh8() = default;
bvh8::bvh8(bvh8&&) noexcept = default;
auto bvh8::operator=(bvh8&&) noexcept -> bvh8& = default;

auto bvh8::make_triangle_bbox(const rtc::scene_model& scene, const std::uint32_t triangle_index) noexcept
    -> rtc::bounding_box
{
  const auto& triangle = scene.triangles[triangle_index];
  return rtc::bounding_box{
      scene.points[triangle.vertex_a()],
      scene.points[triangle.vertex_b()],
      scene.points[triangle.vertex_c()],
  };
}

auto bvh8::make_node_bbox(const std::vector<rtc::bounding_box>& primitive_bboxes,
                          const value_type& triangles) noexcept -> rtc::bounding_box
{
  auto min = primitive_bboxes[triangles.front()].min_boundary();
  auto max = primitive_bboxes[triangles.front()].max_boundary();

  for (const auto triangle_index : triangles)
  {
    const auto& box = primitive_bboxes[triangle_index];
    for (const auto axis : {rtc::axis::x, rtc::axis::y, rtc::axis::z})
    {
      min.axis(axis) = std::min(min.axis(axis), box.min_boundary().axis(axis));
      max.axis(axis) = std::max(max.axis(axis), box.max_boundary().axis(axis));
    }
  }

  return make_bbox_from_bounds(min, max);
}

auto bvh8::build_tree(const std::vector<rtc::bounding_box>& primitive_bboxes,
                      value_type triangles,
                      const std::uint32_t depth) -> std::unique_ptr<tree_node>
{
  const auto node_bbox = make_node_bbox(primitive_bboxes, triangles);
  auto node = std::make_unique<tree_node>(node_bbox);

  if (triangles.size() <= leaf_triangle_limit || !depth)
  {
    node->triangles = std::make_unique<value_type>(std::move(triangles));
    return node;
  }

  std::vector<value_type> groups{};
  groups.push_back(std::move(triangles));

  while (groups.size() < tree_node::max_children)
  {
    auto best_group = groups.size();
    auto best_saving = rtc_float{};

    for (std::size_t i{}; i < groups.size(); ++i)
    {
      if (groups[i].size() <= leaf_triangle_limit)
        continue;

      const auto split = find_sah_split(primitive_bboxes, groups[i]);
      if (!split)
        continue;

      const auto bounds = accumulate_bounds(primitive_bboxes, groups[i]);
      const auto parent_cost = bounds.surface_area() * bounds.count;
      const auto saving = parent_cost - split->cost;
      if (saving > best_saving)
      {
        best_saving = saving;
        best_group = i;
      }
    }

    if (best_group == groups.size())
      break;

    auto group = std::move(groups[best_group]);
    const auto group_bbox = make_node_bbox(primitive_bboxes, group);
    auto children = split_group(primitive_bboxes, std::move(group), group_bbox);
    if (children[0].empty() || children[1].empty())
      break;

    groups[best_group] = std::move(children[0]);
    groups.push_back(std::move(children[1]));
  }

  if (groups.size() == 1)
  {
    node->triangles = std::make_unique<value_type>(std::move(groups.front()));
    return node;
  }

  for (auto& group : groups)
  {
    node->children[node->child_count] = build_tree(primitive_bboxes, std::move(group), depth - 1);
    ++node->child_count;
  }

  return node;
}
}  // namespace rtc
