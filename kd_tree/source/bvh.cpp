#include "bvh.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "bvh_node.hpp"
#include "chrome_trace.hpp"

namespace rtc
{
namespace
{
constexpr std::size_t leaf_triangle_limit{4};

auto centroid_axis(const rtc::bounding_box& box, const rtc::axis axis) noexcept -> rtc_float
{
  return 0.5F * (box.min_boundary().axis(axis) + box.max_boundary().axis(axis));
}

auto make_bbox_from_bounds(const rtc::math_point& min, const rtc::math_point& max) noexcept -> rtc::bounding_box
{
  return rtc::bounding_box{min, max};
}
}  // namespace

bvh::bvh(const rtc::scene_model& scene)
{
  RTC_TRACE_SCOPE_CAT("bvh::bvh", "bvh");

  std::vector<rtc::bounding_box> primitive_bboxes;
  primitive_bboxes.reserve(scene.triangles.size());

  for (std::uint32_t i{}; i < scene.triangles.size(); ++i)
    primitive_bboxes.emplace_back(make_triangle_bbox(scene, i));

  value_type triangles(scene.triangles.size());
  std::iota(triangles.begin(), triangles.end(), 0U);

  const auto max_depth = static_cast<std::uint32_t>(1.3F * std::log2(std::max<std::size_t>(1, triangles.size())) + 8);
  root = build_tree(primitive_bboxes, std::move(triangles), max_depth);
}

auto bvh::cbegin(const rtc::math_ray& ray) const noexcept -> bvh::const_iterator
{
  return root ? const_iterator{ray, root.get()} : const_iterator{};
}

auto bvh::cend(const rtc::math_ray&) const noexcept -> bvh::const_iterator { return const_iterator{}; }

bvh::~bvh() = default;
bvh::bvh(bvh&&) noexcept = default;
auto bvh::operator=(bvh&&) noexcept -> bvh& = default;

auto bvh::make_triangle_bbox(const rtc::scene_model& scene, const std::uint32_t triangle_index) noexcept
    -> rtc::bounding_box
{
  const auto& triangle = scene.triangles[triangle_index];
  return rtc::bounding_box{
      scene.points[triangle.vertex_a()],
      scene.points[triangle.vertex_b()],
      scene.points[triangle.vertex_c()],
  };
}

auto bvh::make_node_bbox(const std::vector<rtc::bounding_box>& primitive_bboxes,
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

auto bvh::build_tree(const std::vector<rtc::bounding_box>& primitive_bboxes,
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

  const auto split_axis = node_bbox.maximum_extent();
  const auto mid = triangles.begin() + static_cast<std::ptrdiff_t>(triangles.size() / 2);

  std::nth_element(triangles.begin(), mid, triangles.end(), [&](const auto lhs, const auto rhs) {
    return centroid_axis(primitive_bboxes[lhs], split_axis) < centroid_axis(primitive_bboxes[rhs], split_axis);
  });

  value_type left{triangles.begin(), mid};
  value_type right{mid, triangles.end()};

  if (left.empty() || right.empty())
  {
    node->triangles = std::make_unique<value_type>(std::move(triangles));
    return node;
  }

  node->left = build_tree(primitive_bboxes, std::move(left), depth - 1);
  node->right = build_tree(primitive_bboxes, std::move(right), depth - 1);
  return node;
}
}  // namespace rtc
