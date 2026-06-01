#include "bvh_iterator.hpp"

#include <algorithm>
#include <cassert>

#include "bvh_node.hpp"

namespace rtc
{
auto bvh::const_iterator::operator*() const noexcept -> const value_type&
{
  assert(current_node);
  return *current_node->triangles;
}

auto bvh::const_iterator::operator==(const bvh::const_iterator& i) const noexcept -> bool
{
  return current_node == i.current_node;
}

auto bvh::const_iterator::operator!=(const bvh::const_iterator& i) const noexcept -> bool
{
  return !(*this == i);
}

bvh::const_iterator::const_iterator(const rtc::math_ray& r, tree_node* node) : ray{r}
{
  push_if_hit(node);
  operator++();
}

auto bvh::const_iterator::operator++() noexcept -> const_iterator&
{
  current_node = nullptr;

  while (!nodes.empty())
  {
    auto node = nodes.back().node;
    const auto tmin = nodes.back().tmin;
    nodes.pop_back();

    if (!node || nearest_intersect_ray_value < tmin)
      continue;

    if (node->is_leaf())
    {
      current_node = node;
      return *this;
    }

    const auto left_t = intersection_value(node->left.get());
    const auto right_t = intersection_value(node->right.get());

    if (left_t < right_t)
    {
      if (right_t < std::numeric_limits<rtc_float>::max())
        nodes.push_back({node->right.get(), right_t});
      if (left_t < std::numeric_limits<rtc_float>::max())
        nodes.push_back({node->left.get(), left_t});
    }
    else
    {
      if (left_t < std::numeric_limits<rtc_float>::max())
        nodes.push_back({node->left.get(), left_t});
      if (right_t < std::numeric_limits<rtc_float>::max())
        nodes.push_back({node->right.get(), right_t});
    }
  }

  return *this;
}

auto bvh::const_iterator::intersection_value(const tree_node* node) const noexcept -> rtc_float
{
  if (!node)
    return std::numeric_limits<rtc_float>::max();

  const auto inv_ray_dir = 1.0F / ray.direction();
  rtc_float t_in{}, t_out = std::numeric_limits<rtc_float>::max();

  for (const auto axis : {rtc::axis::x, rtc::axis::y, rtc::axis::z})
  {
    auto t_far = (node->bbox.max_boundary().axis(axis) - ray.origin().axis(axis)) * inv_ray_dir.axis(axis);
    auto t_near = (node->bbox.min_boundary().axis(axis) - ray.origin().axis(axis)) * inv_ray_dir.axis(axis);

    if (t_near > t_far)
      std::swap(t_near, t_far);

    t_in = std::max(t_near, t_in);
    t_out = std::min(t_far, t_out);

    if (t_in > t_out)
      return std::numeric_limits<rtc_float>::max();
  }

  return t_in;
}

auto bvh::const_iterator::push_if_hit(tree_node* node) noexcept -> void
{
  const auto t = intersection_value(node);
  if (t < std::numeric_limits<rtc_float>::max())
    nodes.push_back({node, t});
}

auto bvh::const_iterator::triangle_hit_value(const rtc_float t) noexcept -> const_iterator&
{
  return nearest_intersect_ray_value = t, *this;
}
}  // namespace rtc
