#include "bvh8_iterator.hpp"

#include <algorithm>
#include <cassert>

#include "bvh8_node.hpp"

namespace rtc
{
auto bvh8::const_iterator::operator*() const noexcept -> const value_type&
{
  assert(current_node);
  return *current_node->triangles;
}

auto bvh8::const_iterator::operator==(const bvh8::const_iterator& i) const noexcept -> bool
{
  return current_node == i.current_node;
}

auto bvh8::const_iterator::operator!=(const bvh8::const_iterator& i) const noexcept -> bool
{
  return !(*this == i);
}

bvh8::const_iterator::const_iterator(const rtc::math_ray& r, tree_node* node)
    : ray{r}, inv_ray_direction{1.0F / r.direction()}
{
  push_if_hit(node);
  operator++();
}

auto bvh8::const_iterator::operator++() noexcept -> const_iterator&
{
  current_node = nullptr;

  while (nodes_size)
  {
    const auto item = nodes[--nodes_size];
    auto node = item.node;
    const auto tmin = item.tmin;

    if (!node || nearest_intersect_ray_value < tmin)
      continue;

    if (node->is_leaf())
    {
      current_node = node;
      return *this;
    }

    std::array<node_t, tree_node::max_children> children{};
    std::size_t child_hits{};
    for (std::size_t i{}; i < node->child_count; ++i)
    {
      const auto child = node->children[i].get();
      const auto child_t = intersection_value(child);
      if (child_t < nearest_intersect_ray_value)
      {
        auto insert_at = child_hits;
        while (insert_at > 0 && children[insert_at - 1].tmin < child_t)
        {
          children[insert_at] = children[insert_at - 1];
          --insert_at;
        }

        children[insert_at] = {child, child_t};
        ++child_hits;
      }
    }

    for (std::size_t i{}; i < child_hits && nodes_size < nodes.size(); ++i)
      nodes[nodes_size++] = children[i];
  }

  return *this;
}

auto bvh8::const_iterator::intersection_value(const tree_node* node) const noexcept -> rtc_float
{
  if (!node)
    return std::numeric_limits<rtc_float>::max();

  rtc_float t_in{}, t_out = std::numeric_limits<rtc_float>::max();

  for (const auto axis : {rtc::axis::x, rtc::axis::y, rtc::axis::z})
  {
    auto t_far = (node->bbox.max_boundary().axis(axis) - ray.origin().axis(axis)) * inv_ray_direction.axis(axis);
    auto t_near = (node->bbox.min_boundary().axis(axis) - ray.origin().axis(axis)) * inv_ray_direction.axis(axis);

    if (t_near > t_far)
      std::swap(t_near, t_far);

    t_in = std::max(t_near, t_in);
    t_out = std::min(t_far, t_out);

    if (t_in > t_out)
      return std::numeric_limits<rtc_float>::max();
  }

  return t_in;
}

auto bvh8::const_iterator::push_if_hit(tree_node* node) noexcept -> void
{
  const auto t = intersection_value(node);
  if (t < nearest_intersect_ray_value && nodes_size < nodes.size())
    nodes[nodes_size++] = {node, t};
}

auto bvh8::const_iterator::triangle_hit_value(const rtc_float t) noexcept -> const_iterator&
{
  return nearest_intersect_ray_value = t, *this;
}
}  // namespace rtc
