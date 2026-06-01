#include "kd_tree_iterator.hpp"

#include "kd_tree.hpp"
#include "kd_tree_node.hpp"
#include "utility.hpp"

namespace rtc
{
auto kd_tree::const_iterator::operator*() const noexcept -> triangle_range
{
  assert(tree_nodes && leaf_triangles && current_node != invalid_node);
  const auto& node = (*tree_nodes)[current_node];
  const auto first = leaf_triangles->data() + node.triangles_begin;
  return {first, first + node.triangles_count};
}


auto kd_tree::const_iterator::operator==(const kd_tree::const_iterator& i) const noexcept -> bool
{
  if (current_node == invalid_node && i.current_node == invalid_node)
    return true;

  return current_node == i.current_node;
}

auto kd_tree::const_iterator::operator!=(const kd_tree::const_iterator& i) const noexcept -> bool
{
  return !(*this == i);
}

kd_tree::const_iterator::const_iterator(const rtc::math_ray& r,
                                        const std::vector<tree_node>* tree,
                                        const std::vector<std::uint32_t>* triangle_storage,
                                        node_t node)
    : tree_nodes{tree}, leaf_triangles{triangle_storage}
{
  if (tree_nodes && node.node != invalid_node && nodes_size < nodes.size())
    nodes[nodes_size++] = node;
  ray = {1.0F / r.direction(), r.origin()};

  (*this).operator++();
}

auto kd_tree::const_iterator::operator++() noexcept -> const_iterator&
{
  if (rtc_unlikely(!tree_nodes || !nodes_size))
    return current_node = invalid_node, *this;

  auto [node, tmin, tmax] = nodes[--nodes_size];

  while (rtc_likely(node != invalid_node))
  {
    if (rtc_unlikely(nearest_intersect_ray_value < tmin))
    {
      current_node = invalid_node;
      break;
    }

    const auto& tree_node = (*tree_nodes)[node];
    if (!tree_node.is_leaf())
    {
      const auto [near, far, tsplit] = get_children_and_split_value(tree_node, ray);

      if ((tsplit > tmax) || (tsplit <= 0))
      {
        node = near;
      }
      else if (tsplit < tmin)
      {
        node = far;
      }
      else
      {
        if (nodes_size < nodes.size())
          nodes[nodes_size++] = {far, tsplit, tmax};
        node = near;
        tmax = tsplit;
      }
    }
    else
    {
      return current_node = node, *this;
    }
  }

  return *this;
}

auto kd_tree::const_iterator::get_children_and_split_value(const tree_node& node, const math_ray& r) const noexcept
    -> std::tuple<std::uint32_t, std::uint32_t, rtc_float>
{
  const auto axis = node.axis();
  const auto tsplit = (node.split_value - r.origin().axis(axis)) * r.direction().axis(axis);
  const bool left_is_near = (r.origin().axis(axis) < node.split_value) ||
                            (r.origin().axis(axis) == node.split_value && r.direction().axis(axis) <= 0);

  if (left_is_near)
  {
    return {node.left, node.right, tsplit};
  }
  else
  {
    return {node.right, node.left, tsplit};
  }
}

auto kd_tree::const_iterator::triangle_hit_value(const rtc_float t) noexcept -> const_iterator&
{
  return nearest_intersect_ray_value = t, *this;
}

}  // namespace rtc
