#pragma once

#include <array>

#include "kd_tree.hpp"
#include "utility.hpp"

namespace rtc
{
class kd_tree::const_iterator
{
  struct node_t
  {
    std::uint32_t node;
    rtc_float tmin;
    rtc_float tmax;
  };

 public:
  const_iterator() = default;
  rtc_hot const_iterator(const rtc::math_ray& r,
                         const std::vector<tree_node>* nodes,
                         const std::vector<std::uint32_t>* leaf_triangles,
                         node_t node);

  rtc_hot auto operator++() noexcept -> const_iterator&;
  auto operator*() const noexcept -> triangle_range;
  auto operator!=(const const_iterator&) const noexcept -> bool;
  auto operator==(const const_iterator&) const noexcept -> bool;
  auto triangle_hit_value(const rtc_float t) noexcept -> const_iterator&;

 private:
  static constexpr std::size_t stack_capacity{128};

  rtc::math_ray ray;
  const std::vector<tree_node>* tree_nodes{};
  const std::vector<std::uint32_t>* leaf_triangles{};
  std::array<node_t, stack_capacity> nodes;
  std::size_t nodes_size{};
  std::uint32_t current_node{invalid_node};
  rtc_float nearest_intersect_ray_value = std::numeric_limits<rtc_float>::max();

  auto get_children_and_split_value(const tree_node&, const math_ray&) const noexcept
      -> std::tuple<std::uint32_t, std::uint32_t, rtc_float>;
};

}  // namespace rtc
