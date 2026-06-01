#pragma once

#include <array>
#include <limits>

#include "bvh8.hpp"
#include "utility.hpp"

namespace rtc
{
class bvh8::const_iterator
{
  struct node_t
  {
    tree_node* node{};
    rtc_float tmin{};
  };

 public:
  const_iterator() = default;
  rtc_hot const_iterator(const rtc::math_ray& r, tree_node* node);

  rtc_hot auto operator++() noexcept -> const_iterator&;
  auto operator*() const noexcept -> const value_type&;
  auto operator!=(const const_iterator&) const noexcept -> bool;
  auto operator==(const const_iterator&) const noexcept -> bool;
  auto triangle_hit_value(rtc_float t) noexcept -> const_iterator&;

 private:
  static constexpr std::size_t stack_capacity{192};

  rtc::math_ray ray;
  rtc::math_vector inv_ray_direction;
  std::array<node_t, stack_capacity> nodes;
  std::size_t nodes_size{};
  tree_node* current_node{};
  rtc_float nearest_intersect_ray_value = std::numeric_limits<rtc_float>::max();

  [[nodiscard]] auto intersection_value(const tree_node*) const noexcept -> rtc_float;
  auto push_if_hit(tree_node*) noexcept -> void;
};
}  // namespace rtc
