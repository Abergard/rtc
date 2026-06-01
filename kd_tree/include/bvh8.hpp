#pragma once

#include <memory>
#include <vector>

#include "bounding_box.hpp"
#include "math_ray.hpp"
#include "scene_model.hpp"

namespace rtc
{
class bvh8
{
 public:
  struct tree_node;
  using value_type = std::vector<std::uint32_t>;
  class const_iterator;

  rtc_hot explicit bvh8(const rtc::scene_model&);
  bvh8(bvh8&&) noexcept;
  auto operator=(bvh8&&) noexcept -> bvh8&;

  ~bvh8();
  bvh8(const bvh8&) = delete;
  auto operator=(const bvh8&) -> bvh8& = delete;

  [[nodiscard]] auto cbegin(const rtc::math_ray&) const noexcept -> const_iterator;
  [[nodiscard]] auto cend(const rtc::math_ray&) const noexcept -> const_iterator;

 private:
  std::unique_ptr<tree_node> root;

  [[nodiscard]] static auto make_triangle_bbox(const rtc::scene_model&, std::uint32_t) noexcept -> rtc::bounding_box;
  [[nodiscard]] static auto make_node_bbox(const std::vector<rtc::bounding_box>&,
                                           const value_type&) noexcept -> rtc::bounding_box;

  auto build_tree(const std::vector<rtc::bounding_box>& primitive_bboxes,
                  value_type triangles,
                  std::uint32_t depth) -> std::unique_ptr<tree_node>;
};
}  // namespace rtc

#include "bvh8_iterator.hpp"
