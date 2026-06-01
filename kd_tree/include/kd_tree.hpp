#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <tuple>
#include <vector>

#include "bounding_edge.hpp"
#include "math_ray.hpp"
#include "math_vector.hpp"
#include "ray_box_intersection_test.hpp"
#include "scene_model.hpp"

namespace rtc
{
class kd_tree
{
  static constexpr std::uint32_t invalid_node{std::numeric_limits<std::uint32_t>::max()};

  struct alignas(8) tree_node
  {
    rtc_float split_value{};
    std::uint32_t left{invalid_node};
    std::uint32_t right{invalid_node};
    std::uint32_t triangles_begin{};
    std::uint32_t triangles_count{};
    std::uint8_t split_axis{};
    std::uint8_t flags{};

    [[nodiscard]] auto is_leaf() const noexcept -> bool { return left == invalid_node && right == invalid_node; }
    [[nodiscard]] auto axis() const noexcept -> rtc::axis { return static_cast<rtc::axis>(split_axis); }
  };
  static_assert(sizeof(tree_node) == 24);
  static_assert(alignof(tree_node) == 8);

  using edge_buffer_t = std::vector<rtc::bounding_edge_point>;
  using edge_buffer_array_t = std::array<edge_buffer_t, 3>;

  template <typename... T>
  using vector_tuple = std::tuple<std::vector<T>...>;

 public:
  struct triangle_range
  {
    const std::uint32_t* first{};
    const std::uint32_t* last{};

    [[nodiscard]] auto begin() const noexcept -> const std::uint32_t* { return first; }
    [[nodiscard]] auto end() const noexcept -> const std::uint32_t* { return last; }
  };

  class const_iterator;

  rtc_hot explicit kd_tree(const rtc::scene_model &sm);
  kd_tree(kd_tree &&) noexcept;
  auto operator=(kd_tree &&) noexcept -> kd_tree &;

  ~kd_tree();
  kd_tree(const kd_tree &) = delete;
  auto operator=(const kd_tree &) -> kd_tree & = delete;

  [[nodiscard]] auto cbegin(const rtc::math_ray &) const noexcept -> const_iterator;
  [[nodiscard]] auto cend(const rtc::math_ray &) const noexcept -> const_iterator;

 private:
  ray_box_intersection_test bbox;
  std::vector<tree_node> nodes;
  std::vector<std::uint32_t> leaf_triangles;
  std::uint32_t root{invalid_node};

  rtc_hot auto build_tree(rtc::bounding_box b,
                          std::vector<std::uint32_t> tr,
                          const std::vector<rtc::bounding_box> &primitive_bboxes,
                          edge_buffer_array_t &edges,
                          const std::uint32_t depth,
                          std::uint32_t bad_refines = 0) -> std::uint32_t;

  rtc_hot auto compute_node_split_paramters(edge_buffer_array_t &edges,
                                            const std::vector<std::uint32_t> &tr,
                                            const rtc::bounding_box &node_bbox,
                                            const std::vector<rtc::bounding_box> &primitive_bboxes);

  rtc_hot auto split_triangles(std::vector<std::uint32_t> &&,
                               const edge_buffer_array_t &edges,
                               const std::uint32_t best_axis,
                               const std::uint32_t best_offset);
};

}  // namespace rtc

#include "kd_tree_iterator.hpp"
