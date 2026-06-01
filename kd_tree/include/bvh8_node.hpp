#pragma once

#include <array>
#include <memory>

#include "bvh8.hpp"

namespace rtc
{
struct bvh8::tree_node
{
  static constexpr std::size_t max_children{8};

  explicit tree_node(rtc::bounding_box box) noexcept : bbox{box} {}

  rtc::bounding_box bbox;
  std::unique_ptr<bvh8::value_type> triangles;
  std::array<std::unique_ptr<tree_node>, max_children> children{};
  std::uint8_t child_count{};

  [[nodiscard]] auto is_leaf() const noexcept -> bool { return child_count == 0; }
};
}  // namespace rtc
