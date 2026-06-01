#pragma once

#include <memory>

#include "bvh.hpp"

namespace rtc
{
struct bvh::tree_node
{
  explicit tree_node(rtc::bounding_box box) noexcept : bbox{box} {}

  rtc::bounding_box bbox;
  std::unique_ptr<bvh::value_type> triangles;
  std::unique_ptr<tree_node> left{}, right{};

  [[nodiscard]] auto is_leaf() const noexcept -> bool { return !left && !right; }
};
}  // namespace rtc
