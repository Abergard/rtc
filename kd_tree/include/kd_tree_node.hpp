#pragma once

#include <memory>

#include "kd_tree.hpp"

namespace rtc
{

struct alignas(8) kd_tree::tree_node
{
  struct axis_data
  {
    rtc_float value{};
    rtc::axis split{};
  };

  struct leaf_data
  {
    std::uint32_t begin{};
    std::uint32_t count{};
  };

  static_assert(sizeof(axis_data) <= sizeof(leaf_data));

  union
  {
    axis_data axis{};
    leaf_data triangles;
  };

  std::unique_ptr<tree_node> left{}, right{};

  tree_node() : triangles{} {}

  [[nodiscard]] auto is_leaf() const noexcept -> bool { return !left && !right; }
};

}  // namespace rtc
