#pragma once

#include <initializer_list>
#include <ostream>
#include <vector>

#include "math_axis.hpp"
#include "math_point.hpp"
#include "math_vector.hpp"
#include "utility.hpp"

namespace rtc
{
class bounding_box
{
 public:
  explicit bounding_box(const std::vector<math_point>&) noexcept;
  bounding_box(std::initializer_list<math_point>) noexcept;

  [[nodiscard]] rtc_hot static auto from_boundaries(math_point min, math_point max) noexcept -> bounding_box
  {
    return bounding_box{boundaries_tag{}, min, max};
  }

  [[nodiscard]] auto min_boundary() const noexcept -> auto& { return pmin; }
  [[nodiscard]] auto max_boundary() const noexcept -> auto& { return pmax; }
  auto min_boundary() noexcept -> auto& { return pmin; }
  auto max_boundary() noexcept -> auto& { return pmax; }

  [[nodiscard]] auto diagonal() const noexcept { return pmax - pmin; }
  [[nodiscard]] rtc_hot auto maximum_extent() const noexcept -> rtc::axis
  {
    const auto x = pmax.x() - pmin.x();
    const auto y = pmax.y() - pmin.y();
    const auto z = pmax.z() - pmin.z();

    return x > y ? (x > z ? axis::x : axis::z) : (y > z ? axis::y : axis::z);
  }

  [[nodiscard]] rtc_hot auto surface_area() const noexcept -> rtc_float
  {
    const auto x = pmax.x() - pmin.x();
    const auto y = pmax.y() - pmin.y();
    const auto z = pmax.z() - pmin.z();

    return 2.0F * (x * y + x * z + y * z);
  }

 private:
  struct boundaries_tag
  {
  };

  bounding_box(boundaries_tag, math_point min, math_point max) noexcept : pmin{min}, pmax{max} {}

  template <typename B, typename E>
  bounding_box(B b, E e) noexcept;

  rtc::math_point pmin, pmax;
};

inline auto operator<<(std::ostream& ss, const bounding_box& bb) noexcept -> std::ostream&
{
  return ss << "[ " << bb.min_boundary() << " , " << bb.max_boundary() << " ]bb";
}

}  // namespace rtc
