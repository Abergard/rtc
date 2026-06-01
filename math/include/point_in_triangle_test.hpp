#pragma once

#include "math_vector.hpp"

namespace rtc
{
class point_in_triangle_test
{
 public:
  rtc_hot point_in_triangle_test(const math_point& p1, const math_point& p2, const math_point& p3) noexcept
  {
    const auto ux = p1.x() - p2.x();
    const auto uy = p1.y() - p2.y();
    const auto uz = p1.z() - p2.z();
    const auto vx = p3.x() - p2.x();
    const auto vy = p3.y() - p2.y();
    const auto vz = p3.z() - p2.z();
    const auto nx = std::fabs(uy * vz - uz * vy);
    const auto ny = std::fabs(uz * vx - ux * vz);
    const auto nz = std::fabs(ux * vy - uy * vx);

    if (nx > ny && nx > nz)
    {
      projection_x_axis = rtc::axis::y;
      projection_y_axis = rtc::axis::z;
    }
    else if (ny > nz)
    {
      projection_x_axis = rtc::axis::x;
      projection_y_axis = rtc::axis::z;
    }
    else
    {
      projection_x_axis = rtc::axis::x;
      projection_y_axis = rtc::axis::y;
    }

    compute_all_needed_factors(p1, p2, p3);
  }

  [[nodiscard]] rtc_hot auto triangle_contains(const math_point& p) const noexcept -> bool
  {
    const auto vec_x = p.axis(projection_x_axis) - origin_x;
    const auto vec_y = p.axis(projection_y_axis) - origin_y;
    const auto ac = ac_x_factor * vec_x + ac_y_factor * vec_y;
    const auto bc = bc_x_factor * vec_x + bc_y_factor * vec_y;

    return (ac >= 0.0F) && (bc >= 0.0F) && (ac + bc <= 1.0F);
  }

 private:
  rtc_hot auto compute_all_needed_factors(const math_point& p1, const math_point& p2, const math_point& p3) noexcept
      -> void
  {
    const auto x1 = p1.axis(projection_x_axis);
    const auto y1 = p1.axis(projection_y_axis);
    const auto x2 = p2.axis(projection_x_axis);
    const auto y2 = p2.axis(projection_y_axis);
    const auto x3 = p3.axis(projection_x_axis);
    const auto y3 = p3.axis(projection_y_axis);
    const rtc_float c = 1.0F / ((x1 - x2) * (y3 - y2) - (x3 - x2) * (y1 - y2));

    ac_x_factor = c * (y3 - y2);
    ac_y_factor = c * (x2 - x3);
    bc_x_factor = c * (y2 - y1);
    bc_y_factor = c * (x1 - x2);
    origin_x = x2;
    origin_y = y2;
  }

  rtc_float ac_x_factor{};
  rtc_float ac_y_factor{};
  rtc_float bc_x_factor{};
  rtc_float bc_y_factor{};
  rtc_float origin_x{};
  rtc_float origin_y{};
  rtc::axis projection_x_axis{};
  rtc::axis projection_y_axis{};
};

}  // namespace rtc
