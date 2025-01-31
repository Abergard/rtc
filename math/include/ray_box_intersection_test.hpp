#pragma once

#include <vector>
#include <array>

#include "bounding_box.hpp"
#include "math_point.hpp"
#include "math_ray.hpp"
#include "utility.hpp"

namespace rtc
{

class ray_box_intersection_test
{
public:
  using tmin_t = rtc_float;
  using tmax_t = rtc_float;
  using ray_intersection_tuple = rtc::optional_tuple<tmin_t, tmax_t>;

  explicit ray_box_intersection_test(const std::vector<rtc::math_point>&) noexcept;
  rtc_hot [[nodiscard]] auto intersection_values_for(const rtc::math_ray& ray) const noexcept -> ray_intersection_tuple;

private:
  rtc::bounding_box bbox;
};

}
