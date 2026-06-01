#pragma once

#include <memory>

#include "bvh8.hpp"
#include "ray_tracer.hpp"
#include "scene_model.hpp"

namespace rtc
{
struct bvh8_rt : private rtc::ray_tracer<rtc::bvh8>
{
  static constexpr bool is_thread_safe{true};

 private:
  using base_t = rtc::ray_tracer<rtc::bvh8>;

 public:
  explicit bvh8_rt(const std::shared_ptr<const rtc::scene_model>& sc) : base_t{rtc::bvh8{*sc}, sc} {}

  using base_t::trace_ray;
};
}  // namespace rtc
