#pragma once

#include <memory>

#include "bvh.hpp"
#include "ray_tracer.hpp"
#include "scene_model.hpp"

namespace rtc
{
struct bvh_rt : private rtc::ray_tracer<rtc::bvh>
{
  static constexpr bool is_thread_safe{true};

 private:
  using base_t = rtc::ray_tracer<rtc::bvh>;

 public:
  explicit bvh_rt(const std::shared_ptr<const rtc::scene_model>& sc) : base_t{rtc::bvh{*sc}, sc} {}

  using base_t::trace_ray;
};
}  // namespace rtc
