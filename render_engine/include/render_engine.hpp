#pragma once

#include <cstdint>
#include <memory>

#include "backward_al.hpp"
#include "bitmap.hpp"
#include "distributed_ray_tracing.hpp"
#include "intersection.hpp"
#include "math_ray.hpp"
#include "optical_camera_plane.hpp"
#include "rt_service.hpp"
#include "scene_model.hpp"
#include "scoped_timer.hpp"

namespace rtc
{
template <typename rt_algorithm = backward_al<::rtc::kdtree_rt, distributed_ray_tracing_shadows<>, void>>
class render_engine
{
 public:
  explicit render_engine(std::shared_ptr<const rtc::scene_model>);

  render_engine(render_engine&&) noexcept = default;
  render_engine(const render_engine&) = delete;
  auto operator=(render_engine&&) noexcept -> render_engine& = default;
  auto operator=(const render_engine&) -> render_engine& = delete;

  rtc_hot auto bitmap() -> rtc::bitmap;
  ~render_engine() = default;

 private:
  using rt_service = ::rtc::rt_service<typename rt_algorithm::rt_paramters>;

  static constexpr std::uint16_t tile_size{rt_service::default_tile_size};

  struct sync_trace_result
  {
    rtc::intersection intersection;

    [[nodiscard]] auto get() const noexcept -> rtc::intersection { return intersection; }
  };

  struct sync_rt_adapter
  {
    const rt_service& rt;

    [[nodiscard]] rtc_hot auto trace_ray(const rtc::math_ray& ray) const -> sync_trace_result
    {
      return {rt.trace_ray_sync(ray)};
    }
  };

  rt_service rt;
  const std::shared_ptr<const rtc::scene_model> scene;

  static auto make_rgb(const rtc::color& c) -> rtc::color_rgb
  {
    using type = rtc::color_rgb::value_type;
    return {c.red<type>(), c.green<type>(), c.blue<type>()};
  }
};

template <typename T>
render_engine<T>::render_engine(std::shared_ptr<const rtc::scene_model> s) : rt{s}, scene{std::move(s)}
{
}

template <typename rt_algorithm>
auto render_engine<rt_algorithm>::bitmap() -> rtc::bitmap
{
  const auto& res = scene->optical_system.screen.resolution;

  rtc::bitmap bmp(res.x, res.y);
  const rtc::optical_camera_plane op{scene->optical_system};

  rt_algorithm rt_alg{scene};
  {
    RTC_TRACE_SCOPE_CAT("ray_al::prework", "render_engine::bitmap");
    rt_alg.prework(bmp);
  }

  rt.for_each_tile(static_cast<std::uint16_t>(res.x),
                   static_cast<std::uint16_t>(res.y),
                   tile_size,
                   [&](const rtc::rt_tile& tile, auto& rt_backend) {
                     sync_rt_adapter sync_rt{rt_backend};
                     for (auto y = tile.y_begin; y < tile.y_end; ++y)
                     {
                       for (auto x = tile.x_begin; x < tile.x_end; ++x)
                       {
                         RTC_TRACE_SCOPE_CAT("pixel generation", "render_engine::tile");
                         const auto primary = op.emit_ray(x, y);
                         const auto c = rt_alg.make_color(primary, rtc::black, sync_rt);

                         DEBUG << "pixel[" << x << "," << y << "]"
                               << "ray " << primary.direction() << " color: " << c;
                         bmp.assign(x, y, make_rgb(c));
                       }
                     }
                   });

  rt_alg.postwork(bmp);

  return bmp;
}

}  // namespace rtc
