#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "backward_al.hpp"
#include "bitmap.hpp"
#include "distributed_ray_tracing.hpp"
#include "rt_service.hpp"
#include "scene_model.hpp"
#include "scoped_timer.hpp"

namespace rtc
{
template <typename rt_algorithm = backward_al<::rtc::kdtree_rt, distributed_ray_tracing_shadows<>, void>>
class render_engine
{
 public:
  explicit render_engine(std::shared_ptr<const rtc::scene_model>, std::shared_ptr<rtc::rt_render_progress> = {});

  render_engine(render_engine&&) noexcept = default;
  render_engine(const render_engine&) = delete;
  auto operator=(render_engine&&) noexcept -> render_engine& = default;
  auto operator=(const render_engine&) -> render_engine& = delete;

  rtc_hot auto bitmap() -> rtc::screen_surface;
  ~render_engine() = default;

 private:
  using rt_service = ::rtc::rt_service<typename rt_algorithm::rt_paramters>;

  static constexpr std::uint16_t tile_size{rt_service::default_tile_size};
  static constexpr std::uint16_t sample_number{1};

  rt_service rt;
  const std::shared_ptr<const rtc::scene_model> scene;
  std::shared_ptr<rtc::rt_render_progress> progress;

  [[nodiscard]] static auto make_tiles(std::uint16_t width, std::uint16_t height) -> std::vector<rtc::rt_tile>;
};

template <typename T>
render_engine<T>::render_engine(std::shared_ptr<const rtc::scene_model> s, std::shared_ptr<rtc::rt_render_progress> p)
    : rt{s}, scene{std::move(s)}, progress{std::move(p)}
{
}

template <typename rt_algorithm>
auto render_engine<rt_algorithm>::make_tiles(const std::uint16_t width,
                                             const std::uint16_t height) -> std::vector<rtc::rt_tile>
{
  std::vector<rtc::rt_tile> tiles;
  tiles.reserve(((width + tile_size - 1) / tile_size) * ((height + tile_size - 1) / tile_size));

  for (std::uint16_t y{}; y < height; y = static_cast<std::uint16_t>(y + tile_size))
  {
    for (std::uint16_t x{}; x < width; x = static_cast<std::uint16_t>(x + tile_size))
    {
      tiles.push_back({
          x,
          y,
          static_cast<std::uint16_t>(std::min<std::uint32_t>(width, x + tile_size)),
          static_cast<std::uint16_t>(std::min<std::uint32_t>(height, y + tile_size)),
      });
    }
  }

  return tiles;
}

template <typename rt_algorithm>
auto render_engine<rt_algorithm>::bitmap() -> rtc::screen_surface
{
  const auto& res = scene->optical_system.screen.resolution;
  const auto width = static_cast<std::uint16_t>(res.x);
  const auto height = static_cast<std::uint16_t>(res.y);

  rtc::screen_surface bmp(width, height);

  rt_algorithm rt_alg{scene};
  {
    RTC_TRACE_SCOPE_CAT("ray_al::prework", "render_engine::bitmap");
    rt_alg.prework(bmp);
  }

  const auto tiles = make_tiles(width, height);
  if (progress)
    progress->reset(static_cast<std::uint64_t>(tiles.size()) * sample_number);

  for (std::uint16_t sample{}; sample < sample_number; ++sample)
  {
    for (const auto& tile : tiles)
      rt.execute(tile, bmp, rt_alg, progress.get());

    rt.finish();
  }

  rt_alg.postwork(bmp);

  return bmp;
}

}  // namespace rtc
