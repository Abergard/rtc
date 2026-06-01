#pragma once

#include <algorithm>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "bitmap.hpp"
#include "color.hpp"
#include "intersection.hpp"
#include "kdtree_rt.hpp"
#include "math_ray.hpp"
#include "optical_camera_plane.hpp"
#include "scene_model.hpp"
#include "utility.hpp"

namespace rtc
{
struct rt_tile
{
  std::uint16_t x_begin{};
  std::uint16_t y_begin{};
  std::uint16_t x_end{};
  std::uint16_t y_end{};
};

template <typename _rt = ::rtc::kdtree_rt>
class rt_service
{
 public:
  static constexpr std::size_t queue_capacity{256};
  static constexpr std::uint16_t default_tile_size{16};

  using ray_tracer = _rt;
  using trace_result = std::future<rtc::intersection>;

  explicit rt_service(std::shared_ptr<const rtc::scene_model>);
  ~rt_service() noexcept;

  rt_service(rt_service&&) noexcept = default;
  rt_service(const rt_service&) = delete;
  auto operator=(rt_service&&) noexcept -> rt_service& = default;
  auto operator=(const rt_service&) -> rt_service& = delete;

  [[nodiscard]] rtc_hot auto trace_ray(const rtc::math_ray&) const -> trace_result;
  [[nodiscard]] rtc_hot auto trace_ray_sync(const rtc::math_ray&) const -> rtc::intersection;

  template <typename InputIt, typename OutputIt>
  auto trace_batch(InputIt first, InputIt last, OutputIt out) const -> void;

  template <typename RtAlgorithm>
  auto execute(const rtc::rt_tile& tile, rtc::bitmap& bmp, RtAlgorithm& rt_alg) const -> void;
  auto finish() const -> void;

  auto thread_number() const noexcept -> std::uint32_t { return worker_thread_number(); }

 private:
  struct scheduler_state
  {
    mutable std::mutex mutex;
    std::vector<std::future<void>> pending_jobs;
  };

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

  const std::shared_ptr<const rtc::scene_model> scene;
  std::unique_ptr<const ray_tracer> rt_search;
  std::shared_ptr<scheduler_state> scheduler{std::make_shared<scheduler_state>()};

  template <typename RtAlgorithm>
  auto render_tile(const rtc::rt_tile& tile, rtc::bitmap& bmp, RtAlgorithm& rt_alg) const -> void;
  [[nodiscard]] static auto make_rgb(const rtc::color& c) -> rtc::color_rgb;
  static auto ready_trace_result(rtc::intersection intersection) -> trace_result;
  auto wait_next_job() const -> bool;
  auto finish_noexcept() const noexcept -> void;
  static auto worker_thread_number() noexcept -> std::uint32_t;
};

template <typename T>
rt_service<T>::rt_service(std::shared_ptr<const rtc::scene_model> sc) : scene{std::move(sc)}
{
  if (rtc_unlikely(!scene))
    throw std::runtime_error{"Pointer to scene_model is null."};

  rt_search = std::make_unique<ray_tracer>(scene);
}

template <typename T>
rt_service<T>::~rt_service() noexcept
{
  finish_noexcept();
}

template <typename T>
auto rt_service<T>::trace_ray(const rtc::math_ray& ray) const -> trace_result
{
  return ready_trace_result(trace_ray_sync(ray));
}

template <typename T>
auto rt_service<T>::trace_ray_sync(const rtc::math_ray& ray) const -> rtc::intersection
{
  if (rtc_unlikely(!rt_search))
    throw std::runtime_error{"ray_trace is not able to trace, probably it was moved from"};

  return rt_search->trace_ray(ray);
}

template <typename T>
auto rt_service<T>::ready_trace_result(rtc::intersection intersection) -> trace_result
{
  std::promise<rtc::intersection> promise;
  promise.set_value(std::move(intersection));
  return promise.get_future();
}

template <typename T>
template <typename InputIt, typename OutputIt>
auto rt_service<T>::trace_batch(InputIt first, InputIt last, OutputIt out) const -> void
{
  std::transform(first, last, out, [this](const auto& ray) { return trace_ray_sync(ray); });
}

template <typename T>
template <typename RtAlgorithm>
auto rt_service<T>::execute(const rtc::rt_tile& tile, rtc::bitmap& bmp, RtAlgorithm& rt_alg) const -> void
{
  if (rtc_unlikely(!scheduler))
    throw std::runtime_error{"rt_service is not able to schedule, probably it was moved from"};

  const auto workers = std::max<std::size_t>(1, worker_thread_number());
  while (true)
  {
    {
      const std::lock_guard<std::mutex> lock{scheduler->mutex};
      if (scheduler->pending_jobs.size() < workers)
        break;
    }

    wait_next_job();
  }

  auto job = std::async(std::launch::async, [this, tile, &bmp, &rt_alg] { render_tile(tile, bmp, rt_alg); });

  const std::lock_guard<std::mutex> lock{scheduler->mutex};
  scheduler->pending_jobs.emplace_back(std::move(job));
}

template <typename T>
auto rt_service<T>::finish() const -> void
{
  while (wait_next_job())
  {
  }
}

template <typename T>
template <typename RtAlgorithm>
auto rt_service<T>::render_tile(const rtc::rt_tile& tile, rtc::bitmap& bmp, RtAlgorithm& rt_alg) const -> void
{
  const rtc::optical_camera_plane op{scene->optical_system};
  sync_rt_adapter sync_rt{*this};

  for (auto y = tile.y_begin; y < tile.y_end; ++y)
  {
    for (auto x = tile.x_begin; x < tile.x_end; ++x)
    {
      RTC_TRACE_SCOPE_CAT("pixel generation", "rt_service::execute");
      const auto primary = op.emit_ray(x, y);
      const auto c = rt_alg.make_color(primary, rtc::black, sync_rt);

      DEBUG << "pixel[" << x << "," << y << "]"
            << "ray " << primary.direction() << " color: " << c;
      bmp.assign(x, y, make_rgb(c));
    }
  }
}

template <typename T>
auto rt_service<T>::wait_next_job() const -> bool
{
  if (!scheduler)
    return false;

  std::future<void> job;
  {
    const std::lock_guard<std::mutex> lock{scheduler->mutex};
    if (scheduler->pending_jobs.empty())
      return false;

    job = std::move(scheduler->pending_jobs.front());
    scheduler->pending_jobs.erase(scheduler->pending_jobs.begin());
  }

  job.get();
  return true;
}

template <typename T>
auto rt_service<T>::finish_noexcept() const noexcept -> void
{
  try
  {
    finish();
  }
  catch (...)
  {
  }
}

template <typename T>
auto rt_service<T>::make_rgb(const rtc::color& c) -> rtc::color_rgb
{
  using type = rtc::color_rgb::value_type;
  return {c.red<type>(), c.green<type>(), c.blue<type>()};
}

template <typename T>
auto rt_service<T>::worker_thread_number() noexcept -> std::uint32_t
{
  if (!T::is_thread_safe)
    return 1U;

  return std::max(1U, std::thread::hardware_concurrency());
}

}  // namespace rtc
