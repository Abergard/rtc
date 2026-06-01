#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
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
#include "rtc_log.hpp"
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

struct rt_service_stats
{
  std::uint64_t submitted_tiles{};
  std::uint64_t completed_tiles{};
  std::uint64_t rendered_pixels{};
  std::uint64_t traced_rays{};
  std::uint64_t tile_time_us{};
  std::uint64_t trace_time_us{};
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
  auto execute(const rtc::rt_tile& tile, rtc::screen_surface& bmp, RtAlgorithm& rt_alg) const -> void;
  auto finish() const -> void;
  [[nodiscard]] auto stats() const noexcept -> rt_service_stats;

  auto thread_number() const noexcept -> std::uint32_t { return worker_thread_number(); }

 private:
  struct scheduler_state
  {
    explicit scheduler_state(std::uint32_t worker_count);
    ~scheduler_state() noexcept;

    scheduler_state(scheduler_state&&) = delete;
    scheduler_state(const scheduler_state&) = delete;
    auto operator=(scheduler_state&&) -> scheduler_state& = delete;
    auto operator=(const scheduler_state&) -> scheduler_state& = delete;

    auto submit(std::function<void()> job) -> void;
    auto finish() -> void;

    mutable std::mutex mutex;
    std::condition_variable job_available;
    std::condition_variable job_finished;
    std::deque<std::function<void()>> jobs;
    std::vector<std::thread> workers;
    std::exception_ptr worker_exception;
    std::size_t active_jobs{};
    bool stopping{};

    std::atomic<std::uint64_t> submitted_tiles{};
    std::atomic<std::uint64_t> completed_tiles{};
    std::atomic<std::uint64_t> rendered_pixels{};
    std::atomic<std::uint64_t> traced_rays{};
    std::atomic<std::uint64_t> tile_time_us{};
    std::atomic<std::uint64_t> trace_time_us{};

   private:
    auto worker_loop() -> void;
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
  std::shared_ptr<scheduler_state> scheduler;

  template <typename RtAlgorithm>
  auto render_tile(const rtc::rt_tile& tile, rtc::screen_surface& bmp, RtAlgorithm& rt_alg) const -> void;
  [[nodiscard]] static auto make_rgb(const rtc::color& c) -> rtc::color_rgb;
  static auto ready_trace_result(rtc::intersection intersection) -> trace_result;
  auto finish_noexcept() const noexcept -> void;
  [[nodiscard]] static auto now() noexcept;
  template <typename Duration>
  [[nodiscard]] static auto to_us(Duration duration) noexcept -> std::uint64_t;
  static auto worker_thread_number() noexcept -> std::uint32_t;
};

template <typename T>
rt_service<T>::scheduler_state::scheduler_state(const std::uint32_t worker_count)
{
  const auto count = std::max(1U, worker_count);
  workers.reserve(count);

  for (std::uint32_t i{}; i < count; ++i)
    workers.emplace_back([this] { worker_loop(); });
}

template <typename T>
rt_service<T>::scheduler_state::~scheduler_state() noexcept
{
  {
    const std::lock_guard<std::mutex> lock{mutex};
    stopping = true;
  }

  job_available.notify_all();

  for (auto& worker : workers)
  {
    if (worker.joinable())
      worker.join();
  }
}

template <typename T>
auto rt_service<T>::scheduler_state::submit(std::function<void()> job) -> void
{
  {
    const std::lock_guard<std::mutex> lock{mutex};
    if (stopping)
      throw std::runtime_error{"rt_service scheduler is stopping"};

    jobs.emplace_back(std::move(job));
    ++submitted_tiles;
  }

  job_available.notify_one();
}

template <typename T>
auto rt_service<T>::scheduler_state::finish() -> void
{
  std::exception_ptr exception;
  {
    std::unique_lock<std::mutex> lock{mutex};
    job_finished.wait(lock, [this] { return jobs.empty() && active_jobs == 0; });
    exception = worker_exception;
    worker_exception = nullptr;
  }

  if (exception)
    std::rethrow_exception(exception);
}

template <typename T>
auto rt_service<T>::scheduler_state::worker_loop() -> void
{
  while (true)
  {
    std::function<void()> job;
    {
      std::unique_lock<std::mutex> lock{mutex};
      job_available.wait(lock, [this] { return stopping || !jobs.empty(); });

      if (stopping && jobs.empty())
        return;

      job = std::move(jobs.front());
      jobs.pop_front();
      ++active_jobs;
    }

    try
    {
      job();
    }
    catch (...)
    {
      const std::lock_guard<std::mutex> lock{mutex};
      if (!worker_exception)
        worker_exception = std::current_exception();
    }

    {
      const std::lock_guard<std::mutex> lock{mutex};
      --active_jobs;
      ++completed_tiles;
    }
    job_finished.notify_all();
  }
}

template <typename T>
rt_service<T>::rt_service(std::shared_ptr<const rtc::scene_model> sc)
    : scene{std::move(sc)}, scheduler{std::make_shared<scheduler_state>(worker_thread_number())}
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

  const auto start = now();
  auto intersection = rt_search->trace_ray(ray);
  if (scheduler)
  {
    ++scheduler->traced_rays;
    scheduler->trace_time_us += to_us(now() - start);
  }

  return intersection;
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
auto rt_service<T>::execute(const rtc::rt_tile& tile, rtc::screen_surface& bmp, RtAlgorithm& rt_alg) const -> void
{
  if (rtc_unlikely(!scheduler))
    throw std::runtime_error{"rt_service is not able to schedule, probably it was moved from"};

  scheduler->submit([this, tile, &bmp, &rt_alg] { render_tile(tile, bmp, rt_alg); });
}

template <typename T>
auto rt_service<T>::finish() const -> void
{
  if (scheduler)
    scheduler->finish();

  const auto snapshot = stats();
  if (snapshot.completed_tiles)
  {
    RELEASE << "rt_service stats: tiles " << snapshot.completed_tiles << "/" << snapshot.submitted_tiles
            << ", pixels " << snapshot.rendered_pixels
            << ", rays " << snapshot.traced_rays
            << ", tile_time_us " << snapshot.tile_time_us
            << ", trace_time_us " << snapshot.trace_time_us;
  }
}

template <typename T>
auto rt_service<T>::stats() const noexcept -> rt_service_stats
{
  if (!scheduler)
    return {};

  return {
      scheduler->submitted_tiles.load(std::memory_order_relaxed),
      scheduler->completed_tiles.load(std::memory_order_relaxed),
      scheduler->rendered_pixels.load(std::memory_order_relaxed),
      scheduler->traced_rays.load(std::memory_order_relaxed),
      scheduler->tile_time_us.load(std::memory_order_relaxed),
      scheduler->trace_time_us.load(std::memory_order_relaxed),
  };
}

template <typename T>
template <typename RtAlgorithm>
auto rt_service<T>::render_tile(const rtc::rt_tile& tile, rtc::screen_surface& bmp, RtAlgorithm& rt_alg) const -> void
{
  const auto tile_start = now();
  const rtc::optical_camera_plane op{scene->optical_system};
  sync_rt_adapter sync_rt{*this};
  std::uint64_t rendered_pixels{};

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
      ++rendered_pixels;
    }
  }

  if (scheduler)
  {
    scheduler->rendered_pixels += rendered_pixels;
    scheduler->tile_time_us += to_us(now() - tile_start);
  }
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
auto rt_service<T>::now() noexcept
{
  return std::chrono::steady_clock::now();
}

template <typename T>
template <typename Duration>
auto rt_service<T>::to_us(Duration duration) noexcept -> std::uint64_t
{
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
}

template <typename T>
auto rt_service<T>::worker_thread_number() noexcept -> std::uint32_t
{
  if (!T::is_thread_safe)
    return 1U;

  return std::max(1U, std::thread::hardware_concurrency());
}

}  // namespace rtc
