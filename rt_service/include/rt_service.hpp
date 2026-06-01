#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>

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

struct rt_render_progress
{
  std::atomic<std::uint64_t> total_tiles{};
  std::atomic<std::uint64_t> submitted_tiles{};
  std::atomic<std::uint64_t> completed_tiles{};
  std::atomic<std::uint64_t> processed_pixels{};
  std::atomic<std::uint64_t> tile_time_us{};

  auto reset(const std::uint64_t total) noexcept -> void
  {
    total_tiles.store(total, std::memory_order_relaxed);
    submitted_tiles.store(0, std::memory_order_relaxed);
    completed_tiles.store(0, std::memory_order_relaxed);
    processed_pixels.store(0, std::memory_order_relaxed);
    tile_time_us.store(0, std::memory_order_relaxed);
  }
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
  auto execute(const rtc::rt_tile& tile,
               rtc::screen_surface& bmp,
               RtAlgorithm& rt_alg,
               rtc::rt_render_progress* progress = nullptr) const -> void;
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

    std::vector<std::function<void()>> pending_jobs;
    std::vector<std::function<void()>> active_jobs;
    std::vector<std::thread> workers;
    boost::mutex wait_mutex;
    boost::condition_variable wait_condition;
    std::mutex exception_mutex;
    std::exception_ptr worker_exception;
    std::atomic<std::size_t> next_job{};
    std::atomic<std::size_t> jobs_left{};
    std::atomic<int> workers_in_batch{};
    std::atomic<int> batch_generation{};
    std::atomic<bool> stopping{};

    std::atomic<std::uint64_t> submitted_tiles{};
    std::atomic<std::uint64_t> completed_tiles{};
    std::atomic<std::uint64_t> rendered_pixels{};
    std::atomic<std::uint64_t> traced_rays{};
    std::atomic<std::uint64_t> tile_time_us{};
    std::atomic<std::uint64_t> trace_time_us{};

   private:
    auto worker_loop() -> void;
    auto wait_for_change(std::atomic<int>& value, int expected) -> void;
    auto notify_one() -> void;
    auto notify_all() -> void;
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
  stopping.store(true, std::memory_order_release);
  batch_generation.fetch_add(1, std::memory_order_release);
  notify_all();

  for (auto& worker : workers)
  {
    if (worker.joinable())
      worker.join();
  }
}

template <typename T>
auto rt_service<T>::scheduler_state::submit(std::function<void()> job) -> void
{
  if (stopping.load(std::memory_order_acquire))
    throw std::runtime_error{"rt_service scheduler is stopping"};

  pending_jobs.emplace_back(std::move(job));
  ++submitted_tiles;
}

template <typename T>
auto rt_service<T>::scheduler_state::finish() -> void
{
  if (pending_jobs.empty())
    return;

  std::exception_ptr exception;
  {
    const std::lock_guard<std::mutex> lock{exception_mutex};
    exception = worker_exception;
    worker_exception = nullptr;
  }

  {
    active_jobs = std::move(pending_jobs);
    pending_jobs.clear();
    next_job.store(0, std::memory_order_relaxed);
    jobs_left.store(active_jobs.size(), std::memory_order_release);
    workers_in_batch.store(static_cast<int>(workers.size()), std::memory_order_release);
  }

  batch_generation.fetch_add(1, std::memory_order_release);
  notify_all();

  auto remaining_workers = workers_in_batch.load(std::memory_order_acquire);
  while (remaining_workers != 0)
  {
    wait_for_change(workers_in_batch, remaining_workers);
    remaining_workers = workers_in_batch.load(std::memory_order_acquire);
  }

  {
    const std::lock_guard<std::mutex> lock{exception_mutex};
    exception = worker_exception;
    worker_exception = nullptr;
  }

  active_jobs.clear();

  if (exception)
    std::rethrow_exception(exception);
}

template <typename T>
auto rt_service<T>::scheduler_state::worker_loop() -> void
{
  auto observed_generation = batch_generation.load(std::memory_order_acquire);

  while (true)
  {
    wait_for_change(batch_generation, observed_generation);

    if (stopping.load(std::memory_order_acquire))
      return;

    observed_generation = batch_generation.load(std::memory_order_acquire);

    while (true)
    {
      const auto index = next_job.fetch_add(1, std::memory_order_relaxed);
      if (index >= active_jobs.size())
        break;

      try
      {
        active_jobs[index]();
      }
      catch (...)
      {
        const std::lock_guard<std::mutex> lock{exception_mutex};
        if (!worker_exception)
          worker_exception = std::current_exception();
      }

      ++completed_tiles;
      jobs_left.fetch_sub(1, std::memory_order_acq_rel);
    }

    if (workers_in_batch.fetch_sub(1, std::memory_order_acq_rel) == 1)
    {
      notify_one();
    }
  }
}

template <typename T>
auto rt_service<T>::scheduler_state::wait_for_change(std::atomic<int>& value, const int expected) -> void
{
  boost::unique_lock<boost::mutex> lock{wait_mutex};
  wait_condition.wait(lock, [&] { return value.load(std::memory_order_acquire) != expected; });
}

template <typename T>
auto rt_service<T>::scheduler_state::notify_one() -> void
{
  wait_condition.notify_one();
}

template <typename T>
auto rt_service<T>::scheduler_state::notify_all() -> void
{
  wait_condition.notify_all();
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
auto rt_service<T>::execute(const rtc::rt_tile& tile,
                            rtc::screen_surface& bmp,
                            RtAlgorithm& rt_alg,
                            rtc::rt_render_progress* progress) const -> void
{
  if (rtc_unlikely(!scheduler))
    throw std::runtime_error{"rt_service is not able to schedule, probably it was moved from"};

  if (progress)
    ++progress->submitted_tiles;

  scheduler->submit([this, tile, &bmp, &rt_alg, progress] {
    const auto tile_start = now();
    render_tile(tile, bmp, rt_alg);
    if (progress)
    {
      const auto tile_pixels = static_cast<std::uint64_t>(tile.x_end - tile.x_begin) *
                               static_cast<std::uint64_t>(tile.y_end - tile.y_begin);
      progress->processed_pixels += tile_pixels;
      progress->tile_time_us += to_us(now() - tile_start);
      ++progress->completed_tiles;
    }
  });
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
