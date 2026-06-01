#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "intersection.hpp"
#include "kdtree_rt.hpp"
#include "math_ray.hpp"
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
  ~rt_service() = default;

  rt_service(rt_service&&) noexcept = default;
  rt_service(const rt_service&) = delete;
  auto operator=(rt_service&&) noexcept -> rt_service& = default;
  auto operator=(const rt_service&) -> rt_service& = delete;

  [[nodiscard]] rtc_hot auto trace_ray(const rtc::math_ray&) const -> trace_result;
  [[nodiscard]] rtc_hot auto trace_ray_sync(const rtc::math_ray&) const -> rtc::intersection;

  template <typename InputIt, typename OutputIt>
  auto trace_batch(InputIt first, InputIt last, OutputIt out) const -> void;

  template <typename Job>
  auto for_each_tile(std::uint16_t width,
                     std::uint16_t height,
                     std::uint16_t tile_size,
                     Job&& job) const -> void;

  auto thread_number() const noexcept -> std::uint32_t { return worker_thread_number(); }

 private:
  std::unique_ptr<const ray_tracer> rt_search;

  static auto ready_trace_result(rtc::intersection intersection) -> trace_result;
  static auto worker_thread_number() noexcept -> std::uint32_t;
};

template <typename T>
rt_service<T>::rt_service(std::shared_ptr<const rtc::scene_model> sc)
{
  if (rtc_unlikely(!sc))
    throw std::runtime_error{"Pointer to scene_model is null."};

  rt_search = std::make_unique<ray_tracer>(std::move(sc));
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
template <typename Job>
auto rt_service<T>::for_each_tile(const std::uint16_t width,
                                  const std::uint16_t height,
                                  const std::uint16_t tile_size,
                                  Job&& job) const -> void
{
  const auto effective_tile = std::max<std::uint16_t>(1, tile_size);
  std::vector<rt_tile> tiles;
  tiles.reserve(((width + effective_tile - 1) / effective_tile) * ((height + effective_tile - 1) / effective_tile));

  for (std::uint16_t y{}; y < height; y = static_cast<std::uint16_t>(y + effective_tile))
  {
    for (std::uint16_t x{}; x < width; x = static_cast<std::uint16_t>(x + effective_tile))
    {
      tiles.push_back({
          x,
          y,
          static_cast<std::uint16_t>(std::min<std::uint32_t>(width, x + effective_tile)),
          static_cast<std::uint16_t>(std::min<std::uint32_t>(height, y + effective_tile)),
      });
    }
  }

  const auto workers = std::min<std::size_t>(std::max<std::size_t>(1, worker_thread_number()), tiles.size());
  std::atomic<std::size_t> next_tile{};
  std::exception_ptr worker_exception;
  std::mutex worker_exception_mutex;
  std::vector<std::thread> threads;
  threads.reserve(workers);

  for (std::size_t worker{}; worker < workers; ++worker)
  {
    threads.emplace_back([&] {
      try
      {
        while (true)
        {
          const auto index = next_tile.fetch_add(1, std::memory_order_relaxed);
          if (index >= tiles.size())
            break;

          job(tiles[index], *this);
        }
      }
      catch (...)
      {
        const std::lock_guard<std::mutex> lock{worker_exception_mutex};
        if (!worker_exception)
          worker_exception = std::current_exception();
      }
    });
  }

  for (auto& thread : threads)
    thread.join();

  if (worker_exception)
    std::rethrow_exception(worker_exception);
}

template <typename T>
auto rt_service<T>::worker_thread_number() noexcept -> std::uint32_t
{
  if (!T::is_thread_safe)
    return 1U;

  return std::max(1U, std::thread::hardware_concurrency());
}

}  // namespace rtc
