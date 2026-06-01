#include <array>
#include <cstddef>
#include <memory>

#include <benchmark/benchmark.h>

#include "brs.hpp"
#include "kd_tree.hpp"
#include "math_ray.hpp"
#include "ray_tracer.hpp"

namespace
{
struct scene_case
{
  std::shared_ptr<rtc::brs> scene;
  std::array<rtc::math_ray, 3> rays;
};

auto room_case() -> scene_case&
{
  static scene_case data = [] {
    auto scene = std::make_shared<rtc::brs>("./room.xml");
    return scene_case{
        scene,
        {
            rtc::math_ray{{-6.788838F, 2.300512F, 4.642227F}, scene->optical_system.view_point},
            rtc::math_ray{{-5.145645F, -0.382301F, 5.584966F}, scene->optical_system.view_point},
            rtc::math_ray{{-4.396932F, -0.778016F, 6.014521F}, scene->optical_system.view_point},
        },
    };
  }();
  return data;
}

auto ulica_case() -> scene_case&
{
  static scene_case data{
      std::make_shared<rtc::brs>("./ulica.xml"),
      {
          rtc::math_ray{{35.994164F, 12.552099F, 10.576778F}, {78.202606F, 28.281248F, 25.230000F}},
          rtc::math_ray{{-12.702606F, -25.481249F, 39.070004F}, {78.202606F, 28.281248F, 25.230000F}},
          rtc::math_ray{{-22.110607F, -23.501247F, 21.443001F}, {78.202606F, 28.281248F, 25.230000F}},
      },
  };
  return data;
}

void run_traversal_benchmark(benchmark::State& state, scene_case& data)
{
  const rtc::kd_tree tree{*data.scene};
  std::size_t leaves{};
  std::size_t candidates{};

  for (auto _ : state)
  {
    leaves = 0;
    candidates = 0;

    for (const auto& ray : data.rays)
    {
      for (auto it = tree.cbegin(ray); it != tree.cend(ray); ++it)
      {
        ++leaves;
        for (auto triangle_index : *it)
        {
          benchmark::DoNotOptimize(triangle_index);
          ++candidates;
        }
      }
    }

    benchmark::DoNotOptimize(leaves);
    benchmark::DoNotOptimize(candidates);
  }

  state.counters["leaves"] = benchmark::Counter(static_cast<double>(leaves), benchmark::Counter::kAvgIterations);
  state.counters["candidates"] = benchmark::Counter(static_cast<double>(candidates), benchmark::Counter::kAvgIterations);
}

void run_trace_benchmark(benchmark::State& state, scene_case& data)
{
  rtc::ray_tracer<rtc::kd_tree> tracer{rtc::kd_tree{*data.scene}, data.scene};

  for (auto _ : state)
  {
    for (const auto& ray : data.rays)
    {
      auto intersection = tracer.trace_ray(ray);
      benchmark::DoNotOptimize(intersection);
    }
  }
}

void BM_KdTreeTraversalRoom(benchmark::State& state) { run_traversal_benchmark(state, room_case()); }
void BM_KdTreeTraversalUlica(benchmark::State& state) { run_traversal_benchmark(state, ulica_case()); }
void BM_KdTreeTraceRoom(benchmark::State& state) { run_trace_benchmark(state, room_case()); }
void BM_KdTreeTraceUlica(benchmark::State& state) { run_trace_benchmark(state, ulica_case()); }
}  // namespace

BENCHMARK(BM_KdTreeTraversalRoom);
BENCHMARK(BM_KdTreeTraversalUlica);
BENCHMARK(BM_KdTreeTraceRoom);
BENCHMARK(BM_KdTreeTraceUlica);

BENCHMARK_MAIN();
