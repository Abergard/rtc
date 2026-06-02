#include <array>
#include <cstddef>
#include <memory>
#include <vector>

#include <benchmark/benchmark.h>

#include "distributed_ray_tracing.hpp"
#include "scene_model.hpp"
#include "shadow_ray.hpp"

namespace
{
struct fake_trace_result
{
  rtc::intersection value{};
  auto get() const noexcept -> rtc::intersection { return value; }
};

struct fake_rt_service
{
  std::vector<rtc::intersection> intersections{};
  std::size_t next{};
  std::size_t trace_calls{};

  auto reset() noexcept -> void
  {
    next = 0;
    trace_calls = 0;
  }

  auto trace_ray(const rtc::math_ray&) noexcept -> fake_trace_result
  {
    ++trace_calls;
    if (next < intersections.size())
      return {intersections[next++]};

    return {};
  }
};

auto benchmark_scene() -> std::shared_ptr<rtc::scene_model>
{
  auto scene = std::make_shared<rtc::scene_model>();
  scene->ambient = rtc::black;
  scene->optical_system.view_point = {0.0F, 0.0F, 100.0F};
  scene->points = {
      {-1.0F, -1.0F, 0.0F},
      {1.0F, -1.0F, 0.0F},
      {0.0F, 1.0F, 0.0F},
      {-1.0F, -1.0F, 5.0F},
      {1.0F, -1.0F, 5.0F},
      {0.0F, 1.0F, 5.0F},
  };
  scene->triangles = {
      {0, 1, 2},
      {3, 4, 5},
  };
  scene->material_id = {0, 1};
  scene->materials.resize(2);

  auto& receiver = scene->materials[0];
  receiver.kd = 1.0F;
  receiver.ka = 0.0F;
  receiver.ks = 0.0F;
  receiver.ktd = 0.0F;
  receiver.kts = 0.0F;
  receiver.shadowfall = true;
  receiver.material_color = {1.0F, 1.0F, 1.0F};

  auto& blocker = scene->materials[1];
  blocker.kd = 1.0F;
  blocker.ka = 0.0F;
  blocker.kts = 0.25F;
  blocker.ktd = 0.25F;
  blocker.material_color = {1.0F, 1.0F, 1.0F};

  rtc::light light;
  light.light_color = {1.0F, 1.0F, 1.0F};
  light.position = {0.0F, 0.0F, 10.0F};
  light.inv_square = {0.0F, 0.0F, 1.0F};
  scene->lights = {light};

  return scene;
}

struct benchmark_context
{
  std::shared_ptr<rtc::scene_model> scene{benchmark_scene()};
  rtc::distributed_ray_tracing_shadows<> shadows{scene};
  rtc::math_ray camera_ray{{0.0F, 0.0F, -1.0F}, {0.0F, 0.0F, 100.0F}};
  rtc::intersection receiver{0, 100.0F};
};

auto context() -> benchmark_context&
{
  static benchmark_context ctx;
  return ctx;
}

void run_compute_color(benchmark::State& state, fake_rt_service& rt)
{
  auto& ctx = context();
  std::size_t trace_calls{};

  for (auto _ : state)
  {
    rt.reset();
    auto color = ctx.shadows.compute_color(ctx.camera_ray, ctx.receiver, {}, {}, rt);
    trace_calls += rt.trace_calls;
    benchmark::DoNotOptimize(color);
  }

  state.counters["trace_calls"] = benchmark::Counter(static_cast<double>(trace_calls), benchmark::Counter::kAvgIterations);
}

void BM_ComputeColorShadowfallDisabled(benchmark::State& state)
{
  auto& ctx = context();
  auto shadowfall = ctx.scene->materials[0].shadowfall;
  ctx.scene->materials[0].shadowfall = false;
  fake_rt_service rt;

  run_compute_color(state, rt);

  ctx.scene->materials[0].shadowfall = shadowfall;
}

void BM_ComputeColorUnblockedLight(benchmark::State& state)
{
  fake_rt_service rt{{rtc::no_intersection}};
  run_compute_color(state, rt);
}

void BM_ComputeColorOpaqueBlocker(benchmark::State& state)
{
  auto& ctx = context();
  auto kts = ctx.scene->materials[1].kts;
  auto ktd = ctx.scene->materials[1].ktd;
  ctx.scene->materials[1].kts = 0.0F;
  ctx.scene->materials[1].ktd = 0.0F;
  fake_rt_service rt{{rtc::intersection{1, 0.5F}}};

  run_compute_color(state, rt);

  ctx.scene->materials[1].kts = kts;
  ctx.scene->materials[1].ktd = ktd;
}

void BM_ComputeColorTransparentBlocker(benchmark::State& state)
{
  fake_rt_service rt{{rtc::intersection{1, 0.5F}, rtc::no_intersection}};
  run_compute_color(state, rt);
}

void BM_ShadowRayTraceUnblocked(benchmark::State& state)
{
  auto& ctx = context();
  fake_rt_service rt{{rtc::no_intersection}};
  std::size_t trace_calls{};

  for (auto _ : state)
  {
    rt.reset();
    rtc::shadow_ray ray{*ctx.scene, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 10.0F}, ctx.scene->lights.front()};
    auto result = ray.trace(rt);
    trace_calls += rt.trace_calls;
    benchmark::DoNotOptimize(result);
  }

  state.counters["trace_calls"] = benchmark::Counter(static_cast<double>(trace_calls), benchmark::Counter::kAvgIterations);
}

void BM_ShadowRayTraceTransparentBlocker(benchmark::State& state)
{
  auto& ctx = context();
  fake_rt_service rt{{rtc::intersection{1, 0.5F}, rtc::no_intersection}};
  std::size_t trace_calls{};

  for (auto _ : state)
  {
    rt.reset();
    rtc::shadow_ray ray{*ctx.scene, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 10.0F}, ctx.scene->lights.front()};
    auto result = ray.trace(rt);
    trace_calls += rt.trace_calls;
    benchmark::DoNotOptimize(result);
  }

  state.counters["trace_calls"] = benchmark::Counter(static_cast<double>(trace_calls), benchmark::Counter::kAvgIterations);
}
}  // namespace

BENCHMARK(BM_ComputeColorShadowfallDisabled);
BENCHMARK(BM_ComputeColorUnblockedLight);
BENCHMARK(BM_ComputeColorOpaqueBlocker);
BENCHMARK(BM_ComputeColorTransparentBlocker);
BENCHMARK(BM_ShadowRayTraceUnblocked);
BENCHMARK(BM_ShadowRayTraceTransparentBlocker);

BENCHMARK_MAIN();
