#include <array>
#include <vector>

#include <benchmark/benchmark.h>

#include "bounding_box.hpp"

namespace
{
constexpr std::array<rtc::math_point, 3> triangle_points{
    rtc::math_point{-1.0F, 3.0F, 0.5F},
    rtc::math_point{4.0F, -2.0F, 7.0F},
    rtc::math_point{2.0F, 5.0F, -3.0F},
};

const std::vector<rtc::math_point> point_cloud{
    {-8.0F, 3.0F, 1.0F},   {4.0F, -2.0F, 6.0F},   {1.0F, 9.0F, -7.0F},  {-5.0F, 2.0F, 8.0F},
    {7.0F, -6.0F, 0.5F},   {-3.0F, 4.0F, -9.0F},  {6.0F, 1.0F, 3.0F},   {-2.0F, -8.0F, 5.0F},
    {9.0F, 7.0F, -1.0F},   {-7.0F, -4.0F, 2.0F},  {3.0F, 6.0F, -5.0F},  {0.0F, -1.0F, 9.0F},
    {-6.0F, 8.0F, -2.0F},  {5.0F, -9.0F, 4.0F},   {2.5F, 3.5F, -6.0F}, {-4.5F, 0.5F, 7.5F},
    {8.5F, -3.5F, 1.5F},   {-1.5F, 5.5F, -8.5F}, {6.5F, 2.5F, 9.5F},  {-9.5F, -5.5F, 3.5F},
    {4.5F, 8.5F, -4.5F},   {-2.5F, -7.5F, 6.5F}, {7.5F, 1.5F, -0.5F}, {-8.5F, 4.5F, 2.5F},
    {1.5F, -6.5F, -9.5F},  {-5.5F, 9.5F, 5.5F},  {3.5F, -0.5F, 8.5F}, {-7.5F, 6.5F, -3.5F},
    {9.5F, -2.5F, 4.5F},   {-0.5F, 7.5F, -6.5F}, {5.5F, -8.5F, 0.0F},  {-4.0F, 1.0F, 10.0F},
};

const rtc::bounding_box query_box = rtc::bounding_box::from_boundaries(
    {-9.5F, -9.0F, -9.5F},
    {9.5F, 9.5F, 10.0F});

void BM_BoundingBoxTriangleConstruction(benchmark::State& state)
{
  for (auto _ : state)
  {
    auto a = triangle_points[0];
    auto b = triangle_points[1];
    auto c = triangle_points[2];
    benchmark::DoNotOptimize(a);
    benchmark::DoNotOptimize(b);
    benchmark::DoNotOptimize(c);

    rtc::bounding_box box{a, b, c};
    benchmark::DoNotOptimize(box);
  }
}

void BM_BoundingBoxVectorConstruction(benchmark::State& state)
{
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(point_cloud.data());
    rtc::bounding_box box{point_cloud};
    benchmark::DoNotOptimize(box);
  }
}

void BM_BoundingBoxDirectBoundaries(benchmark::State& state)
{
  for (auto _ : state)
  {
    rtc::math_point min{-9.5F, -9.0F, -9.5F};
    rtc::math_point max{9.5F, 9.5F, 10.0F};
    benchmark::DoNotOptimize(min);
    benchmark::DoNotOptimize(max);

    auto box = rtc::bounding_box::from_boundaries(min, max);
    benchmark::DoNotOptimize(box);
  }
}

void BM_BoundingBoxSurfaceArea(benchmark::State& state)
{
  for (auto _ : state)
  {
    auto box = query_box;
    benchmark::DoNotOptimize(box);
    benchmark::DoNotOptimize(box.surface_area());
  }
}

void BM_BoundingBoxMaximumExtent(benchmark::State& state)
{
  for (auto _ : state)
  {
    auto box = query_box;
    benchmark::DoNotOptimize(box);
    benchmark::DoNotOptimize(box.maximum_extent());
  }
}
}  // namespace

BENCHMARK(BM_BoundingBoxTriangleConstruction);
BENCHMARK(BM_BoundingBoxVectorConstruction);
BENCHMARK(BM_BoundingBoxDirectBoundaries);
BENCHMARK(BM_BoundingBoxSurfaceArea);
BENCHMARK(BM_BoundingBoxMaximumExtent);

BENCHMARK_MAIN();
