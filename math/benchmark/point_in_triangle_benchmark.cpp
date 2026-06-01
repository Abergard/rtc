#include <array>
#include <tuple>

#include <benchmark/benchmark.h>

#include "point_in_triangle_test.hpp"

namespace
{
class legacy_point_in_triangle_test
{
 public:
  legacy_point_in_triangle_test(const rtc::math_point& p1, const rtc::math_point& p2, const rtc::math_point& p3) noexcept
  {
    const auto n = cross(p1 - p2, p3 - p2);
    biggest_axis = rtc::axis_of_biggest_vec_comp(n);
    compute_all_needed_factors(p1, p2, p3);
  }

  [[nodiscard]] auto triangle_contains(const rtc::math_point& p) const noexcept -> bool
  {
    const auto [x, y] = get_projected_point(p);

    const auto vec_x = x - a[4];
    const auto vec_y = y - a[5];
    const auto ac = a[0] * vec_x + a[1] * vec_y;
    const auto bc = a[2] * vec_x + a[3] * vec_y;

    return (ac >= 0.0F) && (bc >= 0.0F) && (ac + bc <= 1.0F);
  }

 private:
  [[nodiscard]] auto get_projected_point(const rtc::math_point& p) const noexcept -> std::tuple<rtc_float, rtc_float>
  {
    switch (biggest_axis)
    {
      case rtc::axis::x:
        return std::tuple{p.y(), p.z()};
      case rtc::axis::y:
        return std::tuple{p.x(), p.z()};
      case rtc::axis::z:
        return std::tuple{p.x(), p.y()};
    }

    return std::tuple<rtc_float, rtc_float>{};
  }

  auto compute_all_needed_factors(const rtc::math_point& p1,
                                  const rtc::math_point& p2,
                                  const rtc::math_point& p3) noexcept -> void
  {
    const auto [x1, y1] = get_projected_point(p1);
    const auto [x2, y2] = get_projected_point(p2);
    const auto [x3, y3] = get_projected_point(p3);
    const rtc_float c = 1.0F / ((x1 - x2) * (y3 - y2) - (x3 - x2) * (y1 - y2));

    a[0] = c * (y3 - y2);
    a[1] = c * (x2 - x3);
    a[2] = c * (y2 - y1);
    a[3] = c * (x1 - x2);
    a[4] = x2;
    a[5] = y2;
  }

  std::array<rtc_float, 6> a{};
  rtc::axis biggest_axis{};
};

constexpr std::array<rtc::math_point, 3> triangle{
    rtc::math_point{1.25F, -2.0F, 0.5F},
    rtc::math_point{7.0F, 1.0F, 2.0F},
    rtc::math_point{-1.5F, 6.5F, 4.0F},
};

constexpr std::array<rtc::math_point, 8> points{
    rtc::math_point{2.0F, 0.0F, 1.25F},
    rtc::math_point{2.5F, 2.0F, 2.25F},
    rtc::math_point{0.5F, 3.5F, 2.75F},
    rtc::math_point{4.0F, 1.0F, 1.75F},
    rtc::math_point{-4.0F, 1.0F, 0.0F},
    rtc::math_point{9.0F, 4.0F, 2.0F},
    rtc::math_point{2.0F, -5.0F, -1.0F},
    rtc::math_point{1.0F, 8.0F, 6.0F},
};

void BM_PointInTriangleContains(benchmark::State& state)
{
  const rtc::point_in_triangle_test test{triangle[0], triangle[1], triangle[2]};

  for (auto _ : state)
  {
    std::uint32_t inside{};
    for (const auto& point : points)
    {
      auto tested_point = point;
      benchmark::DoNotOptimize(tested_point);
      inside += test.triangle_contains(tested_point) ? 1U : 0U;
    }

    benchmark::DoNotOptimize(inside);
  }
}

void BM_PointInTriangleConstructAndContains(benchmark::State& state)
{
  for (auto _ : state)
  {
    std::uint32_t inside{};
    for (const auto& point : points)
    {
      auto p1 = triangle[0];
      auto p2 = triangle[1];
      auto p3 = triangle[2];
      auto tested_point = point;
      benchmark::DoNotOptimize(p1);
      benchmark::DoNotOptimize(p2);
      benchmark::DoNotOptimize(p3);
      benchmark::DoNotOptimize(tested_point);

      const rtc::point_in_triangle_test test{p1, p2, p3};
      inside += test.triangle_contains(tested_point) ? 1U : 0U;
    }

    benchmark::DoNotOptimize(inside);
  }
}

void BM_LegacyPointInTriangleContains(benchmark::State& state)
{
  const legacy_point_in_triangle_test test{triangle[0], triangle[1], triangle[2]};

  for (auto _ : state)
  {
    std::uint32_t inside{};
    for (const auto& point : points)
    {
      auto tested_point = point;
      benchmark::DoNotOptimize(tested_point);
      inside += test.triangle_contains(tested_point) ? 1U : 0U;
    }

    benchmark::DoNotOptimize(inside);
  }
}

void BM_LegacyPointInTriangleConstructAndContains(benchmark::State& state)
{
  for (auto _ : state)
  {
    std::uint32_t inside{};
    for (const auto& point : points)
    {
      auto p1 = triangle[0];
      auto p2 = triangle[1];
      auto p3 = triangle[2];
      auto tested_point = point;
      benchmark::DoNotOptimize(p1);
      benchmark::DoNotOptimize(p2);
      benchmark::DoNotOptimize(p3);
      benchmark::DoNotOptimize(tested_point);

      const legacy_point_in_triangle_test test{p1, p2, p3};
      inside += test.triangle_contains(tested_point) ? 1U : 0U;
    }

    benchmark::DoNotOptimize(inside);
  }
}
}  // namespace

BENCHMARK(BM_PointInTriangleContains);
BENCHMARK(BM_PointInTriangleConstructAndContains);
BENCHMARK(BM_LegacyPointInTriangleContains);
BENCHMARK(BM_LegacyPointInTriangleConstructAndContains);

BENCHMARK_MAIN();
