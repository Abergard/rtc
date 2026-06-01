#include <array>

#include <benchmark/benchmark.h>

#include "math_vector.hpp"

namespace
{
constexpr std::array<rtc::math_vector, 8> vectors_a{
    rtc::math_vector{1.0F, 2.0F, 3.0F},
    rtc::math_vector{-4.0F, 5.0F, 6.0F},
    rtc::math_vector{7.0F, -8.0F, 9.0F},
    rtc::math_vector{10.0F, 11.0F, -12.0F},
    rtc::math_vector{-13.0F, 14.0F, 15.0F},
    rtc::math_vector{16.0F, -17.0F, 18.0F},
    rtc::math_vector{19.0F, 20.0F, -21.0F},
    rtc::math_vector{-22.0F, 23.0F, 24.0F},
};

constexpr std::array<rtc::math_vector, 8> vectors_b{
    rtc::math_vector{-2.0F, 1.0F, 0.5F},
    rtc::math_vector{3.0F, -7.0F, 2.0F},
    rtc::math_vector{0.25F, 5.0F, -4.0F},
    rtc::math_vector{-9.0F, 0.75F, 8.0F},
    rtc::math_vector{6.0F, -3.0F, 1.25F},
    rtc::math_vector{-1.5F, 2.5F, -6.0F},
    rtc::math_vector{4.5F, -5.5F, 7.5F},
    rtc::math_vector{8.5F, 9.5F, -10.5F},
};

auto scalar_dot(const rtc::math_vector& a, const rtc::math_vector& b) noexcept -> rtc_float
{
  return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
}

auto scalar_cross(const rtc::math_vector& a, const rtc::math_vector& b) noexcept -> rtc::math_vector
{
  return {
      a.y() * b.z() - a.z() * b.y(),
      a.z() * b.x() - a.x() * b.z(),
      a.x() * b.y() - a.y() * b.x(),
  };
}

auto scalar_normalize(const rtc::math_vector& vector) noexcept -> rtc::math_vector
{
  const auto length = scalar_dot(vector, vector);
  if (rtc_likely(length > 0.0F))
    return vector / std::sqrt(length);

  return rtc::math_vector{
      std::numeric_limits<rtc_float>::max(),
      std::numeric_limits<rtc_float>::max(),
      std::numeric_limits<rtc_float>::max(),
  };
}

void BM_MathVectorDot(benchmark::State& state)
{
  for (auto _ : state)
  {
    rtc_float value{};
    for (std::size_t i{}; i < vectors_a.size(); ++i)
    {
      auto a = vectors_a[i];
      auto b = vectors_b[i];
      benchmark::DoNotOptimize(a);
      benchmark::DoNotOptimize(b);
      value += rtc::dot(a, b);
    }

    benchmark::DoNotOptimize(value);
  }
}

void BM_MathVectorCross(benchmark::State& state)
{
  for (auto _ : state)
  {
    rtc::math_vector value{};
    for (std::size_t i{}; i < vectors_a.size(); ++i)
    {
      auto a = vectors_a[i];
      auto b = vectors_b[i];
      benchmark::DoNotOptimize(a);
      benchmark::DoNotOptimize(b);
      value = value + rtc::cross(a, b);
    }

    benchmark::DoNotOptimize(value);
  }
}

void BM_MathVectorNormalize(benchmark::State& state)
{
  for (auto _ : state)
  {
    rtc::math_vector value{};
    for (auto vector : vectors_a)
    {
      benchmark::DoNotOptimize(vector);
      value = value + rtc::normalize(vector);
    }

    benchmark::DoNotOptimize(value);
  }
}

void BM_ScalarMathVectorDot(benchmark::State& state)
{
  for (auto _ : state)
  {
    rtc_float value{};
    for (std::size_t i{}; i < vectors_a.size(); ++i)
    {
      auto a = vectors_a[i];
      auto b = vectors_b[i];
      benchmark::DoNotOptimize(a);
      benchmark::DoNotOptimize(b);
      value += scalar_dot(a, b);
    }

    benchmark::DoNotOptimize(value);
  }
}

void BM_ScalarMathVectorCross(benchmark::State& state)
{
  for (auto _ : state)
  {
    rtc::math_vector value{};
    for (std::size_t i{}; i < vectors_a.size(); ++i)
    {
      auto a = vectors_a[i];
      auto b = vectors_b[i];
      benchmark::DoNotOptimize(a);
      benchmark::DoNotOptimize(b);
      value = value + scalar_cross(a, b);
    }

    benchmark::DoNotOptimize(value);
  }
}

void BM_ScalarMathVectorNormalize(benchmark::State& state)
{
  for (auto _ : state)
  {
    rtc::math_vector value{};
    for (auto vector : vectors_a)
    {
      benchmark::DoNotOptimize(vector);
      value = value + scalar_normalize(vector);
    }

    benchmark::DoNotOptimize(value);
  }
}
}  // namespace

BENCHMARK(BM_MathVectorDot);
BENCHMARK(BM_MathVectorCross);
BENCHMARK(BM_MathVectorNormalize);
BENCHMARK(BM_ScalarMathVectorDot);
BENCHMARK(BM_ScalarMathVectorCross);
BENCHMARK(BM_ScalarMathVectorNormalize);

BENCHMARK_MAIN();
