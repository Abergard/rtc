#pragma once

#include <cmath>
#include <optional>

#include "math_point.hpp"

namespace rtc_gui
{
constexpr float pi = 3.14159265358979323846F;

struct Vec3
{
  float x{};
  float y{};
  float z{};
};

struct Ray
{
  Vec3 origin{};
  Vec3 direction{};
};

inline auto operator+(const Vec3& a, const Vec3& b) -> Vec3 { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline auto operator-(const Vec3& a, const Vec3& b) -> Vec3 { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline auto operator*(const Vec3& a, const float s) -> Vec3 { return {a.x * s, a.y * s, a.z * s}; }
inline auto operator/(const Vec3& a, const float s) -> Vec3 { return {a.x / s, a.y / s, a.z / s}; }

inline auto dot(const Vec3& a, const Vec3& b) -> float { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline auto cross(const Vec3& a, const Vec3& b) -> Vec3
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline auto length(const Vec3& v) -> float { return std::sqrt(dot(v, v)); }

inline auto normalized(const Vec3& v) -> Vec3
{
  const auto len = length(v);
  return len > 0.00001F ? v / len : Vec3{0.0F, 0.0F, 1.0F};
}

inline auto to_vec3(const rtc::math_point& p) -> Vec3 { return {p.x(), p.y(), p.z()}; }
inline auto to_point(const Vec3& v) -> rtc::math_point { return {v.x, v.y, v.z}; }

inline auto intersect_triangle(const Ray& ray, const Vec3& a, const Vec3& b, const Vec3& c) -> std::optional<float>
{
  constexpr float epsilon = 0.000001F;
  const auto edge1 = b - a;
  const auto edge2 = c - a;
  const auto p = cross(ray.direction, edge2);
  const auto determinant = dot(edge1, p);

  if (std::fabs(determinant) < epsilon)
    return std::nullopt;

  const auto invDeterminant = 1.0F / determinant;
  const auto t = ray.origin - a;
  const auto u = dot(t, p) * invDeterminant;

  if (u < 0.0F || u > 1.0F)
    return std::nullopt;

  const auto q = cross(t, edge1);
  const auto v = dot(ray.direction, q) * invDeterminant;

  if (v < 0.0F || u + v > 1.0F)
    return std::nullopt;

  const auto distance = dot(edge2, q) * invDeterminant;
  if (distance <= epsilon)
    return std::nullopt;

  return distance;
}
}  // namespace rtc_gui
