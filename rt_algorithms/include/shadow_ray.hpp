#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>

#include "intersection.hpp"
#include "light.hpp"
#include "math_ray.hpp"
#include "scene_model.hpp"
#include "utility.hpp"

#ifndef RTC_SHADOW_RAY_DIAGNOSTICS
#define RTC_SHADOW_RAY_DIAGNOSTICS 0
#endif

#if RTC_SHADOW_RAY_DIAGNOSTICS
#include <atomic>

#include "rtc_log.hpp"
#endif

namespace rtc
{
class shadow_ray
{
 public:
  struct sample
  {
    rtc::intersection object{};
    rtc_float transmittance{1.0F};
  };

  struct sentinel
  {
  };

  shadow_ray(const rtc::scene_model& scene,
             const rtc::math_point& origin,
             const rtc::math_vector& direction_to_light,
             const rtc::light& source) noexcept
      : shadow_ray{scene, origin, direction_to_light, source, rtc::length(direction_to_light)}
  {
  }

  shadow_ray(const rtc::scene_model& scene,
             const rtc::math_point& origin,
             const rtc::math_vector& direction_to_light,
             const rtc::light& source,
             rtc_float /*direction_to_light_length*/) noexcept
      : scene{&scene}, ray{make_ray_to_light(origin, source.position, direction_to_light)}, light{&source},
        ray_length{rtc::length(ray.direction())}
  {
  }

  shadow_ray(const shadow_ray&) = delete;
  shadow_ray(shadow_ray&&) = delete;
  auto operator=(const shadow_ray&) -> shadow_ray& = delete;
  auto operator=(shadow_ray&&) -> shadow_ray& = delete;

  template <typename rt_serv>
  class iterator
  {
   public:
    using value_type = sample;
    using difference_type = std::ptrdiff_t;
    using pointer = const value_type*;
    using reference = const value_type&;
    using iterator_category = std::input_iterator_tag;

    iterator() noexcept = default;
    iterator(const shadow_ray& owner, rt_serv& rt)
        : scene{owner.scene}, light{owner.light}, rt{&rt}, current_ray{owner.ray},
          current_ray_length{owner.ray_length}, active{true}
    {
      current.object = shadow_ray::trace_one(rt, current_ray);
    }

    auto operator*() const noexcept -> reference { return current; }
    auto operator->() const noexcept -> pointer { return &current; }

    auto operator++() -> iterator&
    {
      constexpr std::uint32_t max_transparent_hits{64};
      if (!active)
        return *this;

      if (++transparent_hits >= max_transparent_hits)
      {
        active = false;
        return *this;
      }

      if (current.object.is_none() || current.object.hit_value() >= 1.0F)
      {
        active = false;
        return *this;
      }

      const auto& material = current.object.attribute(*scene);
      const auto transmittance = shadow_transmittance(material);
      if (!material.shadowcast || transmittance > 0.0F)
      {
        const auto hit_value = current.object.hit_value();
        const auto ray_hit = point_at(current_ray, hit_value);
        if (material.shadowcast && transmittance < 1.0F)
          current.transmittance *= std::pow(transmittance, current_ray_length * hit_value);

        const auto next_direction = light->position - ray_hit;
        current_ray_length = remaining_ray_length(current_ray_length, hit_value);
        current_ray =
            make_ray_to_light(ray_hit, light->position, next_direction, triangle_normal(*scene, current.object.triangle_index()));
        current.object = shadow_ray::trace_one(*rt, current_ray);
        return *this;
      }

      active = false;
      return *this;
    }

    auto operator!=(sentinel) const noexcept -> bool { return active; }
    auto operator==(sentinel) const noexcept -> bool { return !active; }
    auto operator!=(const iterator& other) const noexcept -> bool { return active != other.active; }
    auto operator==(const iterator& other) const noexcept -> bool { return !(*this != other); }

   private:
    const rtc::scene_model* scene{};
    const rtc::light* light{};
    rt_serv* rt{};
    rtc::math_ray current_ray{};
    rtc_float current_ray_length{};
    value_type current{};
    std::uint32_t transparent_hits{};
    bool active{};
  };

  template <typename rt_serv>
  auto begin(rt_serv& rt) const -> iterator<rt_serv>
  {
    return {*this, rt};
  }

  template <typename rt_serv>
  auto end(rt_serv&) const noexcept -> sentinel
  {
    return {};
  }

  template <typename rt_serv>
  rtc_hot auto trace(rt_serv& rt) const -> sample
  {
    constexpr std::uint32_t max_transparent_hits{64};
    constexpr std::uint32_t max_same_object_skips{8};
    sample result{};
    auto current_ray{ray};
    auto current_ray_length = ray_length;
    std::uint32_t transparent_hits{};
    auto previous_hit_index = std::numeric_limits<std::uint32_t>::max();
    rtc::math_vector previous_hit_normal{};
    std::uint32_t same_object_hits{};

    for (;;)
    {
      if (rtc_unlikely(transparent_hits++ >= max_transparent_hits))
        return result;

      result.object = trace_one(rt, current_ray);
      if (rtc_likely(result.object.is_none() || result.object.hit_value() >= 1.0F))
        return result;

      const auto hit_index = result.object.triangle_index();
      const auto hit_value = result.object.hit_value();
      if (rtc_unlikely(hit_index == previous_hit_index))
      {
        ++same_object_hits;
        log_same_object_issue(
            current_ray, result.object, current_ray_length, same_object_hits, result.transmittance, "skipped");

        if (rtc_unlikely(same_object_hits > max_same_object_skips))
        {
          log_same_object_issue(
              current_ray, result.object, current_ray_length, same_object_hits, result.transmittance, "gave-up");
          result.object = rtc::no_intersection;
          return result;
        }

        const auto ray_hit = point_at(current_ray, hit_value);
        const auto next_direction = light->position - ray_hit;
        current_ray = make_ray_to_light(ray_hit, light->position, next_direction, previous_hit_normal);
        current_ray_length = remaining_ray_length(current_ray_length, hit_value);
        continue;
      }

      previous_hit_index = hit_index;
      previous_hit_normal = triangle_normal(*scene, hit_index);
      same_object_hits = 1U;

      const auto& material = scene->materials[scene->material_id[hit_index]];
      const auto transmittance = shadow_transmittance(material);
      if (rtc_likely(material.shadowcast && transmittance <= 0.0F))
        return result;

      const auto ray_hit = point_at(current_ray, hit_value);
      if (rtc_unlikely(material.shadowcast && transmittance < 1.0F))
        result.transmittance *= std::pow(transmittance, current_ray_length * hit_value);

      const auto next_direction = light->position - ray_hit;
      current_ray = make_ray_to_light(ray_hit, light->position, next_direction, previous_hit_normal);
      current_ray_length = remaining_ray_length(current_ray_length, hit_value);
    }
  }

 private:
  static auto shadow_transmittance(const rtc::surface_material& material) noexcept -> rtc_float
  {
    return std::clamp(material.kts + material.ktd, 0.0F, 1.0F);
  }

  template <typename tracer>
  static auto trace_one(tracer& rt, const rtc::math_ray& ray) -> rtc::intersection
  {
    return rt.trace_ray(ray).get();
  }

  static constexpr rtc_float ray_origin_bias{0.0001F};
  static constexpr rtc_float surface_normal_bias{0.0001F};

  static auto point_at(const rtc::math_ray& ray, const rtc_float hit_value) noexcept -> rtc::math_point
  {
    return ray.origin() + ray.direction() * hit_value;
  }

  static auto offset_origin(const rtc::math_point& origin, const rtc::math_vector& direction) noexcept -> rtc::math_point
  {
    return origin + direction * ray_origin_bias;
  }

  static auto triangle_normal(const rtc::scene_model& scene, const std::uint32_t triangle_index) noexcept -> rtc::math_vector
  {
    if (triangle_index < scene.normals.size() && rtc::lengthSQ(scene.normals[triangle_index]) > 0.0F)
      return scene.normals[triangle_index];

    const auto& triangle = scene.triangles[triangle_index];
    const auto& p1 = scene.points[triangle.vertex_a()];
    const auto& p2 = scene.points[triangle.vertex_b()];
    const auto& p3 = scene.points[triangle.vertex_c()];
    return rtc::normalize(rtc::cross(p1 - p2, p3 - p2));
  }

  static auto offset_origin(const rtc::scene_model& scene,
                            const rtc::math_point& origin,
                            const rtc::math_vector& direction,
                            const std::uint32_t triangle_index) noexcept -> rtc::math_point
  {
    const auto normal = triangle_normal(scene, triangle_index);
    return offset_origin(origin, direction, normal);
  }

  static auto offset_origin(const rtc::math_point& origin,
                            const rtc::math_vector& direction,
                            const rtc::math_vector& normal) noexcept -> rtc::math_point
  {
    const auto normal_direction = rtc::dot(normal, direction) >= 0.0F ? normal : -normal;
    return offset_origin(origin, direction) + normal_direction * surface_normal_bias;
  }

  static auto make_ray_to_light(const rtc::math_point& origin,
                                const rtc::math_point& light_position,
                                const rtc::math_vector& direction_to_light) noexcept -> rtc::math_ray
  {
    const auto biased_origin = offset_origin(origin, direction_to_light);
    return {light_position - biased_origin, biased_origin};
  }

  static auto make_ray_to_light(const rtc::scene_model& scene,
                                const rtc::math_point& origin,
                                const rtc::math_point& light_position,
                                const rtc::math_vector& direction_to_light,
                                const std::uint32_t triangle_index) noexcept -> rtc::math_ray
  {
    const auto biased_origin = offset_origin(scene, origin, direction_to_light, triangle_index);
    return {light_position - biased_origin, biased_origin};
  }

  static auto make_ray_to_light(const rtc::math_point& origin,
                                const rtc::math_point& light_position,
                                const rtc::math_vector& direction_to_light,
                                const rtc::math_vector& normal) noexcept -> rtc::math_ray
  {
    const auto biased_origin = offset_origin(origin, direction_to_light, normal);
    return {light_position - biased_origin, biased_origin};
  }

  static auto remaining_ray_length(const rtc_float current_length, const rtc_float hit_value) noexcept -> rtc_float
  {
    return std::max(0.0F, current_length * std::max(0.0F, 1.0F - hit_value) * (1.0F - ray_origin_bias));
  }

  auto log_same_object_issue(const rtc::math_ray& current_ray,
                             const rtc::intersection& object,
                             const rtc_float current_ray_length,
                             const std::uint32_t same_object_hits,
                             const rtc_float transmittance,
                             const char* action) const -> void
  {
#if RTC_SHADOW_RAY_DIAGNOSTICS
    constexpr auto max_logged_issues{64U};
    static std::atomic_uint logged_issues{};

    const auto log_index = logged_issues.fetch_add(1U, std::memory_order_relaxed);
    if (log_index >= max_logged_issues)
      return;

    const auto& triangle = object.triangle(*scene);
    const auto* triangle_data = scene->triangles.data();
    const auto triangle_index = static_cast<std::size_t>(&triangle - triangle_data);
    const auto material_index =
        triangle_index < scene->material_id.size() ? static_cast<std::uint32_t>(scene->material_id[triangle_index]) : 0U;
    const auto normal_dot_ray = triangle_normal_dot_ray(triangle, current_ray);

    RELEASE << "shadow_ray same-object self-hit"
            << " occurrence=" << log_index + 1U << "/" << max_logged_issues
            << " action=" << action
            << " triangle=" << triangle_index
            << " material=" << material_index
            << " same_object_hits=" << same_object_hits
            << " hit_value=" << object.hit_value()
            << " normal_dot_ray=" << normal_dot_ray
            << " grazing=" << (std::abs(normal_dot_ray) < 0.01F)
            << " ray_length=" << current_ray_length
            << " transmittance=" << transmittance
            << " ray_origin=" << current_ray.origin()
            << " ray_direction=" << current_ray.direction()
            << " light_position=" << light->position;
#else
    (void)current_ray;
    (void)object;
    (void)current_ray_length;
    (void)same_object_hits;
    (void)transmittance;
    (void)action;
#endif
  }

#if RTC_SHADOW_RAY_DIAGNOSTICS
  auto triangle_normal_dot_ray(const rtc::triangle3d& triangle, const rtc::math_ray& current_ray) const noexcept -> rtc_float
  {
    const auto* triangle_data = scene->triangles.data();
    const auto triangle_index = static_cast<std::uint32_t>(&triangle - triangle_data);
    return rtc::dot(triangle_normal(*scene, triangle_index), rtc::normalize(current_ray.direction()));
  }
#endif

  const rtc::scene_model* scene{};
  rtc::math_ray ray{};
  const rtc::light* light{};
  rtc_float ray_length{};
};
}  // namespace rtc
