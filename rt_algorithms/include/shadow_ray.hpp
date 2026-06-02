#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <iterator>

#include "intersection.hpp"
#include "light.hpp"
#include "math_ray.hpp"
#include "scene_model.hpp"

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
             rtc_float direction_to_light_length) noexcept
      : scene{&scene}, ray{direction_to_light, origin}, light{&source},
        ray_length{direction_to_light_length}
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
      if (!active)
        return *this;

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
        const auto ray_hit = current_ray[hit_value];
        if (material.shadowcast && transmittance < 1.0F)
          current.transmittance *= std::pow(transmittance, current_ray_length * hit_value);

        current_ray_length *= std::max(0.0F, 1.0F - hit_value);
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
    const rtc::math_ray& current_ray{};
    rtc_float current_ray_length{};
    value_type current{};
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
  auto trace(rt_serv& rt) const -> sample
  {
    sample result{};
    auto current_ray{ray};
    auto current_ray_length = ray_length;
    const rtc::surface_material* material{};

    do
    {
      result.object = trace_one(rt, current_ray);
      if (result.object.is_none() || result.object.hit_value() >= 1.0F)
         return result;

      material = &result.object.attribute(*scene);
      const auto transmittance = shadow_transmittance(*material);
      if (material->shadowcast && transmittance <= 0.0F)
        return result;

      const auto hit_value = result.object.hit_value();
      const auto ray_hit = current_ray[hit_value];
      if (material->shadowcast && transmittance < 1.0F)
        result.transmittance *= std::pow(transmittance, current_ray_length * hit_value);

      current_ray = {light->position - ray_hit, ray_hit};
      current_ray_length *= std::max(0.0F, 1.0F - hit_value);
    } while (result.object.is_present() && (!material->shadowcast || shadow_transmittance(*material) > 0.0F) &&
             result.object.hit_value() < 1.0F);

    return result;
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

  const rtc::scene_model* scene{};
  const rtc::math_ray& ray{};
  const rtc::light* light{};
  rtc_float ray_length{};
};
}  // namespace rtc
