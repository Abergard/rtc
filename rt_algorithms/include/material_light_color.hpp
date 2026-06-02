#pragma once

#include <optional>

#include "color.hpp"
#include "light.hpp"
#include "math_ray.hpp"
#include "scene_model.hpp"

namespace rtc::material_light
{
struct light_sample
{
  rtc::math_vector from_light{};
  rtc_float distance{};
  rtc::math_vector direction_from_light{};
  rtc_float front_dot{};
  rtc_float back_dot{};
  rtc_float diffuse_weight{};
};

auto saturated(rtc_float value) noexcept -> rtc_float;

auto base_color(const rtc::surface_material& material,
                const rtc::color& material_color,
                const rtc::color& ambient) noexcept -> rtc::color;

auto make_light_sample(const rtc::surface_material& material,
                       const rtc::math_point& hit_point,
                       const rtc::math_vector& normal,
                       const rtc::light& source) noexcept -> light_sample;

auto has_direct_contribution(const rtc::surface_material& material, const light_sample& sample) noexcept -> bool;

auto direct_contribution(const rtc::surface_material& material,
                         const rtc::color& material_color,
                         const rtc::math_point& hit_point,
                         const rtc::math_vector& normal,
                         const rtc::math_point& view_point,
                         const rtc::light& source,
                         const light_sample& sample,
                         rtc_float shadow_transmittance) noexcept -> rtc::color;

auto fresnel_factor(const rtc::surface_material& material,
                    const rtc::math_ray& ray,
                    const rtc::math_vector& normal) noexcept -> rtc_float;

auto fresnel_reflection_scale(const rtc::surface_material& material, rtc_float fresnel) noexcept -> rtc_float;

auto fresnel_transmission_scale(const rtc::surface_material& material, rtc_float fresnel) noexcept -> rtc_float;

auto reflection_weight(const rtc::surface_material& material, rtc_float fresnel) noexcept -> rtc_float;

auto transmission_weight(const rtc::surface_material& material, rtc_float fresnel) noexcept -> rtc_float;

auto remaining_weight(rtc_float used_weight) noexcept -> rtc_float;

auto compose(const rtc::surface_material& material,
             const rtc::math_ray& ray,
             const rtc::math_vector& normal,
             const rtc::color& local_color,
             const std::optional<rtc::color>& reflected,
             const std::optional<rtc::color>& refracted) noexcept -> rtc::color;
}  // namespace rtc::material_light
