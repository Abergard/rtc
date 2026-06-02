#include "material_light_color.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>

#include "fast_pow.hpp"
#include "light.hpp"

namespace rtc::material_light
{
auto saturated(const rtc_float value) noexcept -> rtc_float
{
  return std::clamp(value, 0.0F, 1.0F);
}

auto base_color(const rtc::surface_material& material,
                const rtc::color& material_color,
                const rtc::color& ambient) noexcept -> rtc::color
{
  return (material.ka * material_color) * ambient + material.selfLuminance * material_color;
}

auto make_light_sample(const rtc::surface_material& material,
                       const rtc::math_point& hit_point,
                       const rtc::math_vector& normal,
                       const rtc::light& source) noexcept -> light_sample
{
  light_sample sample;
  sample.from_light = hit_point - source.position;
  sample.distance = rtc::length(sample.from_light);
  sample.direction_from_light = sample.distance > 0.0F ? sample.from_light / sample.distance : sample.from_light;
  sample.front_dot = std::max(0.0F, rtc::dot(normal, sample.direction_from_light));
  sample.back_dot = std::max(0.0F, rtc::dot(-normal, sample.direction_from_light));
  sample.diffuse_weight = material.kd * sample.front_dot + material.ktd * sample.back_dot;
  return sample;
}

auto has_direct_contribution(const rtc::surface_material& material, const light_sample& sample) noexcept -> bool
{
  return sample.diffuse_weight > 0.0F || (material.ks > 0.0F && sample.front_dot > 0.0F);
}

auto direct_contribution(const rtc::surface_material& material,
                         const rtc::color& material_color,
                         const rtc::math_point& hit_point,
                         const rtc::math_vector& normal,
                         const rtc::math_point& view_point,
                         const rtc::light& source,
                         const light_sample& sample,
                         rtc_float shadow_transmittance) noexcept -> rtc::color
{
  auto specular = 0.0F;
  if (material.ks > 0.0F)
  {
    const auto reflected_light = 2.0F * rtc::dot(sample.direction_from_light, normal) * normal - sample.direction_from_light;
    const auto view = rtc::normalize(view_point - hit_point);
    const auto specular_angle = std::max(0.0F, rtc::dot(reflected_light, view));
    const auto shininess = static_cast<std::uint32_t>(std::max(1.0F, material.gs));
    specular = material.ks * rtc::pow(specular_angle, shininess);
  }

  const auto attenuation = rtc::inverse_square_factor(source, sample.distance);
  return (shadow_transmittance / attenuation) *
         (sample.diffuse_weight * (material_color * source.light_color) + specular * source.light_color);
}

auto fresnel_factor(const rtc::surface_material& material,
                    const rtc::math_ray& ray,
                    const rtc::math_vector& normal) noexcept -> rtc_float
{
  const auto eta = std::max(material.eta, std::numeric_limits<rtc_float>::epsilon());
  const auto f0_base = (eta - 1.0F) / (eta + 1.0F);
  const auto f0 = f0_base * f0_base;
  const auto incident = rtc::normalize(ray.direction());
  const auto n = rtc::normalize(normal);
  const auto cos_theta = saturated(std::fabs(rtc::dot(incident, n)));
  const auto one_minus_cos = 1.0F - cos_theta;

  return f0 + (1.0F - f0) * rtc::pow(one_minus_cos, 5);
}

auto fresnel_reflection_scale(const rtc::surface_material& material, const rtc_float fresnel) noexcept -> rtc_float
{
  const auto kf = saturated(material.kf);
  return (1.0F - kf) + kf * fresnel;
}

auto fresnel_transmission_scale(const rtc::surface_material& material, const rtc_float fresnel) noexcept -> rtc_float
{
  const auto kf = saturated(material.kf);
  return (1.0F - kf) + kf * (1.0F - fresnel);
}

auto reflection_weight(const rtc::surface_material& material, const rtc_float fresnel) noexcept -> rtc_float
{
  if (material.mirror)
    return 1.0F;

  return material.reflection ? saturated(material.ks) * fresnel_reflection_scale(material, fresnel) : 0.0F;
}

auto transmission_weight(const rtc::surface_material& material, const rtc_float fresnel) noexcept -> rtc_float
{
  return saturated(material.kts) * fresnel_transmission_scale(material, fresnel);
}

auto remaining_weight(const rtc_float used_weight) noexcept -> rtc_float
{
  return std::max(0.0F, 1.0F - used_weight);
}

auto compose(const rtc::surface_material& material,
             const rtc::math_ray& ray,
             const rtc::math_vector& normal,
             const rtc::color& local_color,
             const std::optional<rtc::color>& reflected,
             const std::optional<rtc::color>& refracted) noexcept -> rtc::color
{
  const auto fresnel = fresnel_factor(material, ray, normal);
  const auto reflected_weight = reflected ? reflection_weight(material, fresnel) : 0.0F;
  const auto refracted_weight =
      refracted ? std::min(transmission_weight(material, fresnel), remaining_weight(reflected_weight)) : 0.0F;
  const auto local_weight = remaining_weight(reflected_weight + refracted_weight);

  auto result = local_weight * local_color;
  if (reflected)
    result += reflected_weight * reflected.value();

  if (refracted)
    result += refracted_weight * refracted.value();

  return rtc::clamp(result);
}
}  // namespace rtc::material_light
