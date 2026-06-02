#pragma once

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>

#include "color.hpp"
#include "fast_pow.hpp"
#include "intersection.hpp"
#include "math_ray.hpp"
#include "shadow_ray.hpp"

namespace rtc
{
namespace detail
{
inline auto saturated(const rtc_float value) noexcept -> rtc_float
{
  return std::clamp(value, 0.0F, 1.0F);
}

inline auto reflection_weight(const rtc::surface_material& material) noexcept -> rtc_float
{
  if (material.mirror)
    return 1.0F;

  return material.reflection ? saturated(material.ks) : 0.0F;
}

inline auto transmission_weight(const rtc::surface_material& material) noexcept -> rtc_float
{
  return saturated(material.kts);
}

inline auto fresnel_factor(const rtc::surface_material& material,
                           const rtc::math_ray& ray,
                           const rtc::math_vector& normal) noexcept -> rtc_float
{
  const auto eta = std::max(material.eta, std::numeric_limits<rtc_float>::epsilon());
  const auto f0_base = (eta - 1.0F) / (eta + 1.0F);
  const auto f0 = f0_base * f0_base;
  const auto incident = normalize(ray.direction());
  const auto n = normalize(normal);
  const auto cos_theta = saturated(std::fabs(dot(incident, n)));
  const auto one_minus_cos = 1.0F - cos_theta;

  return f0 + (1.0F - f0) * rtc::pow(one_minus_cos, 5);
}

inline auto fresnel_reflection_scale(const rtc::surface_material& material,
                                     const rtc_float fresnel) noexcept -> rtc_float
{
  const auto kf = saturated(material.kf);
  return (1.0F - kf) + kf * fresnel;
}

inline auto fresnel_transmission_scale(const rtc::surface_material& material,
                                       const rtc_float fresnel) noexcept -> rtc_float
{
  const auto kf = saturated(material.kf);
  return (1.0F - kf) + kf * (1.0F - fresnel);
}

inline auto reflection_weight(const rtc::surface_material& material, const rtc_float fresnel) noexcept -> rtc_float
{
  if (material.mirror)
    return 1.0F;

  return material.reflection ? saturated(material.ks) * fresnel_reflection_scale(material, fresnel) : 0.0F;
}

inline auto transmission_weight(const rtc::surface_material& material, const rtc_float fresnel) noexcept -> rtc_float
{
  return transmission_weight(material) * fresnel_transmission_scale(material, fresnel);
}

inline auto remaining_weight(const rtc_float used_weight) noexcept -> rtc_float
{
  return std::max(0.0F, 1.0F - used_weight);
}
}  // namespace detail

template <typename = void>
struct distributed_ray_tracing_shadows
{
 public:
  using optional_color = std::optional<rtc::color>;
  explicit distributed_ray_tracing_shadows(std::shared_ptr<const rtc::scene_model> sc) : scene(std::move(sc)) {}

  template <typename rt_serv>
  auto compute_color(const rtc::math_ray& ray,
                     const rtc::intersection& object,
                     const optional_color& reflected,
                     const optional_color& refracted,
                     rt_serv& rt) -> rtc::color
  {
    const auto& m = object.attribute(*scene);
    const auto n = object.normal_vector(ray, *scene);
    const auto hit_point = object.hit_point(ray);
    const auto material_color = object.color(*scene);

#if 1
    auto local_color = (m.ka * material_color) * scene->ambient + m.selfLuminance * material_color;
    if (m.shadowfall)
    {
      for (const auto& light : scene->lights)
      {
        // TODO: Extract this into rtc::shadow_ray class probably with ctor which takes reference to
        //      Illumination
        // rtc::shadow_ray<rtc::color> sr{illumination};

        const auto l = hit_point - light.position;
        const auto light_distance = length(l);
        const auto nl = light_distance > 0.0F ? l / light_distance : l;
        const auto dot_ln = std::max(0.0F, dot(n, nl));
        const auto dot_back_ln = std::max(0.0F, dot(-n, nl));
        const auto diffuse_weight = m.kd * dot_ln + m.ktd * dot_back_ln;

        if (diffuse_weight > 0.0F || (m.ks > 0.0F && dot_ln > 0.0F))
        {
          const auto shadow = rtc::shadow_ray{*scene, hit_point, -l, light, light_distance}.trace(rt);

          if (shadow.object.is_none() || !(shadow.object < intersection(_, 1.0F)))
          {
            auto specular = 0.0F;
            if (m.ks > 0.0F)
            {
              const auto R = 2.0F * dot(nl, n) * n - nl;
              const auto V = normalize(scene->optical_system.view_point - hit_point);
              const auto specular_angle = std::max(0.0F, dot(R, V));
              const auto shininess = static_cast<std::uint32_t>(std::max(1.0F, m.gs));
              specular = m.ks * rtc::pow(specular_angle, shininess);
            }
            const auto d = inverse_square_factor(light, light_distance);

            local_color += (shadow.transmittance / d) *
                           (diffuse_weight * (material_color * light.light_color) + specular * light.light_color);
          }
        }
      }
    }
#else
    rtc::color illumination{1, 1, 1};
#endif

    const auto fresnel = detail::fresnel_factor(m, ray, n);
    const auto reflected_weight = reflected ? detail::reflection_weight(m, fresnel) : 0.0F;
    const auto refracted_weight =
        refracted ? std::min(detail::transmission_weight(m, fresnel), detail::remaining_weight(reflected_weight)) : 0.0F;
    const auto local_weight = detail::remaining_weight(reflected_weight + refracted_weight);

    auto r = local_weight * local_color;

    if (reflected)
      r += reflected_weight * reflected.value();

    if (refracted)
      r += refracted_weight * refracted.value();

    return rtc::clamp(r);
  }

 private:
  const std::shared_ptr<const rtc::scene_model> scene;
};

}  // namespace rtc
