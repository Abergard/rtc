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

#if 1
    auto local_color = (m.ka * object.color(*scene)) * scene->ambient + m.selfLuminance * object.color(*scene);
    if (m.shadowfall)
    {
      for (const auto& light : scene->lights)
      {
        // TODO: Extract this into rtc::shadow_ray class probably with ctor which takes reference to
        //      Illumination
        // rtc::shadow_ray<rtc::color> sr{illumination};

        const auto l = object.hit_point(ray) - light.position;
        const auto nl = normalize(l);
        const auto dot_ln = std::max(0.0F, dot(n, nl));
        const auto dot_back_ln = std::max(0.0F, dot(-n, nl));

        if (dot_ln > 0.0F || (m.ktd > 0.0F && dot_back_ln > 0.0F))
        {
          const auto [i, f] = get_intersection_with_light_ray(object, ray, -l, light, rt);

          if (i.is_none() || !(i < intersection(_, 1.0F)))
          {
            auto specular = 0.0F;
            if (m.ks > 0.0F)
            {
              const auto R = 2.0F * dot(nl, n) * n - nl;
              const auto V = normalize(scene->optical_system.view_point - object.hit_point(ray));
              const auto specular_angle = std::max(0.0F, dot(R, V));
              const auto shininess = static_cast<std::uint32_t>(std::max(1.0F, m.gs));
              specular = m.ks * rtc::pow(specular_angle, shininess);
            }
            const auto d = inverse_square_factor(light, l);

            local_color += (f / d) *
                           ((m.kd * dot_ln + m.ktd * dot_back_ln) * (object.color(*scene) * light.light_color) +
                            specular * light.light_color);
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

  template <typename _rt>
  auto get_intersection_with_light_ray(const rtc::intersection& object,
                                       const rtc::math_ray& ray,
                                       const rtc::math_vector& L,
                                       const rtc::light& light,
                                       _rt& rt) const -> std::tuple<intersection, float>
  {
    rtc::math_ray shadow_ray{L, object.hit_point(ray)};
    rtc::intersection intersect{};
    rtc_float acc{1};

    do
    {
      if (intersect)
      {
        const auto ray_hit = intersect.hit_point(shadow_ray);
        const auto segment_length = rtc::length(ray_hit - shadow_ray.origin());
        shadow_ray = {light.position - ray_hit, ray_hit};
        const auto& material = intersect.attribute(*scene);

        if (material.shadowcast)
        {
          const auto transparent_shadow = detail::saturated(0.5F * (material.kts + material.ktd));
          acc *= std::pow(transparent_shadow, segment_length);
        }
      }

      intersect = rt.trace_ray(shadow_ray).get();
    } while (intersect.is_present() &&
             (!intersect.attribute(*scene).shadowcast || intersect.is_refractive(*scene)) &&
             intersect < intersection(_, 1.0F));

    assert(intersect.is_present() && intersect.is_refractive(*scene) && !(intersect < intersection{_, 1.0F}) ||
           intersect.is_present() && !intersect.is_refractive(*scene) && !(intersect < intersection{_, 1.0F}) ||
           intersect.is_present() && !intersect.is_refractive(*scene) && (intersect < intersection{_, 1.0F}) ||
           intersect.is_none());

    return {intersect, std::clamp<rtc_float>(acc, 0, 1)};
  }
};

}  // namespace rtc
