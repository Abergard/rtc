#pragma once

#include <algorithm>
#include <cstdint>
#include <cmath>
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

inline auto local_weight(const rtc::surface_material& material) noexcept -> rtc_float
{
  return std::max(0.0F, 1.0F - reflection_weight(material) - transmission_weight(material));
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
    rtc::color illumination{m.ka * scene->ambient};
    if (m.shadowfall)
    {
      for (const auto& light : scene->lights)
      {
        // TODO: Extract this into rtc::shadow_ray class probably with ctor which takes reference to
        //      Illumination
        // rtc::shadow_ray<rtc::color> sr{illumination};

        const auto l = light.position - object.hit_point(ray);

        if (cos(n, l) > 0)
        {
          const auto [i, f] = get_intersection_with_light_ray(object, ray, l, light, rt);

          if (i.is_none() || !(i < intersection(_, 1.0F)))
          {
            const auto nl = normalize(l);
            const auto dot_ln = std::max(0.0F, dot(n, nl));
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

            illumination += f * ((m.kd * dot_ln + specular) / d) * light.light_color;
          }
        }
      }
    }
#else
    rtc::color illumination{1, 1, 1};
#endif

    rtc::color r = rtc::clamp(object.color(*scene) * illumination + m.selfLuminance * object.color(*scene));

    if (refracted)
      r = detail::local_weight(m) * r + detail::transmission_weight(m) * refracted.value();

    if (reflected)
      r = (1.0F - detail::reflection_weight(m)) * r + detail::reflection_weight(m) * reflected.value();

    return r;
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
