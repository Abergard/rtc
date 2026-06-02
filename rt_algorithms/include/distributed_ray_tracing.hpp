#pragma once

#include <memory>
#include <optional>

#include "color.hpp"
#include "intersection.hpp"
#include "material_light_color.hpp"
#include "math_ray.hpp"
#include "shadow_ray.hpp"

namespace rtc
{
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

    auto local_color = rtc::material_light::base_color(m, material_color, scene->ambient);
    if (m.shadowfall)
    {
      for (const auto& light : scene->lights)
      {
        const auto light_sample = rtc::material_light::make_light_sample(m, hit_point, n, light);
        if (rtc::material_light::has_direct_contribution(m, light_sample))
        {
          const auto shadow = rtc::shadow_ray{*scene, hit_point, -light_sample.from_light, light, light_sample.distance}.trace(rt);

          if (shadow.object.is_none() || !(shadow.object < intersection(_, 1.0F)))
            local_color += rtc::material_light::direct_contribution(
                m, material_color, hit_point, n, scene->optical_system.view_point, light, light_sample, shadow.transmittance);
        }
      }
    }

    return rtc::material_light::compose(m, ray, n, local_color, reflected, refracted);
  }

 private:
  const std::shared_ptr<const rtc::scene_model> scene;
};

}  // namespace rtc
