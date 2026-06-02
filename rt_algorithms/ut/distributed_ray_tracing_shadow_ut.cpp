#include <memory>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "distributed_ray_tracing.hpp"
#include "scene_model.hpp"

namespace rtc::ut
{
namespace
{
using ::testing::FloatNear;

struct fake_trace_result
{
  rtc::intersection value{};
  auto get() noexcept -> rtc::intersection { return value; }
};

struct fake_rt_service
{
  std::vector<rtc::intersection> intersections{};
  std::size_t next{};

  auto trace_ray(const rtc::math_ray&) noexcept -> fake_trace_result
  {
    if (next < intersections.size())
      return {intersections[next++]};

    return {};
  }
};

auto shadow_scene() -> std::shared_ptr<rtc::scene_model>
{
  auto scene = std::make_shared<rtc::scene_model>();
  scene->ambient = rtc::black;
  scene->optical_system.view_point = {0.0F, 0.0F, 100.0F};
  scene->points = {
      {-1.0F, -1.0F, 0.0F},
      {1.0F, -1.0F, 0.0F},
      {0.0F, 1.0F, 0.0F},
      {-1.0F, -1.0F, 5.0F},
      {1.0F, -1.0F, 5.0F},
      {0.0F, 1.0F, 5.0F},
  };
  scene->triangles = {
      {0, 1, 2},
      {3, 4, 5},
  };
  scene->material_id = {0, 1};
  scene->materials.resize(2);

  auto& receiver = scene->materials[0];
  receiver.kd = 1.0F;
  receiver.ka = 0.0F;
  receiver.ks = 0.0F;
  receiver.shadowfall = true;
  receiver.material_color = {1.0F, 1.0F, 1.0F};

  auto& blocker = scene->materials[1];
  blocker.kd = 1.0F;
  blocker.ka = 0.0F;
  blocker.material_color = {1.0F, 1.0F, 1.0F};

  rtc::light light;
  light.light_color = {1.0F, 1.0F, 1.0F};
  light.position = {0.0F, 0.0F, 10.0F};
  light.inv_square = {0.0F, 0.0F, 1.0F};
  scene->lights = {light};

  return scene;
}

auto lit_receiver_color(const std::shared_ptr<rtc::scene_model>& scene, fake_rt_service& rt) -> rtc::color
{
  rtc::distributed_ray_tracing_shadows<> shadows{scene};
  const rtc::math_ray camera_ray{{0.0F, 0.0F, -1.0F}, {0.0F, 0.0F, 100.0F}};
  return shadows.compute_color(camera_ray, rtc::intersection{0, 100.0F}, {}, {}, rt);
}

auto material_mix_scene() -> std::shared_ptr<rtc::scene_model>
{
  auto scene = shadow_scene();
  scene->lights.clear();
  scene->materials[0].kd = 0.0F;
  scene->materials[0].ka = 0.0F;
  scene->materials[0].ks = 0.0F;
  scene->materials[0].kts = 0.0F;
  scene->materials[0].reflection = false;
  scene->materials[0].mirror = false;
  scene->materials[0].shadowfall = false;
  scene->materials[0].selfLuminance = 1.0F;
  scene->materials[0].material_color = {100.0F, 10.0F, 1.0F};

  return scene;
}

auto material_color(const std::shared_ptr<rtc::scene_model>& scene,
                    const rtc::distributed_ray_tracing_shadows<>::optional_color& reflected,
                    const rtc::distributed_ray_tracing_shadows<>::optional_color& refracted) -> rtc::color
{
  fake_rt_service rt;
  rtc::distributed_ray_tracing_shadows<> shadows{scene};
  const rtc::math_ray camera_ray{{0.0F, 0.0F, -1.0F}, {0.0F, 0.0F, 100.0F}};
  return shadows.compute_color(camera_ray, rtc::intersection{0, 100.0F}, reflected, refracted, rt);
}

auto expect_color_near(const rtc::color& actual, const rtc::color& expected)
{
  ASSERT_THAT(actual.red(), FloatNear(expected.red(), 0.00001F));
  ASSERT_THAT(actual.green(), FloatNear(expected.green(), 0.00001F));
  ASSERT_THAT(actual.blue(), FloatNear(expected.blue(), 0.00001F));
}
}  // namespace

TEST(distributed_ray_tracing_shadow_ut, opaque_object_between_surface_and_light_blocks_light)
{
  auto scene = shadow_scene();
  fake_rt_service rt{{rtc::intersection{1, 0.5F}}};

  const auto color = lit_receiver_color(scene, rt);

  ASSERT_THAT(color.red(), FloatNear(0.0F, 0.00001F));
  ASSERT_THAT(color.green(), FloatNear(0.0F, 0.00001F));
  ASSERT_THAT(color.blue(), FloatNear(0.0F, 0.00001F));
}

TEST(distributed_ray_tracing_shadow_ut, object_behind_light_does_not_block_light)
{
  auto scene = shadow_scene();
  fake_rt_service rt{{rtc::intersection{1, 1.5F}}};

  const auto color = lit_receiver_color(scene, rt);

  ASSERT_THAT(color.red(), FloatNear(1.0F, 0.00001F));
  ASSERT_THAT(color.green(), FloatNear(1.0F, 0.00001F));
  ASSERT_THAT(color.blue(), FloatNear(1.0F, 0.00001F));
}

TEST(distributed_ray_tracing_shadow_ut, transparent_object_between_surface_and_light_attenuates_light)
{
  auto scene = shadow_scene();
  scene->materials[1].kts = 0.5F;
  scene->materials[1].ktd = 0.5F;
  fake_rt_service rt{{rtc::intersection{1, 0.5F}, rtc::no_intersection}};

  const auto color = lit_receiver_color(scene, rt);

  ASSERT_THAT(color.red(), FloatNear(0.03125F, 0.00001F));
  ASSERT_THAT(color.green(), FloatNear(0.03125F, 0.00001F));
  ASSERT_THAT(color.blue(), FloatNear(0.03125F, 0.00001F));
}

TEST(distributed_ray_tracing_shadow_ut, non_shadow_casting_object_between_surface_and_light_is_skipped)
{
  auto scene = shadow_scene();
  scene->materials[1].shadowcast = false;
  fake_rt_service rt{{rtc::intersection{1, 0.5F}, rtc::no_intersection}};

  const auto color = lit_receiver_color(scene, rt);

  ASSERT_THAT(color.red(), FloatNear(1.0F, 0.00001F));
  ASSERT_THAT(color.green(), FloatNear(1.0F, 0.00001F));
  ASSERT_THAT(color.blue(), FloatNear(1.0F, 0.00001F));
}

TEST(distributed_ray_tracing_shadow_ut, local_material_color_is_used_when_no_reflection_or_refraction_exists)
{
  const auto scene = material_mix_scene();

  expect_color_near(material_color(scene, {}, {}), {100.0F, 10.0F, 1.0F});
}

TEST(distributed_ray_tracing_shadow_ut, refracted_color_is_blended_by_kts)
{
  auto scene = material_mix_scene();
  scene->materials[0].kts = 0.25F;

  expect_color_near(material_color(scene, {}, rtc::color{0.0F, 100.0F, 0.0F}), {75.0F, 32.5F, 0.75F});
}

TEST(distributed_ray_tracing_shadow_ut, refracted_color_uses_specular_transmission_only)
{
  auto scene = material_mix_scene();
  scene->materials[0].kts = 0.25F;
  scene->materials[0].ktd = 0.25F;

  expect_color_near(material_color(scene, {}, rtc::color{0.0F, 100.0F, 0.0F}), {75.0F, 32.5F, 0.75F});
}

TEST(distributed_ray_tracing_shadow_ut, diffuse_transmission_lights_back_side_of_surface)
{
  auto scene = shadow_scene();
  scene->triangles[0] = {0, 2, 1};
  scene->materials[0].kd = 0.0F;
  scene->materials[0].ktd = 1.0F;
  fake_rt_service rt{{rtc::no_intersection}};

  const auto color = lit_receiver_color(scene, rt);

  ASSERT_THAT(color.red(), FloatNear(1.0F, 0.00001F));
  ASSERT_THAT(color.green(), FloatNear(1.0F, 0.00001F));
  ASSERT_THAT(color.blue(), FloatNear(1.0F, 0.00001F));
}

TEST(distributed_ray_tracing_shadow_ut, missing_refracted_color_does_not_reserve_transmission_weight)
{
  auto scene = material_mix_scene();
  scene->materials[0].kts = 0.5F;

  expect_color_near(material_color(scene, {}, {}), {100.0F, 10.0F, 1.0F});
}

TEST(distributed_ray_tracing_shadow_ut, reflected_color_is_blended_by_ks_when_reflection_is_enabled)
{
  auto scene = material_mix_scene();
  scene->materials[0].reflection = true;
  scene->materials[0].ks = 0.25F;

  expect_color_near(material_color(scene, rtc::color{0.0F, 0.0F, 100.0F}, {}), {75.0F, 7.5F, 25.75F});
}

TEST(distributed_ray_tracing_shadow_ut, reflected_and_refracted_colors_are_combined_once_with_material_weights)
{
  auto scene = material_mix_scene();
  scene->materials[0].reflection = true;
  scene->materials[0].ks = 0.25F;
  scene->materials[0].kts = 0.5F;

  expect_color_near(material_color(scene, rtc::color{0.0F, 0.0F, 100.0F}, rtc::color{0.0F, 100.0F, 0.0F}),
                    {25.0F, 52.5F, 25.25F});
}

TEST(distributed_ray_tracing_shadow_ut, fresnel_uses_kf_and_eta_to_split_reflection_and_refraction)
{
  auto scene = material_mix_scene();
  scene->materials[0].reflection = true;
  scene->materials[0].ks = 1.0F;
  scene->materials[0].kts = 1.0F;
  scene->materials[0].kf = 1.0F;
  scene->materials[0].eta = 1.5F;

  expect_color_near(material_color(scene, rtc::color{100.0F, 0.0F, 0.0F}, rtc::color{0.0F, 100.0F, 0.0F}),
                    {4.0F, 96.0F, 0.0F});
}

TEST(distributed_ray_tracing_shadow_ut, mirror_uses_reflected_color_without_local_or_refracted_contribution)
{
  auto scene = material_mix_scene();
  scene->materials[0].mirror = true;
  scene->materials[0].kts = 0.5F;

  expect_color_near(material_color(scene, rtc::color{5.0F, 6.0F, 7.0F}, rtc::color{100.0F, 100.0F, 100.0F}),
                    {5.0F, 6.0F, 7.0F});
}
}  // namespace rtc::ut
