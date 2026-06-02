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
}  // namespace rtc::ut
