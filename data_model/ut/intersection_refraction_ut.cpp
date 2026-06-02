#include <cmath>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "intersection.hpp"

namespace rtc::ut
{
namespace
{
using ::testing::FloatNear;

constexpr auto tolerance = 0.00001F;

auto refractive_scene() -> scene_model
{
  scene_model scene;
  scene.points = {
      {-1.0F, -1.0F, 0.0F},
      {1.0F, -1.0F, 0.0F},
      {0.0F, 1.0F, 0.0F},
  };
  scene.triangles = {{0, 2, 1}};
  scene.material_id = {0};
  scene.materials.resize(1);
  scene.materials[0].kts = 1.0F;
  scene.materials[0].eta = 1.5F;

  return scene;
}

auto expect_direction_near(const math_vector& actual, const math_vector& expected)
{
  ASSERT_THAT(actual.x(), FloatNear(expected.x(), tolerance));
  ASSERT_THAT(actual.y(), FloatNear(expected.y(), tolerance));
  ASSERT_THAT(actual.z(), FloatNear(expected.z(), tolerance));
}
}  // namespace

TEST(intersection_refraction_ut, bends_towards_normal_when_entering_material)
{
  const auto scene = refractive_scene();
  const intersection hit{0, 1.0F};
  const auto incident = normalize(math_vector{0.5F, 0.0F, -std::sqrt(0.75F)});
  const math_ray ray{incident, {0.0F, 0.0F, std::sqrt(0.75F)}};

  const auto refracted = hit.refract(ray, scene);

  ASSERT_TRUE(refracted);
  expect_direction_near(normalize(refracted->direction()), {1.0F / 3.0F, 0.0F, -std::sqrt(8.0F / 9.0F)});
  ASSERT_EQ(refracted->origin(), hit.hit_point(ray));
}

TEST(intersection_refraction_ut, bends_away_from_normal_when_leaving_material)
{
  const auto scene = refractive_scene();
  const intersection hit{0, 1.0F};
  const auto incident = normalize(math_vector{0.25F, 0.0F, std::sqrt(1.0F - 0.25F * 0.25F)});
  const math_ray ray{incident, {-0.25F, 0.0F, -std::sqrt(1.0F - 0.25F * 0.25F)}};

  const auto refracted = hit.refract(ray, scene);

  ASSERT_TRUE(refracted);
  expect_direction_near(normalize(refracted->direction()), {0.375F, 0.0F, std::sqrt(1.0F - 0.375F * 0.375F)});
  ASSERT_EQ(refracted->origin(), hit.hit_point(ray));
}

TEST(intersection_refraction_ut, returns_no_ray_for_total_internal_reflection)
{
  const auto scene = refractive_scene();
  const intersection hit{0, 1.0F};
  const auto incident = normalize(math_vector{0.8F, 0.0F, 0.6F});
  const math_ray ray{incident, {-0.8F, 0.0F, -0.6F}};

  ASSERT_FALSE(hit.refract(ray, scene));
}

TEST(intersection_refraction_ut, returns_no_ray_for_non_refractive_material)
{
  auto scene = refractive_scene();
  scene.materials[0].kts = 0.0F;
  const intersection hit{0, 1.0F};
  const math_ray ray{{0.0F, 0.0F, -1.0F}, {0.0F, 0.0F, 1.0F}};

  ASSERT_FALSE(hit.refract(ray, scene));
}
}  // namespace rtc::ut
