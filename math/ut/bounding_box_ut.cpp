#include <gtest/gtest.h>

#include "bounding_box.hpp"

namespace rtc::ut
{
TEST(bounding_box_ut, direct_boundaries_are_preserved)
{
  const math_point min{-1.0F, -2.0F, -3.0F};
  const math_point max{4.0F, 5.0F, 6.0F};

  const auto box = bounding_box::from_boundaries(min, max);

  ASSERT_EQ(box.min_boundary(), min);
  ASSERT_EQ(box.max_boundary(), max);
}

TEST(bounding_box_ut, computes_surface_area_from_boundaries)
{
  const auto box = bounding_box::from_boundaries({0.0F, 0.0F, 0.0F}, {2.0F, 3.0F, 4.0F});

  ASSERT_EQ(box.surface_area(), 52.0F);
}

TEST(bounding_box_ut, computes_maximum_extent_from_boundaries)
{
  const auto box = bounding_box::from_boundaries({0.0F, 0.0F, 0.0F}, {2.0F, 7.0F, 4.0F});

  ASSERT_EQ(box.maximum_extent(), axis::y);
}
}  // namespace rtc::ut
