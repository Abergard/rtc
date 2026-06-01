#include "gtest/gtest.h"
#include "rt_service.hpp"
#include "brs.hpp"
#include <atomic>
#include <memory>

namespace rtc::ut
{
struct cancel_after_first_pixel_algorithm
{
  rtc::rt_render_progress* progress{};
  std::atomic<std::uint32_t>* calls{};

  template <typename RtService>
  auto make_color(const rtc::math_ray&, const rtc::color& background, RtService&) noexcept -> rtc::color
  {
    calls->fetch_add(1, std::memory_order_relaxed);
    progress->request_cancel();
    return background;
  }
};

TEST(rt_service_ut, basic_move_test)
{
  auto data = std::make_shared<rtc::brs>("room.xml");
  rtc::rt_service<> r{data};

  auto rt_service{std::move(r)};
}

TEST(rt_service_ut, basic_test)
{
  auto data = std::make_shared<rtc::brs>("room.xml");
  rtc::rt_service<> rt{data};

  const rtc::math_vector v{-4.396932F, -0.778016F, 6.014521F};
  const rtc::math_ray ray{v, data->optical_system.view_point};

  auto intersection = rt.trace_ray(ray).get();

  ASSERT_TRUE(intersection.is_present());
  ASSERT_TRUE(intersection.is_with(619));
}

TEST(rt_service_ut, cancelled_render_stops_inside_tile)
{
  auto data = std::make_shared<rtc::brs>("room.xml");
  rtc::rt_service<> rt{data};
  rtc::screen_surface surface{64, 64};
  rtc::rt_render_progress progress;
  std::atomic<std::uint32_t> calls{};
  cancel_after_first_pixel_algorithm algorithm{&progress, &calls};

  progress.reset(1);
  rt.execute({0, 0, 64, 64}, surface, algorithm, &progress);
  rt.finish();

  ASSERT_GT(calls.load(std::memory_order_relaxed), 0U);
  ASSERT_LT(calls.load(std::memory_order_relaxed), 64U * 64U);
  ASSERT_TRUE(progress.is_cancelled());
}

}
