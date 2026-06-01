#pragma once

#include <QImage>

#include <cstddef>

#include "bitmap.hpp"

namespace rtc_gui
{
auto bitmap_to_image(const rtc::screen_surface& bitmap) -> QImage;
auto non_black_pixels(const QImage& image) -> std::size_t;
}  // namespace rtc_gui
