#include "image_conversion.hpp"

#include <QColor>

#include <cstdint>

namespace rtc_gui
{
auto bitmap_to_image(const rtc::screen_surface& bitmap) -> QImage
{
  QImage image(static_cast<int>(bitmap.width()), static_cast<int>(bitmap.height()), QImage::Format_RGB32);

  for (int y = 0; y < image.height(); ++y)
  {
    for (int x = 0; x < image.width(); ++x)
    {
      const auto& c = bitmap(static_cast<std::uint16_t>(x), static_cast<std::uint16_t>(y));
      image.setPixel(x, y, qRgb(c.red, c.green, c.blue));
    }
  }

  return image;
}

auto non_black_pixels(const QImage& image) -> std::size_t
{
  std::size_t result{};

  for (int y = 0; y < image.height(); ++y)
  {
    for (int x = 0; x < image.width(); ++x)
    {
      const auto pixel = image.pixel(x, y);
      if (qRed(pixel) != 0 || qGreen(pixel) != 0 || qBlue(pixel) != 0)
        ++result;
    }
  }

  return result;
}
}  // namespace rtc_gui
