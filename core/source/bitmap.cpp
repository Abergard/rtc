#include <cstring>
#include <algorithm>
#include <cassert>

#include "bitmap.hpp"

namespace rtc
{

bitmap::bitmap() noexcept
{
  reset();
}

bitmap::bitmap(const std::string& path) : bitmap()
{
  if(!path.empty())
    load_from(path);
}

bitmap::bitmap(const std::uint16_t x, const std::uint16_t y) : bitmap()
{
  resize(x, y);
}

void bitmap::load_from(const std::string& path)
{
  std::ifstream out_file(path.c_str(), std::ios::binary);

  if (!out_file.is_open() || !out_file.good())
    throw std::runtime_error{"bitmap::load: file is not open"};

  out_file.read(reinterpret_cast<char*>(&header_data.bfType), sizeof(header_data.bfType));
  out_file.read(reinterpret_cast<char*>(&header_data.bfSize), sizeof(header_data.bfSize));
  out_file.read(reinterpret_cast<char*>(&header_data.bfReserved1), sizeof(header_data.bfReserved1));
  out_file.read(reinterpret_cast<char*>(&header_data.bfReserved1), sizeof(header_data.bfReserved1));
  out_file.read(reinterpret_cast<char*>(&header_data.bfOffBits), sizeof(header_data.bfOffBits));
  out_file.read(reinterpret_cast<char*>(&header_data.biSize), sizeof(header_data.biSize));
  out_file.read(reinterpret_cast<char*>(&header_data.biWidth), sizeof(header_data.biWidth));
  out_file.read(reinterpret_cast<char*>(&header_data.biHeight), sizeof(header_data.biHeight));
  out_file.read(reinterpret_cast<char*>(&header_data.biPlanes), sizeof(header_data.biPlanes));
  out_file.read(reinterpret_cast<char*>(&header_data.biBitCount), sizeof(header_data.biBitCount));
  out_file.read(reinterpret_cast<char*>(&header_data.biCompression), sizeof(header_data.biCompression));
  out_file.read(reinterpret_cast<char*>(&header_data.biSizeImage), sizeof(header_data.biSizeImage));
  out_file.read(reinterpret_cast<char*>(&header_data.biXPelsPerMeter), sizeof(header_data.biXPelsPerMeter));
  out_file.read(reinterpret_cast<char*>(&header_data.biYPelsPerMeter), sizeof(header_data.biYPelsPerMeter));
  out_file.read(reinterpret_cast<char*>(&header_data.biClrUsed), sizeof(header_data.biClrUsed));
  out_file.read(reinterpret_cast<char*>(&header_data.biClrImportant), sizeof(header_data.biClrImportant));
  out_file.read(reinterpret_cast<char*>(&header_data.biCrlRotation), sizeof(header_data.biCrlRotation));
  out_file.read(reinterpret_cast<char*>(&header_data.biReserved), sizeof(header_data.biReserved));
  out_file.seekg(header_data.bfOffBits, std::ios::beg);

  resize(width(), height());

  const std::uint8_t align = (4 - ((3 * width()) % 4)) % 4;
  color_rgb l_tmp{};

  for (std::uint32_t j = 0; j < height(); j++)
  {
    for (std::uint32_t i = 0; i < width(); i++)
    {
      out_file.read(reinterpret_cast<char*>(&l_tmp.blue), sizeof(l_tmp.blue));
      out_file.read(reinterpret_cast<char*>(&l_tmp.green), sizeof(l_tmp.green));
      out_file.read(reinterpret_cast<char*>(&l_tmp.red), sizeof(l_tmp.red));
      operator()(i, height() - j - 1) = l_tmp;
    }

    for (int a = 0; a < align; a++)
      out_file.read(reinterpret_cast<char*>(std::addressof(l_tmp.blue)), sizeof(l_tmp.blue));
  }
}

void bitmap::save(const std::string& path) const
{
  if (base::empty())
    throw std::runtime_error("bitmap is empty");

  std::ofstream out_file(path.c_str(), std::ios::binary);

  if (!out_file.is_open() || !out_file.good())
    throw std::runtime_error("bitmap::save: file is not open");

  out_file.write(reinterpret_cast<const char*>(&header_data.bfType), sizeof(header_data.bfType));
  out_file.write(reinterpret_cast<const char*>(&header_data.bfSize), sizeof(header_data.bfSize));
  out_file.write(reinterpret_cast<const char*>(&header_data.bfReserved1), sizeof(header_data.bfReserved1));
  out_file.write(reinterpret_cast<const char*>(&header_data.bfReserved1), sizeof(header_data.bfReserved1));
  out_file.write(reinterpret_cast<const char*>(&header_data.bfOffBits), sizeof(header_data.bfOffBits));
  out_file.write(reinterpret_cast<const char*>(&header_data.biSize), sizeof(header_data.biSize));
  out_file.write(reinterpret_cast<const char*>(&header_data.biWidth), sizeof(header_data.biWidth));
  out_file.write(reinterpret_cast<const char*>(&header_data.biHeight), sizeof(header_data.biHeight));
  out_file.write(reinterpret_cast<const char*>(&header_data.biPlanes), sizeof(header_data.biPlanes));
  out_file.write(reinterpret_cast<const char*>(&header_data.biBitCount), sizeof(header_data.biBitCount));
  out_file.write(reinterpret_cast<const char*>(&header_data.biCompression), sizeof(header_data.biCompression));
  out_file.write(reinterpret_cast<const char*>(&header_data.biSizeImage), sizeof(header_data.biSizeImage));
  out_file.write(reinterpret_cast<const char*>(&header_data.biXPelsPerMeter), sizeof(header_data.biXPelsPerMeter));
  out_file.write(reinterpret_cast<const char*>(&header_data.biYPelsPerMeter), sizeof(header_data.biYPelsPerMeter));
  out_file.write(reinterpret_cast<const char*>(&header_data.biClrUsed), sizeof(header_data.biClrUsed));
  out_file.write(reinterpret_cast<const char*>(&header_data.biClrImportant), sizeof(header_data.biClrImportant));
  out_file.write(reinterpret_cast<const char*>(&header_data.biCrlRotation), sizeof(header_data.biCrlRotation));
  out_file.write(reinterpret_cast<const char*>(&header_data.biReserved), sizeof(header_data.biReserved));

  const std::uint8_t l_tmp{0};
  const std::uint8_t align = (4 - ((width() * 3) % 4)) % 4;

  out_file.seekp(header_data.bfOffBits, std::ios::beg);

  for (std::uint32_t j = 0; j < height(); j++)
  {
    for (std::uint32_t i = 0; i < width(); i++)
    {
      out_file.write((char*)&(*this)(i, height() - j - 1).blue,  sizeof(std::uint8_t));
      out_file.write((char*)&(*this)(i, height() - j - 1).green, sizeof(std::uint8_t));
      out_file.write((char*)&(*this)(i, height() - j - 1).red,   sizeof(std::uint8_t));
    }

    for (std::uint32_t k = 0; k < align; k++)
      out_file.write((char*)std::addressof(l_tmp), sizeof(l_tmp));
  }
}

std::size_t bitmap::width() const noexcept
{
  return header_data.biWidth;
}

std::size_t bitmap::height() const noexcept
{
  return header_data.biHeight;
}

bool bitmap::assign(const std::uint16_t x, const std::uint16_t y, const color_rgb& color) noexcept
{
  return assign({x, y, color});
}

bool bitmap::assign(const rtc::pixel& p) noexcept
{
  if (in_range(p.x, p.y))
    operator()(p.x, p.y) = p.color;

  return in_range(p.x, p.y);
}

bitmap& bitmap::resize(const std::uint16_t width, const std::uint16_t height)
{
  base old{std::move(*this)};

  reset();
  base::reserve(width * height);
  header_data.biWidth  = width;
  header_data.biHeight = height;

  for(std::size_t i{}; i < old.size() ; ++i)
    base::emplace_back(old[i].x, old[i].y, old[i].color);

  for(std::size_t i{old.size()}; i < width * height ; ++i)
    base::emplace_back(i % width, i / width, color_rgb{});

  base::shrink_to_fit();
  return *this;
}

std::size_t bitmap::pixel_amount() const noexcept
{
  return size();
}

color_rgb& bitmap::operator()(const std::uint16_t x, const std::uint16_t y)
{
  assert(in_range(x, y));
#ifndef NDEBUG
  return operator[](y * width() + x).color;
#else
  return base::at(y * width() + x).color;
#endif
}

const color_rgb& bitmap::operator()(const std::uint16_t x, const std::uint16_t y) const
{
  assert(in_range(x, y));
#ifndef NDEBUG
  return operator[](y * width() + x).color;
#else
  return base::at(y * width() + x).color;
#endif
}

bitmap& bitmap::clear(const rtc::color_rgb& color) noexcept
{
  std::for_each(begin(), end(), [&](auto& c){ c.color = color; });
  return *this;
}

bitmap& bitmap::revert() noexcept
{
  std::for_each(begin(), end(), [](auto& c){ c.color = c.color.revert(); });
  return *this;
}

bitmap bitmap::operator!() const
{
  return bitmap(*this).revert();
}

bitmap& bitmap::insert(const std::uint16_t x, const std::uint16_t y, const bitmap& bmp)
{
  if(empty())
    resize(x + bmp.width(), y + bmp.height());

  for(const auto& p : *this)
  {
    const auto l_x{x + p.x}, l_y{y + p.y};

    if(in_range(l_x, l_y))
      operator()(l_x, l_y) = p.color;
  }
  return *this;
}

void bitmap::reset() noexcept
{
  const std::uint8_t align = (4 - ((header_data.biWidth * 3) % 4)) % 4;

  header_data = {};
  header_data.bfType = 'B' | ('M' << 8);
  header_data.bfSize = sizeof(header_data) + header_data.biWidth * header_data.biHeight * (sizeof(color_rgb) + align) ;
  header_data.biSizeImage = header_data.biWidth * header_data.biHeight * sizeof(color_rgb);
  header_data.bfOffBits = sizeof(header_data);
  header_data.biSize = 40;
  header_data.biPlanes = 1;
  header_data.biBitCount = 24;
  header_data.biXPelsPerMeter = 2834;
  header_data.biYPelsPerMeter = 2834;
}

bitmap bitmap::trim(const std::uint16_t x,
                    const std::uint16_t y,
                    const std::uint16_t width,
                    const std::uint16_t height) const
{
  if(empty())
    return bitmap{};

  bitmap bmp{width, height};

  for(pixel& p : bmp)
  {
    const auto l_x{x + p.x}, l_y{y + p.y};

    if (in_range(l_x, l_y))
      p.color = operator()(l_x, l_y);
  }
  return bmp;
}

bool bitmap::in_range(const std::uint16_t x, const std::uint16_t y) const noexcept
{
  return (x < width()) && (y < height());
}

color_rgb& bitmap::at(const std::uint16_t x, const std::uint16_t y)
{
  if(in_range(x, y))
    return operator()(x, y);

  throw std::out_of_range{"bitmap::at -> in_range false"};
}

const color_rgb& bitmap::at(const std::uint16_t x, const std::uint16_t y) const
{
  if(in_range(x, y))
    return operator()(x, y);

  throw std::out_of_range("bitmap::at const -> in_range false");
}

bitmap& bitmap::swap(bitmap& bmp) noexcept(noexcept(std::declval<base>().swap(std::declval<base&>())))
{
  base::swap(bmp);
  return *this;
}

bitmap& bitmap::draw(const std::function<color_rgb(std::uint16_t, std::uint16_t)>& fn) noexcept
{
  for(auto i{0u}; i < width(); ++i)
    for(auto j{0u}; j < height(); ++j)
      operator()(i, j) = fn(i, j);

  return *this;
}

} //namespace rtc