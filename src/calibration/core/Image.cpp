#include "slamcpp/calibration/core/Image.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace slamcpp {
namespace calib {

double Image::sampleBilinear(double x, double y) const {
  CALIB_ASSERT_FALSE_DBG(empty(), "Sampling an empty image");

  // Clamp into the interior so that x0 + 1 and y0 + 1 stay in range. Clamping
  // the coordinate rather than the index keeps the interpolation weights
  // consistent with the clamped position.
  x = std::min(std::max(x, 0.0), static_cast<double>(width_ - 1));
  y = std::min(std::max(y, 0.0), static_cast<double>(height_ - 1));

  const int x0 = std::min(static_cast<int>(std::floor(x)), width_ - 2 > 0 ? width_ - 2 : 0);
  const int y0 = std::min(static_cast<int>(std::floor(y)), height_ - 2 > 0 ? height_ - 2 : 0);
  const int x1 = std::min(x0 + 1, width_ - 1);
  const int y1 = std::min(y0 + 1, height_ - 1);

  const double ax = x - x0;
  const double ay = y - y0;

  const double v00 = row(y0)[x0];
  const double v01 = row(y0)[x1];
  const double v10 = row(y1)[x0];
  const double v11 = row(y1)[x1];

  const double top = v00 + ax * (v01 - v00);
  const double bottom = v10 + ax * (v11 - v10);
  return top + ay * (bottom - top);
}

void ImageBuffer::resize(int width, int height) {
  CALIB_ASSERT_GE(width, 0, "Negative image width");
  CALIB_ASSERT_GE(height, 0, "Negative image height");
  width_ = width;
  height_ = height;
  pixels_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);
}

void ImageBuffer::assign(const Image& src) {
  resize(src.width(), src.height());
  for (int y = 0; y < height_; ++y) {
    std::memcpy(row(y), src.row(y), static_cast<std::size_t>(width_));
  }
}

}  // namespace calib
}  // namespace slamcpp
