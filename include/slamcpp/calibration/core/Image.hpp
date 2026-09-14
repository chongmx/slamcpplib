// A dependency-free 8-bit grayscale image at the library boundary.
//
// This is the type that keeps OpenCV out of the core. Upstream passes cv::Mat
// through every layer, which drags OpenCV into the camera models, the target
// geometry and the error terms, none of which do any image processing. The
// port pushes OpenCV behind the optional target-detection component, and this
// is the type at that seam.
//
// Image is a non-owning view: it does not copy and does not free. It is
// constructible from a cv::Mat's data pointer without naming cv::Mat, which is
// the point. ImageBuffer owns storage for code that needs scratch space, such
// as the AprilTag detector's pyramids.
#ifndef CALIB_CORE_IMAGE_HPP
#define CALIB_CORE_IMAGE_HPP

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "slamcpp/calibration/core/Assert.hpp"

namespace slamcpp {
namespace calib {

class Image {
 public:
  constexpr Image() = default;

  // stride is the byte offset between consecutive rows. Zero means packed,
  // that is, stride == width. Non-packed strides matter because a cv::Mat ROI
  // is a view into a larger buffer and its rows are not contiguous.
  Image(const std::uint8_t* data, int width, int height, int stride = 0)
      : data_(data),
        width_(width),
        height_(height),
        stride_(stride > 0 ? stride : width) {
    CALIB_ASSERT_GE(width, 0, "Negative image width");
    CALIB_ASSERT_GE(height, 0, "Negative image height");
    CALIB_ASSERT_GE(stride_, width_, "Stride is narrower than the image");
  }

  bool empty() const { return data_ == nullptr || width_ == 0 || height_ == 0; }

  const std::uint8_t* data() const { return data_; }
  int width() const { return width_; }
  int height() const { return height_; }
  int stride() const { return stride_; }

  const std::uint8_t* row(int y) const {
    CALIB_ASSERT_GE_LT_DBG(y, 0, height_, "Row index out of range");
    return data_ + static_cast<std::ptrdiff_t>(y) * stride_;
  }

  std::uint8_t at(int x, int y) const {
    CALIB_ASSERT_GE_LT_DBG(x, 0, width_, "Column index out of range");
    return row(y)[x];
  }

  // Bilinear sample. Out-of-range coordinates clamp to the border, which is
  // what upstream's corner refinement assumes when a window overhangs an edge.
  double sampleBilinear(double x, double y) const;

  bool contains(double x, double y) const {
    return x >= 0.0 && y >= 0.0 && x <= width_ - 1.0 && y <= height_ - 1.0;
  }

 private:
  const std::uint8_t* data_ = nullptr;
  int width_ = 0;
  int height_ = 0;
  int stride_ = 0;
};

class ImageBuffer {
 public:
  ImageBuffer() = default;
  ImageBuffer(int width, int height) { resize(width, height); }

  void resize(int width, int height);

  // Copies the source, densifying it: the result is always packed, even when
  // the source had a wider stride.
  void assign(const Image& src);

  Image view() const { return Image(pixels_.data(), width_, height_, width_); }

  std::uint8_t* row(int y) {
    return pixels_.data() + static_cast<std::ptrdiff_t>(y) * width_;
  }
  const std::uint8_t* row(int y) const {
    return pixels_.data() + static_cast<std::ptrdiff_t>(y) * width_;
  }

  std::uint8_t& at(int x, int y) { return row(y)[x]; }
  std::uint8_t at(int x, int y) const { return row(y)[x]; }

  int width() const { return width_; }
  int height() const { return height_; }
  bool empty() const { return pixels_.empty(); }

 private:
  std::vector<std::uint8_t> pixels_;
  int width_ = 0;
  int height_ = 0;
};

}  // namespace calib
}  // namespace slamcpp

#endif  // CALIB_CORE_IMAGE_HPP
