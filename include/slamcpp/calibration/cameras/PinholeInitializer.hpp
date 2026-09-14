// Closed-form focal length seed for the pinhole model.
//
// A 1:1 port of PinholeProjection<D>::initializeIntrinsics, upstream
// aslam_cv/aslam_cameras/include/aslam/cameras/implementation/PinholeProjection.hpp:713
// at 1f60227.
//
// Method: C. Hughes, P. Denny, M. Glavin and E. Jones, "Equidistant Fish-Eye
// Calibration and Rectification by Vanishing Point Extraction", PAMI 2010.
// Each row of target corners is fitted with a circle; each pair of circles
// intersects at two vanishing points; the distance between them over pi is one
// focal estimate. The seed is the median over every pair in every image.
//
// Two upstream behaviours are reproduced deliberately rather than corrected.
// Both are recorded in docs/kalibr/09-upstream-quirks.md.
#ifndef SLAMCPP_CALIBRATION_CAMERAS_PINHOLE_INITIALIZER_HPP
#define SLAMCPP_CALIBRATION_CAMERAS_PINHOLE_INITIALIZER_HPP

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace slamcpp {
namespace calib {

// One view of the calibration target, as a dense rows x cols grid. A corner
// that was not detected has valid == false; its point is ignored.
struct GridObservation {
  int rows = 0;
  int cols = 0;
  std::vector<bool> valid;             // rows * cols, row-major
  std::vector<Eigen::Vector2d> points; // rows * cols, row-major

  // Upstream maps grid coordinates to a point index as r * cols + c; see
  // GridCalibrationTargetAprilgrid.cpp, which fills _points.row(r * _cols + c).
  std::size_t index(int r, int c) const {
    return static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) +
           static_cast<std::size_t>(c);
  }
};

struct FocalLengthSeed {
  bool success = false;
  double f = 0.0;                 // the median guess, assigned to both fu and fv
  double cu = 0.0;                // (imageWidth  - 1) / 2
  double cv = 0.0;                // (imageHeight - 1) / 2
  std::vector<double> guesses;    // every finite per-pair guess, in order
  int imagesUsed = 0;             // images with a complete target view
};

// Returns the seed. success is false when no finite guess could be formed,
// which upstream reports by returning false after offering manual entry.
FocalLengthSeed initializePinholeFocalLength(
    const std::vector<GridObservation>& observations, int imageWidth,
    int imageHeight);

}  // namespace calib
}  // namespace slamcpp

#endif  // SLAMCPP_CALIBRATION_CAMERAS_PINHOLE_INITIALIZER_HPP
