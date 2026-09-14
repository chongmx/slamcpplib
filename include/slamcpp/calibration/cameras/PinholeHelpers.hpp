// Geometric helpers for pinhole intrinsics initialization.
//
// A 1:1 port of the PinholeHelpers class defined inline in upstream's
// aslam_cv/aslam_cameras/include/aslam/cameras/implementation/PinholeProjection.hpp
// (lines 598-705 at 1f60227). cv::Point2d becomes Eigen::Vector2d; nothing
// else changes.
//
// The arithmetic is reproduced operation for operation, including the order of
// the summations in fitCircle, because the port has to match upstream's output
// bit for bit before it is allowed to be better than it. Anything that looks
// like it wants tidying is deliberate; see the notes on each function.
#ifndef SLAMCPP_CALIBRATION_CAMERAS_PINHOLE_HELPERS_HPP
#define SLAMCPP_CALIBRATION_CAMERAS_PINHOLE_HELPERS_HPP

#include <vector>

#include <Eigen/Core>

namespace slamcpp {
namespace calib {
namespace pinhole_helpers {

inline double square(double x) { return x * x; }

// Upstream spells this out rather than calling std::hypot. std::hypot is more
// accurate for extreme magnitudes, and differs in the last bit, so using it
// here would break the match. Kept as-is deliberately.
inline double hypot(double a, double b) {
  return std::sqrt(square(a) + square(b));
}

// Intersection points of two circles. Returns 0, 1 or 2 points.
std::vector<Eigen::Vector2d> intersectCircles(double x1, double y1, double r1,
                                              double x2, double y2, double r2);

// Modified least-squares circle fit, from D. Umbach and K. Jones, "A Few
// Methods for Fitting Circles to Data", IEEE Trans. Instrumentation and
// Measurement, 2000.
//
// Upstream performs no check that (A * C - B^2) is non-zero before dividing by
// it, so a degenerate point set yields inf or NaN in the centre. That is
// preserved: the caller filters non-finite focal guesses downstream, and
// adding a check here would change which guesses reach the median.
void fitCircle(const std::vector<Eigen::Vector2d>& points, double& centerX,
               double& centerY, double& radius);

// Median by value, taking the vector by value so the caller's order survives.
// Even-sized input averages the two middle elements.
double medianOfVectorElements(std::vector<double> values);

}  // namespace pinhole_helpers
}  // namespace calib
}  // namespace slamcpp

#endif  // SLAMCPP_CALIBRATION_CAMERAS_PINHOLE_HELPERS_HPP
