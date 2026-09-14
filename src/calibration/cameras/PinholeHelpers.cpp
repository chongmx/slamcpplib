#include "slamcpp/calibration/cameras/PinholeHelpers.hpp"

#include <algorithm>
#include <cmath>

namespace slamcpp {
namespace calib {
namespace pinhole_helpers {

std::vector<Eigen::Vector2d> intersectCircles(double x1, double y1, double r1,
                                              double x2, double y2, double r2) {
  std::vector<Eigen::Vector2d> ipts;

  const double d = hypot(x1 - x2, y1 - y2);
  if (d > r1 + r2) {
    // circles are separate
    return ipts;
  }
  if (d < std::fabs(r1 - r2)) {
    // one circle is contained within the other
    return ipts;
  }

  const double a = (square(r1) - square(r2) + square(d)) / (2.0 * d);
  const double h = std::sqrt(square(r1) - square(a));

  const double x3 = x1 + a * (x2 - x1) / d;
  const double y3 = y1 + a * (y2 - y1) / d;

  if (h < 1e-10) {
    // two circles touch at one point
    ipts.push_back(Eigen::Vector2d(x3, y3));
    return ipts;
  }

  ipts.push_back(Eigen::Vector2d(x3 + h * (y2 - y1) / d, y3 - h * (x2 - x1) / d));
  ipts.push_back(Eigen::Vector2d(x3 - h * (y2 - y1) / d, y3 + h * (x2 - x1) / d));
  return ipts;
}

void fitCircle(const std::vector<Eigen::Vector2d>& points, double& centerX,
               double& centerY, double& radius) {
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  double sum_xy = 0.0;
  double sum_yy = 0.0;
  double sum_xxx = 0.0;
  double sum_xxy = 0.0;
  double sum_xyy = 0.0;
  double sum_yyy = 0.0;

  const int n = static_cast<int>(points.size());
  for (int i = 0; i < n; ++i) {
    const double x = points.at(i).x();
    const double y = points.at(i).y();

    sum_x += x;
    sum_y += y;
    sum_xx += x * x;
    sum_xy += x * y;
    sum_yy += y * y;
    sum_xxx += x * x * x;
    sum_xxy += x * x * y;
    sum_xyy += x * y * y;
    sum_yyy += y * y * y;
  }

  const double A = n * sum_xx - square(sum_x);
  const double B = n * sum_xy - sum_x * sum_y;
  const double C = n * sum_yy - square(sum_y);
  const double D =
      0.5 * (n * sum_xyy - sum_x * sum_yy + n * sum_xxx - sum_x * sum_xx);
  const double E =
      0.5 * (n * sum_xxy - sum_y * sum_xx + n * sum_yyy - sum_y * sum_yy);

  centerX = (D * C - B * E) / (A * C - square(B));
  centerY = (A * E - B * D) / (A * C - square(B));

  double sum_r = 0.0;
  for (int i = 0; i < n; ++i) {
    const double x = points.at(i).x();
    const double y = points.at(i).y();
    sum_r += hypot(x - centerX, y - centerY);
  }
  radius = sum_r / n;
}

double medianOfVectorElements(std::vector<double> values) {
  double median;
  const std::size_t size = values.size();
  std::sort(values.begin(), values.end());
  if (size % 2 == 0) {
    median = (values[size / 2 - 1] + values[size / 2]) / 2;
  } else {
    median = values[size / 2];
  }
  return median;
}

}  // namespace pinhole_helpers
}  // namespace calib
}  // namespace slamcpp
