#include "slamcpp/calibration/core/Statistics.hpp"

#include <cmath>
#include <sstream>

namespace slamcpp {
namespace calib {

void RunningStatistics::add(double x) {
  ++count_;
  const double delta = x - mean_;
  mean_ += delta / static_cast<double>(count_);
  // Uses the updated mean, which is what makes this Welford's method rather
  // than a biased variant.
  m2_ += delta * (x - mean_);
  sum_ += x;
  sumSquares_ += x * x;
  if (x < min_) min_ = x;
  if (x > max_) max_ = x;
}

double RunningStatistics::variance() const {
  if (count_ < 2) return 0.0;
  return m2_ / static_cast<double>(count_ - 1);
}

double RunningStatistics::standardDeviation() const {
  return std::sqrt(variance());
}

double RunningStatistics::rms() const {
  if (count_ == 0) return 0.0;
  return std::sqrt(sumSquares_ / static_cast<double>(count_));
}

void RunningStatistics::clear() { *this = RunningStatistics(); }

std::string RunningStatistics::toString() const {
  std::stringstream ss;
  ss << "n=" << count_ << " mean=" << mean() << " std=" << standardDeviation()
     << " rms=" << rms() << " min=" << min() << " max=" << max();
  return ss.str();
}

}  // namespace calib
}  // namespace slamcpp
