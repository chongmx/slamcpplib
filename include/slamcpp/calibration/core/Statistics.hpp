// Running statistics, replacing the accumulator half of sm_timing.
//
// Upstream's sm::timing::Timing keeps per-tag running statistics using a
// hand-rolled incremental mean and variance. That accumulator is useful well
// beyond timing: the port needs it for reprojection-error summaries, residual
// statistics and solver benchmarks. It is lifted out here as a standalone
// type, and Timer.hpp is left behind entirely, since std::chrono plus this
// class covers every upstream use.
//
// The variance update is Welford's method, as upstream uses. It is chosen over
// the sum-of-squares form because the latter loses all precision when the mean
// is large relative to the spread, which is exactly the case for absolute
// timestamps.
#ifndef CALIB_CORE_STATISTICS_HPP
#define CALIB_CORE_STATISTICS_HPP

#include <cstddef>
#include <limits>
#include <string>

namespace slamcpp {
namespace calib {

class RunningStatistics {
 public:
  void add(double x);

  std::size_t count() const { return count_; }
  double mean() const { return count_ > 0 ? mean_ : 0.0; }

  // Sample variance, with the count - 1 denominator. Zero for fewer than two
  // samples rather than a division by zero.
  double variance() const;
  double standardDeviation() const;

  double min() const { return count_ > 0 ? min_ : 0.0; }
  double max() const { return count_ > 0 ? max_ : 0.0; }
  double sum() const { return sum_; }

  // Root mean square, which is what reprojection-error reporting quotes.
  // Distinct from standardDeviation() whenever the mean is non-zero.
  double rms() const;

  void clear();

  std::string toString() const;

 private:
  std::size_t count_ = 0;
  double mean_ = 0.0;
  double m2_ = 0.0;
  double sum_ = 0.0;
  double sumSquares_ = 0.0;
  double min_ = std::numeric_limits<double>::max();
  double max_ = std::numeric_limits<double>::lowest();
};

}  // namespace calib
}  // namespace slamcpp

#endif  // CALIB_CORE_STATISTICS_HPP
