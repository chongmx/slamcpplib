#include "slamcpp/calibration/core/Timestamp.hpp"

#include <cmath>
#include <cstdio>

namespace slamcpp {
namespace calib {
namespace {
constexpr double kNanosecondsPerSecond = 1e9;
}

Timestamp Timestamp::fromSeconds(double seconds) {
  return Timestamp(secToNsec(seconds));
}

Timestamp Timestamp::fromChrono(
    const std::chrono::system_clock::time_point& time) {
  return Timestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(
                       time.time_since_epoch())
                       .count());
}

Timestamp Timestamp::now() {
  return fromChrono(std::chrono::system_clock::now());
}

double Timestamp::seconds() const { return nsecToSec(nsec_); }

std::chrono::system_clock::time_point Timestamp::toChrono() const {
  return std::chrono::system_clock::time_point(
      std::chrono::duration_cast<std::chrono::system_clock::duration>(
          std::chrono::nanoseconds(nsec_)));
}

std::string Timestamp::toString() const {
  // Floor division, so that negative times (before the epoch) still print a
  // non-negative nanosecond remainder rather than "-3.-500000000".
  NsecTime sec = nsec_ / 1000000000;
  NsecTime rem = nsec_ % 1000000000;
  if (rem < 0) {
    rem += 1000000000;
    sec -= 1;
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%lld.%09lld",
                static_cast<long long>(sec), static_cast<long long>(rem));
  return std::string(buf);
}

std::ostream& operator<<(std::ostream& out, Timestamp t) {
  return out << t.toString();
}

double nsecToSec(NsecTime nsec) {
  return static_cast<double>(nsec) / kNanosecondsPerSecond;
}

NsecTime secToNsec(double seconds) {
  // std::llround, not a truncating cast: upstream's cast makes secToNsec of
  // 0.1 come back as 99999999 on some platforms, and the round trip through
  // seconds is used when reading CSV timestamps.
  return static_cast<NsecTime>(std::llround(seconds * kNanosecondsPerSecond));
}

}  // namespace calib
}  // namespace slamcpp
