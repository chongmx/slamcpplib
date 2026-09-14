// Nanosecond epoch time, replacing sm_timing's NsecTimeUtilities and
// aslam_time's Time/Duration.
//
// Upstream carries two parallel time types: sm::timing::NsecTime, a bare
// int64 of nanoseconds, and aslam::Time, a ROS-shaped sec/nsec pair with
// arithmetic operators. They convert back and forth at layer boundaries. This
// keeps one type. Timestamp is a strongly typed int64 of nanoseconds since the
// Unix epoch, so a raw integer cannot be passed where a time is expected, and
// the sec/nsec split is a formatting concern rather than a representation.
//
// boost::int64_t becomes std::int64_t and boost::posix_time becomes
// std::chrono, which is the whole of the Boost removal here.
#ifndef CALIB_CORE_TIMESTAMP_HPP
#define CALIB_CORE_TIMESTAMP_HPP

#include <chrono>
#include <cstdint>
#include <ostream>
#include <string>

namespace slamcpp {
namespace calib {

using NsecTime = std::int64_t;

class Timestamp {
 public:
  constexpr Timestamp() = default;
  constexpr explicit Timestamp(NsecTime nsec) : nsec_(nsec) {}

  static constexpr Timestamp fromNanoseconds(NsecTime nsec) {
    return Timestamp(nsec);
  }

  // Seconds are double, so this loses resolution for absolute epoch times
  // beyond about 100 ns. Upstream has the same property; every call site that
  // matters works in differences, where the resolution is exact.
  static Timestamp fromSeconds(double seconds);

  static Timestamp fromChrono(
      const std::chrono::system_clock::time_point& time);

  // Wall clock now, as nanoseconds since the Unix epoch.
  static Timestamp now();

  constexpr NsecTime nanoseconds() const { return nsec_; }
  double seconds() const;
  std::chrono::system_clock::time_point toChrono() const;

  // "<seconds>.<9-digit nanoseconds>", the format upstream writes into
  // reports and dataset indices.
  std::string toString() const;

  constexpr Timestamp& operator+=(NsecTime d) { nsec_ += d; return *this; }
  constexpr Timestamp& operator-=(NsecTime d) { nsec_ -= d; return *this; }

 private:
  NsecTime nsec_ = 0;
};

constexpr Timestamp operator+(Timestamp t, NsecTime d) {
  return Timestamp(t.nanoseconds() + d);
}
constexpr Timestamp operator-(Timestamp t, NsecTime d) {
  return Timestamp(t.nanoseconds() - d);
}
// Difference of two timestamps is a duration in nanoseconds, not a timestamp.
constexpr NsecTime operator-(Timestamp a, Timestamp b) {
  return a.nanoseconds() - b.nanoseconds();
}

constexpr bool operator==(Timestamp a, Timestamp b) {
  return a.nanoseconds() == b.nanoseconds();
}
constexpr bool operator!=(Timestamp a, Timestamp b) { return !(a == b); }
constexpr bool operator<(Timestamp a, Timestamp b) {
  return a.nanoseconds() < b.nanoseconds();
}
constexpr bool operator>(Timestamp a, Timestamp b) { return b < a; }
constexpr bool operator<=(Timestamp a, Timestamp b) { return !(b < a); }
constexpr bool operator>=(Timestamp a, Timestamp b) { return !(a < b); }

std::ostream& operator<<(std::ostream& out, Timestamp t);

// Free functions matching the upstream sm::timing spelling, for ported code.
double nsecToSec(NsecTime nsec);
NsecTime secToNsec(double seconds);

}  // namespace calib
}  // namespace slamcpp

#endif  // CALIB_CORE_TIMESTAMP_HPP
