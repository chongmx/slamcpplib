// Random sampling, replacing sm_random and sm_eigen's random.hpp.
//
// Upstream seeds a global boost generator and exposes free functions. That is
// kept, because ported code calls the free functions everywhere, but the
// generator is now a std::mt19937_64 in thread-local storage. Upstream's is a
// single shared generator with no synchronisation, which is a data race in the
// threaded optimiser; making it thread-local removes the race and costs
// nothing, at the price that seeding is per-thread. Tests that need
// reproducibility are single-threaded, so this is the right trade.
#ifndef CALIB_CORE_RANDOM_HPP
#define CALIB_CORE_RANDOM_HPP

#include <cstdint>
#include <random>

#include <Eigen/Core>

namespace slamcpp {
namespace calib {
namespace random {

// Seeds the calling thread's generator.
void seed(std::uint64_t s);

std::mt19937_64& generator();

// Zero mean, unit standard deviation.
double normal();

// Half-open [0, 1).
double uniform();

// Half-open [lowerInclusive, upperExclusive).
double uniform(double lowerInclusive, double upperExclusive);
int uniformInt(int lowerInclusive, int upperExclusive);

// Upstream spellings, kept so ported call sites need no edit.
inline double randn() { return normal(); }
inline double rand() { return uniform(); }
inline double randLU(double lower, double upper) {
  return uniform(lower, upper);
}
inline int randLUi(int lower, int upper) { return uniformInt(lower, upper); }

// Fills any fixed- or dynamic-size Eigen expression with standard normals.
// Dynamic-size matrices must be sized by the caller first.
template <typename Derived>
void fillNormal(Eigen::MatrixBase<Derived>& m) {
  for (Eigen::Index c = 0; c < m.cols(); ++c) {
    for (Eigen::Index r = 0; r < m.rows(); ++r) {
      m(r, c) = static_cast<typename Derived::Scalar>(normal());
    }
  }
}

template <typename Derived>
void fillUniform(Eigen::MatrixBase<Derived>& m, double lower, double upper) {
  for (Eigen::Index c = 0; c < m.cols(); ++c) {
    for (Eigen::Index r = 0; r < m.rows(); ++r) {
      m(r, c) = static_cast<typename Derived::Scalar>(uniform(lower, upper));
    }
  }
}

}  // namespace random
}  // namespace calib
}  // namespace slamcpp

#endif  // CALIB_CORE_RANDOM_HPP
