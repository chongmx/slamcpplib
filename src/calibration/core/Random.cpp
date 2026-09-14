#include "slamcpp/calibration/core/Random.hpp"

#include "slamcpp/calibration/core/Assert.hpp"

namespace slamcpp {
namespace calib {
namespace random {
namespace {

// The generator and the distributions have to live together, because seeding
// has to reset both.
//
// std::normal_distribution is stateful: the Box-Muller / polar methods produce
// two deviates per call and cache the second. Reseeding the generator without
// resetting the distribution returns that stale cached value first, so the
// sequence after seed(s) is not a function of s alone and tests that reseed to
// reproduce a failure quietly do not. The distribution also cannot be
// constructed per call, or the cached half of every pair is thrown away, which
// doubles the cost and changes the stream. Holding both here and calling
// reset() on seed is what makes seeding actually reproducible.
struct State {
  std::mt19937_64 gen{42};
  std::normal_distribution<double> normalDist{0.0, 1.0};
  std::uniform_real_distribution<double> unitDist{0.0, 1.0};
};

State& state() {
  // Thread-local, unlike upstream's single shared generator, which is an
  // unsynchronised data race under the threaded optimiser. The cost is that
  // seeding is per-thread; the tests that need reproducibility are
  // single-threaded, so that is the right trade.
  static thread_local State s;
  return s;
}

}  // namespace

std::mt19937_64& generator() { return state().gen; }

void seed(std::uint64_t s) {
  State& st = state();
  st.gen.seed(s);
  st.normalDist.reset();
  st.unitDist.reset();
}

double normal() {
  State& st = state();
  return st.normalDist(st.gen);
}

double uniform() {
  State& st = state();
  return st.unitDist(st.gen);
}

double uniform(double lowerInclusive, double upperExclusive) {
  CALIB_ASSERT_LT(lowerInclusive, upperExclusive,
                   "Empty or inverted range requested");
  std::uniform_real_distribution<double> dist(lowerInclusive, upperExclusive);
  return dist(generator());
}

int uniformInt(int lowerInclusive, int upperExclusive) {
  CALIB_ASSERT_LT(lowerInclusive, upperExclusive,
                   "Empty or inverted range requested");
  // uniform_int_distribution is closed on both ends, unlike the half-open
  // range this function promises, so the upper bound is decremented.
  std::uniform_int_distribution<int> dist(lowerInclusive, upperExclusive - 1);
  return dist(generator());
}

}  // namespace random
}  // namespace calib
}  // namespace slamcpp
