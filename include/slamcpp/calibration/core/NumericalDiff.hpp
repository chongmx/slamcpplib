// Central-difference Jacobians, for validating analytic ones.
//
// Replaces sm_eigen's NumericalDiff.hpp. This is the single most important
// piece of test infrastructure in the port. The failure mode that matters when
// porting an estimator is a Jacobian that is subtly wrong: the optimiser still
// converges, just to a slightly different answer, and nothing reports a
// problem. Every expression node and error term is checked against this.
//
// Central differences, not forward: the error is O(h^2) rather than O(h), which
// is what makes a 1e-6 relative tolerance achievable at all. The default step
// is 1e-6, matching the tolerance stated in the validation strategy.
#ifndef CALIB_CORE_NUMERICAL_DIFF_HPP
#define CALIB_CORE_NUMERICAL_DIFF_HPP

#include <cmath>
#include <string>
#include <sstream>

#include <Eigen/Core>

namespace slamcpp {
namespace calib {

// Jacobian of f at x, by central differences.
//
// f maps an Eigen vector to an Eigen vector and may change size with its
// input; the output size is taken from f(x). The step is scaled by the
// magnitude of each component, so that a parameter of order 1e6 is perturbed
// proportionally rather than by an absolute 1e-6 that vanishes into rounding.
template <typename Functor>
Eigen::MatrixXd numericalJacobian(Functor&& f, const Eigen::VectorXd& x,
                                  double step = 1e-6) {
  const Eigen::VectorXd f0 = f(x);
  Eigen::MatrixXd J(f0.size(), x.size());

  Eigen::VectorXd xPlus = x;
  Eigen::VectorXd xMinus = x;

  for (Eigen::Index i = 0; i < x.size(); ++i) {
    const double h = step * std::max(1.0, std::abs(x[i]));
    xPlus[i] = x[i] + h;
    xMinus[i] = x[i] - h;

    // The realised step, which differs from 2h when x[i] + h is not exactly
    // representable. Dividing by the realised value rather than by 2h removes
    // that rounding from the result.
    const double realised = xPlus[i] - xMinus[i];
    J.col(i) = (f(xPlus) - f(xMinus)) / realised;

    xPlus[i] = x[i];
    xMinus[i] = x[i];
  }
  return J;
}

// Relative difference between two matrices, normalised by the larger entry so
// that near-zero Jacobian entries do not report a spurious relative error.
inline double relativeDifference(const Eigen::MatrixXd& a,
                                 const Eigen::MatrixXd& b) {
  if (a.rows() != b.rows() || a.cols() != b.cols()) {
    return std::numeric_limits<double>::infinity();
  }
  double worst = 0.0;
  for (Eigen::Index c = 0; c < a.cols(); ++c) {
    for (Eigen::Index r = 0; r < a.rows(); ++r) {
      const double scale =
          std::max(1.0, std::max(std::abs(a(r, c)), std::abs(b(r, c))));
      worst = std::max(worst, std::abs(a(r, c) - b(r, c)) / scale);
    }
  }
  return worst;
}

}  // namespace calib
}  // namespace slamcpp

#endif  // CALIB_CORE_NUMERICAL_DIFF_HPP
