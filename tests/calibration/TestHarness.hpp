// A minimal test harness.
//
// Upstream uses gtest, pulled in by catkin. The port has no package manager and
// the plan makes Eigen the only mandatory dependency, so fetching gtest at
// configure time would make an offline build impossible and add a dependency
// the library itself does not need. This is roughly 120 lines and covers what
// the ported suites use: registration, assertions, Eigen-aware comparisons, and
// a Jacobian check against finite differences.
//
// If the project later wants gtest, the macro names here are deliberately
// close enough that a port is mechanical.
#ifndef CALIB_TEST_HARNESS_HPP
#define CALIB_TEST_HARNESS_HPP

#include <cmath>
#include <cstdio>
#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "slamcpp/calibration/core/NumericalDiff.hpp"

namespace slamcpp {
namespace calib {
namespace test {

struct TestCase {
  std::string suite;
  std::string name;
  std::function<void()> body;
};

inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> tests;
  return tests;
}

struct Registrar {
  Registrar(const char* suite, const char* name, std::function<void()> body) {
    registry().push_back({suite, name, std::move(body)});
  }
};

// Thrown by a failing assertion. Caught by the runner, which turns it into a
// reported failure rather than a crash, so one bad test does not hide the rest.
struct Failure : std::exception {
  explicit Failure(std::string what) : message(std::move(what)) {}
  const char* what() const noexcept override { return message.c_str(); }
  std::string message;
};

inline int run(const std::string& filter = "") {
  int passed = 0;
  std::vector<std::string> failures;

  for (const auto& t : registry()) {
    const std::string full = t.suite + "." + t.name;
    if (!filter.empty() && full.find(filter) == std::string::npos) continue;

    try {
      t.body();
      ++passed;
      std::printf("  [ ok ] %s\n", full.c_str());
    } catch (const Failure& f) {
      failures.push_back(full + "\n" + f.message);
      std::printf("  [FAIL] %s\n%s\n", full.c_str(), f.message.c_str());
    } catch (const std::exception& e) {
      failures.push_back(full + "\n    unexpected exception: " + e.what());
      std::printf("  [FAIL] %s\n    unexpected exception: %s\n", full.c_str(),
                  e.what());
    }
  }

  std::printf("\n%d passed, %zu failed\n", passed, failures.size());
  return failures.empty() ? 0 : 1;
}

}  // namespace test
}  // namespace calib
}  // namespace slamcpp

#define CALIB_TEST(suite, name)                                            \
  static void calib_test_##suite##_##name();                               \
  static ::slamcpp::calib::test::Registrar calib_registrar_##suite##_##name(       \
      #suite, #name, calib_test_##suite##_##name);                         \
  static void calib_test_##suite##_##name()

#define CALIB_FAIL(msg)                                                    \
  do {                                                                      \
    std::stringstream calib_fail_ss;                                       \
    calib_fail_ss << "    " << __FILE__ << ":" << __LINE__ << "\n    "     \
                   << msg;                                                  \
    throw ::slamcpp::calib::test::Failure(calib_fail_ss.str());                    \
  } while (0)

#define EXPECT_TRUE(cond)                                                   \
  do {                                                                      \
    if (!(cond)) CALIB_FAIL("expected true: " #cond);                      \
  } while (0)

#define EXPECT_FALSE(cond)                                                  \
  do {                                                                      \
    if ((cond)) CALIB_FAIL("expected false: " #cond);                      \
  } while (0)

#define EXPECT_EQ(a, b)                                                     \
  do {                                                                      \
    if (!((a) == (b)))                                                      \
      CALIB_FAIL("expected " #a " == " #b "\n      lhs = "                 \
                  << (a) << "\n      rhs = " << (b));                       \
  } while (0)

#define EXPECT_NEAR(a, b, tol)                                              \
  do {                                                                      \
    const double calib_a = (a), calib_b = (b);                            \
    if (!(std::fabs(calib_a - calib_b) <= (tol)))                         \
      CALIB_FAIL("expected " #a " within " << (tol) << " of " #b           \
                  << "\n      lhs = " << calib_a                           \
                  << "\n      rhs = " << calib_b                           \
                  << "\n      diff = " << std::fabs(calib_a - calib_b));  \
  } while (0)

#define EXPECT_THROWS(stmt)                                                 \
  do {                                                                      \
    bool calib_threw = false;                                              \
    try {                                                                   \
      stmt;                                                                 \
    } catch (...) {                                                         \
      calib_threw = true;                                                  \
    }                                                                       \
    if (!calib_threw) CALIB_FAIL("expected an exception from: " #stmt);   \
  } while (0)

// Entrywise comparison with a shape check first, so a size mismatch reports as
// a shape error rather than an out-of-range read.
#define EXPECT_MATRIX_NEAR(A, B, tol)                                       \
  do {                                                                      \
    const Eigen::MatrixXd calib_A = (A);                                   \
    const Eigen::MatrixXd calib_B = (B);                                   \
    if (calib_A.rows() != calib_B.rows() ||                               \
        calib_A.cols() != calib_B.cols())                                 \
      CALIB_FAIL("shape mismatch: " #A " is " << calib_A.rows() << "x"    \
                  << calib_A.cols() << ", " #B " is " << calib_B.rows()   \
                  << "x" << calib_B.cols());                               \
    const double calib_d = (calib_A - calib_B).cwiseAbs().maxCoeff();    \
    if (!(calib_d <= (tol)))                                               \
      CALIB_FAIL("matrices differ by " << calib_d << " > " << (tol)       \
                  << "\n" #A " =\n" << calib_A << "\n" #B " =\n"           \
                  << calib_B);                                             \
  } while (0)

// The Level 1 check: an analytic Jacobian against central differences.
#define EXPECT_JACOBIAN_NEAR(analytic, functor, x, tol)                     \
  do {                                                                      \
    const Eigen::MatrixXd calib_num =                                      \
        ::slamcpp::calib::numericalJacobian(functor, x);                            \
    const Eigen::MatrixXd calib_ana = (analytic);                          \
    const double calib_rel =                                               \
        ::slamcpp::calib::relativeDifference(calib_ana, calib_num);               \
    if (!(calib_rel <= (tol)))                                             \
      CALIB_FAIL("Jacobian relative error " << calib_rel << " > " << (tol) \
                  << "\n      analytic =\n" << calib_ana                   \
                  << "\n      numeric =\n" << calib_num);                  \
  } while (0)

#endif  // CALIB_TEST_HARNESS_HPP
