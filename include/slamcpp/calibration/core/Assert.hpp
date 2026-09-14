// Assertion macros, ported from sm_common's sm/assert_macros.hpp.
//
// The macro set and message formatting are kept compatible so that ported call
// sites read the same and failure text can be diffed against upstream. Three
// deliberate changes:
//
//   1. Every macro is wrapped in do { ... } while (0). Upstream expands to a
//      bare `if (...) { ... }`, so an `else` following a call site binds to the
//      macro's `if` instead of the caller's. No upstream call site hits this,
//      but it is a trap that costs nothing to close.
//   2. The exception type is a template parameter rather than a macro argument
//      spelled at each site, and CALIB_ASSERT_* default to
//      slamcpp::calib::AssertException.
//   3. std::fabs is used explicitly instead of relying on an unqualified fabs
//      that upstream picks up from a transitively included <cmath>.
#ifndef CALIB_CORE_ASSERT_HPP
#define CALIB_CORE_ASSERT_HPP

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include "slamcpp/calibration/core/SourceFilePos.hpp"

#define CALIB_DEFINE_EXCEPTION(ExceptionName, ExceptionParent)      \
  class ExceptionName : public ExceptionParent {                     \
   public:                                                           \
    explicit ExceptionName(const char* message)                      \
        : ExceptionParent(message) {}                                \
    explicit ExceptionName(const std::string& message)               \
        : ExceptionParent(message) {}                                \
    ~ExceptionName() throw() override = default;                     \
  }

namespace slamcpp {
namespace calib {

CALIB_DEFINE_EXCEPTION(AssertException, std::runtime_error);

namespace detail {

template <typename ExceptionT>
[[noreturn]] inline void throwException(const std::string& exceptionType,
                                        const SourceFilePos& sfp,
                                        const std::string& message) {
  std::stringstream ss;
  ss << exceptionType << sfp << " " << message;
  throw ExceptionT(ss.str());
}

template <typename ExceptionT>
[[noreturn]] inline void throwException(const std::string& exceptionType,
                                        const std::string& function,
                                        const std::string& file, int line,
                                        const std::string& message) {
  throwException<ExceptionT>(exceptionType, SourceFilePos(function, file, line),
                             message);
}

}  // namespace detail

template <typename ExceptionT>
inline void assertThrow(bool condition, const std::string& message,
                        const SourceFilePos& sfp) {
  if (!condition) {
    detail::throwException<ExceptionT>("", sfp, message);
  }
}

}  // namespace calib
}  // namespace slamcpp

// ---------------------------------------------------------------------------
// Internal plumbing. CALIB_ASSERT_IMPL is the single point where a failing
// condition turns into an exception, so the message layout lives in one place.
// ---------------------------------------------------------------------------
#define CALIB_ASSERT_IMPL(ExceptionType, failed, description, message) \
  do {                                                                  \
    if (failed) {                                                       \
      std::stringstream calib_assert_ss;                               \
      calib_assert_ss << description << ": " << message;               \
      ::slamcpp::calib::detail::throwException<ExceptionType>(                  \
          "[" #ExceptionType "] ", __FUNCTION__, __FILE__, __LINE__,    \
          calib_assert_ss.str());                                      \
    }                                                                   \
  } while (0)

#define CALIB_THROW_T(ExceptionType, message)                       \
  do {                                                               \
    std::stringstream calib_assert_ss;                              \
    calib_assert_ss << message;                                     \
    ::slamcpp::calib::detail::throwException<ExceptionType>(                 \
        "[" #ExceptionType "] ", __FUNCTION__, __FILE__, __LINE__,   \
        calib_assert_ss.str());                                     \
  } while (0)

#define CALIB_THROW_SFP(ExceptionType, sfp, message)                        \
  do {                                                                       \
    std::stringstream calib_assert_ss;                                      \
    calib_assert_ss << message;                                             \
    ::slamcpp::calib::detail::throwException<ExceptionType>("[" #ExceptionType "] ", \
                                                    sfp,                     \
                                                    calib_assert_ss.str()); \
  } while (0)

#define CALIB_ASSERT_TRUE_T(ExceptionType, condition, message)  \
  CALIB_ASSERT_IMPL(ExceptionType, !(condition),                \
                     "assert(" #condition ") failed", message)

#define CALIB_ASSERT_FALSE_T(ExceptionType, condition, message)      \
  CALIB_ASSERT_IMPL(ExceptionType, (condition),                      \
                     "assert( not " #condition ") failed", message)

#define CALIB_ASSERT_BINARY_T(ExceptionType, a, op, b, message)               \
  CALIB_ASSERT_IMPL(ExceptionType, !((a)op(b)),                               \
                     "assert(" #a " " #op " " #b ") failed [" << (a) << " "    \
                                                              #op " " << (b)   \
                                                              << "]",          \
                     message)

#define CALIB_ASSERT_LT_T(E, v, ub, m) CALIB_ASSERT_BINARY_T(E, v, <, ub, m)
#define CALIB_ASSERT_LE_T(E, v, ub, m) CALIB_ASSERT_BINARY_T(E, v, <=, ub, m)
#define CALIB_ASSERT_GT_T(E, v, lb, m) CALIB_ASSERT_BINARY_T(E, v, >, lb, m)
#define CALIB_ASSERT_GE_T(E, v, lb, m) CALIB_ASSERT_BINARY_T(E, v, >=, lb, m)
#define CALIB_ASSERT_EQ_T(E, v, t, m) CALIB_ASSERT_BINARY_T(E, v, ==, t, m)
#define CALIB_ASSERT_NE_T(E, v, t, m) CALIB_ASSERT_BINARY_T(E, v, !=, t, m)

#define CALIB_ASSERT_GE_LT_T(ExceptionType, value, lowerBound, upperBound,   \
                              message)                                        \
  CALIB_ASSERT_IMPL(                                                         \
      ExceptionType, ((value) < (lowerBound) || (value) >= (upperBound)),     \
      "assert(" #lowerBound " <= " #value " < " #upperBound ") failed ["      \
          << (lowerBound) << " <= " << (value) << " < " << (upperBound)       \
          << "]",                                                             \
      message)

#define CALIB_ASSERT_NEAR_T(ExceptionType, value, testValue, absError,       \
                             message)                                          \
  CALIB_ASSERT_IMPL(                                                          \
      ExceptionType,                                                           \
      !(std::fabs((testValue) - (value)) <= std::fabs(absError)),              \
      "assert(" #value " == " #testValue ") failed [" << (value)               \
          << " == " << (testValue) << " ("                                     \
          << std::fabs((testValue) - (value)) << " > "                         \
          << std::fabs(absError) << ")]",                                      \
      message)

// ---------------------------------------------------------------------------
// Default-exception forms. These are what ported code should use.
// ---------------------------------------------------------------------------
#define CALIB_THROW(message) \
  CALIB_THROW_T(::slamcpp::calib::AssertException, message)
#define CALIB_ASSERT_TRUE(c, m) \
  CALIB_ASSERT_TRUE_T(::slamcpp::calib::AssertException, c, m)
#define CALIB_ASSERT_FALSE(c, m) \
  CALIB_ASSERT_FALSE_T(::slamcpp::calib::AssertException, c, m)
#define CALIB_ASSERT_LT(v, ub, m) \
  CALIB_ASSERT_LT_T(::slamcpp::calib::AssertException, v, ub, m)
#define CALIB_ASSERT_LE(v, ub, m) \
  CALIB_ASSERT_LE_T(::slamcpp::calib::AssertException, v, ub, m)
#define CALIB_ASSERT_GT(v, lb, m) \
  CALIB_ASSERT_GT_T(::slamcpp::calib::AssertException, v, lb, m)
#define CALIB_ASSERT_GE(v, lb, m) \
  CALIB_ASSERT_GE_T(::slamcpp::calib::AssertException, v, lb, m)
#define CALIB_ASSERT_EQ(v, t, m) \
  CALIB_ASSERT_EQ_T(::slamcpp::calib::AssertException, v, t, m)
#define CALIB_ASSERT_NE(v, t, m) \
  CALIB_ASSERT_NE_T(::slamcpp::calib::AssertException, v, t, m)
#define CALIB_ASSERT_GE_LT(v, lb, ub, m) \
  CALIB_ASSERT_GE_LT_T(::slamcpp::calib::AssertException, v, lb, ub, m)
#define CALIB_ASSERT_NEAR(v, t, e, m) \
  CALIB_ASSERT_NEAR_T(::slamcpp::calib::AssertException, v, t, e, m)

// ---------------------------------------------------------------------------
// Debug forms, compiled out under NDEBUG exactly as upstream does.
// ---------------------------------------------------------------------------
#ifndef NDEBUG
#define CALIB_THROW_DBG(m) CALIB_THROW(m)
#define CALIB_ASSERT_TRUE_DBG(c, m) CALIB_ASSERT_TRUE(c, m)
#define CALIB_ASSERT_FALSE_DBG(c, m) CALIB_ASSERT_FALSE(c, m)
#define CALIB_ASSERT_LT_DBG(v, ub, m) CALIB_ASSERT_LT(v, ub, m)
#define CALIB_ASSERT_LE_DBG(v, ub, m) CALIB_ASSERT_LE(v, ub, m)
#define CALIB_ASSERT_GT_DBG(v, lb, m) CALIB_ASSERT_GT(v, lb, m)
#define CALIB_ASSERT_GE_DBG(v, lb, m) CALIB_ASSERT_GE(v, lb, m)
#define CALIB_ASSERT_EQ_DBG(v, t, m) CALIB_ASSERT_EQ(v, t, m)
#define CALIB_ASSERT_NE_DBG(v, t, m) CALIB_ASSERT_NE(v, t, m)
#define CALIB_ASSERT_GE_LT_DBG(v, lb, ub, m) CALIB_ASSERT_GE_LT(v, lb, ub, m)
#define CALIB_ASSERT_NEAR_DBG(v, t, e, m) CALIB_ASSERT_NEAR(v, t, e, m)
#else
#define CALIB_THROW_DBG(m) static_cast<void>(0)
#define CALIB_ASSERT_TRUE_DBG(c, m) static_cast<void>(0)
#define CALIB_ASSERT_FALSE_DBG(c, m) static_cast<void>(0)
#define CALIB_ASSERT_LT_DBG(v, ub, m) static_cast<void>(0)
#define CALIB_ASSERT_LE_DBG(v, ub, m) static_cast<void>(0)
#define CALIB_ASSERT_GT_DBG(v, lb, m) static_cast<void>(0)
#define CALIB_ASSERT_GE_DBG(v, lb, m) static_cast<void>(0)
#define CALIB_ASSERT_EQ_DBG(v, t, m) static_cast<void>(0)
#define CALIB_ASSERT_NE_DBG(v, t, m) static_cast<void>(0)
#define CALIB_ASSERT_GE_LT_DBG(v, lb, ub, m) static_cast<void>(0)
#define CALIB_ASSERT_NEAR_DBG(v, t, e, m) static_cast<void>(0)
#endif

#endif  // CALIB_CORE_ASSERT_HPP
