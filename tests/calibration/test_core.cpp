#include <cmath>
#include <sstream>
#include <vector>

#include "TestHarness.hpp"
#include "slamcpp/calibration/core/Assert.hpp"
#include "slamcpp/calibration/core/Image.hpp"
#include "slamcpp/calibration/core/Logging.hpp"
#include "slamcpp/calibration/core/NumericalDiff.hpp"
#include "slamcpp/calibration/core/Random.hpp"
#include "slamcpp/calibration/core/Statistics.hpp"
#include "slamcpp/calibration/core/Timestamp.hpp"

// ---------------------------------------------------------------------------
// Assert
// ---------------------------------------------------------------------------

CALIB_TEST(Assert, PassingAssertionsDoNotThrow) {
  CALIB_ASSERT_TRUE(1 + 1 == 2, "arithmetic");
  CALIB_ASSERT_FALSE(1 + 1 == 3, "arithmetic");
  CALIB_ASSERT_LT(1, 2, "");
  CALIB_ASSERT_LE(2, 2, "");
  CALIB_ASSERT_GT(3, 2, "");
  CALIB_ASSERT_GE(2, 2, "");
  CALIB_ASSERT_EQ(2, 2, "");
  CALIB_ASSERT_NE(2, 3, "");
  CALIB_ASSERT_GE_LT(1, 0, 2, "");
  CALIB_ASSERT_NEAR(1.0, 1.0 + 1e-9, 1e-6, "");
}

CALIB_TEST(Assert, FailingAssertionsThrow) {
  EXPECT_THROWS(CALIB_ASSERT_TRUE(false, "boom"));
  EXPECT_THROWS(CALIB_ASSERT_LT(2, 1, ""));
  EXPECT_THROWS(CALIB_ASSERT_GE_LT(5, 0, 2, ""));
  EXPECT_THROWS(CALIB_ASSERT_NEAR(1.0, 2.0, 1e-6, ""));
  EXPECT_THROWS(CALIB_THROW("explicit"));
}

CALIB_TEST(Assert, MessageCarriesLocationAndValues) {
  try {
    CALIB_ASSERT_LT(7, 3, "custom text");
    CALIB_FAIL("assertion did not throw");
  } catch (const slamcpp::calib::AssertException& e) {
    const std::string what = e.what();
    EXPECT_TRUE(what.find("custom text") != std::string::npos);
    EXPECT_TRUE(what.find("test_core.cpp") != std::string::npos);
    // The offending values are interpolated, not just the expression text.
    EXPECT_TRUE(what.find("7") != std::string::npos);
    EXPECT_TRUE(what.find("3") != std::string::npos);
  }
}

// The upstream macros expand to a bare if, so a trailing else binds to the
// macro instead of the caller. This is the regression test for that fix.
CALIB_TEST(Assert, DoesNotCaptureATrailingElse) {
  bool tookElse = false;
  if (1 + 1 == 2)
    CALIB_ASSERT_TRUE(true, "");
  else
    tookElse = true;
  EXPECT_FALSE(tookElse);
}

// ---------------------------------------------------------------------------
// Timestamp
// ---------------------------------------------------------------------------

CALIB_TEST(Timestamp, RoundTripsThroughSeconds) {
  const slamcpp::calib::Timestamp t = slamcpp::calib::Timestamp::fromNanoseconds(1403636579763555584LL);
  // Absolute epoch times exceed double's integer resolution, so seconds()
  // round trips only to microseconds. Documented, not accidental.
  EXPECT_NEAR(slamcpp::calib::Timestamp::fromSeconds(t.seconds()).seconds(), t.seconds(), 1e-6);
}

CALIB_TEST(Timestamp, SecToNsecRoundsRatherThanTruncates) {
  // The truncating cast upstream uses turns 0.1 s into 99999999 ns on some
  // platforms. Rounding is what makes CSV timestamp parsing exact.
  EXPECT_EQ(slamcpp::calib::secToNsec(0.1), 100000000LL);
  EXPECT_EQ(slamcpp::calib::secToNsec(0.3), 300000000LL);
  EXPECT_EQ(slamcpp::calib::secToNsec(-0.1), -100000000LL);
}

CALIB_TEST(Timestamp, DifferenceIsADurationAndIsExact) {
  const slamcpp::calib::Timestamp a = slamcpp::calib::Timestamp::fromNanoseconds(1000000000LL);
  const slamcpp::calib::Timestamp b = slamcpp::calib::Timestamp::fromNanoseconds(1000000123LL);
  EXPECT_EQ(b - a, 123LL);
  EXPECT_TRUE(a < b);
  EXPECT_TRUE(b > a);
  EXPECT_TRUE(a <= a);
  EXPECT_TRUE(a != b);
}

CALIB_TEST(Timestamp, FormatsWithNineDigitNanoseconds) {
  EXPECT_EQ(slamcpp::calib::Timestamp::fromNanoseconds(3000000500LL).toString(),
            std::string("3.000000500"));
  // Negative times keep a non-negative nanosecond remainder.
  EXPECT_EQ(slamcpp::calib::Timestamp::fromNanoseconds(-500LL).toString(),
            std::string("-1.999999500"));
}

CALIB_TEST(Timestamp, ChronoRoundTrip) {
  const auto now = std::chrono::system_clock::now();
  const slamcpp::calib::Timestamp t = slamcpp::calib::Timestamp::fromChrono(now);
  EXPECT_TRUE(t.toChrono() == now);
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

CALIB_TEST(Statistics, MatchesClosedFormOnASmallSample) {
  slamcpp::calib::RunningStatistics s;
  const std::vector<double> xs{2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0};
  for (double x : xs) s.add(x);

  EXPECT_EQ(s.count(), std::size_t(8));
  EXPECT_NEAR(s.mean(), 5.0, 1e-12);
  // Sample variance with the n-1 denominator: 32/7.
  EXPECT_NEAR(s.variance(), 32.0 / 7.0, 1e-12);
  EXPECT_NEAR(s.standardDeviation(), std::sqrt(32.0 / 7.0), 1e-12);
  EXPECT_NEAR(s.min(), 2.0, 1e-12);
  EXPECT_NEAR(s.max(), 9.0, 1e-12);
  EXPECT_NEAR(s.sum(), 40.0, 1e-12);
  EXPECT_NEAR(s.rms(), std::sqrt(232.0 / 8.0), 1e-12);
}

CALIB_TEST(Statistics, DegenerateCountsDoNotDivideByZero) {
  slamcpp::calib::RunningStatistics s;
  EXPECT_NEAR(s.variance(), 0.0, 0.0);
  EXPECT_NEAR(s.mean(), 0.0, 0.0);
  EXPECT_NEAR(s.rms(), 0.0, 0.0);
  s.add(3.0);
  EXPECT_NEAR(s.variance(), 0.0, 0.0);
  EXPECT_NEAR(s.mean(), 3.0, 1e-12);
}

// This is why Welford's method is used rather than accumulating sums of
// squares: a large offset destroys the naive form's precision entirely.
CALIB_TEST(Statistics, VarianceSurvivesALargeOffset) {
  slamcpp::calib::RunningStatistics s;
  const double offset = 1e9;
  for (double x : {1.0, 2.0, 3.0, 4.0, 5.0}) s.add(offset + x);
  EXPECT_NEAR(s.mean(), offset + 3.0, 1e-6);
  EXPECT_NEAR(s.variance(), 2.5, 1e-6);
}

CALIB_TEST(Statistics, ClearResets) {
  slamcpp::calib::RunningStatistics s;
  s.add(1.0);
  s.add(2.0);
  s.clear();
  EXPECT_EQ(s.count(), std::size_t(0));
  EXPECT_NEAR(s.mean(), 0.0, 0.0);
}

// ---------------------------------------------------------------------------
// Random
// ---------------------------------------------------------------------------

CALIB_TEST(Random, SeedingIsReproducible) {
  slamcpp::calib::random::seed(1234);
  const double a1 = slamcpp::calib::random::normal();
  const double a2 = slamcpp::calib::random::uniform();
  slamcpp::calib::random::seed(1234);
  EXPECT_NEAR(slamcpp::calib::random::normal(), a1, 0.0);
  EXPECT_NEAR(slamcpp::calib::random::uniform(), a2, 0.0);
}

CALIB_TEST(Random, NormalHasTheRightMomentsInBulk) {
  slamcpp::calib::random::seed(7);
  slamcpp::calib::RunningStatistics s;
  for (int i = 0; i < 200000; ++i) s.add(slamcpp::calib::random::normal());
  EXPECT_NEAR(s.mean(), 0.0, 0.02);
  EXPECT_NEAR(s.standardDeviation(), 1.0, 0.02);
}

CALIB_TEST(Random, UniformRespectsItsHalfOpenRange) {
  slamcpp::calib::random::seed(9);
  for (int i = 0; i < 10000; ++i) {
    const double u = slamcpp::calib::random::uniform(-2.0, 5.0);
    EXPECT_TRUE(u >= -2.0 && u < 5.0);
    const int k = slamcpp::calib::random::uniformInt(3, 7);
    EXPECT_TRUE(k >= 3 && k < 7);
  }
}

// uniform_int_distribution is closed on both ends, so the upper bound must be
// decremented. This checks the top of the range is actually reachable and the
// one past it is not.
CALIB_TEST(Random, UniformIntCoversExactlyItsRange) {
  slamcpp::calib::random::seed(11);
  bool sawLow = false, sawHigh = false;
  for (int i = 0; i < 2000; ++i) {
    const int k = slamcpp::calib::random::uniformInt(0, 3);
    EXPECT_TRUE(k >= 0 && k <= 2);
    if (k == 0) sawLow = true;
    if (k == 2) sawHigh = true;
  }
  EXPECT_TRUE(sawLow);
  EXPECT_TRUE(sawHigh);
}

CALIB_TEST(Random, EmptyRangesAreRejected) {
  EXPECT_THROWS(slamcpp::calib::random::uniform(1.0, 1.0));
  EXPECT_THROWS(slamcpp::calib::random::uniformInt(4, 2));
}

CALIB_TEST(Random, FillsEigenExpressions) {
  slamcpp::calib::random::seed(3);
  Eigen::MatrixXd m(4, 3);
  slamcpp::calib::random::fillNormal(m);
  EXPECT_TRUE(m.allFinite());
  EXPECT_TRUE(m.cwiseAbs().maxCoeff() > 0.0);
}

// ---------------------------------------------------------------------------
// NumericalDiff
// ---------------------------------------------------------------------------

CALIB_TEST(NumericalDiff, MatchesAnAnalyticJacobian) {
  // f(x) = [x0^2 * x1, sin(x0) + x2^3, x1 * x2]
  auto f = [](const Eigen::VectorXd& x) {
    Eigen::VectorXd y(3);
    y[0] = x[0] * x[0] * x[1];
    y[1] = std::sin(x[0]) + x[2] * x[2] * x[2];
    y[2] = x[1] * x[2];
    return y;
  };

  Eigen::VectorXd x(3);
  x << 0.7, -1.3, 2.1;

  Eigen::MatrixXd J(3, 3);
  J << 2.0 * x[0] * x[1], x[0] * x[0], 0.0,
       std::cos(x[0]), 0.0, 3.0 * x[2] * x[2],
       0.0, x[2], x[1];

  EXPECT_JACOBIAN_NEAR(J, f, x, 1e-6);
}

CALIB_TEST(NumericalDiff, DetectsAWrongJacobian) {
  auto f = [](const Eigen::VectorXd& x) {
    Eigen::VectorXd y(1);
    y[0] = x[0] * x[0];
    return y;
  };
  Eigen::VectorXd x(1);
  x << 3.0;

  Eigen::MatrixXd wrong(1, 1);
  wrong << 5.0;  // correct is 6.0
  const Eigen::MatrixXd numeric = slamcpp::calib::numericalJacobian(f, x);
  EXPECT_TRUE(slamcpp::calib::relativeDifference(wrong, numeric) > 1e-6);
}

CALIB_TEST(NumericalDiff, ScalesTheStepWithMagnitude) {
  // An absolute 1e-6 step vanishes into rounding at this magnitude; the
  // proportional step is what keeps the result accurate.
  auto f = [](const Eigen::VectorXd& x) {
    Eigen::VectorXd y(1);
    y[0] = x[0] * x[0];
    return y;
  };
  Eigen::VectorXd x(1);
  x << 1e7;
  Eigen::MatrixXd J(1, 1);
  J << 2e7;
  EXPECT_JACOBIAN_NEAR(J, f, x, 1e-6);
}

// ---------------------------------------------------------------------------
// Image
// ---------------------------------------------------------------------------

CALIB_TEST(Image, ReadsThroughAStride) {
  // A 2x2 image embedded in a 4-wide buffer, as a cv::Mat ROI would be.
  const std::uint8_t buf[8] = {10, 20, 99, 99, 30, 40, 99, 99};
  const slamcpp::calib::Image img(buf, 2, 2, 4);
  EXPECT_EQ(int(img.at(0, 0)), 10);
  EXPECT_EQ(int(img.at(1, 0)), 20);
  EXPECT_EQ(int(img.at(0, 1)), 30);
  EXPECT_EQ(int(img.at(1, 1)), 40);
  EXPECT_FALSE(img.empty());
}

CALIB_TEST(Image, RejectsAStrideNarrowerThanTheImage) {
  const std::uint8_t buf[4] = {1, 2, 3, 4};
  EXPECT_THROWS(slamcpp::calib::Image(buf, 4, 1, 2));
}

CALIB_TEST(Image, BilinearSamplingIsExactAtPixelCentresAndMidpoints) {
  const std::uint8_t buf[4] = {0, 100, 200, 100};
  const slamcpp::calib::Image img(buf, 2, 2);
  EXPECT_NEAR(img.sampleBilinear(0.0, 0.0), 0.0, 1e-12);
  EXPECT_NEAR(img.sampleBilinear(1.0, 0.0), 100.0, 1e-12);
  EXPECT_NEAR(img.sampleBilinear(0.5, 0.0), 50.0, 1e-12);
  EXPECT_NEAR(img.sampleBilinear(0.0, 0.5), 100.0, 1e-12);
  // Centre of all four: (0 + 100 + 200 + 100) / 4.
  EXPECT_NEAR(img.sampleBilinear(0.5, 0.5), 100.0, 1e-12);
}

CALIB_TEST(Image, SamplingClampsAtTheBorder) {
  const std::uint8_t buf[4] = {0, 100, 200, 100};
  const slamcpp::calib::Image img(buf, 2, 2);
  EXPECT_NEAR(img.sampleBilinear(-5.0, 0.0), 0.0, 1e-12);
  EXPECT_NEAR(img.sampleBilinear(50.0, 0.0), 100.0, 1e-12);
  EXPECT_FALSE(img.contains(-0.1, 0.0));
  EXPECT_TRUE(img.contains(1.0, 1.0));
  EXPECT_FALSE(img.contains(1.01, 1.0));
}

CALIB_TEST(Image, BufferDensifiesAStridedSource) {
  const std::uint8_t buf[8] = {10, 20, 99, 99, 30, 40, 99, 99};
  const slamcpp::calib::Image src(buf, 2, 2, 4);
  slamcpp::calib::ImageBuffer dst;
  dst.assign(src);
  EXPECT_EQ(dst.width(), 2);
  EXPECT_EQ(dst.height(), 2);
  EXPECT_EQ(dst.view().stride(), 2);
  EXPECT_EQ(int(dst.at(1, 1)), 40);
}

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

CALIB_TEST(Logging, SinkReceivesLevelAndMessage) {
  std::vector<slamcpp::calib::logging::Event> seen;
  slamcpp::calib::logging::setSink([&](const slamcpp::calib::logging::Event& e) { seen.push_back(e); });
  slamcpp::calib::logging::setLevel(slamcpp::calib::logging::Level::All);

  CALIB_INFO_STREAM("hello " << 42);
  CALIB_WARN_STREAM_NAMED("solver", "careful");

  EXPECT_EQ(seen.size(), std::size_t(2));
  EXPECT_EQ(seen[0].message, std::string("hello 42"));
  EXPECT_TRUE(seen[0].level == slamcpp::calib::logging::Level::Info);
  EXPECT_EQ(std::string(seen[1].stream), std::string("solver"));
  EXPECT_TRUE(seen[1].level == slamcpp::calib::logging::Level::Warn);

  slamcpp::calib::logging::setSink(nullptr);
  slamcpp::calib::logging::setLevel(slamcpp::calib::logging::Level::Info);
}

CALIB_TEST(Logging, LevelSuppressesAndDoesNotEvaluateArguments) {
  int evaluations = 0;
  auto counted = [&]() { ++evaluations; return 1; };

  std::vector<slamcpp::calib::logging::Event> seen;
  slamcpp::calib::logging::setSink([&](const slamcpp::calib::logging::Event& e) { seen.push_back(e); });
  slamcpp::calib::logging::setLevel(slamcpp::calib::logging::Level::Error);

  CALIB_INFO_STREAM("suppressed " << counted());
  EXPECT_EQ(seen.size(), std::size_t(0));
  EXPECT_EQ(evaluations, 0);

  CALIB_ERROR_STREAM("kept " << counted());
  EXPECT_EQ(seen.size(), std::size_t(1));
  EXPECT_EQ(evaluations, 1);

  slamcpp::calib::logging::setSink(nullptr);
  slamcpp::calib::logging::setLevel(slamcpp::calib::logging::Level::Info);
}

CALIB_TEST(Logging, OnceAndCondVariants) {
  std::vector<slamcpp::calib::logging::Event> seen;
  slamcpp::calib::logging::setSink([&](const slamcpp::calib::logging::Event& e) { seen.push_back(e); });
  slamcpp::calib::logging::setLevel(slamcpp::calib::logging::Level::All);

  for (int i = 0; i < 5; ++i) CALIB_INFO_STREAM_ONCE("only once");
  EXPECT_EQ(seen.size(), std::size_t(1));

  seen.clear();
  for (int i = 0; i < 5; ++i) CALIB_INFO_STREAM_COND(i % 2 == 0, "even " << i);
  EXPECT_EQ(seen.size(), std::size_t(3));

  slamcpp::calib::logging::setSink(nullptr);
  slamcpp::calib::logging::setLevel(slamcpp::calib::logging::Level::Info);
}
