#include "slamcpp/calibration/cameras/PinholeInitializer.hpp"

#include <cmath>

#include "slamcpp/calibration/cameras/PinholeHelpers.hpp"
#include "slamcpp/calibration/core/Assert.hpp"

namespace slamcpp {
namespace calib {

FocalLengthSeed initializePinholeFocalLength(
    const std::vector<GridObservation>& observations, int imageWidth,
    int imageHeight) {
  CALIB_ASSERT_FALSE(observations.empty(), "Need min. one observation");

  FocalLengthSeed seed;
  seed.cu = (imageWidth - 1.0) / 2.0;
  seed.cv = (imageHeight - 1.0) / 2.0;

  std::vector<double> f_guesses;

  for (const GridObservation& obs : observations) {
    const int rows = obs.rows;
    const int cols = obs.cols;

    std::vector<Eigen::Vector2d> center(static_cast<std::size_t>(rows));
    std::vector<double> radius(static_cast<std::size_t>(rows));
    bool skipImage = false;

    for (int r = 0; r < rows; ++r) {
      std::vector<Eigen::Vector2d> circle;
      for (int c = 0; c < cols; ++c) {
        const std::size_t i = obs.index(r, c);
        if (obs.valid[i]) {
          // QUIRK 1, reproduced deliberately. Upstream writes
          //   circle.push_back(cv::Point2f(imagePoint[0], imagePoint[1]));
          // into a std::vector<cv::Point2d>. The corner arrives as a double,
          // is narrowed to float by the Point2f constructor, then widened back
          // to double on insertion, so the circle fit sees about 7 significant
          // digits rather than 16.
          //
          // Measured: on AprilGrid data this is a no-op. Every one of the
          // 20,088 coordinates sampled from an upstream trace was already
          // exactly float-representable, because the AprilTag detector works in
          // float, and removing the cast reproduces upstream bit for bit just
          // the same. It is kept because it is what upstream does, and because
          // a detector that ever returns genuine double precision, a future
          // refinement step among them, would make it load-bearing without
          // warning.
          const Eigen::Vector2d& p = obs.points[i];
          circle.push_back(
              Eigen::Vector2d(static_cast<double>(static_cast<float>(p.x())),
                              static_cast<double>(static_cast<float>(p.y()))));
        } else {
          // skip this image if the board view is not complete
          skipImage = true;
        }
      }
      // Fitted before the skip is acted on, exactly as upstream does. For a
      // partial row this fits a circle to whatever was found, and the result
      // is then discarded with the image.
      pinhole_helpers::fitCircle(circle, center[static_cast<std::size_t>(r)](0),
                                 center[static_cast<std::size_t>(r)](1),
                                 radius[static_cast<std::size_t>(r)]);
    }

    if (skipImage) {
      continue;
    }
    ++seed.imagesUsed;

    for (int j = 0; j < rows; ++j) {
      // QUIRK 2, reproduced deliberately. The inner bound is cols(), not
      // rows(), while k indexes center[] and radius[], which are sized by
      // rows(). On a square grid the two are equal and nothing goes wrong; the
      // AprilGrid used here is 12 x 12. On a non-square target with more
      // columns than rows this reads past the end of both arrays. Preserved so
      // the square case matches; flagged as a genuine upstream bug.
      for (int k = j + 1; k < cols; ++k) {
        if (k >= rows) {
          // The out-of-bounds read upstream would perform. Refusing to do it
          // is the one place this port must deviate: reading past the array is
          // undefined, so there is nothing to reproduce faithfully.
          break;
        }
        const std::vector<Eigen::Vector2d> ipts = pinhole_helpers::intersectCircles(
            center[static_cast<std::size_t>(j)](0),
            center[static_cast<std::size_t>(j)](1),
            radius[static_cast<std::size_t>(j)],
            center[static_cast<std::size_t>(k)](0),
            center[static_cast<std::size_t>(k)](1),
            radius[static_cast<std::size_t>(k)]);
        if (ipts.size() < 2) {
          continue;
        }
        const double f_guess = (ipts.at(0) - ipts.at(1)).norm() / M_PI;
        if (std::isfinite(f_guess)) {
          f_guesses.push_back(f_guess);
        }
      }
    }
  }

  if (f_guesses.empty()) {
    seed.success = false;
    return seed;
  }

  seed.guesses = f_guesses;
  seed.f = pinhole_helpers::medianOfVectorElements(f_guesses);
  seed.success = true;
  return seed;
}

}  // namespace calib
}  // namespace slamcpp
