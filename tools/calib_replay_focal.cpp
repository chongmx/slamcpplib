// Replay upstream's focal length seed through the ported code.
//
// Reads the detections captured from upstream Kalibr in Docker (see
// tools/kalibr_trace/ at the repository root), runs the ported
// initializePinholeFocalLength over them, and compares against the value
// upstream produced from the identical input.
//
// This is the first 1:1 check of the port against the original. A difference
// here is a porting bug by definition: the inputs are upstream's own recorded
// detections, so nothing upstream of this stage can be responsible.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "slamcpp/calibration/cameras/PinholeInitializer.hpp"

namespace {

std::vector<slamcpp::calib::GridObservation> loadFixture(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    std::cerr << "cannot open " << path << "\n";
    std::exit(2);
  }
  std::vector<slamcpp::calib::GridObservation> out;
  std::string tag;
  while (in >> tag) {
    if (tag != "obs") {
      std::cerr << "malformed fixture near '" << tag << "'\n";
      std::exit(2);
    }
    int idx, rows, cols, count;
    in >> idx >> rows >> cols >> count;

    slamcpp::calib::GridObservation obs;
    obs.rows = rows;
    obs.cols = cols;
    obs.valid.assign(static_cast<std::size_t>(rows * cols), false);
    obs.points.assign(static_cast<std::size_t>(rows * cols),
                      Eigen::Vector2d::Zero());
    for (int i = 0; i < count; ++i) {
      int pid;
      double x, y;
      in >> pid >> x >> y;
      obs.valid[static_cast<std::size_t>(pid)] = true;
      obs.points[static_cast<std::size_t>(pid)] = Eigen::Vector2d(x, y);
    }
    out.push_back(std::move(obs));
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: calib_replay_focal <detections.txt> <upstream_f> "
                 "[width] [height]\n";
    return 2;
  }
  const std::string fixture = argv[1];
  const double upstreamF = std::strtod(argv[2], nullptr);
  const int width = argc > 3 ? std::atoi(argv[3]) : 752;
  const int height = argc > 4 ? std::atoi(argv[4]) : 480;

  const auto observations = loadFixture(fixture);
  std::cout << "loaded " << observations.size() << " observations\n";

  const auto seed =
      slamcpp::calib::initializePinholeFocalLength(observations, width, height);

  std::cout << std::setprecision(17);
  std::cout << "ported   f  = " << seed.f << "\n";
  std::cout << "upstream f  = " << upstreamF << "\n";
  std::cout << "cu, cv      = " << seed.cu << ", " << seed.cv << "\n";
  std::cout << "images used = " << seed.imagesUsed << "\n";
  std::cout << "guesses     = " << seed.guesses.size() << "\n";

  const double diff = seed.f - upstreamF;
  std::cout << "difference  = " << diff << "\n";
  if (diff == 0.0) {
    std::cout << "RESULT: EXACT MATCH (bit-identical)\n";
    return 0;
  }
  std::cout << "RESULT: MISMATCH\n";
  return 1;
}
