// ---------------------------------------------------------------------------
// slamcpp calibration - the umbrella header for the calibration module.
//
// This is a port of ethz-asl/kalibr @ 1f60227 out of its ROS/catkin workspace
// and into slamcpp, where it is a module rather than a separate library. A
// consumer that links slamcpp gets calibration and SLAM from one target: you
// calibrate a rig and then track with it without leaving the library or
// converting a configuration file between two conventions.
//
// See docs/kalibr/ at the repository root for the subsystem analysis, the
// phased plan this follows, and the parameter specification.
//
// Layers, each depending only on those below it:
//
//   core/         asserts, logging, time, random, statistics, the image type
//   kinematics/   rotations, quaternions, transformations              (Phase 1)
//   linalg/       sparse block matrices and the linear solver interface (Phase 1)
//   backend/      design variables, error terms, the optimiser          (Phase 1)
//   expressions/  the autodiff expression graph                         (Phase 1)
//   splines/      continuous-time B-spline trajectories                 (Phase 2)
//   cameras/      projection, distortion and shutter models             (Phase 2)
//   targets/      checkerboard, circle grid and AprilGrid detectors     (Phase 2)
//   apriltag/     the AprilTag detector                                 (Phase 2)
//   imu/          accelerometer and gyroscope error terms               (Phase 2)
//   io/           YAML configuration and JSON reports
//   pipelines/    the multi-camera and camera-IMU calibrators           (Phase 5)
//
// Everything lives in namespace slamcpp::calib. The module deliberately does
// not use the ORB_SLAM3 namespace the inherited SLAM core still carries: it is
// new code and does not need to keep that name.
//
// Dependencies: Eigen only, for everything up to and including cameras/.
// OpenCV is needed by the checkerboard and circle-grid detectors, and slamcpp
// links it already, so it is not a separate option here as it is in the
// standalone plan.
// ---------------------------------------------------------------------------
#ifndef SLAMCPP_CALIBRATION_H
#define SLAMCPP_CALIBRATION_H

#include "slamcpp/calibration/core/Assert.hpp"
#include "slamcpp/calibration/core/Image.hpp"
#include "slamcpp/calibration/core/Logging.hpp"
#include "slamcpp/calibration/core/NumericalDiff.hpp"
#include "slamcpp/calibration/core/Random.hpp"
#include "slamcpp/calibration/core/Statistics.hpp"
#include "slamcpp/calibration/core/Timestamp.hpp"
#include "slamcpp/calibration/io/Json.hpp"
#include "slamcpp/calibration/io/Yaml.hpp"

#endif  // SLAMCPP_CALIBRATION_H
