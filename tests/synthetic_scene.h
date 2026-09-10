// A deterministic image sequence, shared by the tests that need one.
//
// Synthetic on purpose: the tests then need no camera and no recorded dataset,
// and they fail for the reason under test rather than because a scene was
// featureless that day. A textured plane viewed through a moving virtual
// camera gives real inter-frame motion, which is what ORB_SLAM3's
// homography-based monocular initialiser needs.

#ifndef SLAMCPP_TESTS_SYNTHETIC_SCENE_H
#define SLAMCPP_TESTS_SYNTHETIC_SCENE_H

#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace slamcpp_test
{

constexpr int kWidth = 640;
constexpr int kHeight = 480;

// Structure at several scales, so every level of the pyramid has corners to
// find rather than only the finest one.
inline cv::Mat makeTexture()
{
    cv::RNG rng(20240910);
    cv::Mat base(kHeight * 2, kWidth * 2, CV_8UC1);
    rng.fill(base, cv::RNG::UNIFORM, 0, 255);

    cv::Mat img;
    cv::GaussianBlur(base, img, cv::Size(5, 5), 0);

    for (int i = 0; i < 400; ++i)
    {
        const cv::Point c(rng.uniform(0, img.cols), rng.uniform(0, img.rows));
        cv::circle(img, c, rng.uniform(4, 18), cv::Scalar(rng.uniform(0, 255)), -1);
    }
    for (int i = 0; i < 120; ++i)
    {
        const cv::Point a(rng.uniform(0, img.cols), rng.uniform(0, img.rows));
        const cv::Point b(rng.uniform(0, img.cols), rng.uniform(0, img.rows));
        cv::line(img, a, b, cv::Scalar(rng.uniform(0, 255)), rng.uniform(1, 3));
    }
    return img;
}

// The view at frame i of totalFrames: a sideways sweep with a little rotation
// and scale, which is what produces the parallax the initialiser looks for.
inline cv::Mat renderView(const cv::Mat& texture, int i, int totalFrames)
{
    const double t = static_cast<double>(i) / totalFrames;
    const double dx = 150.0 * t;
    const double dy = 26.0 * std::sin(t * 3.0);
    const double angle = 5.0 * std::sin(t * 2.0);
    const double scale = 1.0 + 0.16 * t;

    const cv::Point2f centre(texture.cols * 0.5f + static_cast<float>(dx),
                             texture.rows * 0.5f + static_cast<float>(dy));
    cv::Mat rot = cv::getRotationMatrix2D(centre, angle, scale);
    rot.at<double>(0, 2) += kWidth * 0.5 - centre.x;
    rot.at<double>(1, 2) += kHeight * 0.5 - centre.y;

    cv::Mat view;
    cv::warpAffine(texture, view, rot, cv::Size(kWidth, kHeight), cv::INTER_LINEAR,
                   cv::BORDER_REFLECT);
    return view;
}

}  // namespace slamcpp_test

#endif  // SLAMCPP_TESTS_SYNTHETIC_SCENE_H
