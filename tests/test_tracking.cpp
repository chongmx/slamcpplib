// End-to-end check that the merged library still tracks.
//
// The vocabulary test proves DBoW3 reads the same words DBoW2 did. This one
// proves the rest of the pipeline still runs on top of it: System comes up,
// the extractor and matcher agree, the bag-of-words path is exercised through
// Frame and KeyFrame, and monocular initialisation converges.
//
// The imagery is synthetic on purpose, so the test is deterministic and needs
// no camera. A random texture is viewed through a moving virtual camera, which
// gives the tracker a planar scene with real inter-frame motion. That is
// enough for ORB_SLAM3's homography-based monocular initialiser.
//
// Usage: test_tracking <ORBvoc.txt> <settings.yaml>

#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "slamcpp/System.h"

#include "synthetic_scene.h"

namespace
{

const int kFrames = 160;

const char* stateName(int s)
{
    switch (s)
    {
        case -1: return "SYSTEM_NOT_READY";
        case 0:  return "NO_IMAGES_YET";
        case 1:  return "NOT_INITIALIZED";
        case 2:  return "OK";
        case 3:  return "RECENTLY_LOST";
        case 4:  return "LOST";
        default: return "UNKNOWN";
    }
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::fprintf(stderr, "usage: %s <ORBvoc.txt> <settings.yaml>\n", argv[0]);
        return 2;
    }

    const cv::Mat texture = slamcpp_test::makeTexture();

    std::printf("bringing up System (this loads the vocabulary)\n");
    ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);

    int  bestState = -1;
    int  framesTracked = 0;
    int  maxKeypoints = 0;

    for (int i = 0; i < kFrames; ++i)
    {
        const cv::Mat view = slamcpp_test::renderView(texture, i, kFrames);
        slam.TrackMonocular(view, i / 30.0);

        const int state = static_cast<int>(slam.GetTrackingState());
        if (state > bestState)
            bestState = state;
        if (state == 2)
            ++framesTracked;

        const int n = static_cast<int>(slam.GetTrackedKeyPointsUn().size());
        if (n > maxKeypoints)
            maxKeypoints = n;
    }

    std::printf("\n  best state reached : %s\n", stateName(bestState));
    std::printf("  frames tracked OK  : %d of %d\n", framesTracked, kFrames);
    std::printf("  most keypoints seen: %d\n", maxKeypoints);

    slam.Shutdown();

    int failures = 0;
    const bool extracted = maxKeypoints > 100;
    std::printf("\n  [%s] the extractor finds features\n", extracted ? "PASS" : "FAIL");
    if (!extracted) ++failures;

    // Reaching OK means the bag-of-words path, the matcher and the initialiser
    // all did their jobs on the DBoW3 vocabulary.
    const bool initialised = bestState >= 2;
    std::printf("  [%s] monocular initialisation reaches OK\n", initialised ? "PASS" : "FAIL");
    if (!initialised) ++failures;

    std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "OK",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
