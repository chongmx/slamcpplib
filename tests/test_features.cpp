// A sweep across the library's feature surface.
//
// The other tests prove one thing each. This one walks the public API: every
// sensor mode, both camera models, localisation mode, the reset entry points
// and the trajectory writers, so that a port which merely compiles is told
// apart from one that still works.
//
// The imagery is synthetic but geometrically consistent. The scene is a
// fronto-parallel textured plane, so a stereo pair is the same image shifted by
// a constant disparity d = fx * b / Z, and the matching depth map is a constant
// Z. That is a real stereo geometry rather than two unrelated pictures, which
// is what lets stereo and RGB-D initialisation actually succeed.
//
// Usage: test_features <ORBvoc.txt>

#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "slamcpp/Atlas.h"
#include "slamcpp/ImuTypes.h"
#include "slamcpp/Map.h"
#include "slamcpp/System.h"

#include "synthetic_scene.h"

namespace
{

const int    kFrames = 140;
const double kFx = 554.26;
const double kBaseline = 0.10;   // metres
const double kDisparity = 20.0;  // pixels
const double kDepth = kFx * kBaseline / kDisparity;  // ~2.77 m

int failures = 0;
int checks = 0;

void check(bool ok, const std::string& what)
{
    ++checks;
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (!ok)
        ++failures;
}

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

std::string writeYaml(const std::string& path, const std::string& body)
{
    std::ofstream os(path);
    os << "%YAML:1.0\n" << body;
    return path;
}

const char* kOrbAndViewer =
    "Camera.fps: 30\n"
    "Camera.RGB: 1\n"
    "ORBextractor.nFeatures: 1250\n"
    "ORBextractor.scaleFactor: 1.2\n"
    "ORBextractor.nLevels: 8\n"
    "ORBextractor.iniThFAST: 20\n"
    "ORBextractor.minThFAST: 7\n"
    "Viewer.KeyFrameSize: 0.05\n"
    "Viewer.KeyFrameLineWidth: 1.0\n"
    "Viewer.GraphLineWidth: 0.9\n"
    "Viewer.PointSize: 2.0\n"
    "Viewer.CameraSize: 0.08\n"
    "Viewer.CameraLineWidth: 3.0\n"
    "Viewer.ViewpointX: 0.0\n"
    "Viewer.ViewpointY: -0.7\n"
    "Viewer.ViewpointZ: -1.8\n"
    "Viewer.ViewpointF: 500.0\n";

std::string pinholeIntrinsics()
{
    std::ostringstream os;
    os << "File.version: \"1.0\"\n"
       << "Camera.type: \"PinHole\"\n"
       << "Camera1.fx: " << kFx << "\n"
       << "Camera1.fy: " << kFx << "\n"
       << "Camera1.cx: 320.0\n"
          "Camera1.cy: 240.0\n"
          "Camera1.k1: 0.0\n"
          "Camera1.k2: 0.0\n"
          "Camera1.p1: 0.0\n"
          "Camera1.p2: 0.0\n"
          "Camera.width: 640\n"
          "Camera.height: 480\n";
    return os.str();
}

// The right eye of a fronto-parallel plane: the same view shifted by the
// disparity that plane's depth implies.
cv::Mat rightOf(const cv::Mat& left)
{
    cv::Mat shift = (cv::Mat_<double>(2, 3) << 1, 0, -kDisparity, 0, 1, 0);
    cv::Mat right;
    cv::warpAffine(left, right, shift, left.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT);
    return right;
}

cv::Mat depthOf(const cv::Mat& left, double factor)
{
    return cv::Mat(left.size(), CV_16UC1,
                   cv::Scalar(static_cast<double>(kDepth * factor)));
}

struct Outcome
{
    int  bestState = -1;
    int  okFrames = 0;
    long keyFrames = 0;
    long mapPoints = 0;
    bool threw = false;
};

long countKeyFrames(ORB_SLAM3::Atlas* atlas)
{
    long n = 0;
    for (ORB_SLAM3::Map* m : atlas->GetAllMaps())
        for (ORB_SLAM3::KeyFrame* kf : m->GetAllKeyFrames())
            if (kf && !kf->isBad())
                ++n;
    return n;
}

long countMapPoints(ORB_SLAM3::Atlas* atlas)
{
    long n = 0;
    for (ORB_SLAM3::Map* m : atlas->GetAllMaps())
        for (ORB_SLAM3::MapPoint* mp : m->GetAllMapPoints())
            if (mp && !mp->isBad())
                ++n;
    return n;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::fprintf(stderr, "usage: %s <ORBvoc.txt>\n", argv[0]);
        return 2;
    }
    const std::string voc = argv[1];
    const cv::Mat     texture = slamcpp_test::makeTexture();

    // -----------------------------------------------------------------------
    std::printf("\nMONOCULAR / PinHole\n");
    {
        const std::string cfg = writeYaml("feat_mono.yaml", pinholeIntrinsics() + kOrbAndViewer);
        Outcome o;
        ORB_SLAM3::System slam(voc, cfg, ORB_SLAM3::System::MONOCULAR, false);
        for (int i = 0; i < kFrames; ++i)
        {
            slam.TrackMonocular(slamcpp_test::renderView(texture, i, kFrames), i / 30.0);
            const int st = slam.GetTrackingState();
            o.bestState = std::max(o.bestState, st);
            if (st == 2)
                ++o.okFrames;
        }
        o.keyFrames = countKeyFrames(slam.GetAtlas());
        o.mapPoints = countMapPoints(slam.GetAtlas());

        std::printf("       best=%s ok=%d kf=%ld mp=%ld scale=%.2f\n", stateName(o.bestState),
                    o.okFrames, o.keyFrames, o.mapPoints, slam.GetImageScale());
        check(o.bestState >= 2, "monocular initialises and tracks");
        check(o.keyFrames > 1, "monocular creates key frames");
        check(o.mapPoints > 100, "monocular triangulates map points");

        // Trajectory writers. TUM and KITTI refuse monocular by design.
        slam.SaveKeyFrameTrajectoryTUM("feat_kf_tum.txt");
        slam.SaveTrajectoryEuRoC("feat_traj_euroc.txt");
        slam.SaveKeyFrameTrajectoryEuRoC("feat_kf_euroc.txt");
        slam.Shutdown();

        auto nonEmpty = [](const char* f) {
            std::ifstream in(f, std::ios::ate);
            return in.good() && in.tellg() > 0;
        };
        check(nonEmpty("feat_kf_tum.txt"), "SaveKeyFrameTrajectoryTUM writes poses");
        check(nonEmpty("feat_traj_euroc.txt"), "SaveTrajectoryEuRoC writes poses");
        check(nonEmpty("feat_kf_euroc.txt"), "SaveKeyFrameTrajectoryEuRoC writes poses");
        std::remove("feat_kf_tum.txt");
        std::remove("feat_traj_euroc.txt");
        std::remove("feat_kf_euroc.txt");
        std::remove(cfg.c_str());
    }

    // -----------------------------------------------------------------------
    std::printf("\nSTEREO / Rectified\n");
    {
        std::ostringstream body;
        body << "File.version: \"1.0\"\n"
             << "Camera.type: \"Rectified\"\n"
             << "Camera1.fx: " << kFx << "\nCamera1.fy: " << kFx << "\n"
             << "Camera1.cx: 320.0\nCamera1.cy: 240.0\n"
             << "Camera.width: 640\nCamera.height: 480\n"
             << "Stereo.b: " << kBaseline << "\n"
             << "Stereo.ThDepth: 40.0\n"
             << kOrbAndViewer;
        const std::string cfg = writeYaml("feat_stereo.yaml", body.str());

        Outcome o;
        ORB_SLAM3::System slam(voc, cfg, ORB_SLAM3::System::STEREO, false);
        for (int i = 0; i < kFrames; ++i)
        {
            const cv::Mat left = slamcpp_test::renderView(texture, i, kFrames);
            slam.TrackStereo(left, rightOf(left), i / 30.0);
            const int st = slam.GetTrackingState();
            o.bestState = std::max(o.bestState, st);
            if (st == 2)
                ++o.okFrames;
        }
        o.keyFrames = countKeyFrames(slam.GetAtlas());
        o.mapPoints = countMapPoints(slam.GetAtlas());
        std::printf("       best=%s ok=%d kf=%ld mp=%ld\n", stateName(o.bestState), o.okFrames,
                    o.keyFrames, o.mapPoints);
        check(o.bestState >= 2, "stereo initialises and tracks");
        check(o.mapPoints > 100, "stereo triangulates map points");

        slam.SaveTrajectoryTUM("feat_stereo_tum.txt");
        slam.SaveTrajectoryKITTI("feat_stereo_kitti.txt");
        slam.Shutdown();
        auto nonEmpty = [](const char* f) {
            std::ifstream in(f, std::ios::ate);
            return in.good() && in.tellg() > 0;
        };
        check(nonEmpty("feat_stereo_tum.txt"), "SaveTrajectoryTUM writes poses for stereo");
        check(nonEmpty("feat_stereo_kitti.txt"), "SaveTrajectoryKITTI writes poses for stereo");
        std::remove("feat_stereo_tum.txt");
        std::remove("feat_stereo_kitti.txt");
        std::remove(cfg.c_str());
    }

    // -----------------------------------------------------------------------
    std::printf("\nRGBD / PinHole\n");
    {
        const double factor = 5000.0;
        std::ostringstream body;
        body << pinholeIntrinsics() << "Stereo.b: " << kBaseline << "\n"
             << "Stereo.ThDepth: 40.0\n"
             // Written with a decimal point on purpose: the reader insists this
             // one be a real, and a bare "5000" is parsed as an integer.
             << "RGBD.DepthMapFactor: " << std::fixed << std::setprecision(1) << factor
             << std::defaultfloat << "\n"
             << kOrbAndViewer;
        const std::string cfg = writeYaml("feat_rgbd.yaml", body.str());

        Outcome o;
        ORB_SLAM3::System slam(voc, cfg, ORB_SLAM3::System::RGBD, false);
        for (int i = 0; i < kFrames; ++i)
        {
            const cv::Mat im = slamcpp_test::renderView(texture, i, kFrames);
            slam.TrackRGBD(im, depthOf(im, factor), i / 30.0);
            const int st = slam.GetTrackingState();
            o.bestState = std::max(o.bestState, st);
            if (st == 2)
                ++o.okFrames;
        }
        o.keyFrames = countKeyFrames(slam.GetAtlas());
        o.mapPoints = countMapPoints(slam.GetAtlas());
        std::printf("       best=%s ok=%d kf=%ld mp=%ld\n", stateName(o.bestState), o.okFrames,
                    o.keyFrames, o.mapPoints);
        check(o.bestState >= 2, "RGB-D initialises and tracks");
        check(o.mapPoints > 100, "RGB-D triangulates map points");
        slam.Shutdown();
        std::remove(cfg.c_str());
    }

    // -----------------------------------------------------------------------
    std::printf("\nMONOCULAR / KannalaBrandt8 (fisheye model)\n");
    {
        std::ostringstream body;
        body << "File.version: \"1.0\"\n"
             << "Camera.type: \"KannalaBrandt8\"\n"
             << "Camera1.fx: 190.978477\nCamera1.fy: 190.973307\n"
             << "Camera1.cx: 320.0\nCamera1.cy: 240.0\n"
             << "Camera1.k1: 0.003482389402\nCamera1.k2: 0.000715034845\n"
             << "Camera1.k3: -0.002053236141\nCamera1.k4: 0.000202936736\n"
             << "Camera.width: 640\nCamera.height: 480\n"
             << kOrbAndViewer;
        const std::string cfg = writeYaml("feat_fisheye.yaml", body.str());

        Outcome o;
        try
        {
            ORB_SLAM3::System slam(voc, cfg, ORB_SLAM3::System::MONOCULAR, false);
            for (int i = 0; i < kFrames; ++i)
            {
                slam.TrackMonocular(slamcpp_test::renderView(texture, i, kFrames), i / 30.0);
                o.bestState = std::max(o.bestState, slam.GetTrackingState());
            }
            o.mapPoints = countMapPoints(slam.GetAtlas());
            slam.Shutdown();
        }
        catch (const std::exception& e)
        {
            o.threw = true;
            std::printf("       threw: %s\n", e.what());
        }
        std::printf("       best=%s mp=%ld\n", stateName(o.bestState), o.mapPoints);
        check(!o.threw, "the KannalaBrandt8 camera model loads and runs");
        check(o.bestState >= 1, "fisheye model reaches the tracking loop");
        std::remove(cfg.c_str());
    }

    // -----------------------------------------------------------------------
    // The synthetic camera motion has no matching inertial truth, so IMU
    // initialisation is not expected to converge. What is checked is that the
    // inertial path builds its preintegration and runs without falling over.
    std::printf("\nIMU_MONOCULAR / PinHole\n");
    {
        std::ostringstream body;
        body << pinholeIntrinsics()
             << "IMU.T_b_c1: !!opencv-matrix\n"
                "   rows: 4\n   cols: 4\n   dt: f\n"
                "   data: [1.0, 0.0, 0.0, 0.0,\n"
                "          0.0, 1.0, 0.0, 0.0,\n"
                "          0.0, 0.0, 1.0, 0.0,\n"
                "          0.0, 0.0, 0.0, 1.0]\n"
                "IMU.NoiseGyro: 1.7e-4\n"
                "IMU.NoiseAcc: 2.0e-3\n"
                "IMU.GyroWalk: 1.9393e-05\n"
                "IMU.AccWalk: 3.0e-03\n"
                "IMU.Frequency: 200.0\n"
             << kOrbAndViewer;
        const std::string cfg = writeYaml("feat_imu.yaml", body.str());

        Outcome o;
        try
        {
            ORB_SLAM3::System slam(voc, cfg, ORB_SLAM3::System::IMU_MONOCULAR, false);
            for (int i = 0; i < kFrames; ++i)
            {
                // 200 Hz samples across the 1/30 s between frames, gravity down.
                std::vector<ORB_SLAM3::IMU::Point> imu;
                const double t0 = i / 30.0;
                for (int k = 0; k < 7; ++k)
                {
                    const double t = t0 + k * (1.0 / 200.0);
                    imu.emplace_back(0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 0.0f, t);
                }
                slam.TrackMonocular(slamcpp_test::renderView(texture, i, kFrames), t0 + 1.0 / 30.0,
                                    imu);
                o.bestState = std::max(o.bestState, slam.GetTrackingState());
            }
            o.mapPoints = countMapPoints(slam.GetAtlas());
            slam.Shutdown();
        }
        catch (const std::exception& e)
        {
            o.threw = true;
            std::printf("       threw: %s\n", e.what());
        }
        std::printf("       best=%s mp=%ld\n", stateName(o.bestState), o.mapPoints);
        check(!o.threw, "the inertial path runs without raising");
        check(o.bestState >= 1, "IMU_MONOCULAR reaches the tracking loop");
        std::remove(cfg.c_str());
    }

    // -----------------------------------------------------------------------
    std::printf("\nlocalisation mode, reset, MapChanged\n");
    {
        const std::string cfg = writeYaml("feat_modes.yaml", pinholeIntrinsics() + kOrbAndViewer);
        ORB_SLAM3::System slam(voc, cfg, ORB_SLAM3::System::MONOCULAR, false);

        for (int i = 0; i < kFrames; ++i)
            slam.TrackMonocular(slamcpp_test::renderView(texture, i, kFrames), i / 30.0);

        const long builtKF = countKeyFrames(slam.GetAtlas());
        check(builtKF > 1, "a map exists before the mode changes");
        // MapChanged() tracks the atlas's big-change counter, which only moves
        // on a loop closure or a global bundle adjustment. This sweep is a
        // short open trajectory with neither, so false is the right answer and
        // what is checked is that it is callable and says so.
        check(!slam.MapChanged(), "MapChanged reports no big change on an open trajectory");

        // Localisation mode: tracking continues, mapping stops.
        slam.ActivateLocalizationMode();
        for (int i = 0; i < 30; ++i)
            slam.TrackMonocular(slamcpp_test::renderView(texture, kFrames + i, kFrames), 
                                (kFrames + i) / 30.0);
        const long afterLocalisation = countKeyFrames(slam.GetAtlas());
        check(afterLocalisation <= builtKF + 1,
              "localisation mode stops adding key frames");
        slam.DeactivateLocalizationMode();

        // Reset clears the atlas back to an empty map.
        slam.Reset();
        for (int i = 0; i < 5; ++i)
            slam.TrackMonocular(slamcpp_test::renderView(texture, i, kFrames), (300 + i) / 30.0);
        const long afterReset = countKeyFrames(slam.GetAtlas());
        std::printf("       kf built=%ld after localisation=%ld after reset=%ld\n", builtKF,
                    afterLocalisation, afterReset);
        check(afterReset < builtKF, "Reset clears the map");

        slam.ResetActiveMap();
        check(true, "ResetActiveMap runs without raising");

        slam.Shutdown();
        std::remove(cfg.c_str());
    }

    // -----------------------------------------------------------------------
    // The SuperPoint front-end, when this build has one. It needs its own
    // vocabulary: a float tree, because its descriptors are floats. The ORB
    // vocabulary cannot stand in.
#if SLAMCPP_WITH_SUPERPOINT
    std::printf("\nMONOCULAR / SuperPoint\n");
    {
        const std::string spVoc = argc > 2
                                      ? std::string(argv[2])
                                      : std::string(SLAMCPP_RESOURCE_DIR) +
                                            "/vocabulary/superpoint_voc.dbow3";

        std::ifstream probe(spVoc, std::ios::binary);
        if (!probe.good())
        {
            std::printf("       skipped: no SuperPoint vocabulary at %s\n", spVoc.c_str());
            std::printf("       build it with: convert_vocabulary superpoint_voc.yml.gz %s\n",
                        spVoc.c_str());
        }
        else
        {
            probe.close();
            const std::string cfg = writeYaml(
                "feat_sp.yaml", pinholeIntrinsics() + kOrbAndViewer +
                                    "Frontend.type: \"SUPERPOINT\"\n");

            Outcome o;
            try
            {
                ORB_SLAM3::System slam(spVoc, cfg, ORB_SLAM3::System::MONOCULAR, false);
                for (int i = 0; i < kFrames; ++i)
                {
                    slam.TrackMonocular(slamcpp_test::renderView(texture, i, kFrames), i / 30.0);
                    const int st = slam.GetTrackingState();
                    o.bestState = std::max(o.bestState, st);
                    if (st == 2)
                        ++o.okFrames;
                }
                o.keyFrames = countKeyFrames(slam.GetAtlas());
                o.mapPoints = countMapPoints(slam.GetAtlas());
                slam.Shutdown();
            }
            catch (const std::exception& e)
            {
                o.threw = true;
                std::printf("       threw: %s\n", e.what());
            }

            std::printf("       best=%s ok=%d kf=%ld mp=%ld\n", stateName(o.bestState),
                        o.okFrames, o.keyFrames, o.mapPoints);
            check(!o.threw, "the SuperPoint front-end builds and runs");
            check(o.bestState >= 2, "SuperPoint initialises and tracks");
            check(o.mapPoints > 50, "SuperPoint triangulates map points");
            std::remove(cfg.c_str());
        }
    }
#else
    std::printf("\nMONOCULAR / SuperPoint\n       skipped: built without libtorch\n");
#endif

    std::printf("\n%s (%d of %d checks failed)\n", failures ? "FAILED" : "OK", failures, checks);
    return failures ? 1 : 0;
}
