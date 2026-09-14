// The front-end factory. Lives outside src/features because it has to compile
// in builds without libtorch, which is exactly when it has to explain that
// SuperPoint is unavailable.

#include "slamcpp/features/FeatureFactory.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "slamcpp/ORBextractor.h"

#if SLAMCPP_WITH_SUPERPOINT
#include "slamcpp/features/SPextractor.h"
#endif

namespace ORB_SLAM3
{

FrontendType ParseFrontendType(const std::string& name)
{
    std::string upper = name;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char c) { return static_cast<char>(::toupper(c)); });

    if (upper == "ORB")
        return FrontendType::ORB;
    if (upper == "SUPERPOINT" || upper == "SP")
        return FrontendType::SuperPoint;

    std::cerr << "Unknown Frontend.type '" << name << "', using ORB" << std::endl;
    return FrontendType::ORB;
}

const char* FrontendTypeName(FrontendType type)
{
    return type == FrontendType::SuperPoint ? "SuperPoint" : "ORB";
}

bool FrontendAvailable(FrontendType type)
{
#if SLAMCPP_WITH_SUPERPOINT
    (void)type;
    return true;
#else
    return type != FrontendType::SuperPoint;
#endif
}

int FrontendDescriptorType(FrontendType type)
{
    return type == FrontendType::SuperPoint ? CV_32F : CV_8U;
}

FeatureExtractor* CreateFeatureExtractor(FrontendType type, int nFeatures, float scaleFactor,
                                         int nLevels, int iniThFAST, int minThFAST,
                                         const std::string& modelPath)
{
    if (type == FrontendType::SuperPoint)
    {
#if SLAMCPP_WITH_SUPERPOINT
        // SuperPoint scores its detections on a 0..1 confidence, so the FAST
        // thresholds are reinterpreted rather than reused: the settings file
        // keeps one pair of numbers and each front-end reads them its own way.
        const float iniConf = iniThFAST > 1 ? iniThFAST / 1000.0f : 0.015f;
        const float minConf = minThFAST > 1 ? minThFAST / 1000.0f : 0.007f;
        return new SPextractor(nFeatures, scaleFactor, nLevels, iniConf, minConf, modelPath);
#else
        throw std::runtime_error(
            "slamcpp: the SuperPoint front-end was requested, but this build has no libtorch. "
            "Rebuild with -DCMAKE_PREFIX_PATH=<libtorch> to enable it.");
#endif
    }

    return new ORBextractor(nFeatures, scaleFactor, nLevels, iniThFAST, minThFAST);
}

}  // namespace ORB_SLAM3
