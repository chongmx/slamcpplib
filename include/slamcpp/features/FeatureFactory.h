/**
 * Choosing a feature front-end at run time.
 *
 * This is the seam that makes ORB and SuperPoint alternatives rather than two
 * different builds. The type is named in the settings file, the factory hands
 * back the matching extractor, and everything above it works through
 * FeatureExtractor.
 *
 * SuperPoint is only present when the library was built with libtorch, so the
 * factory reports what it can actually make rather than failing obscurely
 * deep inside Tracking.
 */

#ifndef SLAMCPP_FEATURES_FEATUREFACTORY_H
#define SLAMCPP_FEATURES_FEATUREFACTORY_H

#include <string>

#include "slamcpp/features/FeatureExtractor.h"

namespace ORB_SLAM3
{

enum class FrontendType
{
    ORB,
    SuperPoint
};

// Accepts "ORB" and "SUPERPOINT", in any case. Unknown names fall back to ORB
// and say so.
FrontendType ParseFrontendType(const std::string& name);

const char* FrontendTypeName(FrontendType type);

// False for SuperPoint in a build without libtorch.
bool FrontendAvailable(FrontendType type);

// The descriptor a front-end emits, without having to build one: CV_8U for
// ORB, CV_32F for SuperPoint. The matcher is configured from this.
int FrontendDescriptorType(FrontendType type);

// Builds the extractor. Raises if the front-end is not available in this
// build, or if SuperPoint cannot find its weights.
FeatureExtractor* CreateFeatureExtractor(FrontendType type, int nFeatures, float scaleFactor,
                                         int nLevels, int iniThFAST, int minThFAST,
                                         const std::string& modelPath = std::string());

}  // namespace ORB_SLAM3

#endif  // SLAMCPP_FEATURES_FEATUREFACTORY_H
