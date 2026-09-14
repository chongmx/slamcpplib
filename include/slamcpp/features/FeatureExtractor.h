/**
 * The feature front-end interface.
 *
 * slamcpp supports more than one kind of feature. ORB is the classic binary
 * descriptor the system was built around; SuperPoint is a learned float
 * descriptor. The two are not variants of one algorithm, they are different
 * front-ends with different descriptor arithmetic, and the rest of the system
 * has to be able to run on either without being rebuilt.
 *
 * The upstream SuperPoint fork solved this by typedef'ing SPextractor to the
 * name ORBextractor and deleting the ORB files, which makes the choice a
 * build-time one and means a single binary can only ever do one of them. This
 * interface is what replaces that: the choice is a pointer, decided when the
 * Tracking object is constructed, from the settings file.
 *
 * The scale-pyramid data lives in the base rather than behind virtual getters.
 * Frame and Tracking read it in inner loops, and it is plain state that every
 * front-end has, so there is nothing to dispatch on.
 */

#ifndef SLAMCPP_FEATURES_FEATUREEXTRACTOR_H
#define SLAMCPP_FEATURES_FEATUREEXTRACTOR_H

#include <vector>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3
{

class FeatureExtractor
{
public:
    virtual ~FeatureExtractor() = default;

    // Extracts key points and descriptors. vLappingArea is the horizontal band
    // a fisheye stereo rig shares between its two eyes; the return value is the
    // index of the first key point inside it, or 0 when there is no overlap.
    virtual int operator()(cv::InputArray image, cv::InputArray mask,
                           std::vector<cv::KeyPoint>& keypoints,
                           cv::OutputArray descriptors,
                           std::vector<int>& vLappingArea) = 0;

    // CV_8U for a binary descriptor, CV_32F for a float one. The matcher picks
    // its distance from this, and the bag-of-words vocabulary has to agree.
    virtual int descriptorType() const = 0;

    // A short name, for logging and for naming the front-end in a saved map.
    virtual const char* name() const = 0;

    int   GetLevels() { return nlevels; }
    float GetScaleFactor() { return static_cast<float>(scaleFactor); }

    std::vector<float> GetScaleFactors() { return mvScaleFactor; }
    std::vector<float> GetInverseScaleFactors() { return mvInvScaleFactor; }
    std::vector<float> GetScaleSigmaSquares() { return mvLevelSigma2; }
    std::vector<float> GetInverseScaleSigmaSquares() { return mvInvLevelSigma2; }

    std::vector<cv::Mat> mvImagePyramid;

protected:
    int    nfeatures = 0;
    double scaleFactor = 1.2;
    int    nlevels = 1;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

}  // namespace ORB_SLAM3

#endif  // SLAMCPP_FEATURES_FEATUREEXTRACTOR_H
