/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SPEXTRACTOR_H
#define SPEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>

#include <torch/torch.h>
#include <memory>
#include <string>

#include "slamcpp/features/ExtractorNode.h"
#include "slamcpp/features/FeatureExtractor.h"
#include "slamcpp/features/SuperPoint.h"
#ifdef EIGEN_MPL2_ONLY
#undef EIGEN_MPL2_ONLY
#endif


namespace ORB_SLAM3
{

class SPextractor : public FeatureExtractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };
    // modelPath names the TorchScript SuperPoint weights. Empty means the
    // copy shipped in resources/models, resolved at run time.
    SPextractor(int nfeatures, float scaleFactor, int nlevels,
                float iniThFAST, float minThFAST,
                const std::string& modelPath = std::string());

    ~SPextractor(){}

    // Compute the SP features and descriptors on an image.
    // SP are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    // Matches FeatureExtractor. SuperPoint has no notion of a stereo lapping
    // area, so the band is ignored and 0 is returned, which is what the
    // pinhole stereo and monocular paths expect.
    int operator()( cv::InputArray image, cv::InputArray mask,
                    std::vector<cv::KeyPoint>& keypoints,
                    cv::OutputArray descriptors,
                    std::vector<int>& vLappingArea) override;

    // 256 floats, compared with an L2 norm.
    int descriptorType() const override { return CV_32F; }
    const char* name() const override { return "SuperPoint"; }

    // The scale pyramid, its factors and their getters live in the base.
    bool mbUseFP16 = false;

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints, cv::Mat &_desc);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    // void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    // std::vector<cv::Point> pattern;

    float iniThFAST;
    float minThFAST;
    std::string mModelPath;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::shared_ptr<SuperPoint> model;
};


} //namespace ORB_SLAM

#endif

