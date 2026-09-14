/**
 * The octree cell used to spread key points evenly over an image.
 *
 * Both front-ends distribute their detections with the same quadtree split, so
 * the node lives here rather than being declared twice. It used to be, once in
 * each extractor's header, with DivideNode defined in each extractor's source,
 * which is a duplicate symbol the moment both are built into one library.
 */

#ifndef SLAMCPP_FEATURES_EXTRACTORNODE_H
#define SLAMCPP_FEATURES_EXTRACTORNODE_H

#include <list>
#include <vector>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3
{

class ExtractorNode
{
public:
    ExtractorNode() : bNoMore(false) {}

    void DivideNode(ExtractorNode& n1, ExtractorNode& n2, ExtractorNode& n3, ExtractorNode& n4);

    std::vector<cv::KeyPoint>          vKeys;
    cv::Point2i                        UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool                               bNoMore;
};

}  // namespace ORB_SLAM3

#endif  // SLAMCPP_FEATURES_EXTRACTORNODE_H
