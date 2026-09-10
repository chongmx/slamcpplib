// Checks the DBoW3 migration against the vocabulary the project actually ships.
//
// This is the part of the DBoW2 removal that could not be settled by reading:
// DBoW2 fixed the descriptor as a 32-byte binary word in its template
// arguments, while DBoW3 decides at run time from the file it loaded. The
// classic ORBvoc.txt has to come back with the same shape as before, and the
// bag-of-words transform and similarity score have to still behave.
//
// Usage: test_vocabulary <path to ORBvoc.txt>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "slamcpp/Vocabulary.h"

namespace
{

int failures = 0;

void check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        ++failures;
}

// A deterministic stand-in for a real ORB descriptor: 32 bytes, CV_8U.
cv::Mat makeDescriptor(int seed)
{
    cv::Mat d(1, 32, CV_8UC1);
    for (int i = 0; i < 32; ++i)
        d.at<uchar>(0, i) = static_cast<uchar>((seed * 37 + i * 11) & 0xFF);
    return d;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::fprintf(stderr, "usage: %s <ORBvoc.txt>\n", argv[0]);
        return 2;
    }
    const std::string path = argv[1];

    std::printf("loading %s\n", path.c_str());
    ORB_SLAM3::Vocabulary voc;
    const bool loaded = ORB_SLAM3::LoadVocabulary(voc, path);
    std::printf("\n");

    check(loaded, "ORBvoc.txt loads through DBoW3");
    if (!loaded)
        return 1;

    check(!voc.empty(), "vocabulary is not empty");

    // ORB_SLAM3 ships a k=10, L=6 tree, so just under a million leaf words.
    std::printf("       words=%u  branching=%d  levels=%d  descriptor type=%d\n",
                voc.size(), voc.getBranchingFactor(), voc.getDepthLevels(),
                voc.getDescritorType());
    check(voc.size() > 900000, "word count is the expected ~1e6");
    check(voc.getBranchingFactor() == 10, "branching factor is 10");
    check(voc.getDepthLevels() == 6, "depth is 6 levels");

    // The whole point of DBoW3 here: it reports the descriptor type rather
    // than having it baked into the C++ type. Text vocabularies are CV_8U.
    check(voc.getDescritorType() == CV_8U, "descriptor type reported as CV_8U");

    // Transform: a set of descriptors has to produce a non-empty bag of words
    // and a feature vector, which is what Frame and KeyFrame rely on.
    std::vector<cv::Mat> descriptors;
    for (int i = 0; i < 64; ++i)
        descriptors.push_back(makeDescriptor(i));

    DBoW3::BowVector bow;
    DBoW3::FeatureVector feat;
    voc.transform(descriptors, bow, feat, 4);
    check(!bow.empty(), "transform produces a non-empty BowVector");
    check(!feat.empty(), "transform produces a non-empty FeatureVector");

    // Scoring: identical input must score above a clearly different one, which
    // is the property loop closing and relocalisation depend on.
    std::vector<cv::Mat> other;
    for (int i = 0; i < 64; ++i)
        other.push_back(makeDescriptor(i + 5000));

    DBoW3::BowVector bowOther;
    DBoW3::FeatureVector featOther;
    voc.transform(other, bowOther, featOther, 4);

    const double self = voc.score(bow, bow);
    const double cross = voc.score(bow, bowOther);
    std::printf("       score(self)=%.6f  score(other)=%.6f\n", self, cross);
    check(self > cross, "a bag of words scores higher against itself");

    std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "OK",
                failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
