// Converts a bag-of-words vocabulary into DBoW3's binary form.
//
// Why this is worth a tool: the SuperPoint vocabulary ships as a 117 MB
// gzipped YAML, and DBoW3 reads that through cv::FileStorage, which takes
// minutes. The same tree written in DBoW3's own compressed binary format loads
// in seconds. Converting once turns an unusable start-up into a normal one.
//
// Usage: convert_vocabulary <in> <out>
//
//   convert_vocabulary superpoint_voc.yml.gz superpoint_voc.dbow3
//   convert_vocabulary ORBvoc.txt            ORBvoc.dbow3
//
// The input may be anything DBoW3 can read: its own binary, a YAML or gzipped
// YAML, or the classic ORB_SLAM text format.

#include <chrono>
#include <cstdio>
#include <string>

#include "slamcpp/Vocabulary.h"

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::fprintf(stderr, "usage: %s <input vocabulary> <output .dbow3>\n", argv[0]);
        return 2;
    }

    const std::string in = argv[1];
    const std::string out = argv[2];

    std::printf("reading %s\n", in.c_str());
    std::fflush(stdout);

    const auto t0 = std::chrono::steady_clock::now();

    ORB_SLAM3::Vocabulary voc;
    if (!ORB_SLAM3::LoadVocabulary(voc, in))
    {
        std::fprintf(stderr, "error: could not read %s\n", in.c_str());
        return 1;
    }

    const auto t1 = std::chrono::steady_clock::now();
    const double readSeconds = std::chrono::duration<double>(t1 - t0).count();

    std::printf("\n  words          %u\n", voc.size());
    std::printf("  branching      %d\n", voc.getBranchingFactor());
    std::printf("  depth          %d\n", voc.getDepthLevels());
    std::printf("  descriptor     %s\n",
                voc.getDescritorType() == CV_8U ? "CV_8U (binary, ORB)"
                                                : "CV_32F (float, SuperPoint)");
    std::printf("  read in        %.1f s\n", readSeconds);

    std::printf("\nwriting %s\n", out.c_str());
    std::fflush(stdout);

    const auto t2 = std::chrono::steady_clock::now();
    voc.save(out, /*binary_compressed=*/true);
    const double writeSeconds = std::chrono::duration<double>(
                                    std::chrono::steady_clock::now() - t2)
                                    .count();

    std::printf("  written in     %.1f s\n\nDone.\n", writeSeconds);
    return 0;
}
