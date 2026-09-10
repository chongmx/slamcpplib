// Round-trips an Atlas through the replacement archive.
//
// This is the test the Boost removal turns on. Atlas save and load had no test
// before, and it is the one feature Boost.Serialization was carrying, so
// swapping the archive underneath it without proving a full round trip would
// be replacing something that worked with something merely plausible.
//
// It runs the real path end to end: build a map from the synthetic sequence,
// let Shutdown() write the .osa file, then bring up a second System pointed at
// that file and compare what came back.
//
// Usage: test_serialization <ORBvoc.txt> <settings.yaml>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "slamcpp/Atlas.h"
#include "slamcpp/KeyFrame.h"
#include "slamcpp/Map.h"
#include "slamcpp/MapPoint.h"
#include "slamcpp/System.h"
#include "slamcpp/serialization/Archive.h"

#include "synthetic_scene.h"

namespace
{

const int         kFrames = 160;

int failures = 0;

void check(bool ok, const std::string& what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (!ok)
        ++failures;
}

// Copies the project's settings file and appends atlas keys, so the test drives
// the same configuration path the application does.
std::string writeSettings(const std::string& base, const std::string& out,
                          const std::vector<std::pair<std::string, std::string>>& keys)
{
    std::ifstream in(base);
    std::stringstream ss;
    ss << in.rdbuf();

    std::ofstream os(out);
    os << ss.str() << "\n";
    for (const auto& kv : keys)
        os << kv.first << ": \"" << kv.second << "\"\n";
    return out;
}

// Every pointer the loader rebuilt has to be usable. A dangling reference used
// to arrive as a null in the covisibility graph and crash the first traversal.
void checkGraph(ORB_SLAM3::Atlas* atlas)
{
    int badKF = 0, badMP = 0, nullCam = 0;
    for (ORB_SLAM3::Map* m : atlas->GetAllMaps())
    {
        for (ORB_SLAM3::KeyFrame* kf : m->GetAllKeyFrames())
        {
            if (!kf || kf->isBad())
                continue;
            if (kf->GetMap() == nullptr)
                ++badKF;
            if (kf->mpCamera == nullptr)
                ++nullCam;
            for (ORB_SLAM3::KeyFrame* c : kf->GetVectorCovisibleKeyFrames())
                if (c == nullptr)
                    ++badKF;
        }
        for (ORB_SLAM3::MapPoint* mp : m->GetAllMapPoints())
            if (mp && !mp->isBad() && mp->GetMap() == nullptr)
                ++badMP;
    }
    check(badKF == 0, "no null or map-less key frames in the restored graph");
    check(badMP == 0, "no map-less map points in the restored graph");
    check(nullCam == 0, "every restored key frame kept its camera model");
}

struct Counts
{
    int           nonEmptyMaps = 0;
    unsigned long keyFrames = 0;
    unsigned long mapPoints = 0;
};

// Summed over every map, not just the active one.
//
// Atlas::KeyFramesInMap() reports the current map only, and loading a saved
// atlas leaves the restored map in place while starting a fresh empty one for
// the new session. Asking the atlas for "the" count would therefore read the
// new empty map and report nothing, which says nothing about what was loaded.
Counts measure(ORB_SLAM3::Atlas* atlas)
{
    Counts c;
    for (ORB_SLAM3::Map* m : atlas->GetAllMaps())
    {
        // Counts only what PreSave actually writes. KeyFramesInMap() reports
        // the size of the whole set, bad entries included, while PreSave skips
        // those, so comparing raw sizes would report a difference that is not
        // one.
        unsigned long kf = 0;
        for (ORB_SLAM3::KeyFrame* p : m->GetAllKeyFrames())
            if (p && !p->isBad())
                ++kf;

        unsigned long mp = 0;
        for (ORB_SLAM3::MapPoint* p : m->GetAllMapPoints())
            if (p && !p->isBad())
                ++mp;

        c.keyFrames += kf;
        c.mapPoints += mp;
        if (kf != 0 || mp != 0)
            ++c.nonEmptyMaps;
    }
    return c;
}

// ---------------------------------------------------------------------------
// A focused check of the archive itself, so that a failure points at the
// serializer rather than at several thousand lines of SLAM around it.
// ---------------------------------------------------------------------------
void archiveBasics(slamcpp::serialization::Format format, const char* name)
{
    std::printf("\narchive primitives (%s)\n", name);

    const std::string path = std::string("slamcpp_archive_basics_") + name;

    std::map<int, std::string>     mapOut{{1, "one"}, {7, "seven"}};
    std::vector<double>            vecOut{1.5, -2.25, 3.125};
    std::set<long>                 setOut{5, 9, 11};
    std::string                    strOut = "a string with spaces";
    float                          arrOut[4] = {1.f, 2.f, 3.f, 4.f};

    {
        std::ofstream ofs(path, std::ios::binary);
        slamcpp::serialization::OutputArchive ar(ofs, format);
        ar& mapOut& vecOut& setOut& strOut;
        ar& slamcpp::serialization::make_array(arrOut, 4);
    }

    std::map<int, std::string> mapIn;
    std::vector<double>        vecIn;
    std::set<long>             setIn;
    std::string                strIn;
    float                      arrIn[4] = {0, 0, 0, 0};
    {
        std::ifstream ifs(path, std::ios::binary);
        slamcpp::serialization::InputArchive ar(ifs, format);
        ar& mapIn& vecIn& setIn& strIn;
        ar& slamcpp::serialization::make_array(arrIn, 4);
    }

    check(mapIn == mapOut, "std::map round-trips");
    check(vecIn == vecOut, "std::vector round-trips");
    check(setIn == setOut, "std::set round-trips");
    check(strIn == strOut, "std::string round-trips");
    check(std::equal(arrOut, arrOut + 4, arrIn), "make_array round-trips");

    std::remove(path.c_str());

    // A truncated file must be reported, not read as garbage.
    {
        std::ofstream ofs(path, std::ios::binary);
        slamcpp::serialization::OutputArchive ar(ofs, format);
        ar& vecOut;
    }
    {
        std::ifstream src(path, std::ios::binary);
        std::string   whole((std::istreambuf_iterator<char>(src)),
                            std::istreambuf_iterator<char>());
        // Cut well into the payload. Trimming a few bytes only removes
        // trailing structure in XML, which a reader is right not to miss.
        std::ofstream trunc(path, std::ios::binary);
        trunc << whole.substr(0, whole.size() / 2);
    }
    bool threw = false;
    try
    {
        std::ifstream ifs(path, std::ios::binary);
        slamcpp::serialization::InputArchive ar(ifs, format);
        std::vector<double> v;
        ar& v;
    }
    catch (const std::exception&)
    {
        threw = true;
    }
    check(threw, "a truncated archive raises rather than returning junk");
    std::remove(path.c_str());
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::fprintf(stderr, "usage: %s <ORBvoc.txt> <settings.yaml>\n", argv[0]);
        return 2;
    }
    const std::string voc = argv[1];
    const std::string baseSettings = argv[2];

    archiveBasics(slamcpp::serialization::Format::Binary, "binary");
    archiveBasics(slamcpp::serialization::Format::Text, "text");
    archiveBasics(slamcpp::serialization::Format::Xml, "xml");

    // Phase 1 builds a map and writes it. Its live counts are deliberately not
    // used as the reference: Map::PreSave mutates the map it is about to write,
    // erasing observations and marking entries bad as it goes, so a count taken
    // from the live map afterwards does not describe the file.
    //
    // What must hold instead is that the archive is a fixed point, and that the
    // three formats agree. The atlas is written as binary, read back and
    // written as text, read back and written as XML, then read once more. If
    // any format loses or invents anything, the counts stop matching.
    const char* kGen[] = {"slamcpp_rt_bin", "slamcpp_rt_txt", "slamcpp_rt_xml"};

    const std::string buildBin =
        writeSettings(baseSettings, "slamcpp_rt_0.yaml",
                      {{"System.SaveAtlasToFile", kGen[0]}, {"System.AtlasFormat", "binary"}});

    const cv::Mat texture = slamcpp_test::makeTexture();

    std::printf("\nphase 1: build a map, write it as binary\n");
    {
        ORB_SLAM3::System slam(voc, buildBin, ORB_SLAM3::System::MONOCULAR, false);
        for (int i = 0; i < kFrames; ++i)
            slam.TrackMonocular(slamcpp_test::renderView(texture, i, kFrames), i / 30.0);
        slam.Shutdown();
    }
    {
        std::ifstream probe(std::string("./") + kGen[0] + ".osa", std::ios::binary);
        check(probe.good(), "binary atlas written");
    }

    // Each step reads the previous format and writes the next one.
    struct Step
    {
        const char* readFrom;
        const char* readFormat;
        const char* writeTo;
        const char* writeFormat;
        const char* label;
    };
    const Step steps[] = {
        {kGen[0], "binary", kGen[1], "text", "binary -> text"},
        {kGen[1], "text", kGen[2], "xml", "text -> xml"},
        {kGen[2], "xml", nullptr, nullptr, "xml -> (read only)"},
    };

    Counts first;
    bool   haveFirst = false;
    for (const Step& step : steps)
    {
        std::printf("\nphase: %s\n", step.label);

        std::vector<std::pair<std::string, std::string>> keys;
        keys.push_back({"System.LoadAtlasFromFile", step.readFrom});
        keys.push_back({"System.AtlasFormat", step.readFormat});
        if (step.writeTo != nullptr)
        {
            keys.push_back({"System.SaveAtlasToFile", step.writeTo});
            keys.push_back({"System.AtlasSaveFormat", step.writeFormat});
        }

        const std::string cfg =
            writeSettings(baseSettings, std::string("slamcpp_rt_") + step.readFormat + ".yaml",
                          keys);

        Counts c;
        {
            ORB_SLAM3::System slam(voc, cfg, ORB_SLAM3::System::MONOCULAR, false);
            c = measure(slam.GetAtlas());
            checkGraph(slam.GetAtlas());
            slam.Shutdown();
        }
        std::printf("  %s holds %d map(s), %lu key frames, %lu map points\n",
                    step.readFormat, c.nonEmptyMaps, c.keyFrames, c.mapPoints);

        if (!haveFirst)
        {
            first = c;
            haveFirst = true;
            check(c.nonEmptyMaps > 0, "the restored atlas has a map");
            check(c.keyFrames > 0, "the restored atlas has key frames");
            check(c.mapPoints > 0, "the restored atlas has map points");
        }
        else
        {
            check(c.nonEmptyMaps == first.nonEmptyMaps,
                  std::string(step.readFormat) + ": map count matches binary");
            check(c.keyFrames == first.keyFrames,
                  std::string(step.readFormat) + ": key frame count matches binary");
            check(c.mapPoints == first.mapPoints,
                  std::string(step.readFormat) + ": map point count matches binary");
        }

        std::remove(cfg.c_str());
    }

    for (const char* g : kGen)
        std::remove((std::string("./") + g + ".osa").c_str());
    std::remove(buildBin.c_str());

    std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "OK", failures,
                failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
