#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "TestHarness.hpp"
#include "slamcpp/calibration/io/Json.hpp"
#include "slamcpp/calibration/io/Yaml.hpp"

namespace yaml = slamcpp::calib::yaml;
namespace json = slamcpp::calib::json;

// A camchain-imucam.yaml in upstream's exact layout. Schema compatibility is a
// hard requirement from the plan: existing files must load and output must be
// consumable by downstream tools, so this is the shape that has to parse.
static const char* kCamChain = R"(# Kalibr camera chain
cam0:
  camera_model: pinhole
  intrinsics: [461.629, 460.152, 362.680, 246.049]
  distortion_model: radtan
  distortion_coeffs: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
  resolution: [752, 480]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: 0.0021343643
  T_cam_imu:
  - [0.0148655, -0.999880, 0.00414029, -0.0216401]
  - [0.999557, 0.0149672, 0.0257744, -0.064676]
  - [0.0257744, 0.0, 0.999368, 0.00981073]
  - [0.0, 0.0, 0.0, 1.0]
cam1:
  camera_model: pinhole
  intrinsics: [460.316, 458.833, 379.242, 255.284]
  resolution: [752, 480]
  T_cn_cnm1:
  - [0.999997, 0.00231207, 0.000376008, -0.110074]
  - [-0.00231714, 0.999898, 0.0140907, 0.000399122]
  - [-0.000343393, -0.0140916, 0.999901, -0.000853703]
  - [0.0, 0.0, 0.0, 1.0]
)";

// ---------------------------------------------------------------------------
// YAML
// ---------------------------------------------------------------------------

CALIB_TEST(Yaml, ParsesAnUpstreamCamChain) {
  const yaml::Node root = yaml::parse(kCamChain);
  EXPECT_TRUE(root.isMap());
  EXPECT_TRUE(root.has("cam0"));
  EXPECT_TRUE(root.has("cam1"));

  const yaml::Node& cam0 = root["cam0"];
  EXPECT_EQ(cam0["camera_model"].asString(), std::string("pinhole"));
  EXPECT_EQ(cam0["distortion_model"].asString(), std::string("radtan"));
  EXPECT_EQ(cam0["rostopic"].asString(), std::string("/cam0/image_raw"));

  const std::vector<double> intr = cam0["intrinsics"].asDoubleVector();
  EXPECT_EQ(intr.size(), std::size_t(4));
  EXPECT_NEAR(intr[0], 461.629, 1e-9);
  EXPECT_NEAR(intr[3], 246.049, 1e-9);

  // Scientific notation has to survive, or a distortion coefficient is lost.
  const std::vector<double> dist = cam0["distortion_coeffs"].asDoubleVector();
  EXPECT_NEAR(dist[3], 1.76187114e-05, 1e-15);
  EXPECT_NEAR(dist[0], -0.28340811, 1e-12);

  EXPECT_EQ(cam0["resolution"][0].asInt(), 752);
  EXPECT_EQ(cam0["resolution"][1].asInt(), 480);
  EXPECT_NEAR(cam0["timeshift_cam_imu"].asDouble(), 0.0021343643, 1e-15);
}

// A block sequence sitting at the same indentation as its key is legal YAML
// and is exactly how upstream writes every transformation matrix.
CALIB_TEST(Yaml, ParsesAMatrixAtTheSameIndentAsItsKey) {
  const yaml::Node root = yaml::parse(kCamChain);
  const auto T = root["cam0"]["T_cam_imu"].asMatrix();
  EXPECT_EQ(T.size(), std::size_t(4));
  EXPECT_EQ(T[0].size(), std::size_t(4));
  EXPECT_NEAR(T[0][0], 0.0148655, 1e-12);
  EXPECT_NEAR(T[0][1], -0.999880, 1e-12);
  EXPECT_NEAR(T[1][3], -0.064676, 1e-12);
  EXPECT_NEAR(T[3][3], 1.0, 1e-12);

  const auto B = root["cam1"]["T_cn_cnm1"].asMatrix();
  EXPECT_NEAR(B[0][3], -0.110074, 1e-12);
}

CALIB_TEST(Yaml, HandlesCommentsQuotesAndDocumentMarkers) {
  const yaml::Node n = yaml::parse(
      "---\n"
      "# leading comment\n"
      "a: 1  # trailing comment\n"
      "b: \"quoted # not a comment\"\n"
      "c: 'single ''escaped'' quotes'\n"
      "d: plain string with spaces\n"
      "e: \"tab\\there\"\n");
  EXPECT_EQ(n["a"].asInt(), 1);
  EXPECT_EQ(n["b"].asString(), std::string("quoted # not a comment"));
  EXPECT_EQ(n["c"].asString(), std::string("single 'escaped' quotes"));
  EXPECT_EQ(n["d"].asString(), std::string("plain string with spaces"));
  EXPECT_EQ(n["e"].asString(), std::string("tab\there"));
}

CALIB_TEST(Yaml, NestedMapsAndBooleans) {
  const yaml::Node n = yaml::parse(
      "target:\n"
      "  target_type: aprilgrid\n"
      "  tagRows: 6\n"
      "  tagCols: 6\n"
      "  tagSize: 0.088\n"
      "  tagSpacing: 0.3\n"
      "options:\n"
      "  verbose: true\n"
      "  showExtraction: no\n");
  EXPECT_EQ(n["target"]["target_type"].asString(), std::string("aprilgrid"));
  EXPECT_EQ(n["target"]["tagRows"].asInt(), 6);
  EXPECT_NEAR(n["target"]["tagSize"].asDouble(), 0.088, 1e-12);
  EXPECT_TRUE(n["options"]["verbose"].asBool());
  EXPECT_FALSE(n["options"]["showExtraction"].asBool());
}

CALIB_TEST(Yaml, NestedFlowSequences) {
  const yaml::Node n = yaml::parse("m: [[1, 2], [3, 4]]\n");
  const auto m = n["m"].asMatrix();
  EXPECT_EQ(m.size(), std::size_t(2));
  EXPECT_NEAR(m[1][0], 3.0, 1e-12);
}

CALIB_TEST(Yaml, BlockSequenceOfMaps) {
  const yaml::Node n = yaml::parse(
      "imus:\n"
      "  - rostopic: /imu0\n"
      "    update_rate: 200.0\n"
      "  - rostopic: /imu1\n"
      "    update_rate: 400.0\n");
  EXPECT_EQ(n["imus"].size(), std::size_t(2));
  EXPECT_EQ(n["imus"][0]["rostopic"].asString(), std::string("/imu0"));
  EXPECT_NEAR(n["imus"][1]["update_rate"].asDouble(), 400.0, 1e-12);
}

// Missing keys read as null rather than throwing, so optional fields such as
// T_cn_cnm1 on cam0 can be probed with has() and skipped.
CALIB_TEST(Yaml, MissingKeysAreNull) {
  const yaml::Node root = yaml::parse(kCamChain);
  EXPECT_TRUE(root["cam0"]["T_cn_cnm1"].isNull());
  EXPECT_FALSE(root["cam0"].has("T_cn_cnm1"));
  EXPECT_TRUE(root["nonexistent"].isNull());
}

// The unsupported constructs must be rejected with a line number, never
// silently mis-parsed: a config that quietly drops a coefficient is far worse
// than one that fails to load.
CALIB_TEST(Yaml, RejectsUnsupportedConstructs) {
  EXPECT_THROWS(yaml::parse("a: &anchor 1\nb: *anchor\n"));
  EXPECT_THROWS(yaml::parse("a: {b: 1}\n"));
  EXPECT_THROWS(yaml::parse("a: |\n  block scalar\n"));
  EXPECT_THROWS(yaml::parse("a: [1, 2\n"));
  EXPECT_THROWS(yaml::parse("a: 1\na: 2\n"));
  EXPECT_THROWS(yaml::parse("not a mapping line\n"));
}

CALIB_TEST(Yaml, ReportsTheOffendingLineNumber) {
  try {
    yaml::parse("a: 1\nb: 2\nc: {d: 3}\n");
    CALIB_FAIL("expected a ParseError");
  } catch (const yaml::ParseError& e) {
    EXPECT_EQ(e.line, 3);
  }
}

CALIB_TEST(Yaml, TypedReadsRejectMismatchedNodes) {
  const yaml::Node n = yaml::parse("a: hello\nb: [1, 2]\n");
  EXPECT_THROWS(n["a"].asDouble());
  EXPECT_THROWS(n["a"].asInt());
  EXPECT_THROWS(n["a"].asBool());
  EXPECT_THROWS(n["b"].asString());
  EXPECT_THROWS(n["a"].asDoubleVector());
}

CALIB_TEST(Yaml, RaggedMatrixIsRejected) {
  const yaml::Node n = yaml::parse("m:\n- [1, 2, 3]\n- [4, 5]\n");
  EXPECT_THROWS(n["m"].asMatrix());
}

// Round-tripping is what lets the port be validated against upstream by direct
// file comparison, so it has to be exact through a parse-emit-parse cycle.
CALIB_TEST(Yaml, RoundTripsThroughTheEmitter) {
  const yaml::Node first = yaml::parse(kCamChain);
  const std::string emitted = yaml::emit(first);
  const yaml::Node second = yaml::parse(emitted);

  EXPECT_EQ(second["cam0"]["camera_model"].asString(), std::string("pinhole"));
  EXPECT_NEAR(second["cam0"]["intrinsics"].asDoubleVector()[0], 461.629, 1e-9);
  EXPECT_NEAR(second["cam0"]["distortion_coeffs"].asDoubleVector()[3],
              1.76187114e-05, 1e-15);

  const auto T1 = first["cam0"]["T_cam_imu"].asMatrix();
  const auto T2 = second["cam0"]["T_cam_imu"].asMatrix();
  EXPECT_EQ(T1.size(), T2.size());
  for (std::size_t r = 0; r < T1.size(); ++r) {
    for (std::size_t c = 0; c < T1[r].size(); ++c) {
      EXPECT_NEAR(T1[r][c], T2[r][c], 0.0);
    }
  }
  // Key order is preserved, so an emitted file diffs cleanly against upstream.
  EXPECT_EQ(second.keys()[0], std::string("cam0"));
  EXPECT_EQ(second["cam0"].keys()[0], std::string("camera_model"));
}

CALIB_TEST(Yaml, EmitsMatricesAsBlocksOfFlowRows) {
  yaml::Node root;
  yaml::Node row = yaml::Node::sequence();
  row.push_back(yaml::Node::scalar("1"));
  row.push_back(yaml::Node::scalar("2"));
  yaml::Node m = yaml::Node::sequence();
  m.push_back(row);
  m.push_back(row);
  root["T"] = m;

  const std::string out = yaml::emit(root);
  EXPECT_EQ(out, std::string("T:\n- [1, 2]\n- [1, 2]\n"));
}

CALIB_TEST(Yaml, EmptyInputIsNull) {
  EXPECT_TRUE(yaml::parse("").isNull());
  EXPECT_TRUE(yaml::parse("# just a comment\n").isNull());
}

// ---------------------------------------------------------------------------
// JSON
// ---------------------------------------------------------------------------

CALIB_TEST(Json, WritesANestedReport) {
  std::ostringstream out;
  json::Writer w(out);
  w.beginObject();
  w.keyValue("converged", true);
  w.keyValue("iterations", 12);
  w.key("reprojection");
  w.beginObject();
  w.keyValue("mean", 0.184);
  w.keyValue("count", 4821);
  w.endObject();
  w.keyValue("intrinsics", std::vector<double>{461.629, 460.152});
  w.endObject();

  const std::string s = out.str();
  EXPECT_TRUE(s.find("\"converged\": true") != std::string::npos);
  EXPECT_TRUE(s.find("\"iterations\": 12") != std::string::npos);
  EXPECT_TRUE(s.find("\"mean\": 0.184") != std::string::npos);
  EXPECT_TRUE(s.find("461.629") != std::string::npos);
  // No trailing comma before a closing brace or bracket.
  EXPECT_TRUE(s.find(",}") == std::string::npos);
  EXPECT_TRUE(s.find(",]") == std::string::npos);
  EXPECT_TRUE(s.find(", }") == std::string::npos);
}

CALIB_TEST(Json, WritesMatrices) {
  std::ostringstream out;
  json::Writer w(out);
  w.beginObject();
  w.keyValue("T_cam_imu", std::vector<std::vector<double>>{{1.0, 0.0}, {0.0, 1.0}});
  w.endObject();
  const std::string s = out.str();
  EXPECT_TRUE(s.find("[") != std::string::npos);
  EXPECT_TRUE(s.find(",}") == std::string::npos);
}

// Reports are compared against reference results, so a double has to come back
// bit-identical from the text.
CALIB_TEST(Json, DoublesRoundTripExactly) {
  const double v = 0.1234567890123456789;
  std::ostringstream out;
  json::Writer w(out);
  w.beginArray();
  w.value(v);
  w.endArray();
  const std::string s = out.str();
  const std::string num = s.substr(s.find_first_of("0123456789-"));
  EXPECT_NEAR(std::strtod(num.c_str(), nullptr), v, 0.0);
}

// NaN and infinity are not valid JSON. Emitting them bare produces a document
// no parser accepts, so they become null.
CALIB_TEST(Json, NonFiniteBecomesNull) {
  std::ostringstream out;
  json::Writer w(out);
  w.beginArray();
  w.value(std::nan(""));
  w.value(std::numeric_limits<double>::infinity());
  w.value(1.0);
  w.endArray();
  const std::string s = out.str();
  EXPECT_TRUE(s.find("nan") == std::string::npos);
  EXPECT_TRUE(s.find("inf") == std::string::npos);

  std::size_t nulls = 0;
  for (std::size_t i = s.find("null"); i != std::string::npos;
       i = s.find("null", i + 4)) {
    ++nulls;
  }
  EXPECT_EQ(nulls, std::size_t(2));
  // The finite value is untouched.
  EXPECT_TRUE(s.find("1") != std::string::npos);
}

CALIB_TEST(Json, EscapesStrings) {
  std::ostringstream out;
  json::Writer w(out);
  w.beginObject();
  w.keyValue("path", std::string("a\"b\\c\nd\te"));
  w.endObject();
  const std::string s = out.str();
  EXPECT_TRUE(s.find("\\\"") != std::string::npos);
  EXPECT_TRUE(s.find("\\\\") != std::string::npos);
  EXPECT_TRUE(s.find("\\n") != std::string::npos);
  EXPECT_TRUE(s.find("\\t") != std::string::npos);
  EXPECT_TRUE(s.find('\n') != std::string::npos);  // pretty-printing newlines
}

CALIB_TEST(Json, EmptyContainers) {
  std::ostringstream out;
  json::Writer w(out);
  w.beginObject();
  w.key("empty");
  w.beginArray();
  w.endArray();
  w.endObject();
  EXPECT_EQ(out.str(), std::string("{\n  \"empty\": []\n}"));
}
