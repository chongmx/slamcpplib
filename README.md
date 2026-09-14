# slamcpp

A unified visual-inertial SLAM library.

It descends from ORB_SLAM3 v1.0 and is being merged with the SuperPoint fork
(SP_SLAM3) so that both feature front-ends live in one build and are chosen at
run time rather than at compile time.

## Layout

    include/slamcpp/     public headers, included as <slamcpp/System.h>
    include/slamcpp/calibration/
                         the calibration module, namespace slamcpp::calib
    src/                 implementation
    3rdparty/            vendored dependencies: DBoW3, g2o, Sophus
    resources/           runtime assets: vocabularies and model weights
    tests/               build with -DSLAMCPP_BUILD_TESTS=ON, run with ctest
    docs/                design and porting notes
    examples/            (empty for now)
    tools/               convert_vocabulary

## Using it

    add_subdirectory(3rdparty/slamcpplib)
    target_link_libraries(your_app PRIVATE slamcpp)

That is the whole integration. Include paths, the C++ standard and the vendored
g2o and DBoW3 all come across as usage requirements. Calibration comes with it:
`slamcpp_calib` is linked `PUBLIC` into `slamcpp`, so one target covers both
calibrating a rig and tracking with it.

## Calibration

A port of [Kalibr](https://github.com/ethz-asl/kalibr) into this library, so
that a rig can be calibrated and then tracked without leaving it or converting
a configuration file between two conventions. Everything lives in namespace
`slamcpp::calib`, under `include/slamcpp/calibration/`.

    #include <slamcpp/calibration/Calibration.h>

**This is a port in progress and cannot calibrate anything yet.** The `core/`
layer is done: asserts, logging, time, random sampling, statistics, a
dependency-free image type and a finite-difference helper. The camera models,
target detectors, splines, optimiser and pipelines are not written. See
`docs/kalibr/08-integration-status.md` at the repository root for what is
built, what is blocked, and the phase plan.

Build options:

    SLAMCPP_BUILD_CALIBRATION       build the module at all           (ON)
    SLAMCPP_CALIB_WITH_SUITESPARSE  use CHOLMOD for the sparse solves (OFF)

The module depends on Eigen alone. It does not use the `ORB_SLAM3` namespace
the inherited SLAM core still carries, and its core stays free of OpenCV behind
its own `Image` type, so the camera models and error terms carry no
image-processing dependency.

## Dependencies

Required: OpenCV, Eigen 3, OpenSSL, a threads library.
Vendored: DBoW3, g2o, Sophus.
Optional: libtorch, which enables the SuperPoint front-end, LightGlue and
learned place recognition. Without it the library still builds and the ORB
front-end still works.

Deliberately absent:

- **Pangolin.** The Viewer and MapDrawer are not part of this library.
  Rendering belongs to the application.
- **glog.** Nothing ever called it. Only a stale `find_package(Glog)` referred
  to it.
- **DBoW2.** Replaced outright by DBoW3.
- **Boost.** Atlas save and load used Boost.Serialization. It is replaced by
  `include/slamcpp/serialization/`, which keeps all three of Boost's formats.

See `docs/porting-notes.md` for the reasoning behind both.

## Feature front-ends

Two, selected in the settings file:

    Frontend.type: "ORB"          # or "SUPERPOINT"
    Frontend.modelPath: ""        # optional; defaults to resources/models

ORB is the classic 32-byte binary descriptor compared with a Hamming distance.
SuperPoint is a learned 256-float descriptor compared with an L2 norm. The
matcher picks its distance from the descriptor itself and its thresholds from
the active front-end, so the two cannot be crossed by accident.

Each needs a vocabulary built from its own descriptor type. ORB uses the
classic `ORBvoc.txt`; SuperPoint needs the float tree in `resources/vocabulary`.
Convert that one to DBoW3's binary form before using it, or start-up takes
minutes rather than seconds:

    convert_vocabulary superpoint_voc.yml.gz superpoint_voc.dbow3

SuperPoint is only present when the library was built against libtorch:

    cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/libtorch

Without it the library still builds and ORB still works; asking for SuperPoint
then fails with a clear message rather than something obscure.

### Optional learned components

Two more parts of the SuperPoint fork are built in but inactive until you point
them at weights, which this repository does not ship:

    LightGlue.model_path: "lightglue.pt"
    PlaceRecognition.model_path: "netvlad.pt"

LightGlue replaces the brute-force search during monocular initialisation.
Place recognition adds candidates to loop detection alongside the bag of words
rather than replacing it. Both degrade silently to the classic paths when no
model is given or the file will not load.

## Atlas files

The atlas is saved and loaded in one of three formats, chosen in the settings
file:

    System.SaveAtlasToFile: "session"     # writes ./session.osa
    System.LoadAtlasFromFile: "session"
    System.AtlasFormat: "binary"          # binary (default), text or xml
    System.AtlasSaveFormat: "xml"         # optional, overrides the write format

`binary` is compact and is the default. `text` is indented and diffable. `xml`
is for handing the state to something else. Setting `AtlasSaveFormat` on its own
converts: read a binary atlas, write an XML one.

Text and XML carry values rather than the machine's representation of them, so
they move between builds and architectures. Binary does not, and refuses a file
whose type widths disagree with the running build. Floating point is written
with enough digits to come back bit-for-bit in every format.

## Resources

    resources/models/superpoint.pt              SuperPoint weights
    resources/vocabulary/ORBvoc.txt.tar.gz      ORB vocabulary, DBoW3 text format
    resources/vocabulary/superpoint_voc.yml.gz  SuperPoint vocabulary, DBoW3 YAML

These total about 163 MB. The SuperPoint vocabulary is 117 MB, past GitHub's
100 MB per-file limit, so it is committed as 45 MiB chunks named
`superpoint_voc.yml.gz.part00` and upward.

Nothing needs doing by hand. `cmake/JoinVocabulary.cmake` runs at configure
time and reassembles the archive, so a fresh clone builds as it always did.
The join is skipped once the file is in place, and needs CMake 3.18 or newer
for `cmake -E cat`. Turn it off with `-DSLAMCPP_JOIN_VOCABULARY=OFF`.

For a checkout that is not being configured, the same job by hand:

    ./scripts/join_vocabulary.sh

Both read the expected digest from `superpoint_voc.yml.gz.sha256` and refuse
to leave a file that does not match it. Both write to a temporary and rename,
so an interrupted run cannot leave a truncated archive that later looks whole.
The joined file is gitignored and will not be committed back.

To re-split after replacing the vocabulary:

    cd resources/vocabulary
    sha256sum superpoint_voc.yml.gz > superpoint_voc.yml.gz.sha256
    rm -f superpoint_voc.yml.gz.part*
    split -b 45M -d superpoint_voc.yml.gz superpoint_voc.yml.gz.part

## Tests

    cmake -S . -B build -DSLAMCPP_BUILD_TESTS=ON
    cmake --build build -j3
    ctest --test-dir build --output-on-failure

`vocabulary` checks that DBoW3 reads the shipped ORB vocabulary and that the
transform and scoring still behave. `tracking` brings up a System on synthetic
imagery and checks that monocular initialisation converges. `serialization`
round-trips an Atlas through the archive and checks the restored graph holds no
null references.
