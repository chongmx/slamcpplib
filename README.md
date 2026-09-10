# slamcpp

A unified visual-inertial SLAM library.

It descends from ORB_SLAM3 v1.0 and is being merged with the SuperPoint fork
(SP_SLAM3) so that both feature front-ends live in one build and are chosen at
run time rather than at compile time.

## Layout

    include/slamcpp/     public headers, included as <slamcpp/System.h>
    src/                 implementation
    3rdparty/            vendored dependencies: DBoW3, g2o, Sophus
    resources/           runtime assets: vocabularies and model weights
    tests/               build with -DSLAMCPP_BUILD_TESTS=ON, run with ctest
    docs/                design and porting notes
    examples/            (empty for now)
    tools/               (empty for now)

## Using it

    add_subdirectory(3rdparty/slamcpplib)
    target_link_libraries(your_app PRIVATE slamcpp)

That is the whole integration. Include paths, the C++ standard and the vendored
g2o and DBoW3 all come across as usage requirements.

## Dependencies

Required: OpenCV, Eigen 3, OpenSSL, a threads library.
Vendored: DBoW3, g2o, Sophus.
Optional: libtorch, which enables the SuperPoint front-end. Without it the
library still builds and the ORB front-end still works.

Deliberately absent:

- **Pangolin.** The Viewer and MapDrawer are not part of this library.
  Rendering belongs to the application.
- **glog.** Nothing ever called it. Only a stale `find_package(Glog)` referred
  to it.
- **DBoW2.** Replaced outright by DBoW3.
- **Boost.** Atlas save and load used Boost.Serialization. It is replaced by
  `include/slamcpp/serialization/`, which keeps all three of Boost's formats.

See `docs/porting-notes.md` for the reasoning behind both.

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
