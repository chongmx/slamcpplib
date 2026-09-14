# ---------------------------------------------------------------------------
# slamcpp - a unified visual-inertial SLAM library
#
# One library, one target, one set of headers. It descends from ORB_SLAM3 v1.0
# and is being merged with the SuperPoint fork (SP_SLAM3) so that both feature
# front-ends live in a single build and are chosen at run time.
#
# Deliberately absent:
#
#   Pangolin  the Viewer and MapDrawer are not part of this library. Rendering
#             belongs to the application, which already draws the map itself.
#   glog      nothing in the sources ever called it; only a stale
#             find_package(Glog) referred to it.
#   DBoW2     replaced outright by DBoW3, which carries the descriptor type in
#             the vocabulary file instead of fixing it in template arguments.
#             That is what lets one build serve both front-ends. See
#             include/Vocabulary.h.
#
#   Boost     the Atlas save/load path used Boost.Serialization. It is
#             replaced by include/slamcpp/serialization/Archive.h, which is
#             about 600 lines of C++17 and needs no tracking table, because the
#             core already flattens its object graph to integer ids in
#             PreSave() and restores the pointers in PostLoad().
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.12)
project(slamcpp CXX C)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# SuperPoint front-end. libtorch is optional: without it the library still
# builds and the ORB front-end still works, so a machine that does not have
# libtorch is not shut out of the project.
# ---------------------------------------------------------------------------
option(SLAMCPP_WITH_SUPERPOINT "Build the SuperPoint front-end (needs libtorch)" ON)

set(SLAMCPP_SUPERPOINT_ENABLED OFF)
if(SLAMCPP_WITH_SUPERPOINT)
    # libtorch 2.7 still asks for CUDA::nvToolsExt, which the CUDA 13 toolkit
    # removed when NVTX moved to the header-only NVTX3. Without this shim the
    # generate step fails outright on a machine with a newer toolkit than the
    # one libtorch was built against. NVTX carries profiler annotations only,
    # so an empty interface target costs nothing at run time.
    find_package(CUDAToolkit QUIET)
    if(NOT TARGET CUDA::nvToolsExt)
        add_library(CUDA::nvToolsExt INTERFACE IMPORTED)
    endif()

    find_package(Torch QUIET)
    if(Torch_FOUND)
        set(SLAMCPP_SUPERPOINT_ENABLED ON)
        message(STATUS "slamcpp: SuperPoint front-end enabled (libtorch found)")
    else()
        message(STATUS "slamcpp: libtorch not found, building the ORB front-end only")
    endif()
endif()

# ---------------------------------------------------------------------------
# Resources.
#
# The SuperPoint vocabulary is committed in chunks because it is over GitHub's
# 100 MB per-file limit. Join it here so a fresh clone needs no manual step.
# ---------------------------------------------------------------------------
option(SLAMCPP_JOIN_VOCABULARY
       "Reassemble the chunked SuperPoint vocabulary at configure time" ON)

if(SLAMCPP_JOIN_VOCABULARY)
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/JoinVocabulary.cmake)
    slamcpp_join_vocabulary(
        ${CMAKE_CURRENT_SOURCE_DIR}/resources/vocabulary/superpoint_voc.yml.gz)
endif()

# ---------------------------------------------------------------------------
# Vendored dependencies.
#
# Both are built as ordinary in-tree targets and linked by target name. The
# upstream trees instead produced a .so into a fixed lib/ directory and were
# linked through a hardcoded path, which left CMake unable to infer any build
# order: a clean tree failed with "No rule to make target .../libg2o.so"
# because nothing knew how to produce the file it was being asked to link.
# ---------------------------------------------------------------------------
configure_file(3rdparty/g2o/config.h.in
               ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/g2o/config.h)

file(GLOB_RECURSE G2O_SOURCES CONFIGURE_DEPENDS 3rdparty/g2o/g2o/*.cpp)
add_library(slamcpp_g2o STATIC ${G2O_SOURCES})
target_include_directories(slamcpp_g2o SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/g2o
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/g2o/g2o/core
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/g2o/g2o/types
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/g2o/g2o/stuff
    ${EIGEN3_INCLUDE_DIR}
)
set_target_properties(slamcpp_g2o PROPERTIES POSITION_INDEPENDENT_CODE ON)

file(GLOB DBOW3_SOURCES CONFIGURE_DEPENDS
     3rdparty/DBoW3/src/*.cpp
     3rdparty/DBoW3/src/*.c
     3rdparty/DBoW3/DUtils/*.cpp)
add_library(slamcpp_dbow3 STATIC ${DBOW3_SOURCES})
target_include_directories(slamcpp_dbow3 SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/DBoW3
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(slamcpp_dbow3 PUBLIC ${OpenCV_LIBS})
set_target_properties(slamcpp_dbow3 PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Neither tree is ours to clean up, and between them they emit about a
# thousand warnings that would bury anything the core has to say.
target_compile_options(slamcpp_g2o PRIVATE -w)
target_compile_options(slamcpp_dbow3 PRIVATE -w)

# ---------------------------------------------------------------------------
# The library itself
# ---------------------------------------------------------------------------
file(GLOB SLAMCPP_SOURCES CONFIGURE_DEPENDS src/*.cc src/*.cpp src/CameraModels/*.cpp)

# The SuperPoint front-end. Every file under src/features needs libtorch, so
# the whole directory is conditional and the ORB path is unaffected by its
# absence.
file(GLOB SLAMCPP_SUPERPOINT_SOURCES CONFIGURE_DEPENDS src/features/*.cc)
if(SLAMCPP_SUPERPOINT_ENABLED)
    list(APPEND SLAMCPP_SOURCES ${SLAMCPP_SUPERPOINT_SOURCES})
endif()

add_library(slamcpp STATIC ${SLAMCPP_SOURCES})

# The root of the tree is on the path because the public headers include their
# vendored dependencies by repo-relative path, as in
# "3rdparty/DBoW3/src/BowVector.h".
# SYSTEM, so that a consumer building with -Wall -Wextra hears about its own
# code and not about this one. The core is inherited and still emits plenty of
# sign-compare and unused-parameter noise, and it drags in g2o, Sophus and
# deprecated Eigen behind it.
target_include_directories(slamcpp SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories(slamcpp SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/Sophus
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(slamcpp
    PUBLIC
        slamcpp_g2o
        slamcpp_dbow3
        ${OpenCV_LIBS}
        Threads::Threads
    PRIVATE
        OpenSSL::Crypto
)

target_compile_features(slamcpp PUBLIC cxx_std_17)
set_target_properties(slamcpp PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Calibration module
#
# A port of ethz-asl/kalibr into this library, so that one target covers both
# calibrating a rig and tracking with it. See docs/kalibr/ at the repository
# root for the analysis and the phased plan.
#
# Built as its own static library rather than folded into the slamcpp glob,
# for two reasons. It is new code held to -Werror, which the inherited SLAM
# core could not survive. And its layers land over several phases, so listing
# sources explicitly keeps a half-finished layer from being picked up by a
# glob before it compiles.
#
# It is still linked into slamcpp PUBLIC, so a consumer sees one target.
# ---------------------------------------------------------------------------
option(SLAMCPP_BUILD_CALIBRATION "Build the calibration module" ON)
option(SLAMCPP_CALIB_WITH_SUITESPARSE
       "Use SuiteSparse/CHOLMOD for the calibration sparse solves" OFF)

if(SLAMCPP_BUILD_CALIBRATION)
    # Phase 1 adds kinematics, linalg, backend and expressions; Phase 2 adds
    # splines, cameras, targets, apriltag and imu; Phase 5 adds pipelines.
    add_library(slamcpp_calib STATIC
        src/calibration/core/Image.cpp
        src/calibration/core/Logging.cpp
        src/calibration/core/Random.cpp
        src/calibration/core/Statistics.cpp
        src/calibration/core/Timestamp.cpp
        src/calibration/cameras/PinholeHelpers.cpp
        src/calibration/cameras/PinholeInitializer.cpp
    )

    # Not SYSTEM, unlike the includes on slamcpp itself: this is our code and
    # we want to hear its warnings.
    target_include_directories(slamcpp_calib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_include_directories(slamcpp_calib SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
    target_compile_features(slamcpp_calib PUBLIC cxx_std_17)
    set_target_properties(slamcpp_calib PROPERTIES POSITION_INDEPENDENT_CODE ON)

    # Warnings are errors in the module. Ported estimation code that trips a
    # real warning is a bug worth stopping for, and enforcing this from the
    # first file is far cheaper than retrofitting it across the ~45,000 lines
    # the plan carries across.
    if(MSVC)
        target_compile_options(slamcpp_calib PRIVATE /W4)
    else()
        target_compile_options(slamcpp_calib PRIVATE
            -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Werror)
    endif()

    if(SLAMCPP_CALIB_WITH_SUITESPARSE)
        # Phase 1 of the plan keeps CHOLMOD to preserve numerical equivalence
        # with upstream. Phase 3 adds an Eigen SimplicialLDLT solver behind the
        # same interface and makes it the default, leaving this as an opt-in so
        # discrepancies can be bisected against the original numerics.
        find_package(SuiteSparse REQUIRED)
        target_link_libraries(slamcpp_calib PUBLIC SuiteSparse::CHOLMOD)
        target_compile_definitions(slamcpp_calib PUBLIC SLAMCPP_CALIB_WITH_SUITESPARSE=1)
    endif()

    target_link_libraries(slamcpp PUBLIC slamcpp_calib)

    # Replays upstream's recorded detections through the ported code and
    # compares the result. Not a unit test: it needs a trace captured from
    # upstream Kalibr in Docker, which is far too large to commit.
    add_executable(calib_replay_focal tools/calib_replay_focal.cpp)
    target_link_libraries(calib_replay_focal PRIVATE slamcpp_calib)
endif()

if(SLAMCPP_SUPERPOINT_ENABLED)
    target_compile_definitions(slamcpp PUBLIC SLAMCPP_WITH_SUPERPOINT=1)
    # --no-as-needed around the CUDA half on purpose. Nothing in this library
    # names a symbol in libtorch_cuda directly: the GPU backend registers
    # itself from a static initialiser. With the default --as-needed the linker
    # drops the library, the registration never runs, and
    # torch::cuda::is_available() answers false on a machine with a working
    # GPU, quietly putting SuperPoint on the CPU.
    target_link_libraries(slamcpp PUBLIC
        -Wl,--no-as-needed ${TORCH_LIBRARIES} -Wl,--as-needed)

    # libtorch is built against the C++11 ABI in the cxx11-abi distribution and
    # needs its own flags; TORCH_CXX_FLAGS carries them.
    if(TORCH_CXX_FLAGS)
        target_compile_options(slamcpp PUBLIC ${TORCH_CXX_FLAGS})
    endif()
endif()

# Where the shipped weights and vocabularies live, so nothing has to be started
# from a particular working directory to find them.
target_compile_definitions(slamcpp PUBLIC
    SLAMCPP_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

# The core is inherited code that we are not going to re-warning-clean in one
# pass. Turned back on deliberately once it is ours.
target_compile_options(slamcpp PRIVATE -w)

# ---------------------------------------------------------------------------
# Tests
#
# Off by default so that consuming projects, which add this library through
# add_subdirectory, do not build them. Turn on with -DSLAMCPP_BUILD_TESTS=ON.
# ---------------------------------------------------------------------------
option(SLAMCPP_BUILD_TESTS "Build slamcpp's tests" OFF)

if(SLAMCPP_BUILD_TESTS)
    enable_testing()

    add_executable(test_vocabulary tests/test_vocabulary.cpp)
    target_link_libraries(test_vocabulary PRIVATE slamcpp)

    # Needs the extracted vocabulary, which is large, so the path is passed in
    # rather than discovered. Skipped by CTest when it is not there.
    set(SLAMCPP_TEST_VOCABULARY "${CMAKE_BINARY_DIR}/ORBvoc.txt"
        CACHE FILEPATH "ORBvoc.txt used by the vocabulary test")
    add_test(NAME vocabulary
             COMMAND test_vocabulary "${SLAMCPP_TEST_VOCABULARY}")

    add_executable(test_tracking tests/test_tracking.cpp)
    target_link_libraries(test_tracking PRIVATE slamcpp ${OpenCV_LIBS})
    target_include_directories(test_tracking PRIVATE tests)

    set(SLAMCPP_TEST_SETTINGS "${CMAKE_SOURCE_DIR}/config/webcam_mono.yaml"
        CACHE FILEPATH "camera settings used by the tracking test")
    add_test(NAME tracking
             COMMAND test_tracking "${SLAMCPP_TEST_VOCABULARY}" "${SLAMCPP_TEST_SETTINGS}")

    add_executable(test_serialization tests/test_serialization.cpp)
    target_link_libraries(test_serialization PRIVATE slamcpp ${OpenCV_LIBS})
    target_include_directories(test_serialization PRIVATE tests)
    add_test(NAME serialization
             COMMAND test_serialization "${SLAMCPP_TEST_VOCABULARY}" "${SLAMCPP_TEST_SETTINGS}")

    add_executable(test_features tests/test_features.cpp)
    target_link_libraries(test_features PRIVATE slamcpp ${OpenCV_LIBS})
    target_include_directories(test_features PRIVATE tests)
    add_test(NAME features COMMAND test_features "${SLAMCPP_TEST_VOCABULARY}")

    # Calibration module. Self-contained: no vocabulary, no dataset, no
    # fixture, so it runs anywhere the library builds. The harness is
    # in-tree and header-only (tests/calibration/TestHarness.hpp); upstream
    # Kalibr uses gtest via catkin, and fetching it here would make an
    # offline build impossible and add a dependency the library does not need.
    if(SLAMCPP_BUILD_CALIBRATION)
        add_executable(test_calibration
            tests/calibration/test_main.cpp
            tests/calibration/test_core.cpp
            tests/calibration/test_io.cpp
        )
        target_link_libraries(test_calibration PRIVATE slamcpp_calib)
        target_include_directories(test_calibration
                                   PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests/calibration)
        if(NOT MSVC)
            target_compile_options(test_calibration PRIVATE
                -Wall -Wextra -Wpedantic -Werror)
        endif()
        add_test(NAME calibration COMMAND test_calibration)
    endif()
endif()

# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------
option(SLAMCPP_BUILD_TOOLS "Build slamcpp's command line tools" OFF)

if(SLAMCPP_BUILD_TOOLS)
    add_executable(convert_vocabulary tools/convert_vocabulary.cpp)
    target_link_libraries(convert_vocabulary PRIVATE slamcpp)
endif()
