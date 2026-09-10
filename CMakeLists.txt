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

if(SLAMCPP_SUPERPOINT_ENABLED)
    target_compile_definitions(slamcpp PUBLIC SLAMCPP_WITH_SUPERPOINT=1)
    target_link_libraries(slamcpp PUBLIC ${TORCH_LIBRARIES})
endif()

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
endif()
