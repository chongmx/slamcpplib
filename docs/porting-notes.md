# Porting notes

What changed relative to the two upstream trees, and why.

## The two ancestors are not a fork and its parent

`ORB_SLAM3_Fixed` is ORB_SLAM3 v1.0: it has `Settings`, `GeometricTools` and
`TwoViewReconstruction`. `SP_SLAM3` sits on an older base that still carries
`Initializer`, `PnPsolver`, `Random` and `Timestamp`. Between them `Optimizer`
differs by about 3,500 lines and `Tracking` by about 2,400.

v1.0 is the base here. The older files that v1.0 replaced are not carried over:
`TwoViewReconstruction` supersedes `Initializer`, and `MLPnPsolver` supersedes
`PnPsolver`.

## SuperPoint substitutes for ORB, it does not sit beside it

In SP_SLAM3 the SuperPoint front-end wears ORB's names. `SPextractor` is
typedef'd to `ORBextractor`, the vocabulary is typedef'd to a DBoW3 vocabulary,
and the ORB headers are deleted outright. Descriptor distance changes from an
integer Hamming distance to a float L2 distance.

Supporting both at run time therefore needs a real feature abstraction across
`Frame`, `KeyFrame`, `KeyFrameDatabase`, the matcher and the vocabulary. The
vocabulary half of that is already done, by the DBoW3 move below. The extractor
and matcher halves are not.

## DBoW2 is gone, permanently

DBoW2 fixed the descriptor as a 32-byte binary word in its template arguments.
Any second descriptor type meant a second instantiation of the whole core,
which is exactly what forced SP_SLAM3 into a compile-time substitution.

DBoW3 stores the descriptor type inside the vocabulary file and dispatches on
it at run time, so a binary ORB vocabulary and a float SuperPoint one are the
same C++ type. That is what makes a runtime front-end switch possible at all.

One asymmetry is worth knowing: DBoW3's text loader hardcodes `CV_8U`, so the
classic `ORBvoc.txt` still loads exactly as before, while a float vocabulary has
to be supplied in DBoW3's binary or YAML format.

Verified by `tests/test_vocabulary.cpp`: the shipped vocabulary loads with
971,814 words, branching factor 10, depth 6, descriptor type `CV_8U`, and the
transform and similarity score behave.

### Serialization

The DBoW2 fork this code grew up with had an intrusive `serialize()` member
bolted onto `BowVector` and `FeatureVector`, which `KeyFrame` relies on. Stock
DBoW3 has no such member. Both types derive from a `std::map` that Boost
already knows how to serialize, so the support lives in
`include/slamcpp/Vocabulary.h` as free functions and the vendored tree stays
pristine.

## Defects fixed while porting

**`LoopClosing::mnFullBAIdx` was a `bool`.** It is a generation counter:
`RunGlobalBundleAdjustment` snapshots it and bails out if it has moved by the
time the adjustment finishes. Incrementing a `bool` saturates at `true`, so
only the first abort was ever observed and every later one was silently missed.
It is an `int` now. This also happens to be what C++17 requires, since `++` on
`bool` was removed.

**`MLPnPsolver` used `DUtils::Random` without including it.** It had been
arriving by accident through a DBoW2 header. Now included explicitly.

**`System` called `exit(-1)` on a bad vocabulary path.** A library should not
kill the process. It raises now, which the application already catches.

## Build system

The upstream trees produced `libg2o.so` and `libDBoW2.so` into fixed `lib/`
directories and linked them through hardcoded paths, which left CMake no way to
derive a build order. A clean tree failed with `No rule to make target
.../libg2o.so`. Both are ordinary in-tree targets here, linked by target name.

The consuming project's CMakeLists lost, as a result: a separate DBoW2
subdirectory, a loop excluding 24 example executables one at a time, and
explicit `add_dependencies` calls working around the hardcoded paths.

The library builds as C++17. ORB_SLAM3 pinned itself to C++11 and broke under
C++17 solely because of the `bool` increment above.

## Boost is gone

Boost.Serialization was the last Boost dependency, carried for one feature:
Atlas save and load. `include/slamcpp/serialization/Archive.h` replaces it in
about 600 lines of C++17.

Most of Boost.Serialization's weight is pointer tracking, so that an object
reachable twice is written once and restored as one shared object. The core
never needed it. It flattens its own graph first: `PreSave()` rewrites every
cross-reference as an integer id and `PostLoad()` turns the ids back into
pointers, leaving a strict ownership tree where Atlas owns its maps and cameras,
each Map owns its key frames and map points, and every heap object is reached
exactly once. A tree needs no tracking table.

The archive deliberately mirrors the small part of Boost's API the core used,
so the eighteen existing `serialize()` bodies barely changed: `ar & x` chains,
`Archive::is_saving::value` picks a direction inside a shared body,
`make_array()` handles raw buffers, and a friend declaration of `access` reaches
private members. `SLAMCPP_REGISTER_POLYMORPHIC` replaces the `register_type<>()`
calls that let a `GeometricCamera*` come back as the concrete Pinhole or
KannalaBrandt8 it was.

### The three formats

Boost offered binary, text and XML archives, and all three are kept.
`Archive.h` walks the object graph and `Format.h` decides how the result is
spelled, so a format costs a backend rather than a second traversal.

The readers are much simpler than a parser normally has to be, for one reason:
saving and loading run the same serialize() bodies in the same order, so the
reader already knows what comes next. It never interprets the document, it only
pulls the next value out of it in order. Structural markers are there to make
the text and XML readable, and the readers step over them. The XML is written
and scanned by hand for the same reason; what is needed is a scanner, not a
document model.

Binary is host-native, as Boost's was, and records the type widths the writer
used so a mismatched build is rejected rather than misread. Text and XML carry
values instead of representations, so they move between builds and
architectures. Floating point is written with enough digits to come back
bit-for-bit in all three. Byte buffers, which is mostly key frame descriptors,
go out as one hex blob rather than a few hundred thousand separate values.

### Defects this uncovered

Atlas save and load had no test before, so writing one was part of the job.
`tests/test_serialization.cpp` found a family of crashes that had nothing to do
with the archive.

`Map::PreSave` writes only the key frames and map points that are not bad, but
`KeyFrame::PreSave`, `MapPoint::PreSave` and `Map::PreSave` itself decided
which references to record by asking whether the target was in the map's set,
which includes bad entries. A reference to a bad object was therefore written
as an id that no saved object backed.

On the way back in, `PostLoad` resolved those ids with `operator[]` on a
`std::map`, which inserts a null for a missing key and hands it back. The null
landed in the covisibility graph, and the first traversal dereferenced it.
`KeyFrame::UpdateBestCovisibles` crashed taking a mutex on it. Whether it
happened at all depended on thread timing, which is why it presented as an
intermittent segfault.

Both halves are fixed: PreSave now records a reference only if the target will
actually be written, and PostLoad resolves ids with `find()` and drops what it
cannot resolve, so a file written by the older code cannot crash the loader.

The test checks a fixed point rather than comparing against the live map,
because `Map::PreSave` mutates the very map it is about to write. Loading a
file, writing it again and loading that must produce identical results, which
isolates the serializer from the SLAM layer's own bookkeeping.

## Still to do

- Port the SuperPoint front-end onto the v1.0 core, behind the optional
  libtorch dependency, and add the extractor and matcher abstraction that makes
  the choice a runtime one.
- Calibration, replacing the ROS-era Kalibr workflow.
