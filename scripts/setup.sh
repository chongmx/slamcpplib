#!/usr/bin/env bash
#
# Central setup. Prepares everything a fresh clone needs before it can build,
# test and run.
#
#   ./scripts/setup.sh               run every step
#   ./scripts/setup.sh --list        show the steps and exit
#   ./scripts/setup.sh --only vocab  run one step by name
#   ./scripts/setup.sh --skip orb    skip a step by name (repeatable)
#   ./scripts/setup.sh -j 4          parallel compile jobs (default: MAX_JOBS)
#   ./scripts/setup.sh --force       redo steps even if their output looks current
#
# This is the one entry point. Each step is a function below, and the ones that
# already have a standalone script delegate to it rather than duplicating it,
# so scripts/join_vocabulary.sh keeps working on its own for anyone who wants
# just that piece.
#
# Everything here is generated rather than tracked, because git cannot
# reasonably carry it:
#
#   superpoint_voc.yml.gz   117 MB, over GitHub's 100 MB per-file limit, so it
#                           is committed as 45 MiB chunks and rejoined here.
#   superpoint_voc.dbow3    117 MB, DBoW3's binary form of the above. Also over
#                           the limit, and derived from it, so tracking it would
#                           store the same vocabulary twice. It is what makes
#                           SuperPoint start in seconds rather than minutes.
#   ORBvoc.txt              ~140 MB extracted; tracked compressed, unpacked here.
#
# Safe to re-run: every step is skipped when its output is present and valid.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VOCAB="$ROOT/resources/vocabulary"
BUILD_DIR="${SLAMCPP_BUILD_DIR:-$ROOT/build}"

# Deliberately well below nproc. A full build here is memory hungry and the
# machine should stay usable while it runs.
MAX_JOBS=3
JOBS="$MAX_JOBS"
FORCE=0
ONLY=""
SKIP=()

# Step name, then one-line description. Order is the execution order, and the
# later steps depend on the earlier ones.
STEPS=(
    "vocab:rejoin the chunked SuperPoint vocabulary"
    "orb:extract the ORB vocabulary"
    "tools:build convert_vocabulary"
    "dbow3:convert the SuperPoint vocabulary to DBoW3 binary"
)

usage() { sed -n '3,10p' "$0" | sed 's/^#\ \?//'; }

list_steps() {
    echo "steps, in order:"
    local entry
    for entry in "${STEPS[@]}"; do
        printf '  %-8s %s\n' "${entry%%:*}" "${entry#*:}"
    done
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --force)   FORCE=1; shift ;;
        --only)    ONLY="${2:?--only needs a step name}"; shift 2 ;;
        --skip)    SKIP+=("${2:?--skip needs a step name}"); shift 2 ;;
        --list)    list_steps; exit 0 ;;
        -j)        JOBS="${2:?-j needs a number}"; shift 2 ;;
        -j*)       JOBS="${1#-j}"; shift ;;
        -h|--help) usage; exit 0 ;;
        *)         echo "unknown option: $1 (try --help)" >&2; exit 1 ;;
    esac
done

step_banner() { printf '\n==> %s\n' "$1"; }

should_run() {
    local name="$1" s
    if [[ -n "$ONLY" && "$ONLY" != "$name" ]]; then return 1; fi
    for s in ${SKIP[@]+"${SKIP[@]}"}; do
        [[ "$s" == "$name" ]] && return 1
    done
    return 0
}

# ---------------------------------------------------------------------------
# vocab: rejoin the chunked SuperPoint vocabulary.
#
# Delegates to scripts/join_vocabulary.sh, which verifies the result against a
# committed SHA-256 and refuses to leave a file that does not match.
#
# cmake/JoinVocabulary.cmake does the same at configure time, so configuring is
# enough on its own. It is repeated here because setup has to work before any
# configure has happened, and because the dbow3 step below needs the joined
# file to exist first.
# ---------------------------------------------------------------------------
do_vocab() {
    step_banner "rejoining the SuperPoint vocabulary"
    [[ "$FORCE" -eq 1 ]] && rm -f "$VOCAB/superpoint_voc.yml.gz"
    "$ROOT/scripts/join_vocabulary.sh"
}

# ---------------------------------------------------------------------------
# orb: extract the ORB vocabulary.
#
# Small enough to track compressed, so only the extraction happens here. It
# lands in the build directory because the tests take its path as an argument
# and CMake defaults that to ${CMAKE_BINARY_DIR}/ORBvoc.txt.
# ---------------------------------------------------------------------------
do_orb() {
    step_banner "extracting the ORB vocabulary"
    mkdir -p "$BUILD_DIR"
    if [[ -s "$BUILD_DIR/ORBvoc.txt" && "$FORCE" -eq 0 ]]; then
        echo "ORBvoc.txt already present"
        return
    fi
    tar xzf "$VOCAB/ORBvoc.txt.tar.gz" -C "$BUILD_DIR"
    echo "extracted to $BUILD_DIR/ORBvoc.txt"
}

# ---------------------------------------------------------------------------
# tools: build the converter.
#
# Only this target, so setup is not a full build. It still compiles slamcpp,
# which it links against.
# ---------------------------------------------------------------------------
do_tools() {
    step_banner "building convert_vocabulary"
    if [[ -x "$BUILD_DIR/convert_vocabulary" && "$FORCE" -eq 0 ]]; then
        echo "convert_vocabulary already built"
        return
    fi
    cmake -S "$ROOT" -B "$BUILD_DIR" -DSLAMCPP_BUILD_TOOLS=ON >/dev/null
    cmake --build "$BUILD_DIR" -j"$JOBS" --target convert_vocabulary
}

# ---------------------------------------------------------------------------
# dbow3: convert the SuperPoint vocabulary to DBoW3's binary form.
#
# Reading the gzipped YAML goes through cv::FileStorage and takes minutes. The
# same tree in DBoW3's own format loads in seconds.
# ---------------------------------------------------------------------------
do_dbow3() {
    step_banner "converting the SuperPoint vocabulary to .dbow3"
    local out="$VOCAB/superpoint_voc.dbow3"
    if [[ -s "$out" && "$FORCE" -eq 0 ]]; then
        echo "superpoint_voc.dbow3 already present (use --force to regenerate)"
        return
    fi
    if [[ ! -x "$BUILD_DIR/convert_vocabulary" ]]; then
        echo "error: convert_vocabulary not built; run without --only, or --only tools first" >&2
        exit 1
    fi
    "$BUILD_DIR/convert_vocabulary" "$VOCAB/superpoint_voc.yml.gz" "$out"
}

if [[ -n "$ONLY" ]]; then
    if ! printf '%s\n' "${STEPS[@]%%:*}" | grep -qx "$ONLY"; then
        echo "unknown step: $ONLY" >&2
        list_steps >&2
        exit 1
    fi
fi

for entry in "${STEPS[@]}"; do
    name="${entry%%:*}"
    should_run "$name" && "do_$name"
done

printf '\nSetup complete.\n\n'
printf '  %s\n' "$VOCAB/superpoint_voc.yml.gz"
printf '  %s\n' "$VOCAB/superpoint_voc.dbow3"
[[ -s "$BUILD_DIR/ORBvoc.txt" ]] && printf '  %s\n' "$BUILD_DIR/ORBvoc.txt"

cat <<'TAIL'

None of these are tracked by git. The two SuperPoint files are over GitHub's
100 MB limit, and all of them are reproducible from what is tracked.

Build and test with:

  cmake -S . -B build -DSLAMCPP_BUILD_TESTS=ON
  cmake --build build -j3
  ctest --test-dir build --output-on-failure
TAIL
