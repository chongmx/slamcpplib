#!/usr/bin/env bash
# Reassemble resources/vocabulary/superpoint_voc.yml.gz from its committed parts.
#
# The archive is 117 MiB, over GitHub's 100 MB per-file limit, so it is stored
# split into 45 MiB chunks. Run this once after cloning.
#
# The build does this for you: cmake/JoinVocabulary.cmake runs at configure
# time. This script is for people who are not configuring a build. Both read
# the expected digest from the .sha256 sidecar, so it lives in one place.
set -euo pipefail

dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../resources/vocabulary" && pwd)"
out="$dir/superpoint_voc.yml.gz"
sidecar="$out.sha256"

if [ ! -f "$sidecar" ]; then
    echo "error: missing $sidecar" >&2
    exit 1
fi
expected="$(awk '{print $1; exit}' "$sidecar")"

if [ -f "$out" ] && [ "$(sha256sum "$out" | cut -d' ' -f1)" = "$expected" ]; then
    echo "superpoint_voc.yml.gz already present and verified"
    exit 0
fi

shopt -s nullglob
parts=()
for p in "$out".part*; do
    case "$p" in *.sha256) continue ;; esac
    parts+=("$p")
done

if [ ${#parts[@]} -eq 0 ]; then
    echo "error: no superpoint_voc.yml.gz.part* files in $dir" >&2
    exit 1
fi

# Write to a temporary and move it into place, so an interrupted run cannot
# leave a truncated archive that later looks joined.
cat "${parts[@]}" > "$out.tmp"

actual="$(sha256sum "$out.tmp" | cut -d' ' -f1)"
if [ "$actual" != "$expected" ]; then
    echo "error: checksum mismatch after join" >&2
    echo "  expected $expected" >&2
    echo "  actual   $actual" >&2
    rm -f "$out.tmp"
    exit 1
fi

mv "$out.tmp" "$out"
echo "superpoint_voc.yml.gz reassembled and verified (${#parts[@]} parts)"
