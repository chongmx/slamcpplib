#!/usr/bin/env bash
# Reassemble resources/vocabulary/superpoint_voc.yml.gz from its committed parts.
# The archive is 117 MiB, over GitHub's 100 MB per-file limit, so it is stored
# split into 45 MiB chunks. Run this once after cloning.
set -euo pipefail

dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../resources/vocabulary" && pwd)"
out="$dir/superpoint_voc.yml.gz"
expected=ac555c1f385ea2bbb5a9e1fe287017c913d9cfe5c73f0b14b1b9790382a7761d

if [ -f "$out" ] && [ "$(sha256sum "$out" | cut -d' ' -f1)" = "$expected" ]; then
    echo "superpoint_voc.yml.gz already present and verified"
    exit 0
fi

shopt -s nullglob
parts=("$dir"/superpoint_voc.yml.gz.part*)
if [ ${#parts[@]} -eq 0 ]; then
    echo "error: no superpoint_voc.yml.gz.part* files in $dir" >&2
    exit 1
fi

cat "${parts[@]}" > "$out"

actual="$(sha256sum "$out" | cut -d' ' -f1)"
if [ "$actual" != "$expected" ]; then
    echo "error: checksum mismatch after join" >&2
    echo "  expected $expected" >&2
    echo "  actual   $actual" >&2
    rm -f "$out"
    exit 1
fi

echo "superpoint_voc.yml.gz reassembled and verified (${#parts[@]} parts)"
