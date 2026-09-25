#!/usr/bin/env bash
# Applies this project's local changes to the two upstream submodules.
# Idempotent: safe to run again. Run once after `git clone --recurse-submodules`.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

git submodule update --init GraspGen SAM3

apply_patch() {  # <submodule> <patch>
  if git -C "$1" apply --reverse --check "$2" 2>/dev/null; then
    echo "$1: patch already applied"
  else
    git -C "$1" apply "$2"
    echo "$1: patch applied"
  fi
}

apply_patch GraspGen "$ROOT/patches/GraspGen/local-changes.patch"
apply_patch SAM3     "$ROOT/patches/SAM3/local-changes.patch"

# Files that only exist in this project (the ZMQ server/client and entry point).
for f in sam3_server sam3_client main.py; do
  [ -e "$ROOT/SAM3/$f" ] || cp -r "$ROOT/patches/SAM3/$f" "$ROOT/SAM3/$f"
done
echo "SAM3: sam3_server/, sam3_client/, main.py in place"
