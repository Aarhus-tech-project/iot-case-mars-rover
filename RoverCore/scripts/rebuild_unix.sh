#!/usr/bin/env bash
set -euo pipefail

# Usage: ./scripts/rebuild_unix.sh [Release|Debug]
BUILD_TYPE="${1:-Release}"
BUILD_DIR="build/${BUILD_TYPE}"

# Optional: quick helpers to install deps
# macOS:   brew install cmake ninja
# Debian/RPi: sudo apt update && sudo apt install -y build-essential cmake ninja-build

echo "[*] Nuking ${BUILD_DIR}"
rm -rf "${BUILD_DIR}"

echo "[*] Configuring (${BUILD_TYPE})"
cmake -S . -B "${BUILD_DIR}" -G Ninja -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

echo "[*] Building"
cmake --build "${BUILD_DIR}" -- -v

echo "[*] Binary:"
echo "    ${BUILD_DIR}/bin/rover_core"
