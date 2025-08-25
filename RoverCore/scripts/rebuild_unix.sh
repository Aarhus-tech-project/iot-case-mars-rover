#!/usr/bin/env bash
set -euo pipefail

BUILD_TYPE="${1:-Release}"
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/${BUILD_TYPE}"

# --- Tool check ---
cmd() { command -v "$1" >/dev/null 2>&1; }
echo "[*] Tooling:"
cmake --version || true
cmd gcc  && gcc  --version | head -n1 || echo "gcc: MISSING"
cmd g++  && g++  --version | head -n1 || echo "g++: MISSING"
cmd make && make --version | head -n1 || echo "make: MISSING"

# --- Clean build cache completely ---
echo "[*] Nuking ${ROOT_DIR}/build"
rm -rf "${ROOT_DIR}/build"
mkdir -p "${BUILD_DIR}"

# --- Configure (force Unix Makefiles) ---
echo "[*] Configuring (${BUILD_TYPE})"
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
  -G "Unix Makefiles" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_C_COMPILER=/usr/bin/gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++

# --- Build ---
JOBS="$(nproc 2>/dev/null || echo 2)"
echo "[*] Building (-j${JOBS})"
cmake --build "${BUILD_DIR}" -- -j"${JOBS}" VERBOSE=1

# --- Done ---
BIN="${BUILD_DIR}/bin/rover_core"
echo "[*] Binary: ${BIN}"
[ -x "${BIN}" ] && ls -lh "${BIN}" || true