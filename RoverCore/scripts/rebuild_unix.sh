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
cmd ninja && ninja --version || echo "ninja: (not found, will fallback)"

# --- Choose generator ---
GEN="Unix Makefiles"
if cmd ninja; then GEN="Ninja"; fi
echo "[*] Generator: ${GEN}"

# --- Clean build cache completely (avoid stale CMAKE_MAKE_PROGRAM) ---
echo "[*] Nuking ${ROOT_DIR}/build"
rm -rf "${ROOT_DIR}/build"
mkdir -p "${BUILD_DIR}"

# --- Configure ---
echo "[*] Configuring (${BUILD_TYPE})"
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
  -G "${GEN}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_C_COMPILER=/usr/bin/gcc \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++

# --- Build ---
JOBS="$(nproc || echo 2)"
echo "[*] Building (-j${JOBS})"
if [ "${GEN}" = "Ninja" ]; then
  cmake --build "${BUILD_DIR}" -j"${JOBS}" --verbose
else
  cmake --build "${BUILD_DIR}" -- -j"${JOBS}" VERBOSE=1
fi

# --- Done ---
BIN="${BUILD_DIR}/bin/rover_core"
echo "[*] Binary: ${BIN}"
[ -x "${BIN}" ] && ls -lh "${BIN}" || true
