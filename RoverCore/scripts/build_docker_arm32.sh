#!/usr/bin/env bash
set -euo pipefail

# Locate repo root (where this script lives) and parent (which contains proto/)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"         # RoverCore/
PARENT="$(cd "$ROOT/.." && pwd)"       # parent containing proto/ and RoverCore/
IMAGE_TAG="rovercore:rpi3-builder"

echo "Repo root:      $ROOT"
echo "Build context:  $PARENT"

# Ensure buildx (and QEMU emulation) is set up
docker buildx inspect >/dev/null 2>&1 || docker buildx create --use
# QEMU (one-time; harmless if already installed)
docker run --privileged --rm tonistiigi/binfmt --install all >/dev/null 2>&1 || true

# Build the builder stage for armv7 and load it locally
docker buildx build \
  --platform linux/arm/v7 \
  --target builder \
  -t "$IMAGE_TAG" \
  -f "$ROOT/Dockerfile" \
  --build-arg "PROJECT_SUBDIR=$(basename "$ROOT")" \
  --load \
  "$PARENT"

# Copy the binary out of the image
mkdir -p "$ROOT/out"
CID="$(docker create --platform=linux/arm/v7 "$IMAGE_TAG")"
docker cp "$CID":/src/build/bin/rover_core "$ROOT/out/rover_core_rpi3"
docker rm "$CID" >/dev/null

echo "✅ Built Pi 3 binary: $ROOT/out/rover_core_rpi3"
file "$ROOT/out/rover_core_rpi3" || true