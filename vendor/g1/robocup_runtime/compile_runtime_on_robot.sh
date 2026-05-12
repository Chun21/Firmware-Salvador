#!/usr/bin/env bash
set -Eeuo pipefail

# Build the G1 perception/localization runtime vendored inside Firmware-Salvador.
# This script is intended to run on the Unitree G1 image, where RealSense,
# TensorRT, CUDA, OpenCV, yaml-cpp and unitree_sdk2 are available.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS="${JOBS:-$(nproc)}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
UNITREE_SDK2_ROOT="${UNITREE_SDK2_ROOT:-$ROOT_DIR/unitree_sdk2}"

export LD_LIBRARY_PATH="$ROOT_DIR/g1_comp_servo_service/lib/arm:${LD_LIBRARY_PATH:-}"
if [[ -d "$UNITREE_SDK2_ROOT" ]]; then
  export UNITREE_SDK2_ROOT
  export LD_LIBRARY_PATH="$UNITREE_SDK2_ROOT/lib/aarch64:$UNITREE_SDK2_ROOT/thirdparty/lib/aarch64:$LD_LIBRARY_PATH"
fi

build_cmake_dir() {
  local name="$1"
  local src_dir="$2"
  local build_dir="$src_dir/build"

  echo "[g1-runtime-build] build $name"
  local cmake_args=(-DCMAKE_BUILD_TYPE="$BUILD_TYPE")
  if [[ -d "${UNITREE_SDK2_ROOT:-}" ]]; then
    cmake_args+=(-DUNITREE_SDK2_ROOT="$UNITREE_SDK2_ROOT")
  fi
  cmake -S "$src_dir" -B "$build_dir" "${cmake_args[@]}"
  cmake --build "$build_dir" -j"$JOBS"
}

require_source() {
  if [[ ! -d "$1" ]]; then
    echo "[g1-runtime-build][ERROR] missing source dir: $1" >&2
    exit 1
  fi
}

require_source "$ROOT_DIR/g1_comp_servo_service"
require_source "$ROOT_DIR/football_detectcpp"
require_source "$ROOT_DIR/robocup_locator_v1.1"
require_source "$ROOT_DIR/roboCup_sdk/src/common"

build_cmake_dir "g1_comp_servo_service" "$ROOT_DIR/g1_comp_servo_service"
build_cmake_dir "football_detectcpp" "$ROOT_DIR/football_detectcpp"
build_cmake_dir "robocup_locator" "$ROOT_DIR/robocup_locator_v1.1"

cat <<INFO
[g1-runtime-build] done.
Generated binaries:
  $ROOT_DIR/g1_comp_servo_service/build/main
  $ROOT_DIR/football_detectcpp/build/football_detect
  $ROOT_DIR/robocup_locator_v1.1/build/test_location
  $ROOT_DIR/robocup_locator_v1.1/build/location_fusion
INFO
