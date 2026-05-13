#!/usr/bin/env bash

# Function to check if command succeeded
check_success() {
  if [ $? -ne 0 ]; then
      echo "❌ Error: $1"
      docker run --rm --network host -v $TOOLCHAIN_VOLUME:/l4t -v $(pwd):$(pwd) $CONTAINER_NAME bash -c "
         chown -R $USER_ID:$GROUP_ID /workspaces/firmware_6.0
       "
    exit 1
  else
    echo "✅ Success: $1"
  fi
}

# Function to set the right permissions to SSH files
fix_ssh_key_file_permissions() {
	for fix_this_file in deploy-helpers/your.ssh.key.file deploy-helpers/your.ssh.key.file.pub deploy-helpers/.ssh deploy-helpers/.ssh/authorized_keys ; do
		# this will be 0 if perms is ok, nonzero if ssh would complain about key permissions
		local key_group_perms=$(( ( $(${stat_cmd} -c %a ${fix_this_file}) % 100 ) / 10 ))
		local key_other_perms=$(( $(${stat_cmd} -c %a ${fix_this_file}) % 10 ))

		if [[ ${key_group_perms} -ne 0 || ${key_other_perms} -ne 0 ]] ; then
			echo Tweaking SSH priv key file permissions.
			${chmod_cmd} g-rwx,o-rwx -- ${fix_this_file}
		fi
	done
}

write_g1_runtime_scripts() {
    local deploy_dir="$1"

    mkdir -p "$deploy_dir"

    cat > "$deploy_dir/run_g1_salvador_only.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="$DEPLOY_DIR/lib"
LOADER="$LIB_DIR/aarch64-linux-gnu/ld-linux-aarch64.so.1"
REAL_BIN="$DEPLOY_DIR/bin/fw_salvador.real"

cd "$DEPLOY_DIR"
export G1_NETWORK_INTERFACE="${G1_NETWORK_INTERFACE:-eth0}"
export LD_LIBRARY_PATH="$LIB_DIR:${LD_LIBRARY_PATH:-}"

FW_ARGS=("$@")
HAS_GC=0
for arg in "${FW_ARGS[@]}"; do
  case "$arg" in
    --gc|--gc3|--real_gamecontroller|--no-gc)
      HAS_GC=1
      ;;
  esac
done
if [[ "$HAS_GC" == "0" ]]; then
  FW_ARGS=(--gc "${FW_ARGS[@]}")
fi

if [[ -x "$LOADER" ]]; then
  exec "$LOADER" --library-path "$LIB_DIR:${LD_LIBRARY_PATH:-}" "$REAL_BIN" "${FW_ARGS[@]}"
else
  exec "$REAL_BIN" "${FW_ARGS[@]}"
fi
EOF
    chmod +x "$deploy_dir/run_g1_salvador_only.sh"

    cat > "$deploy_dir/stop_g1_stack.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

IFACE="${G1_NETWORK_INTERFACE:-eth0}"
DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[g1-stop] stopping G1 perception/localization stack on iface=$IFACE"
pkill -TERM -f "[f]ootball_detect .* $IFACE" 2>/dev/null || true
pkill -TERM -f "[t]est_location $IFACE " 2>/dev/null || true
pkill -TERM -f "[l]ocation_fusion $IFACE" 2>/dev/null || true
pkill -TERM -f "$DEPLOY_DIR/bin/[f]w_salvador.real" 2>/dev/null || true
sudo -n pkill -TERM -f "[.]/main $IFACE --serial" 2>/dev/null || true
sleep 0.8
pkill -KILL -f "[f]ootball_detect .* $IFACE" 2>/dev/null || true
pkill -KILL -f "[t]est_location $IFACE " 2>/dev/null || true
pkill -KILL -f "[l]ocation_fusion $IFACE" 2>/dev/null || true
pkill -KILL -f "$DEPLOY_DIR/bin/[f]w_salvador.real" 2>/dev/null || true
sudo -n pkill -KILL -f "[.]/main $IFACE --serial" 2>/dev/null || true
echo "[g1-stop] done"
EOF
    chmod +x "$deploy_dir/stop_g1_stack.sh"

    cat > "$deploy_dir/run_g1.sh" <<'EOF'
#!/usr/bin/env bash
set -Eeuo pipefail

# One-key G1 launcher for Firmware-Salvador.
# By default it starts the vendored G1 perception/localization runtime packaged
# inside this Firmware-Salvador deploy directory:
#   g1_perception/g1_comp_servo_service -> football_detect(RealSense + YOLO)
#   -> marker_locator -> location_fusion -> fw_salvador
#
# Useful overrides:
#   G1_NETWORK_INTERFACE=eth0 ./run_g1.sh
#   FW_G1_PERCEPTION=0 ./run_g1.sh          # only start Salvador
#   ./run_g1.sh --salvador-only             # only start Salvador
#   DETECT_DISPLAY=1 MARKER_DISPLAY=1 ./run_g1.sh

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="$DEPLOY_DIR/lib"
LOADER="$LIB_DIR/aarch64-linux-gnu/ld-linux-aarch64.so.1"
REAL_BIN="$DEPLOY_DIR/bin/fw_salvador.real"

IFACE="${G1_NETWORK_INTERFACE:-eth0}"
DETECT_DISPLAY="${DETECT_DISPLAY:-0}"
MARKER_DISPLAY="${MARKER_DISPLAY:-0}"
FW_G1_PERCEPTION="${FW_G1_PERCEPTION:-1}"
ROBOCUP_SERVO_SERIAL="${ROBOCUP_SERVO_SERIAL:-}"
ORIGINAL_LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
SALVADOR_LD_LIBRARY_PATH="$LIB_DIR${ORIGINAL_LD_LIBRARY_PATH:+:$ORIGINAL_LD_LIBRARY_PATH}"
RUNTIME_LD_LIBRARY_PATH="$ORIGINAL_LD_LIBRARY_PATH"

FW_ARGS=()
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --salvador-only|--no-perception)
      FW_G1_PERCEPTION=0
      shift
      ;;
    --iface=*)
      IFACE="${1#--iface=}"
      shift
      ;;
    --detect-display)
      DETECT_DISPLAY=1
      shift
      ;;
    --marker-display)
      MARKER_DISPLAY=1
      shift
      ;;
    *)
      FW_ARGS+=("$1")
      shift
      ;;
  esac
done

export G1_NETWORK_INTERFACE="$IFACE"

configure_g1_localization_defaults() {
  # Keep startup simple: when the user provides the RGB-D initial pose, reuse it as the
  # marker-locator prior.  The defaults are intentionally mild: odom/RGB-D remains the
  # main estimate, while marker localization can correct it only when reasonably consistent.
  if [[ -n "${ROBOCUP_RGBD_INIT_X:-}" ]]; then
    export ROBOCUP_MARKER_PRIOR_X="${ROBOCUP_MARKER_PRIOR_X:-$ROBOCUP_RGBD_INIT_X}"
  fi
  if [[ -n "${ROBOCUP_RGBD_INIT_Y:-}" ]]; then
    export ROBOCUP_MARKER_PRIOR_Y="${ROBOCUP_MARKER_PRIOR_Y:-$ROBOCUP_RGBD_INIT_Y}"
  fi
  if [[ -n "${ROBOCUP_RGBD_INIT_THETA_DEG:-}" ]]; then
    export ROBOCUP_MARKER_PRIOR_THETA_DEG="${ROBOCUP_MARKER_PRIOR_THETA_DEG:-$ROBOCUP_RGBD_INIT_THETA_DEG}"
  fi

  export ROBOCUP_MARKER_PRIOR_X_WINDOW_M="${ROBOCUP_MARKER_PRIOR_X_WINDOW_M:-1.0}"
  export ROBOCUP_MARKER_PRIOR_Y_WINDOW_M="${ROBOCUP_MARKER_PRIOR_Y_WINDOW_M:-1.0}"
  export ROBOCUP_MARKER_PRIOR_THETA_WINDOW_DEG="${ROBOCUP_MARKER_PRIOR_THETA_WINDOW_DEG:-60}"
  export ROBOCUP_MARKER_MIN_CONFIDENCE="${ROBOCUP_MARKER_MIN_CONFIDENCE:-40}"
  export ROBOCUP_MARKER_FAR_CONFIDENCE_START_M="${ROBOCUP_MARKER_FAR_CONFIDENCE_START_M:-3.0}"
  export ROBOCUP_MARKER_FAR_CONFIDENCE_SLOPE="${ROBOCUP_MARKER_FAR_CONFIDENCE_SLOPE:-5.0}"
  export ROBOCUP_MARKER_FAR_CONFIDENCE_MAX="${ROBOCUP_MARKER_FAR_CONFIDENCE_MAX:-65}"
  export ROBOCUP_MARKER_MIN_COUNT="${ROBOCUP_MARKER_MIN_COUNT:-3}"
  export ROBOCUP_MARKER_RESIDUAL_TOLERANCE="${ROBOCUP_MARKER_RESIDUAL_TOLERANCE:-0.35}"
  export ROBOCUP_MARKER_CONVERGE_TOLERANCE="${ROBOCUP_MARKER_CONVERGE_TOLERANCE:-0.30}"
  export ROBOCUP_MARKER_RESIDUAL_DISTANCE_POWER="${ROBOCUP_MARKER_RESIDUAL_DISTANCE_POWER:-0.70}"
  export ROBOCUP_FUSION_MARKER_ALPHA="${ROBOCUP_FUSION_MARKER_ALPHA:-0.05}"
  export ROBOCUP_FUSION_MARKER_MAX_CORRECTION_M="${ROBOCUP_FUSION_MARKER_MAX_CORRECTION_M:-0.7}"
  export ROBOCUP_FUSION_MARKER_MAX_CORRECTION_DEG="${ROBOCUP_FUSION_MARKER_MAX_CORRECTION_DEG:-40}"
  export ROBOCUP_G1_READY_STABLE_LOC="${ROBOCUP_G1_READY_STABLE_LOC:-1}"
}

configure_g1_localization_defaults

HAS_GC=0
for arg in "${FW_ARGS[@]}"; do
  case "$arg" in
    --gc|--gc3|--real_gamecontroller|--no-gc)
      HAS_GC=1
      ;;
  esac
done
if [[ "$HAS_GC" == "0" ]]; then
  FW_ARGS=(--gc "${FW_ARGS[@]}")
fi

build_runtime_if_needed() {
  local root="$1"

  local servo_bin="$root/g1_comp_servo_service/build/main"
  local detect_bin="$root/football_detectcpp/build/football_detect"
  local marker_bin="$root/robocup_locator_v1.1/build/test_location"
  local fusion_bin="$root/robocup_locator_v1.1/build/location_fusion"

  if [[ -x "$servo_bin" && -x "$detect_bin" && -x "$marker_bin" && -x "$fusion_bin" ]]; then
    return
  fi

  if [[ "${FW_G1_AUTOBUILD_PERCEPTION:-1}" != "1" ]]; then
    return
  fi

  if [[ ! -x "$root/compile_runtime_on_robot.sh" ]]; then
    return
  fi

  echo "[g1] perception binaries missing; building vendored runtime inside this project..."
  env LD_LIBRARY_PATH="$ORIGINAL_LD_LIBRARY_PATH" "$root/compile_runtime_on_robot.sh"
}

source_g1_runtime_env() {
  local root="$1"
  local env_ld_library_path="$ORIGINAL_LD_LIBRARY_PATH"
  if [[ -f "$root/robocup_env.sh" ]]; then
    echo "[g1] source env: $root/robocup_env.sh"
    # shellcheck disable=SC1090
    source "$root/robocup_env.sh"
    env_ld_library_path="${LD_LIBRARY_PATH:-$ORIGINAL_LD_LIBRARY_PATH}"
    export LD_LIBRARY_PATH="$ORIGINAL_LD_LIBRARY_PATH"
  fi

  SALVADOR_LD_LIBRARY_PATH="$LIB_DIR${ORIGINAL_LD_LIBRARY_PATH:+:$ORIGINAL_LD_LIBRARY_PATH}"
  RUNTIME_LD_LIBRARY_PATH="$root/g1_comp_servo_service/lib/arm:$root/unitree_sdk2/thirdparty/lib/aarch64:$root/unitree_sdk2/lib/aarch64:$root/g1_comp_servo_service/build:$root/football_detectcpp/build:$root/robocup_locator_v1.1/build${env_ld_library_path:+:$env_ld_library_path}"
}

require_file() {
  if [[ ! -e "$1" ]]; then
    echo "[g1][ERROR] missing: $1" >&2
    exit 1
  fi
}

is_alive() {
  local pid="$1"
  kill -0 "$pid" 2>/dev/null || sudo -n kill -0 "$pid" 2>/dev/null
}

detect_servo_serial() {
  if [[ -n "${ROBOCUP_SERVO_SERIAL:-}" && -e "${ROBOCUP_SERVO_SERIAL:-}" ]]; then
    printf '%s' "$ROBOCUP_SERVO_SERIAL"
    return 0
  fi
  if [[ -e /dev/ttyUSB0 ]]; then
    printf '/dev/ttyUSB0'
    return 0
  fi
  local dev
  for dev in /dev/ttyUSB* /dev/ttyACM*; do
    if [[ -e "$dev" ]]; then
      printf '%s' "$dev"
      return 0
    fi
  done
  return 1
}

PIDS=()
NAMES=()
TAIL_PID=""
RUN_ID="$(/bin/date +%Y%m%d_%H%M%S 2>/dev/null || true)"
if [[ -z "$RUN_ID" ]]; then
  RUN_ID="pid_$$"
fi
LOG_DIR="$DEPLOY_DIR/logs/g1_stack_$RUN_ID"

cleanup() {
  local code=$?
  set +e
  echo
  echo "[g1] stopping ${#PIDS[@]} processes..."
  if [[ -n "$TAIL_PID" ]]; then
    kill "$TAIL_PID" 2>/dev/null || true
  fi
  local pid
  for pid in "${PIDS[@]}"; do
    if is_alive "$pid"; then
      kill -- "-$pid" 2>/dev/null || true
      kill "$pid" 2>/dev/null || true
      sudo -n kill -- "-$pid" 2>/dev/null || true
      sudo -n kill "$pid" 2>/dev/null || true
    fi
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    if is_alive "$pid"; then
      kill -9 -- "-$pid" 2>/dev/null || true
      kill -9 "$pid" 2>/dev/null || true
      sudo -n kill -9 -- "-$pid" 2>/dev/null || true
      sudo -n kill -9 "$pid" 2>/dev/null || true
    fi
  done
  pkill -TERM -f "[f]ootball_detect .* $IFACE" 2>/dev/null || true
  pkill -TERM -f "[t]est_location $IFACE " 2>/dev/null || true
  pkill -TERM -f "[l]ocation_fusion $IFACE" 2>/dev/null || true
  sudo -n pkill -TERM -f "[.]/main $IFACE --serial" 2>/dev/null || true
  sleep 0.5
  pkill -KILL -f "[f]ootball_detect .* $IFACE" 2>/dev/null || true
  pkill -KILL -f "[t]est_location $IFACE " 2>/dev/null || true
  pkill -KILL -f "[l]ocation_fusion $IFACE" 2>/dev/null || true
  sudo -n pkill -KILL -f "[.]/main $IFACE --serial" 2>/dev/null || true
  echo "[g1] logs: $LOG_DIR"
  exit "$code"
}
trap cleanup INT TERM EXIT

start_bg() {
  local name="$1"
  local workdir="$2"
  shift 2
  local log="$LOG_DIR/${name}.log"

  echo "[g1] start $name"
  echo "     cwd: $workdir"
  echo "     log: $log"
  (
    cd "$workdir"
    echo "===== $name started at $(date) ====="
    echo "cwd=$(pwd)"
    echo "cmd=$*"
    exec "$@"
  ) >"$log" 2>&1 &

  local pid=$!
  PIDS+=("$pid")
  NAMES+=("$name")
  sleep 1
  if ! is_alive "$pid"; then
    echo "[g1][ERROR] $name exited immediately. Check log: $log" >&2
    exit 1
  fi
}

run_firmware_cmd() {
  if [[ -x "$LOADER" ]]; then
    "$LOADER" --library-path "$SALVADOR_LD_LIBRARY_PATH" "$REAL_BIN" "${FW_ARGS[@]}"
  else
    env LD_LIBRARY_PATH="$SALVADOR_LD_LIBRARY_PATH" "$REAL_BIN" "${FW_ARGS[@]}"
  fi
}

mkdir -p "$LOG_DIR"
cd "$DEPLOY_DIR"

if [[ "$FW_G1_PERCEPTION" != "1" ]]; then
  echo "[g1] perception disabled; starting Salvador only."
  exec "$DEPLOY_DIR/run_g1_salvador_only.sh" "${FW_ARGS[@]}"
fi

ROBOCUP_ROOT="$DEPLOY_DIR/g1_perception"
if [[ ! -d "$ROBOCUP_ROOT" ]]; then
  cat >&2 <<ERROR
[g1][ERROR] vendored G1 perception runtime not found: $ROBOCUP_ROOT
Re-run ./install.bash --g1 so Firmware-Salvador packages vendor/g1/robocup_runtime into deploy/g1_perception.
ERROR
  exit 1
fi

source_g1_runtime_env "$ROBOCUP_ROOT"
build_runtime_if_needed "$ROBOCUP_ROOT"

SERVO_BIN="$ROBOCUP_ROOT/g1_comp_servo_service/build/main"
DETECT_BIN="$ROBOCUP_ROOT/football_detectcpp/build/football_detect"
YOLO_ENGINE="$ROBOCUP_ROOT/football_detectcpp/weight/weight.engine"
MARKER_BIN="$ROBOCUP_ROOT/robocup_locator_v1.1/build/test_location"
FUSION_BIN="$ROBOCUP_ROOT/robocup_locator_v1.1/build/location_fusion"
LOC_CONFIG="$ROBOCUP_ROOT/robocup_locator_v1.1/config.yaml"

require_file "$SERVO_BIN"
require_file "$DETECT_BIN"
require_file "$YOLO_ENGINE"
require_file "$MARKER_BIN"
require_file "$FUSION_BIN"
require_file "$LOC_CONFIG"
require_file "$REAL_BIN"

if [[ -z "$ROBOCUP_SERVO_SERIAL" ]]; then
  ROBOCUP_SERVO_SERIAL="$(detect_servo_serial || true)"
fi
if [[ -z "$ROBOCUP_SERVO_SERIAL" || ! -e "$ROBOCUP_SERVO_SERIAL" ]]; then
  echo "[g1][ERROR] servo serial device not found. Set ROBOCUP_SERVO_SERIAL=/dev/ttyUSBx." >&2
  exit 1
fi

cat <<INFO
[g1] deploy: $DEPLOY_DIR
[g1] g1_perception: $ROBOCUP_ROOT
[g1] iface: $IFACE
[g1] logs: $LOG_DIR
[g1] detect_display=$DETECT_DISPLAY marker_display=$MARKER_DISPLAY
[g1] servo_serial=$ROBOCUP_SERVO_SERIAL
[g1] fw_args=${FW_ARGS[*]}
INFO

echo "[g1] checking sudo permission for servo service..."
sudo -v

echo "[g1] cleanup stale perception/localization processes..."
"$DEPLOY_DIR/stop_g1_stack.sh" || true

start_bg servo_service \
  "$ROBOCUP_ROOT/g1_comp_servo_service/build" \
  sudo env LD_LIBRARY_PATH="$RUNTIME_LD_LIBRARY_PATH" ROBOCUP_SERVO_CONFIG="${ROBOCUP_SERVO_CONFIG:-../config/config.yaml}" ./main "$IFACE" --serial "$ROBOCUP_SERVO_SERIAL"
sleep 2

start_bg football_detect \
  "$ROBOCUP_ROOT/football_detectcpp/build" \
  env LD_LIBRARY_PATH="$RUNTIME_LD_LIBRARY_PATH" ./football_detect ../weight/weight.engine "$DETECT_DISPLAY" "$IFACE"
sleep 3

start_bg marker_locator \
  "$ROBOCUP_ROOT/robocup_locator_v1.1/build" \
  env LD_LIBRARY_PATH="$RUNTIME_LD_LIBRARY_PATH" ./test_location "$IFACE" ../config.yaml "$MARKER_DISPLAY"
sleep 2

start_bg location_fusion \
  "$ROBOCUP_ROOT/robocup_locator_v1.1/build" \
  env LD_LIBRARY_PATH="$RUNTIME_LD_LIBRARY_PATH" ./location_fusion "$IFACE"
sleep 2

FW_LOG="$LOG_DIR/fw_salvador.log"
echo "[g1] start fw_salvador"
echo "     log: $FW_LOG"
(
  cd "$DEPLOY_DIR"
  echo "===== fw_salvador started at $(date) ====="
  echo "cwd=$(pwd)"
  echo "cmd=$REAL_BIN ${FW_ARGS[*]}"
  run_firmware_cmd
) >"$FW_LOG" 2>&1 &
PIDS+=("$!")
NAMES+=("fw_salvador")
sleep 1
if ! is_alive "${PIDS[-1]}"; then
  echo "[g1][ERROR] fw_salvador exited immediately. Check log: $FW_LOG" >&2
  exit 1
fi

echo
echo "[g1] stack started. Following fw_salvador log; press Ctrl-C to stop all modules."
echo "[g1] other logs:"
echo "  tail -f $LOG_DIR/football_detect.log"
echo "  tail -f $LOG_DIR/marker_locator.log"
echo "  tail -f $LOG_DIR/location_fusion.log"
echo
tail -n +1 -F "$FW_LOG" &
TAIL_PID="$!"

while true; do
  for i in "${!PIDS[@]}"; do
    pid="${PIDS[$i]}"
    name="${NAMES[$i]}"
    if ! is_alive "$pid"; then
      echo "[g1][ERROR] process exited: $name pid=$pid log=$LOG_DIR/${name}.log" >&2
      exit 1
    fi
  done
  sleep 2
done
EOF
    chmod +x "$deploy_dir/run_g1.sh"
}

LOCAL_DIR=$(pwd)

# Docker image configuration
DOCKER_IMAGE_VERSION="${DOCKER_IMAGE_VERSION:-$(cat TOOLCHAIN_VERSION 2>/dev/null || echo 'unknown')}"
SFTP_HOST="${SFTP_HOST:-your}"
SFTP_PORT="${SFTP_PORT:-infrastructure}"
SFTP_USER="${SFTP_USER:-here}"

DOCKERFILE_PATH="toolchains/Robot.Dockerfile"
CONTAINER_NAME="htwk-robots:compile-booster"
IMAGE_BASE_NAME="compile-booster"
VERSIONED_IMAGE_NAME="${IMAGE_BASE_NAME}:${DOCKER_IMAGE_VERSION}"
ROBOT_MODEL="t1"
##default is ""
ROBOT_SUBMODEL="T1_robocup_version"
CAMERA_MODEL="realsense"
DEPLOY_USER="booster"

find_cmd=find
stat_cmd=stat
chmod_cmd=chmod
if [ "$(uname)" == "Darwin" ] ; then
    # macOS·
    find_cmd=gfind
    stat_cmd=gstat
    chmod_cmd=gchmod
fi

CMAKE_TOOLCHAIN_FILE="$LOCAL_DIR/toolchains.cmake"
BUILD_PATH="build-t1"

# Architecture the last build was made for (assume aarch64 if no build was found)
LAST_BUILD_ARCH="aarch64"
if [[ -f build/fw_salvador ]]; then
    LAST_BUILD_ARCH=$(file build/fw_salvador | tr ', ' '\n' | head -9 | tail -1)
fi

# Parse command line arguments
DEPLOY_IP=""
DEPLOY_ONLY=false
wifi_ssid=''
SIMULATION_MODE="OFF"
while [[ "$#" -gt 0 ]]; do
	case $1 in
        --ssid|-s) 
            wifi_ssid="${2}" 
            shift
            shift
            ;;
        --deploy) 
            echo "Deploy only"
            DEPLOY_ONLY=true
            shift
            ;;
        --simulation)
            echo "Simulation build detected"
            CONTAINER_IMAGE="ubuntu:22.04"
            TOOLCHAIN_VOLUME="sim-toolchain"
            DOCKERFILE_PATH="toolchains/Simulation.Dockerfile"
            CONTAINER_NAME="htwk-robots:compile-booster-sim"
            IMAGE_BASE_NAME="compile-booster-sim"
            VERSIONED_IMAGE_NAME="${IMAGE_BASE_NAME}:${DOCKER_IMAGE_VERSION}"
            CMAKE_TOOLCHAIN_FILE="$LOCAL_DIR/sim_toolchains.cmake"
            SIMULATION_MODE="ON"
            DEPLOY_PATH="deploy_sim"
            BUILD_PATH="build-sim"
            shift
            ;;
        --t1)
            echo "building for T1"
            DEPLOY_PATH="deploy_t1"
            BUILD_PATH="build-t1"
            ROBOT_MODEL="t1"
            CAMERA_MODEL="realsense"
            DEPLOY_USER="booster"
            shift
            ;;
        --t1_robocup_version)
            echo "building for T1"
            DEPLOY_PATH="deploy_t1"
            BUILD_PATH="build-t1"
            ROBOT_MODEL="t1"
            ROBOT_SUBMODEL="T1_robocup_version"
            CAMERA_MODEL="realsense"
            DEPLOY_USER="booster"
            shift
            ;;
        --k1z)
            echo "building for K1 with Zed2 camera"
            DEPLOY_PATH="deploy_k1z"
            BUILD_PATH="build-k1z"
            ROBOT_MODEL="k1"
            CAMERA_MODEL="zed2"
            DEPLOY_USER="booster"
            shift
            ;;
        --k1bc)
            echo "building for K1 with Booster camera"
            DEPLOY_PATH="deploy_k1bc"
            BUILD_PATH="build-k1bc"
            ROBOT_MODEL="k1"
            CAMERA_MODEL="booster_camera"
            DEPLOY_USER="booster"
            shift
            ;;
        --g1)
            echo "building for Unitree G1"
            DEPLOY_PATH="deploy_g1"
            BUILD_PATH="build-g1"
            ROBOT_MODEL="g1"
            ROBOT_SUBMODEL=""
            CAMERA_MODEL="external"
            DEPLOY_USER="unitree"
            shift
            ;;
        -h|--help)
            echo "Usage: ./install.bash [OPTIONS] [ip of booster]"
            echo "     --deploy                         only deploy to robot and not build"
            echo "-s | --ssid [SSID]                TBD update the Wifi of the Robot"
            echo "     --simulation                     build for simulation"
            echo "     --t1                             build for T1"
            echo "     --g1                             build for Unitree G1"
            echo "     --k1                             build for K1 (default)"
            echo "-h | --help                           nobody really knows what this one does …"
            echo "     [ip of booster]                  ip address of the robot to deploy to, does not need to be ipv4"
            shift
            exit 0;;
		*)
            DEPLOY_IP="$1"
            echo "Deploy IP set to $DEPLOY_IP"
            shift
            ;;
esac; done

DEPLOY_PATH="${BUILD_PATH}/deploy"

if [ $DEPLOY_ONLY = false ] ; then
    echo $CONTAINER_NAME > .lsp-container
    echo $BUILD_PATH > .lsp-build-path

    echo "Building using toolchain $DOCKERFILE_PATH in $CONTAINER_NAME last build was for $LAST_BUILD_ARCH"
    echo "Deployment files will be written to ${DEPLOY_PATH}"

    file_exists_in_docker() {
        docker run --rm --network host -v $TOOLCHAIN_VOLUME:/l4t $CONTAINER_NAME bash -c "test -e $1"
    }

    #if [[ "$SIMULATION_MODE" = "OFF" ]] && [[ "$LAST_BUILD_ARCH" != "aarch64" ]]; then
    #    echo "Cleanup required"
    #    rm -rf build/
    #    check_success "Cleanup for real robot after last build was for simulation"
    #fi
    #
    #if [[ "$SIMULATION_MODE" = "ON" ]] && [[ "$LAST_BUILD_ARCH" == "aarch64" ]]; then
    #    echo "Cleanup required"
    #    rm -rf build/
    #    check_success "Cleanup for simulation after last build was for real robot"
    #fi

    # Get current user ID and group ID for docker to use
    USER_ID=$(id -u)
    GROUP_ID=$(id -g)

    # Docker errors are otherwise indistinguishable from "image not found" in
    # the image-inspect branch below. Fail early with an actionable message.
    if ! docker info >/dev/null 2>&1; then
        echo "❌ Docker daemon is not accessible for user '$USER'."
        echo "   Make sure Docker is running and this user can access /var/run/docker.sock,"
        echo "   e.g. add the user to the docker group and open a new shell, or run from a shell"
        echo "   that has Docker daemon permission."
        exit 1
    fi

    G1_DOCKER_ARGS=()
    if [[ "$ROBOT_MODEL" == "g1" ]]; then
        G1_ROBOCUP_DDS_ROOT="${G1_ROBOCUP_DDS_ROOT:-$LOCAL_DIR/vendor/g1/robocup_dds}"
        UNITREE_SDK2_ROOT="${UNITREE_SDK2_ROOT:-$LOCAL_DIR/vendor/g1/unitree_sdk2}"

        if [[ ! -f "$G1_ROBOCUP_DDS_ROOT/detection/DetectionModule.hpp" || \
              ! -f "$G1_ROBOCUP_DDS_ROOT/location/LocationModule.hpp" ]]; then
            echo "❌ G1 build requires vendored RoboCup DDS types."
            echo "   Expected under: $G1_ROBOCUP_DDS_ROOT"
            echo "   Required files:"
            echo "     detection/DetectionModule.hpp"
            echo "     location/LocationModule.hpp"
            exit 1
        fi

        if [[ ! -d "$UNITREE_SDK2_ROOT" ]]; then
            echo "❌ G1 build requires vendored Unitree SDK2."
            echo "   Expected: $UNITREE_SDK2_ROOT"
            exit 1
        fi

        G1_DOCKER_ARGS+=(
            -e "G1_ROBOCUP_DDS_ROOT=$G1_ROBOCUP_DDS_ROOT"
            -e "UNITREE_SDK2_ROOT=$UNITREE_SDK2_ROOT"
        )
        echo "Using G1 vendored RoboCup DDS: $G1_ROBOCUP_DDS_ROOT"
        echo "Using G1 Unitree SDK2: $UNITREE_SDK2_ROOT"
    fi

    # Check if Docker image exists, download from SFTP if needed
    if ! docker image inspect "$VERSIONED_IMAGE_NAME" >/dev/null 2>&1; then
        echo "Docker image $VERSIONED_IMAGE_NAME not found locally, downloading from SFTP..."

        IMAGE_FILENAME="${IMAGE_BASE_NAME}_${DOCKER_IMAGE_VERSION}"

        if [[ -z "$SFTP_HOST" || -z "$SFTP_PORT" || -z "$SFTP_USER" || \
              "$SFTP_HOST" == "your" || "$SFTP_PORT" == "infrastructure" || "$SFTP_USER" == "here" ]]; then
            echo "❌ SFTP settings are not configured."
            echo "   Please export SFTP_HOST, SFTP_PORT, and SFTP_USER before running install.bash."
            echo "   Expected remote file: /toolchains/${IMAGE_FILENAME}.tar.zst"
            exit 1
        fi

        # Preflight: ensure required tools for download/decompression/import are available
        for required_cmd in scp zstd pv; do
            if ! command -v "$required_cmd" >/dev/null 2>&1; then
                echo "❌ Required tool '$required_cmd' is not installed or not in PATH. Please install it and rerun."
                exit 1
            fi
        done

        # Create temporary file for the compressed image (use directory with most available space)
        temp_dir=""
        # Try directories in order of preference for available space
        for dir in "/var/tmp" "$HOME/tmp" "$HOME" "/tmp"; do
            if [ -w "$dir" ] && [ -d "$dir" ]; then
                # Check if directory has at least 30GB free
                if df -k "$dir" | awk 'NR==2 {if ($4 > 30000000) print "ok"}' | grep -q ok 2>/dev/null; then
                    temp_dir="$dir"
                    break
                fi
            fi
        done

        if [ -z "$temp_dir" ]; then
            temp_dir="/tmp"  # fallback
            echo "⚠️ Using /tmp as last resort - may run out of space!"
        fi

        echo "Using temporary directory: $temp_dir"
        temp_compressed="${temp_dir}/${IMAGE_FILENAME}.tar.zst"

        echo "Downloading: $IMAGE_FILENAME (this may take several minutes)"

        # Step 1: Download compressed image from SFTP
        echo "Step 1/3: Downloading compressed image from SFTP..."
        if ! scp -P "$SFTP_PORT" "$SFTP_USER@$SFTP_HOST:/toolchains/$IMAGE_FILENAME.tar.zst" "$temp_compressed" 2>&1; then
            echo "❌ Failed to download image from SFTP"
            rm -f "$temp_compressed"
            exit 1
        fi

        # Step 2: Verify downloaded file integrity
        echo "Step 2/4: Verifying downloaded file..."
        if ! zstd -t "$temp_compressed"; then
            echo "❌ Downloaded file is corrupted or incomplete"
            rm -f "$temp_compressed"
            exit 1
        fi

        # Get compressed file size for progress indication
        compressed_size=$(stat -c%s "$temp_compressed" 2>/dev/null || stat -f%z "$temp_compressed" 2>/dev/null || echo "0")
        temp_uncompressed="${temp_compressed%.tar.zst}.tar"

        # Step 3: Decompress zstd archive with progress
        echo "Step 3/4: Decompressing zstd archive..."
        if ! zstd -dc "$temp_compressed" | pv -s "$compressed_size" > "$temp_uncompressed"; then
            echo "❌ Failed to decompress archive"
            rm -f "$temp_compressed" "$temp_uncompressed"
            exit 1
        fi

        # Get uncompressed file size for import progress
        uncompressed_size=$(stat -c%s "$temp_uncompressed" 2>/dev/null || stat -f%z "$temp_uncompressed" 2>/dev/null || echo "0")

        # Step 4: Import into Docker with progress
        echo "Step 4/4: Importing image into Docker..."
        if ! pv -s "$uncompressed_size" "$temp_uncompressed" | docker import - "$VERSIONED_IMAGE_NAME"; then
            echo "❌ Failed to import Docker image"
            rm -f "$temp_compressed" "$temp_uncompressed"
            exit 1
        fi

        # Clean up uncompressed file
        rm -f "$temp_uncompressed"

        # Success
        echo "✅ Successfully downloaded and loaded image from SFTP"
        rm -f "$temp_compressed"
    else
        echo "Using existing Docker image: $VERSIONED_IMAGE_NAME"
        # Still need to tag it as CONTAINER_NAME for the build process
        docker tag "$VERSIONED_IMAGE_NAME" "$CONTAINER_NAME" 2>/dev/null || true
    fi

    # Find the loaded image and tag it appropriately
    LOADED_IMAGE=$(docker images --format "{{.Repository}}:{{.Tag}}" | grep "^${IMAGE_BASE_NAME}:" | head -1)
    if [ -n "$LOADED_IMAGE" ]; then
        docker tag "$LOADED_IMAGE" "$CONTAINER_NAME"
        echo "✅ Tagged image as: $CONTAINER_NAME"
    fi

    echo "Running the build in $CONTAINER_NAME ..."
    docker run --rm --network host -v $LOCAL_DIR:$LOCAL_DIR -v $(pwd)/$DEPLOY_PATH:/install "${G1_DOCKER_ARGS[@]}" $CONTAINER_NAME bash -c "
        set -e
        cd $LOCAL_DIR
        mkdir -p $BUILD_PATH
        cd $BUILD_PATH
        chown -R root:root ${LOCAL_DIR}/$BUILD_PATH

        # Native build without cross-compilation
        cmake --debug-trycompile -DCMAKE_TOOLCHAIN_FILE=$CMAKE_TOOLCHAIN_FILE -DCMAKE_COLOR_MAKEFILE=ON -DSIMULATION_MODE=$SIMULATION_MODE -DCMAKE_INSTALL_PREFIX=/install -DROBOT_MODEL=$ROBOT_MODEL -DROBOT_SUBMODEL=\"$ROBOT_SUBMODEL\" -DCAMERA_MODEL=$CAMERA_MODEL ..
        CLICOLOR_FORCE=1 make -j$(nproc) install

        # Copy shared libraries from baked sysroot into deployment (only for real robot builds)
        if [ \"$SIMULATION_MODE\" = \"OFF\" ] && [ -d \"/l4t/targetfs/usr/lib\" ]; then
            mkdir -p /install/lib
            shopt -s nullglob
            cp -a /l4t/targetfs/usr/lib/*.so* /install/lib/ 2>/dev/null || true &&
            if [ -d \"/l4t/targetfs/usr/lib/aarch64-linux-gnu\" ]; then
                cp -a /l4t/targetfs/usr/lib/aarch64-linux-gnu/*.so* /install/lib/ 2>/dev/null || true
            fi
            shopt -u nullglob
        fi
        # delete Zed2 libs because they dynamically load libsl_ai.so from /usr/lib, so for now we use the ones present on the bot.
        rm -f /install/lib/libsl_ai.so
        rm -f /install/lib/libsl_zed.so

        if [ \"$ROBOT_MODEL\" = \"g1\" ]; then
            mkdir -p /install/lib/aarch64-linux-gnu

            # Some third-party SDK packages ship linker-script placeholder files
            # for SONAME entries. The runtime loader needs real symlinks.
            if [ -f /install/lib/libddsc.so ]; then
                rm -f /install/lib/libddsc.so.0
                ln -sf libddsc.so /install/lib/libddsc.so.0
            fi
            if [ -f /install/lib/libddscxx.so ]; then
                rm -f /install/lib/libddscxx.so.0
                ln -sf libddscxx.so /install/lib/libddscxx.so.0
            fi

            # Recreate missing SONAME symlinks such as libfastcdr.so.2 and
            # libfastrtps.so.2.13 from the actual shared-library metadata.
            for so in /install/lib/*.so; do
                [ -f \"\$so\" ] || continue
                soname=\$(readelf -d \"\$so\" 2>/dev/null | awk -F'[][]' '/SONAME/ {print \$2; exit}')
                # Do not overwrite real SONAME files that were copied from the sysroot.  Some
                # libraries, e.g. Boost, ship both libfoo.so and libfoo.so.X.Y.Z.  Replacing the
                # real versioned file with a symlink back to libfoo.so creates a circular symlink
                # and the G1 runtime loader then reports \"cannot open shared object file\".
                if [ -n \"\$soname\" ] && [ ! -e \"/install/lib/\$soname\" ]; then
                    ln -sf \"\$(basename \"\$so\")\" \"/install/lib/\$soname\"
                fi
            done

            # The G1 deploy is run on Ubuntu 20.04 robots while the current
            # cross sysroot is Ubuntu 22.04. Bundle the matching loader and the
            # small FastDDS dependency if present in the sysroot.
            for loader in \
                /l4t/targetfs/usr/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1 \
                /l4t/targetfs/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1; do
                if [ -f \"\$loader\" ]; then
                    cp -a \"\$loader\" /install/lib/aarch64-linux-gnu/
                    break
                fi
            done
            find /l4t/targetfs -name 'libtinyxml2.so.9*' -exec cp -a {} /install/lib/ \\; 2>/dev/null || true
            if ! ls /install/lib/libtinyxml2.so.9* >/dev/null 2>&1; then
                cp -a ${LOCAL_DIR}/vendor/g1/aarch64_libs/tinyxml2/libtinyxml2.so.9* /install/lib/ 2>/dev/null || true
            fi
            if [ -f /install/lib/libtinyxml2.so.9.0.0 ] && [ ! -e /install/lib/libtinyxml2.so.9 ]; then
                ln -sf libtinyxml2.so.9.0.0 /install/lib/libtinyxml2.so.9
            fi
            for boost_lib in \
                libboost_program_options \
                libboost_filesystem \
                libboost_serialization; do
                find /l4t/targetfs/usr/lib /l4t/targetfs/usr/lib/aarch64-linux-gnu \
                    -maxdepth 1 -name "\${boost_lib}.so*" -exec cp -a {} /install/lib/ \\; 2>/dev/null || true
            done

            # Make direct execution work on Unitree G1 as well:
            #   ./bin/fw_salvador --gc
            #
            # The real ELF is linked against the bundled cross-sysroot libraries. Running it
            # directly with the robot's system loader can crash before main() on some G1 images.
            # Keep the ELF as fw_salvador.real and install fw_salvador as a small launcher that
            # selects the bundled loader/library path.
            if [ -f /install/bin/fw_salvador ] && readelf -h /install/bin/fw_salvador >/dev/null 2>&1; then
                mv -f /install/bin/fw_salvador /install/bin/fw_salvador.real
            fi

            cat > /install/bin/fw_salvador <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
BIN_DIR=\$(cd \$(dirname \${BASH_SOURCE[0]}) && pwd)
DEPLOY_DIR=\$(cd \$BIN_DIR/.. && pwd)
LIB_DIR=\$DEPLOY_DIR/lib
LOADER=\$LIB_DIR/aarch64-linux-gnu/ld-linux-aarch64.so.1
REAL_BIN=\$BIN_DIR/fw_salvador.real

export G1_NETWORK_INTERFACE=\${G1_NETWORK_INTERFACE:-eth0}
export LD_LIBRARY_PATH=\$LIB_DIR:\${LD_LIBRARY_PATH:-}

if [[ -x \$LOADER ]]; then
  exec \$LOADER --library-path \$LIB_DIR:\${LD_LIBRARY_PATH:-} \$REAL_BIN \$@
else
  exec \$REAL_BIN \$@
fi
EOF
            chmod +x /install/bin/fw_salvador

            cat > /install/run_g1.sh <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
DEPLOY_DIR=\$(cd \$(dirname \${BASH_SOURCE[0]}) && pwd)
LIB_DIR=\$DEPLOY_DIR/lib
LOADER=\$LIB_DIR/aarch64-linux-gnu/ld-linux-aarch64.so.1
REAL_BIN=\$DEPLOY_DIR/bin/fw_salvador.real

cd \$DEPLOY_DIR
export G1_NETWORK_INTERFACE=\${G1_NETWORK_INTERFACE:-eth0}
export LD_LIBRARY_PATH=\$LIB_DIR:\${LD_LIBRARY_PATH:-}

if [[ -x \$LOADER ]]; then
  exec \$LOADER --library-path \$LIB_DIR:\${LD_LIBRARY_PATH:-} \$REAL_BIN --gc \$@
else
  exec \$REAL_BIN --gc \$@
fi
EOF
            chmod +x /install/run_g1.sh
        fi
    "
    check_success "Build process"

    if [ "$ROBOT_MODEL" = "g1" ]; then
        echo "Packaging vendored G1 perception runtime into ${DEPLOY_PATH}/g1_perception ..."
        docker run --rm --network host -v $LOCAL_DIR:$LOCAL_DIR $CONTAINER_NAME bash -c "
           chown -R $USER_ID:$GROUP_ID $LOCAL_DIR/$DEPLOY_PATH
        "
        rm -rf "${DEPLOY_PATH}/g1_perception"
        mkdir -p "${DEPLOY_PATH}/g1_perception"
        rsync -a --delete \
            --exclude '*/build/' \
            --exclude '.git/' \
            --exclude '.pytest_cache/' \
            "vendor/g1/robocup_runtime/" "${DEPLOY_PATH}/g1_perception/"
        rsync -a --delete \
            --exclude '.git/' \
            "vendor/g1/unitree_sdk2/" "${DEPLOY_PATH}/g1_perception/unitree_sdk2/"
        write_g1_runtime_scripts "${DEPLOY_PATH}"
        check_success "Packaged G1 perception runtime"
    fi

    docker run --rm --network host -v $LOCAL_DIR:$LOCAL_DIR $CONTAINER_NAME bash -c "
           chown -R $USER_ID:$GROUP_ID $LOCAL_DIR
    "

    echo "Build completed successfully!"
fi

if [ -n "$DEPLOY_IP" ]; then
    echo "Deploying to robot at $DEPLOY_IP ..."
    fix_ssh_key_file_permissions

    rsync -av --rsh="ssh -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file -l ${DEPLOY_USER}" ${DEPLOY_PATH}/bin/ ${DEPLOY_USER}@$DEPLOY_IP:/home/${DEPLOY_USER}/bin/ --progress
    check_success "Uploaded binaries"
    rsync -av --rsh="ssh -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file -l ${DEPLOY_USER}" ${DEPLOY_PATH}/lib/ ${DEPLOY_USER}@$DEPLOY_IP:/home/${DEPLOY_USER}/lib/ --progress
    check_success "Uploaded libraries"
    rsync -av --rsh="ssh -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file -l ${DEPLOY_USER}" deploy/ ${DEPLOY_USER}@$DEPLOY_IP:/home/${DEPLOY_USER}/etc/ --progress
    check_success "Uploaded files"

    if [ "$wifi_ssid" ]; then
        echo "ToDo: Set Wifi SSID when installing on the robot"
    fi

    NTP_IP=$(./deploy-helpers/local_ipv4.py --no-fail)

    ssh -t -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file ${DEPLOY_USER}@$DEPLOY_IP \
    NTP_SERVER_IP="$NTP_IP" 'bash -s' << 'EOF'

echo "📡 Setting up NTP sync to $NTP_SERVER_IP ..."

echo 123456 | sudo -S bash -c "printf '[Time]\nNTP=$NTP_SERVER_IP\nFallbackNTP=\n' > /etc/systemd/timesyncd.conf"

# important to change different at cuntry
sudo timedatectl set-timezone Europe/Berlin

echo 123456 | sudo -S timedatectl set-ntp true

echo "⏱ Forcing immediate time sync..."
echo 123456 | sudo -S systemctl restart systemd-timesyncd

# Wait a moment for NTP to start
sleep 3

# Function to validate NTP configuration
validate_ntp_config() {
    local ntp_server_ip="$1"

    echo "🔍 Validating NTP configuration..."

    # Check if NTP service is active
    if systemctl is-active --quiet systemd-timesyncd; then
        echo "✅ NTP service is running"
    else
        echo "❌ NTP service is not running"
        return 1
    fi

    # Check if NTP is enabled
    if timedatectl show --property=NTP | grep -q "yes"; then
        echo "✅ NTP synchronization is enabled"
    else
        echo "❌ NTP synchronization is not enabled"
        return 1
    fi

    # Check time synchronization status
    if timedatectl show --property=NTPSynchronized | grep -q "yes"; then
        echo "✅ Time is synchronized with NTP server"
    else
        echo "⚠️  Time is not yet synchronized (this may take a few minutes)"
        echo "   Current time: $(date)"
        echo "   NTP server: $ntp_server_ip"
    fi

    # Check if we can reach the NTP server
    if ping -c 1 -W 3 $ntp_server_ip > /dev/null 2>&1; then
        echo "✅ NTP server $ntp_server_ip is reachable"
    else
        echo "❌ NTP server $ntp_server_ip is not reachable"
        return 1
    fi

    echo "📊 NTP Status Summary:"
    timedatectl status

    return 0
}

# Call the validation function
validate_ntp_config "$NTP_SERVER_IP"

EOF

    echo "Deployment completed successfully!"
else
    echo -e "\n\nIf you want to copy onto the robot:"
	echo "install.bash [robot ip]"
fi

# Deploy to robot if IP is specified
#if [ ! -z "$DEPLOY_IP" ]; then
#    echo "Deploying to robot at $DEPLOY_IP..."
#
#    # Create deployment directories on the robot
#    ssh -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file booster@$DEPLOY_IP "mkdir -p /home/booster/bin"
#    ssh -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file booster@$DEPLOY_IP "mkdir -p /home/booster/lib"
#    ssh -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file booster@$DEPLOY_IP "mkdir -p /home/booster/etc"
#    check_success "Creating deployment directories on robot"
#
#    # Copy binary and libraries
#    scp -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file -r ${DEPLOY_PATH}/bin/* booster@$DEPLOY_IP:/home/booster/bin
#    check_success "Copying binaries to robot"
#
#    scp -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file -r ${DEPLOY_PATH}/lib/* booster@$DEPLOY_IP:/home/booster/lib/
#    check_success "Copying libraries to robot"
#
#    scp -o StrictHostKeyChecking=no -i deploy-helpers/your.ssh.key.file -r ${DEPLOY_PATH}/etc/* booster@$DEPLOY_IP:/home/booster/etc/
#    check_success "Copying libraries to robot"
#
#    echo "Deployment completed successfully!"
#fi
