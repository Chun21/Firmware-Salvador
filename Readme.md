# HTWK Robots Firmware 6.0

This is the firmware for HTWK Robots.
This firmware was successfully used at the RoboCup Salvador 2025, Beijing Masters 2025, and RCAP Abu Dhabi 2025.

**⚠️ Notice:** This firmware comes without any warranty. Use at your own risk. In particular, using the setup scripts can easily lock you out of a robot.

---

## 📋 Table of Contents

- [Supported Robots](#supported-robots)
- [Installation & Build](#installation--build)
- [Unitree G1 RoboCup](#unitree-g1-robocup)
- [Deployment](#deployment)
- [License](#license)

---

## 🤖 Supported Robots

### T1
- **Camera:** Intel RealSense
- **Build Options:** `--t1`, `--t1_robocup_version`
- **Deploy Directory:** `build-t1/deploy`

### K1
- **Camera Options:**
  - ZED2: `--k1z`
  - Booster Camera: `--k1bc`
- **Deploy Directories:** 
  - `build-k1z/deploy` (ZED2)
  - `build-k1bc/deploy` (Booster Camera)
    using the Booster Camera is highly untestet. It might only work in 1 out of 5 cases. 

### Unitree G1
- **Camera:** external RealSense + 2DOF head servo
- **Build Option:** `--g1`
- **Deploy Directory:** `build-g1/deploy`
- **Runtime Stack:** G1 servo service, YOLO detector, marker locator, location fusion, and
  `fw_salvador`
- **Safety:** G1 mode only uses high-level locomotion in the first phase. Low-level
  `JointControl`, T1/K1 policies, true kicking, and dive actions are blocked.

---

### Needed Software
- **scp** (SFTP client)
- **zstd** (compression)
- **pv** (progress display)

## 🔧 Installation, Build & Deployment

### 1. Prepare Toolchain Images
This command creates the Docker images for cross-compilation. Run this on the mashine (not the robot) that will build the firmware . This needs to be executed once. 
It may take a while and requires internet access. You can copy the docker image on other mashines so it needs to be run only once per team.
```bash
./toolchains/build_and_push_images.sh
```



### 2. Build Firmware
This command will build our firmware. The Docker image from step 1 needs to be aviable.
#### For T1 (Standard RoboCup Version)
```bash
./install.bash --t1_robocup_version
```

#### For T1 (Standard)
```bash
./install.bash --t1
```

#### For K1 with ZED2 Camera
```bash
./install.bash --k1z
```

#### For K1 with Booster Camera Camera
```bash
./install.bash --k1bc
```

#### For Unitree G1
```bash
./install.bash --g1
```


### 3.1. Build and Deploy to Single Robot

This command will build and deploy our firmware to a robot. Please note that the robot model needs to be specified.

```bash
./install.bash --t1_robocup_version <ROBOT_IP>
```

Example:
```bash
./install.bash --t1_robocup_version 10.0.44.15
```

### 3.2. Deploy Only (without Build)

The --deploy flag will skip the building. It only works if the last build was sucessful.

```bash
./install.bash --deploy --t1_robocup_version <ROBOT_IP>
```

### Deployment Process

In addition to the firmware, various setup scripts are included.
Using the scripts is at your own risk. Incorrect usage can easily lock you out of a robot.

### Run

Execute on robot
```bash
./bin/fw_salvador
```

---

## Unitree G1 RoboCup

### Build

Build the G1 firmware and package the vendored G1 perception/localization runtime:

```bash
./install.bash --g1
```

The generated runtime is under:

```bash
build-g1/deploy
```

If deploying manually, copy the **whole** `build-g1/deploy` directory to the G1. The
`g1_perception` directory and `run_g1.sh` are required; copying only `bin/` and `lib/` is not
enough for the full RoboCup stack.

### Start the full G1 RoboCup stack

Run this on the G1 from `build-g1/deploy`:

```bash
cd build-g1/deploy

ROBOCUP_SERVO_SERIAL=/dev/ttyUSB0 \
DETECT_DISPLAY=1 \
MARKER_DISPLAY=1 \
./run_g1.sh --gc
```

What this starts:

```text
g1_comp_servo_service
football_detect
marker_locator
location_fusion
fw_salvador --gc
```

Useful environment variables:

| Variable | Meaning |
| --- | --- |
| `G1_NETWORK_INTERFACE` | DDS network interface. Defaults to `eth0`. |
| `ROBOCUP_SERVO_SERIAL` | Head-servo serial device, for example `/dev/ttyUSB0`. |
| `ROBOCUP_MARKER_INITIAL_PARTICLES` | Initial marker-localization particle count. Defaults to `1200`. |
| `ROBOCUP_MARKER_PARTICLES` | Marker-localization particle count after initial odom calibration. Defaults to `300`. |
| `ROBOCUP_MARKER_INITIAL_OUTER_X_M` | Extra initial search margin behind own goal line. Defaults to `0.8`. |
| `ROBOCUP_MARKER_INITIAL_OUTER_Y_M` | Extra initial search margin outside sidelines. Defaults to `0.8`. |
| `ROBOCUP_MARKER_INITIAL_MIDLINE_MARGIN_M` | Initial search may extend this far into the opponent half. Defaults to `0.3`. |
| `ROBOCUP_RGBD_INIT_X/Y/THETA_DEG` | Optional fixed RGB-D initial pose. Not required for marker localization by default. |
| `ROBOCUP_MARKER_PRIOR_FROM_RGBD=1` | Opt in to reusing `ROBOCUP_RGBD_INIT_*` as the marker-localizer prior. Disabled by default. |
| `ROBOCUP_FUSION_INIT_FROM_RGBD=1` | Opt in to initializing fused field pose from RGB-D. Disabled by default; fusion waits for marker initialization. |
| `DETECT_DISPLAY=1` | Show the YOLO detection window. |
| `MARKER_DISPLAY=1` | Show marker/localization debug display. |
| `ROBOCUP_G1_READY_STABLE_LOC=1` | Enable G1 localization stability filtering before Ready walking. Enabled by default; set to `0` to disable. |
| `ROBOCUP_G1_LOC_STABLE_SEC` | Seconds of continuous stable localization required before Ready walking. Defaults to `1.0`. |
| `ROBOCUP_G1_LOC_STABLE_TRANSLATION_M` | Maximum per-update translation considered stable. Defaults to `0.25`. |
| `ROBOCUP_G1_LOC_STABLE_ROTATION_DEG` | Maximum per-update heading change considered stable. Defaults to `15`. |
| `ROBOCUP_G1_LOC_JUMP_TRANSLATION_M` | Translation jump threshold rejected by the G1 filter. Defaults to `0.50`. |
| `ROBOCUP_G1_LOC_JUMP_ROTATION_DEG` | Heading jump threshold rejected by the G1 filter. Defaults to `30`. |
| `FW_G1_PERCEPTION=0` | Start only Salvador, without perception/localization. |

The launcher automatically adds `--gc` if no GameController flag is provided.

By default, G1 marker localization does not depend on a fixed `ROBOCUP_RGBD_INIT_*` entry pose.
Before odometry is calibrated it randomly searches our half plus a small outside margin:
on a 9x6 field the default area is approximately `x=[-5.3,0.3]`, `y=[-3.8,3.8]`.
The G1 defaults are deliberately mild: marker detections below
confidence `40` are ignored, far marker detections require gradually higher confidence, marker
corrections are blended with alpha `0.05`, and marker poses that disagree with the current
fused/odometry pose by more than `0.7 m` or `40°` are rejected
instead of pulling the robot to a bad relocalization.

During `Ready`, G1 will not walk to the kickoff position until localization quality is stable.
Large sudden pose jumps are rejected and logged instead of being passed directly to the strategy.
During `Playing`, G1 uses self ball, teammate ball, and its own last-seen ball for at most `3 s`;
after all of them expire it stays in place and turns to search for the ball.

### Start without the perception stack

Use this only when another process already publishes the required DDS topics
(`detectionresults` and `rt/locationresults`):

```bash
cd build-g1/deploy
./run_g1.sh --salvador-only --gc
```

or:

```bash
FW_G1_PERCEPTION=0 ./run_g1.sh --gc
```

### Stop the stack

Press `Ctrl-C` in the terminal running `run_g1.sh`, or run:

```bash
cd build-g1/deploy
./stop_g1_stack.sh
```


---


## 📝 License

See license information in the repository.

Special license conditions apply for RoboCup teams.

---

**Last Updated:** November 2025
