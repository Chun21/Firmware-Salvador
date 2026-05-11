# Vendored G1 dependencies

This directory contains the G1-specific third-party/runtime inputs that were
previously referenced from `~/robocup_deploy`. G1 builds must be reproducible
from the repository checkout and must not require absolute paths on the build
host.

## Layout

- `unitree_sdk2/`
  - Minimal Unitree SDK2 headers and aarch64 libraries used by `g1_sdk.cmake`.
  - Contains `include/`, `lib/aarch64/`, `thirdparty/include/`, and
    `thirdparty/lib/aarch64/`.
- `robocup_dds/`
  - Minimal generated CycloneDDS C++ types consumed by the firmware adapter:
    - `detection/DetectionModule.*`
    - `location/LocationModule.*`
- `robocup_runtime/`
  - Source/config/weights copied from the G1 RoboCup detection and localization
    runtime for reference and future in-repo integration.
  - Build products, editor files, caches, and screenshots are intentionally not
    vendored.

## Rules

- Do not add a new hard dependency on `~/robocup_deploy` or any other absolute
  developer-machine path.
- If a G1 module needs additional SDK/runtime files, copy the minimal required
  files here and update this README plus the relevant CMake file.
- The firmware currently links against vendored Unitree SDK2 and subscribes to
  the vendored DDS type definitions. Runtime detector/localizer process startup
  should be integrated from `robocup_runtime/` rather than from an external
  checkout.
