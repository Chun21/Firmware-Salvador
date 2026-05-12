# libtinyxml2 for G1 deploy

`libfastrtps.so.2.13` in the G1 firmware deploy has SONAME dependency `libtinyxml2.so.9`.
Some toolchain/sysroot snapshots used for `install.bash --g1` do not contain this runtime library, so it is vendored here for deterministic robot deployment.

Source package: Ubuntu Jammy arm64 `libtinyxml2-9_9.0.0+dfsg-3_arm64.deb`.
