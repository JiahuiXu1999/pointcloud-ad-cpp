# ADR-0001: CMake, Ninja, and vcpkg manifest toolchain

- Status: Accepted
- Date: 2026-08-19

## Context

PointCloudAD must build reproducibly on Windows and Linux, integrate PCL without committing binary
SDKs, export a conventional CMake package, and expose a small set of commands that an AI agent can
run without IDE-specific state.

## Decision

- CMake 3.25 is the minimum build-system version.
- CMake Presets are the only supported configuration entry points.
- Ninja is the default generator with one build directory per preset.
- vcpkg manifest mode owns third-party dependencies and uses a pinned registry baseline.
- MSVC is the primary Windows compiler; GCC is the primary Linux compiler; Clang sanitizers form a
  required secondary validation path.
- CTest provides test execution. The M0 smoke suite is dependency-free so the foundation can be
  verified offline; GoogleTest becomes the default harness for M1 domain test matrices.
- CMake install/export produces the `PointCloudAD::pointcloud_ad` consumer target.

## Consequences

Generated Visual Studio projects are disposable. Developers need vcpkg and Ninja, but the Windows
script discovers Visual Studio bundled copies. A vcpkg baseline update is an explicit reviewed
change. CI must verify both direct builds and an installed-package consumer.
