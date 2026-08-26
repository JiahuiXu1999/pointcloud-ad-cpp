# ADR-0009: declare PCL's LZ4 link dependency explicitly

- Status: Accepted
- Date: 2026-08-26

## Context

PCL and FLANN use LZ4 and optionally OpenMP. LZ4 is already in the pinned vcpkg dependency graph,
and OpenMP is supplied by the compiler toolchain. On Linux, vcpkg builds PCL as static libraries,
but the exported PCL targets do not propagate the complete LZ4/OpenMP link interface. This leaves
unresolved `LZ4_*`, `GOMP_*`, and `omp_*` references in the shared library or isolated backend test
executables.

## Decision

Declare `lz4` directly in the existing pinned vcpkg manifest. Provide one internal CMake interface
target containing `lz4::lz4` and, when PCL found it, `OpenMP::OpenMP_CXX`. Link that closure privately
to `pointcloud_ad`, the CLI, and isolated tests that compile PCL backend sources. Neither dependency
appears in PointCloudAD's public headers or installed target interface.

## Consequences

- Linux static PCL linkage has a complete dependency closure under GCC and Clang/sanitizers.
- Windows keeps using the same LZ4 runtime DLL that was already part of PCL's transitive runtime
  closure and the SDK package.
- The manifest records every library that PointCloudAD names directly instead of relying on a
  transitive dependency remaining visible.
