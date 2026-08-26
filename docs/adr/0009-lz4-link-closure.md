# ADR-0009: declare PCL's LZ4 link dependency explicitly

- Status: Accepted
- Date: 2026-08-26

## Context

PCL's I/O libraries use LZ4 and already bring the port into the pinned vcpkg dependency graph. On
Linux, vcpkg builds PCL as static libraries, but the exported PCL target does not propagate every
LZ4 symbol required when those objects are linked into the shared PointCloudAD library. This leaves
unresolved `LZ4_*` references when an executable consumes `libpointcloud_ad.so`.

## Decision

Declare `lz4` directly in the existing pinned vcpkg manifest and link its `lz4::lz4` imported target
privately to `pointcloud_ad`. LZ4 remains a backend implementation dependency: no LZ4 header, type,
or target appears in PointCloudAD's public headers or installed target interface.

## Consequences

- Linux static PCL linkage has a complete dependency closure under GCC and Clang/sanitizers.
- Windows keeps using the same LZ4 runtime DLL that was already part of PCL's transitive runtime
  closure and the SDK package.
- The manifest records every library that PointCloudAD names directly instead of relying on a
  transitive dependency remaining visible.
