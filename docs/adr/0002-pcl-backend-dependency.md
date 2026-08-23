# ADR-0002: use PCL behind an internal backend facade

- Status: Accepted
- Date: 2026-08-20

## Context

M2 introduces PLY/PCD compatibility and later milestones require spatial indexing, normal
estimation, and registration. Reimplementing these capabilities would create unnecessary format and
algorithm risk. Public headers must remain independent from backend types and dependency headers.

## Decision

Use Point Cloud Library (PCL) 1.15.1#1 within the pinned vcpkg baseline. Disable optional
visualization/application features and consume only required PCL component targets. Every PCL include
and conversion implementation belongs under `src/backends/pcl/`. Public and `src/io/` code exchange
only project-owned values through an internal facade.

The vcpkg manifest is the only dependency declaration. CMake uses imported targets from
`find_package(PCL CONFIG REQUIRED ...)`; it does not contain compiler, linker, or installation paths.
All presets share the generated manifest installation tree, while platform toolchain selection stays
in CMake presets and CI environment configuration.

## Consequences

- Public consumers do not include or manipulate PCL, Eigen, Boost, VTK, or filesystem handles.
- The installed shared library must package its PCL runtime dependencies for downstream execution.
- PCL acquisition needs registry/source network access or a populated vcpkg asset/binary cache.
- PLY/PCD compatibility inherits PCL's supported field and encoding behavior, while project adapters
  enforce required XYZ, optional normals, explicit units/frames, topology, masks, and stable errors.
