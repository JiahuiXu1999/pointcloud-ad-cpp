# Frozen architecture decisions

Changes to these decisions require an ADR before implementation.

| Area | Decision |
|---|---|
| License | Apache-2.0 |
| Language | C++20 |
| Build | CMake Presets and Ninja; generated IDE projects are disposable |
| Dependencies | vcpkg manifest with a pinned registry baseline |
| Backend | PCL remains private under `src/backends/pcl/`; ADR-0010 adds optional CUDA correspondence search under `src/backends/cuda/` |
| Public API | No PCL, Eigen, VTK, Boost, CLI, JSON, or OS handle types |
| Units | Millimetres internally; conversion happens once at input normalization |
| Coordinates | Right-handed frames, column vectors, scan-to-reference transforms |
| Precision | Point storage may use `float`; transforms and accumulations use `double` |
| Ownership | `SurfaceView` borrows; `OwnedSurface` owns; `RegistrationContext` owns an immutable reference snapshot and prepared resources |
| Errors | Public APIs return `Result<T>`; exceptions do not cross the boundary |
| Registration | ADR-0010: shared point-to-plane / point-to-point / GICP objectives, CPU or optional resident GPU objectives with a CPU 6x6 solve, followed by a mandatory quality gate |
| Comparison | Scan-to-reference deviation plus reference-to-scan coverage |
| Outcome | PASS/FAIL/INDETERMINATE is separate from process exit status |
| Schemas | Configuration and result JSON schemas are explicitly versioned |
| Determinism | Stable ordering and explicit random seeds are mandatory |
| Package | Exported target is `PointCloudAD::pointcloud_ad` |
| Windows artifact | Official Debug/Release builds produce `pointcloud_ad.dll`; static builds are optional |
| v0.1 input | PLY and PCD only; depth/height-field inputs are later work |
