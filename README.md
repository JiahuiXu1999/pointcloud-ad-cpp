# PointCloudAD-CPP

Deterministic point-cloud anomaly detection for industrial inspection, implemented as a
portable C++20 library and command-line application.

> Project status: **M0/M0.1 through M3 are complete; M4 registration contracts (PCAD-REG-001) are
> complete and robust ICP is next**. The official library artifact is a DLL/shared library with an
> audited public export. Public contracts cover errors/results, units, frames, right-handed
> scan-to-reference transforms, validated configuration, borrowed/owned surfaces, millimetre/frame
> normalization, and an internal PCL conversion facade, deterministic binary/ASCII PLY and PCD I/O,
> and finite-value/ROI validity preprocessing with per-sample reasons plus deterministic voxel
> centroids and source mappings. M3 also provides PCA/existing-normal preparation, explicit
> orientation evidence, and organized or radius-based boundary flags. M4 introduces backend-neutral
> registration input and metrics contracts (`include/pointcloud_ad/registration.hpp`) with validated
> scan-to-reference transforms, solver parameters, neutral convergence status, and quantitative
> fitness/RMSE/delta metrics. Industrial M3 validation remains pending real sample clouds.

## Build contract

The source of truth is CMake Presets. Generated Visual Studio solutions and local build
directories are disposable and are never committed.

### Windows

Requirements:

- Visual Studio 2022 or newer with Desktop development with C++
- PowerShell 7 or Windows PowerShell 5.1
- CMake 3.25+, Ninja, and vcpkg (the script can discover Visual Studio bundled copies)
- Network access to the vcpkg registry/source archives, or a populated vcpkg asset/binary cache

Run the complete configure/build/test workflow:

```powershell
pwsh ./scripts/build.ps1 -Preset windows-msvc-debug
```

Use `-Fresh` after changing compilers, generators, or toolchain configuration.

Validate Release installation and a downstream `find_package(PointCloudAD)` consumer:

```powershell
pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall
```

Windows build artifacts use a deterministic layout: DLLs and executables are in `bin/`, while the
DLL import library is in `lib/`. The build script uses `dumpbin` to verify the declared public export
and confirm that the CLI, tests, and installed consumer dynamically load `pointcloud_ad.dll`.

### Linux

Set `VCPKG_ROOT` to a vcpkg checkout before using the Linux presets.

Run one of the committed Linux presets:

```bash
./scripts/build-linux.sh linux-gcc-release
./scripts/build-linux.sh linux-clang-asan
```

Direct CMake invocation is also supported:

```bash
cmake --workflow --preset linux-gcc-release
```

## Repository layout

```text
include/pointcloud_ad/   Stable public C++ API; never exposes PCL or Eigen
src/core/                Dependency-free compiled domain primitives
src/backends/pcl/        Private PCL facade and conversions; never installed
src/io/                  Private format validation and backend-neutral I/O orchestration
src/preprocess/          Backend-neutral validity, ROI, sampling, normal, and boundary stages
src/cli/                 Thin command-line adapter
cmake/                   Warnings, sanitizers, analysis, install/export rules
tests/unit/              Fast deterministic unit tests
tests/consumer/          Installed-package smoke consumer
docs/architecture/       Dependency rules and system structure
docs/adr/                Accepted architectural decisions
scripts/                 Reproducible developer entry points
```

Future modules are introduced only when their milestone begins. PCL is isolated under
`src/backends/pcl/`; no PCL include may enter `include/pointcloud_ad/` or `src/io/`.

## Current executable

```text
pcad --help
pcad --version
```

Unknown command-line arguments return exit code `2`. Inspection result semantics are not yet
implemented and will remain separate from process exit status.

## Quality gates

Every change must preserve all of the following:

1. CMake configure succeeds from a clean directory.
2. The library and CLI compile with warnings enabled.
3. CTest passes with no skipped required tests.
4. `cmake --install` produces a usable `PointCloudADConfig.cmake` package.
5. The standalone consumer builds using only the installed package.
6. Public headers remain free of backend types and includes.
7. The Windows DLL exports only declared public ABI symbols and installed consumers receive its
   runtime dependency without using the source or build tree.

The pinned vcpkg manifest installs PCL with optional features disabled. Manifest artifacts are shared
across build presets in `vcpkg_installed/`; this generated directory is never edited manually.

M3 algorithm tests use fixed synthetic planes, masks, topology, depth discontinuities, and orientation
hints. See [docs/validation/M3_INDUSTRIAL_SAMPLES.md](docs/validation/M3_INDUSTRIAL_SAMPLES.md) for
the explicitly unverified industrial-data gate.

See [CONTRIBUTING.md](CONTRIBUTING.md) for the development workflow, [docs/ROADMAP.md](docs/ROADMAP.md)
for atomic tasks, and [AGENTS.md](AGENTS.md) for rules that apply to AI-assisted implementation.

## License

Apache-2.0. See [LICENSE](LICENSE).
