# PCAD-ENG-003: default DLL and runtime packaging

## Identity

- Task ID: PCAD-ENG-003
- Milestone: M0.1 / B00
- Objective: Make the official library artifact a DLL/shared library and prove its export, runtime
  layout, installation, and independent consumption.
- Dependencies: PCAD-ENG-002

## Allowed scope

- Files/modules that may change: top-level CMake, M0 tests and consumer, Windows build/verification
  scripts, CI, and M0 documentation.
- Public API changes: none; `pointcloud_ad::version_string()` remains the sole binary ABI symbol.
- New dependencies: none.

## Contract

- Inputs and ownership: Consumers include installed project-owned headers and link the exported
  `PointCloudAD::pointcloud_ad` target.
- Outputs and ownership: CMake owns generated export headers and build/install artifacts; consumers
  own their executable-local runtime copy.
- Units and coordinate frames: not applicable to the engineering foundation.
- Determinism requirements: fixed `bin/` runtime and `lib/` linker artifact layout; exactly one
  declared DLL export in M0.1.
- Error cases: missing artifacts, missing/unexpected exports, static linkage, misplaced DLLs, broken
  package metadata, or a consumer requiring the build tree fail the acceptance command.
- Performance budget: no runtime performance requirement; verification remains a smoke workflow.

## Exclusions

- Explicitly out of scope: M1 domain contracts, algorithms, PCL, CPack, and third-party runtime
  dependency collection.
- Architecture boundaries that must not move: the public include root and inward dependency rules.

## Acceptance

- Required unit tests: version API through the DLL.
- Required integration/golden tests: build-tree runtime layout, `dumpbin` export/dependency audit,
  clean installation, and standalone `find_package` consumer execution.
- Required build presets: `windows-msvc-debug` and `windows-msvc-release` with `-VerifyInstall`.
- Required documentation updates: roadmap, README, architecture, and changelog.

## Completion evidence

- Commands run:
  - `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`
  - `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`
- Test results: both presets pass all three CTest tests; Release installation and independent
  consumer execution pass and print `0.1.0-dev`.
- Remaining risks: third-party runtime dependency packaging is intentionally deferred until those
  dependencies exist; clean-system release packaging remains M9 scope.
