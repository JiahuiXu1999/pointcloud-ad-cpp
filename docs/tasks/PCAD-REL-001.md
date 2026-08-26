# PCAD-REL-001: open-source SDK release package

## Identity

- Task ID: PCAD-REL-001
- Milestone: M9 / B31-B33
- Objective: Produce a redistributable v0.1 Windows SDK archive and automate its verified GitHub
  release from version tags.
- Dependencies: PCAD-BENCH-001

## Allowed scope

- Files/modules that may change: install/CPack rules, release workflow and verification script,
  standalone SDK example, open-source/release documentation, and version presentation.
- Public API changes: none; the existing public C++ API is packaged as version 0.1.0.
- New dependencies: none.

## Contract

- Inputs and ownership: a Release build produced from the pinned vcpkg manifest; callers own all
  inputs passed through the existing SDK API.
- Outputs and ownership: a versioned ZIP and SHA-256 checksum containing the SDK binaries, import
  library, public headers, CMake package, runtime dependencies, licenses, documentation, and
  consumer source.
- Units and coordinate frames: unchanged; millimetres, right-handed frames, column vectors, and
  scan-to-reference transforms.
- Determinism requirements: archive name and installed layout are stable for a version/platform;
  runtime algorithms retain the existing deterministic contracts.
- Error cases: packaging fails on a missing required file, source/build path leak, consumer build
  failure, or packaged executable that cannot run from the restricted SDK runtime path.
- Performance budget: packaging adds no runtime algorithm work; Release tests retain existing
  benchmark and acceptance guardrails.

## Exclusions

- Explicitly out of scope: a stable C ABI, bindings for other languages, package registries,
  code-signing certificates, installers, and macOS/Linux binary release archives.
- Architecture boundaries that must not move: third-party types remain private and no dependency
  is exposed by the installed CMake target or public headers.

## Acceptance

- Required unit tests: the complete Debug and Release CTest suites remain green.
- Required integration/golden tests: install/export consumer, DLL export/runtime inspection, CPack
  archive extraction, bundled consumer rebuild, restricted-path CLI and consumer execution.
- Required build presets: `windows-msvc-debug` and `windows-msvc-release` with install and package
  verification.
- Required documentation updates: README, SDK guide, security policy, changelog, third-party
  notices, roadmap, task card, and current milestone.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug` and
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall -VerifyPackage`.
- Test results: recorded in the release commit and corresponding GitHub Actions runs.
- Remaining risks: the public C++ ABI is compiler/runtime specific until 1.0; consumers must use a
  compatible toolchain and ship every DLL from the package `bin/` directory together.
