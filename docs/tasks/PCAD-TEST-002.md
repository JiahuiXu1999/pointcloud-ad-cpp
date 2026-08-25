# PCAD-TEST-002: consumer, sanitizer, and determinism gates

## Identity

- Task ID: PCAD-TEST-002
- Milestone: M8 / B27-B28
- Objective: Turn installed-consumer, sanitizer, formatting, and cross-process determinism checks
  into explicit release gates.
- Dependencies: PCAD-TEST-001

## Allowed scope

- Files/modules that may change: CLI integration tests, CTest metadata, CI/build verification,
  formatting-only changes, and M8 documentation.
- Public API changes: none.
- New dependencies: none.

## Contract

- Inputs and ownership: a fixed valid configuration and deterministic identical reference/scan PLY
  fixtures owned by the CLI integration test.
- Outputs and ownership: ten independently generated `result.json` documents plus the existing
  installed-package consumer executable.
- Units and coordinate frames: millimetres; scan frame `scanner`, reference frame `fixture`, and
  scan-to-reference transforms.
- Determinism requirements: across ten CLI processes, every serialized business field is identical;
  only UTC timestamps and measured stage durations are excluded.
- Error cases: any CLI failure, missing result, changed business field, sanitizer finding, public
  header leak, or installed-consumer failure fails the release gate.
- Performance budget: the ten-process deterministic CLI test completes in under 30 seconds on CI.

## Exclusions

- Explicitly out of scope: 100k/1M staged timing baselines (PCAD-BENCH-001), CPack archives, and
  clean-machine release-package validation (PCAD-REL-001).
- Architecture boundaries that must not move: tests may use private I/O/serialization facades, but
  installed public headers remain backend-neutral.

## Acceptance

- Required unit tests: AC-011 ten-process serialized business-field equality.
- Required integration/golden tests: AC-012 Release install, standalone `find_package` consumer,
  public-header audit, Windows DLL dependency/export audit, and formatting check.
- Required build presets: Windows Debug, Windows Release with install verification, Linux GCC
  Release, and Linux Clang ASan/UBSan in CI.
- Required documentation updates: roadmap, changelog, README, task card, and milestone status.

## Completion evidence

- Commands run: `cmake --build --preset windows-msvc-release --target format`,
  `cmake --build --preset windows-msvc-release --target format-check`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`, and
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: all Debug/Release CTest tests, the ten-process CLI determinism gate, DLL audits,
  installation, and the standalone consumer pass; Linux sanitizer coverage is enforced by CI.
- Remaining risks: industrial acceptance remains unverified pending real sample clouds; performance
  baselines arrive in PCAD-BENCH-001.
