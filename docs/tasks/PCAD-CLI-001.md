# PCAD-CLI-001: inspect, validate-config, and version commands

## Identity

- Task ID: PCAD-CLI-001
- Milestone: M7 / B25
- Objective: Complete the `pcad` command-line vertical slice with `inspect`, `validate-config`, and
  `version`, consuming the pipeline and the internal serialization/output facades.
- Dependencies: PCAD-SER-002

## Allowed scope

- Files/modules that may change: `src/cli/main.cpp`, the `pcad` build wiring, its integration test,
  and M7 documentation.
- Public API changes: none. The CLI links the internal I/O and serialization sources directly.
- New dependencies: none.

## Contract

- Inputs and ownership: `validate-config` reads a config JSON file; `inspect` reads a config JSON
  file plus two point-cloud files whose format is chosen by extension (`.ply`/`.pcd`) and whose unit
  and frame come from the validated configuration.
- Outputs and ownership: `inspect` writes `result.json` (versioned result document) and
  `manifest.json` (versioned manifest with the result file's SHA-256 digest) into the output
  directory, and prints the verdict to stdout.
- Units and coordinate frames: millimetres; scan-to-reference.
- Determinism requirements: fixed output file names and stable manifest key ordering.
- Error cases: missing files, malformed or unsupported configuration, unsupported point-cloud
  extensions, and pipeline failures return a non-zero exit code with a message on stderr.
- Performance budget: a single pipeline pass per `inspect` invocation.

## Exclusions

- Explicitly out of scope: mapping inspection verdicts to process exit codes (pass/fail/
  indeterminate always exit zero once the run completes).
- Architecture boundaries that must not move: the CLI never exposes PCL/Eigen through the installed
  headers; no public ABI is added.

## Acceptance

- Required unit tests: `--version` exits zero; `validate-config` accepts a valid document and
  rejects an invalid one; `inspect` completes on identical planar clouds, writes `result.json` with
  a `pass` verdict and `manifest.json` with a 64-character digest; unknown arguments exit 2.
- Required integration/golden tests: existing public-header audit, installed-consumer, I/O, and
  preprocessing gates remain green.
- Required build presets: Windows Debug and Release install verification.
- Required documentation updates: roadmap, changelog, and milestone status.

## Completion evidence

- Commands run: `pwsh ./scripts/build.ps1 -Preset windows-msvc-debug`,
  `pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall`.
- Test results: the new CLI integration test passes alongside the full suite.
- Remaining risks: `inspect` uses an identity initial pose and a fixed run id until richer request
  options arrive; per-point PLY emission awaits a pipeline public API for per-point fields.
