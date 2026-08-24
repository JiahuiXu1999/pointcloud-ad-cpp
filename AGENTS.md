# PointCloudAD-CPP agent rules

These rules apply to every AI or human change in this repository.

## Definition of done

A task is complete only when its implementation, tests, documentation, and relevant build
verification are complete. Do not report success when required tests are failing or were not run.

On Windows, use:

```powershell
pwsh ./scripts/build.ps1 -Preset windows-msvc-debug
```

For install/export changes, also use:

```powershell
pwsh ./scripts/build.ps1 -Preset windows-msvc-release -VerifyInstall
```

## Architectural invariants

- Use C++20 and target-based CMake. Do not add handwritten compiler or linker paths.
- `include/pointcloud_ad/` is the only public include root.
- Public headers must not include or expose PCL, Eigen, VTK, Boost, filesystem handles, or CLI types.
- PCL code belongs exclusively in `src/backends/pcl/` once that module exists.
- Dependencies point inward toward domain code. Core code never depends on I/O, PCL, CLI, or JSON.
- The public error model is `Result<T>`; exceptions must never cross the public API boundary.
- Internal geometry uses millimetres, right-handed frames, column vectors, and scan-to-reference
  transforms unless a type explicitly states otherwise.
- Preserve deterministic output ordering and explicitly seed randomized algorithms.
- Do not combine PASS/FAIL/INDETERMINATE inspection status with process exit codes.
- Add source files explicitly to targets. Globs are allowed only for non-build tooling such as the
  format target.
- Do not introduce a second dependency manager or vendor third-party source trees.

## Change discipline

- Implement one atomic task from `docs/ROADMAP.md` at a time using `docs/AI_TASK_TEMPLATE.md`.
- Add tests before or with behavior changes.
- Keep public API changes small and document ownership, units, frame direction, and failure cases.
- Do not weaken warnings, sanitizers, tests, or CI to make a change pass.
- Do not edit generated content under `out/` or `vcpkg_installed/`.
- New dependencies require an ADR and a pinned vcpkg manifest change.

## Current milestone

M0, M0.1/B00, M1, M2, M3, M4, and M5 are complete. M6 is in progress: PCAD-DET-001 (dent/bump
classification and clustering) is complete. The next permitted atomic task is M6/PCAD-DET-002:
implement missing-material clustering without changing status/exit-code semantics or exposing
PCL/Eigen from installed headers.
