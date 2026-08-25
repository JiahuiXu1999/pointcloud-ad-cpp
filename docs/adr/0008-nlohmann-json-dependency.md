# ADR-0008: use nlohmann/json for configuration and result serialization

- Status: Accepted
- Date: 2026-08-25

## Context

M7 introduces versioned configuration and result JSON. Reimplementing a JSON parser and serializer
would duplicate a well-understood, widely used capability and add format risk. Public headers must
stay free of third-party types, so the JSON library must not leak past the serialization boundary.

## Decision

Use nlohmann/json via the pinned vcpkg manifest. All JSON construction and parsing lives behind the
`src/serialization/` facade, which exchanges only project-owned value types with callers. The
library is linked privately to the `pointcloud_ad` target and never appears in installed headers.

The vcpkg manifest is the only dependency declaration. CMake consumes the imported target from
`find_package(nlohmann_json CONFIG REQUIRED)`.

## Consequences

- Config and result JSON share one parser/serializer with deterministic key ordering and stable
  number formatting.
- Installed headers remain free of nlohmann/json, PCL, Eigen, Boost, and filesystem types.
- JSON numbers must reject NaN/Infinity; unavailable measurements are written as null with a reason.
