# Contributing

## Workflow

1. Select one atomic task from the architecture specification.
2. Record its inputs, outputs, invariants, exclusions, and acceptance commands using
   `docs/AI_TASK_TEMPLATE.md`.
3. Add or update tests with the implementation.
4. Run the appropriate preset workflow from a clean or existing build tree.
5. If public packaging changed, run the installed-consumer verification.
6. Keep commits focused and use the task identifier in the commit subject when available.

## C++ conventions

- C++20, two-space indentation, UTF-8 source files.
- Use RAII and value semantics by default.
- Prefer explicit ownership and `std::span`/views for borrowed contiguous data.
- Use `noexcept` only when the implementation can uphold it.
- Avoid global mutable state and hidden unit conversions.
- Include what you use; do not rely on transitive includes.
- Run `cmake --build --preset <preset> --target format-check` before review.

## Pull request requirements

- State which acceptance criteria are covered.
- List the exact commands that were run.
- Call out public API, schema, dependency, performance, and determinism effects.
- Include benchmark evidence for performance-sensitive code.
- Do not merge with required CI jobs failing.

