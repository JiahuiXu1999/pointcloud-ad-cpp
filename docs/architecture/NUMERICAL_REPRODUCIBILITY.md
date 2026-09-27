# Registration numerical contract (M11-M13)

The existing scalar solver is the reference. Preserve exact search distance bounds and lowest original-index ties, valid-mask ordering, covariance neighbor sorting, rank tests, Huber thresholds, convergence rules and inspection verdict semantics. Do not enable fast-math or FMA contraction in explicit SIMD kernels.

1. Fresh and reused contexts with identical options must have exactly equal transforms and all metrics.
2. CPU thread counts 1, 2, 4 and 8 must have exactly equal transforms, iterations, counts, fitness, RMSE and movement metrics on the regression matrix. Work blocks and reduction merge order do not depend on worker count.
3. Fixed blocks of 256 points may change rounding relative to the original sequential accumulation. Before this change, capture scalar fixture metrics. The budget on the small translated curved fixture is 1e-6 mm translation and RMSE, 1e-6 for rotation matrix entries; count and convergence status must match. No such tolerance replaces exact repeated-run/cross-thread tests or existing acceptance thresholds.
4. SIMD kernels use identical scalar operation grouping without contraction. Test direct outputs, masks, tails, non-aligned buffers and Huber boundary values; compare all public metrics across implementations. Any unavoidable difference needs its own documented change, not relaxed tests.
5. Existing CPU/CUDA tests and industrial metric-specific thresholds remain unchanged. Timing/timestamp metadata is excluded from reproducibility. No cross-compiler/device bitwise guarantee.

Inspection acceptance retains all AC-001..012 behavior. Registration gate and deviation threshold boundary tests remain required. Runtime and allocation failures are Result errors; failed jobs finish/join before public return and contexts remain usable after rejected input.

Reproducibility assumes the default round-to-nearest floating environment. Changing rounding/denormal modes during a context lifetime is outside this contract; the executor does not change process-wide floating-point policy.
