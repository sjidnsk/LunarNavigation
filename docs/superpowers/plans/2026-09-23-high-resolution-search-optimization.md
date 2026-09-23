# High-resolution search optimization implementation plan

> Execute natively in the current authorized deployment worktree. No merge/push or P3 changes.

**Goal:** Reduce legged search and wheel postprocessing latency without changing certified paths.
**Architecture:** Request-scoped lazy cell cache, bounded directed-edge reuse, retained phase mask, exact boundary BVH. Preserve original numeric geometry and all safety gates.
**Tech Stack:** C++20, CMake/CTest/GTest, Release and ASan/UBSan, Python standard-library benchmark comparison.
**Spec:** docs/validation/2026-09-23-high-resolution-search-diagnosis.md; user approved the first three optimization groups and requires extensive tests.

## Global constraints

- Keep 5 cm capability, collision geometry, phase semantics, terrain thresholds, terminal orientation and UNKNOWN unchanged.
- Preserve deterministic results on unlimited-deadline comparisons; real timeout/cancellation must still be honored.
- Work in app/orin-legged-vehicle; do not merge into integration or sync other deployments without reporting final scope.
- Store builds/raw benchmarks outside repository. Do not include existing untracked documents in a commit.

## Review focus

- Negative/nonzero window origins and outside-window elevation queries: cache must exactly match original lookup.
- StartAssumedFree must never become a normal-phase landing/support endpoint.
- Snapshot replacement, including same-sized maps and changed costs, must not reuse stale evidence.
- Boundary-distance ties, decimal cell corners and reversed segments must preserve exact clearance.
- Cancel/timeout during cache/index preparation must not turn into successful output.

## Task 1: Record the baseline and add cache contract tests

- [x] Freeze original core source in external baseline-core and reuse immutable old Release library for differential runs.
- [x] Add test/local_query_cache_test.cpp with randomized state/cost/elevation fixtures, negative indices, overrides and independent views; compare literal phase outcomes and original snapshot results.
- [x] New internal src/local/request_cell_cache.hpp exposes Source(GridIndex), TraversalCost(GridIndex), ElevationRangeAt(GridIndex), AdvancePhase(StartPhase,GridIndex). Lazy bounded pages avoid scanning the full map for short paths.
- [x] Prove tests detect missing behavior, implement cache and run the new CTest target.

## Task 2: Integrate query cache and bound edge cache memory

- [x] In src/legged/legged_local_planner.cpp retain Certify checks and replace unordered_map edge memoization with 64 fixed tagged slots; use one request cache through search, terminal and simplification.
- [x] In src/wheel/wheel_local_astar.cpp cache repeated Source/TraversalCost/AdvancePhase lookups per Plan.
- [x] Run all core tests and differential maze/step cases before the next optimization.

## Task 3: Accelerate wheel simplification with exact geometry

- [x] Retain PhaseGeometry allowed mask for Legal, falling back to original lookup before Build.
- [x] Add internal src/local/boundary_distance_index.hpp with exact segment metric, conservative box lower bounds, median-split BVH and cancellation checks.
- [x] Add test/boundary_distance_index_test.cpp: exhaustive oracle versus index on seeded axis-aligned boundaries and disjoint query segments, degenerate/large/decimal cases, empty input and interrupted build/query.
- [x] Keep same supercover and exact clearance acceptance; build index only after phase boundary extraction.

## Task 4: Rich validation and performance comparison

- [x] Run complete Release core CTest and sanitizer core suite; report any failure by name.
- [x] Compare full raw/final paths, statuses and expansion counts against frozen baseline over maze (three resolutions), steps/gaps, costs, unknowns, phase patches, regional goals and repeated map changes.
- [x] Run serial CPU-pinned ABBA benchmarks including initialization, cache/index costs. Report p50/p95 separately by resolution/platform/mode plus no-path and short-path controls.
- [x] Measure peak RSS per isolated baseline/optimized process. Preserve raw CSV, sources, commands, hashes and environment.
- [x] Update docs/validation/2026-09-23-search-optimization-validation.md with results, regressions/rejected approaches and NOT_RUN deployment boundaries.
- [x] Review final diff and git diff --check. Do not auto-commit/merge/push.

## Implementation refinements from validation

- Completely removing edge memoization failed the existing adjacent-goal work-bound test (9 versus 8 certifications). The final implementation keeps 64 direct-mapped slots with full-key validation; no test threshold was relaxed.
- Request caching uses lazy 256-cell pages, limiting short-path setup to a small page directory.
- BVH tests include 15,000 oracle comparisons and interruption during build/query.
- Randomized differential cancellation/deadline fixtures are pre-stopped; a separate 400-run probe verifies interruption after expanded_states > 0.
- Independent read-only review found no blocking correctness issue. All performance evidence and limitations are recorded in the validation report.

## Deployment commit handoff

The user subsequently authorized organizing and committing the workspace to the appropriate deployment branches. The common implementation is f88f3824 on legged, cherry-picked with provenance to wheel/TCP. Keep performance history separate from the fresh rolling-baseline validation recorded in docs/validation/2026-09-23-search-optimization-validation.md. No integration merge or remote push is authorized by this handoff.
