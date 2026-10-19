# MPPI performance optimization

Authorized execution: implement the accepted profiling / behavior-preserving optimization / equivalence / short concurrency gate workflow directly in the existing MPPI topic worktree. No commit, merge, production cutover or actor200 run in this task.

Frozen: 32 samples, 40 x .05s horizon, iterations1, noise, temperature, cost weights, public arrival, speed limits and every geometric/braking safety condition. Preserve prior causality artifacts; snapshot the current package as the comparison oracle before edits.

1. Profile serial offline straight/wall/corner/offset fixtures. Record cumulative/self times and call counts. Separately preserve ROS uninstrumented deadline statistics.
2. Based on measured hotspots, add a behavioral equivalence harness comparing the frozen package with current package on fixed states/histories/seeds/map updates, including forward/reverse and unsafe cases. Add a failing resource-work regression for the selected redundant operation before changing it.
3. Remove only proven redundant computation or allocations. Retain sampler and all safety semantics. Compare commands, phases, admissibility and full closed-loop traces; abort optimizations with changed decisions.
4. Run controller and affected interface regressions; build isolated Humble overlay. Serial paired baseline/current timings, then proper-driver ROS six fixtures. Run a bounded eight-worker actual ROS pressure comparison using private domains160–167 (baseline wave then optimized wave). Distinguish these fixed-reference controller scenarios from full eight-environment exploration evaluation.
5. Request read-only code review, fix findings, preserve final source and hashes, report measurements and remaining hardware/actor200 boundaries. Update registry as paused/uncommitted pending integration.

## Measured refinement

Phase1 removed duplicated final braking work and batched invariant rollout operations. 480 dtype/history/delay cases were bitwise equal; all18 closed-loop fixtures matched. Serial ROS baseline vs phase1: P95 26.25 ->15.58ms, but deadline misses12/1881 vs11/1913, so this alone did not resolve tails. Per-stage wall/thread CPU profiling localized fallback ticks54–68ms mainly to braking43–52ms, with CPU approximately wall time.

Phase2 therefore preserves full trajectory geometry/dynamic checks, orders the existing fallback set by stable finite costs and fully certifies candidates until the first safe choice. The selected argmin and all per-candidate delay/braking conditions must remain unchanged. Verify tied costs, fine-grid/tangent boundaries, all-failed and expensive-only-safe cases, then repeat final ROS and eight-worker comparisons. Keep phase1 outputs separate; do not call its tails solved.


Worst-case refinement: 5mm grid, short synthetic reference-control horizon, 12 candidates with 11 identical unsafe first targets. Before in-call certificate reuse, lazy search repeated the same failed command. A failing regression established this; exact-byte keys now reuse only within one call, with unchanged state/map/history/dt. Both all-failed and last-only-safe cases matched the full baseline; observed certificate CPU ~59–73 ->11–13ms. Four resolutions and five tangent offsets (20 cases) matched baseline selection and pose. Read-only review found no remaining blocker; full final measurements still required.

## Final outcome

Adopted algorithm retains frozen sampling/importance loop, complete scoring, first-target projection and full brake kernel ASTs. 480 rollout cases,120 candidate-selection cases,20 fine-grid/tangent cases matched; adopted18 closed-loop fixtures matched baseline traces bitwise. Source/applicable regressions318 passed; isolated Humble3-package build passed. Final serial ROS6/6,1951 observed solves,0 deadline misses; eight actual ROS workers8/8,0 collisions, P95 22.44->13.09ms, P99 32.66->18.51ms, misses13/2462->2/2583, max115.64->41.32ms. The two remaining deadline misses are retained; no hard-real-time claim. Actor200 and Orin/hardware not run. Details, scripts, manifests and recoverable source: `/mnt/data/WS/lunar-navigation/analysis-artifacts/mppi-performance-20261018/REPORT.md`. No commit/merge/push/cutover; paused with uncommitted recovery pending integration decision.
