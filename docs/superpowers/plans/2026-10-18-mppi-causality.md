# MPPI failure causality investigation

Reuse MPPI-20261018 and feat/mppi-controller-20261018. Preserve all prior artifacts; no production cutover/commit/merge.

Frozen: batch32, horizon40×0.05s, iteration1, temperature0.15, noise0.12/0.35, all cost weights, public arrival0.1m/0.05rad/stopped, speed±0.2m/s.

1. Instrument baseline ROS publication reasons, paired state/TF stamps, solver inputs/results and history. Reproduce straight/final-yaw/corner before fixes.
2. Use observations to test separate hypotheses: callback-order stops; mismatched nominal/actual execution timeline; stricter-than-public completion. Record rejected hypotheses too.
3. Write failing behavioral regressions; implement only causes established by intervention. Keep experiment variants out of production switches.
4. Paired A/B with same maps/poses/seeds, serial execution, held CPU thread limits; timing-only / terminal-only / combined if justified. Repeats and return-to-baseline control.
5. Full controller tests plus targeted DRL/native checks; raw traces, manifest, report, task registry and recoverable snapshot. Explicitly distinguish toy ROS benchmarks, native action smoke, actor200 and hardware NOT_RUN.


## Outcome

Completed isolated causality investigation and experimental repair. The earlier ROS 1/6 is confounded by the benchmark callback driver: unchanged old MPPI passes 6/6 with continuous callback processing. Proven defects: source/receipt-clock mixing and latest-only state pairing; hidden half-size arrival tolerances; candidate-versus-published-target model mismatch. Fixed targets independently expose 0.105m prediction error under 2tick transport + 1tick observation age. A 631/631 command replay reproduces the remaining corner stall. Unified command projection/model, public completion and paired-state input fix are adopted only in this topic worktree. A publish-only prototype was rejected by the delayed-straight regression.

No sampler/weight change: MppiPolicy and complete scoring AST unchanged. Current tests: 303 passed; isolated Humble build: 3 packages. Final unified model ROS six fixtures 6/6 plus five repeats 5/5, no collision. Corner alternation: timing/arrival-only 2/3 vs unified 3/3; small-N evidence, not a general success-rate estimate or universal speedup. Current code static offline16/16 plus two safe map-update stops. Native smoke results, final manifests and source recovery snapshot are in the report below.

Evidence: `/mnt/data/WS/lunar-navigation/analysis-artifacts/mppi-causality-20261018/REPORT.md`. Formal actor200, Orin/hardware, variable-rate trajectory validation and upper-level map recovery remain NOT_RUN. No production cutover, commit, merge or push; keep recoverable uncommitted work pending integration decision.
