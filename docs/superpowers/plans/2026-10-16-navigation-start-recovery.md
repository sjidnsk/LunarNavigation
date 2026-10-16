# Navigation Start Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Repair temporary-start support and phase-consistent planning, verify stopped-pose recovery, and adopt only at the saved block-14 boundary while preserving the complete formal training state.

**Architecture:** Real map evidence remains immutable. Request-local navigation separates movement assumptions from derivative support. Search and simplification share phase-aware continuous edge legality. The controller retains physical limits; change its short-segment behavior only if recovery verification still requires it. DRL records failure context without inventing next states.

**Tech Stack:** C++20 core, ROS 2 Humble, Python SAC runtime.

**Spec:** User-approved four-layer repair design in this conversation; captured evidence at `/mnt/data/WS/lunar-navigation/analysis-artifacts/input-unavailable-016-20261016/code-guided-diagnosis.md`.

## Global Constraints

- Preserve model, optimizers, RNG, replay contents, sampler, counters, curriculum, run ID, ClearML task and historical evaluation records.
- Keep all training configuration bytes unchanged, including 0.10 m / 0.05 rad arrival, sensor and physical curvature limits.
- No in-place runtime library/source replacement. Build and stage a new immutable runtime.
- No assumptions promoted into measured global map or derivative-only halo promoted into movement support.
- Stop only after the active block-14 trainer naturally finishes and saves; resume only after validation and identity adoption.
- Common navigation changes originate from integration; DRL-only diagnostics and adoption tooling stay on the DRL line. Retain all old branches and worktrees; no remote push.

## Review Focus

- Fractional-cell anchors near blind-zone edges must retain valid support without erasing genuine uncertainty.
- Real obstacles, unsupported geometry and incomplete physical measurements remain authoritative.
- Prefix-to-evidence transitions are monotone; diagonal corner contacts obey the same rule in search and simplification.
- Canceled or naturally failed navigation at a valid stopped pose can create a real transition; unavailable next states are never fabricated.
- Old evaluation proofs remain attached to their old runtime; new evaluation does not reuse them as current proof.

## Tasks

### 1. Request-local support

Files: `src/local/request_local_start_patch.cpp`, new `test/request_local_start_patch_regression_test.cpp`, package CMake test registration.

- [x] Add a portable height-only common fixture and a captured per-center-statistics DRL fixture; shifted starts recover and persistent evidence remains unchanged.
- [x] Observe failure on the old implementation.
- [x] Add derivative support closure around the bounded assumption domain; derivative-only centers remain intrinsically unknown, with measured evidence taking priority.
- [x] Run existing start-patch and fine-map suites plus regression tests, including blocked, boundary and missing-plane negatives.
- [x] Commit only common navigation changes.

### 2. Shared edge legality

Files: `local_planning.hpp`, `src/local/grid_supercover.hpp` or a focused shared local utility, `src/local/phase_aware_path_simplifier.cpp`, `src/wheel/wheel_local_astar.cpp`, affected tests.

- [x] Add real corner-contact regressions and monotone-phase negatives; observe old failure.
- [x] Make search certify actual endpoint segments and reuse the same continuous source/phase rules as postprocessing.
- [x] Verify alternative legal paths, phase transitions, narrow corridors and long-route benchmarks; commit.

### 3. Closed-loop execution recovery and diagnostics

Files: controller execution tests/source only if necessary; DRL `ros_env.py` and affected tests in a separate development worktree.

- [x] Run captured stopped-map replay and isolated Humble real-controller tests with the first two fixes.
- [x] If short-corner recovery still fails or loops, add a failing controller test and bounded certified-segment alignment recovery, preserving physical limits.
- [x] Add complete goal/result/pose/start-status/revision failure evidence with no stale-state replay fallback; test failure logging cannot replace the original error.
- [x] Verify natural recovery, cancellations, backward motion, U-turns, cave and large lunar scenarios.

### 4. Review, preservation and boundary cutover

- [ ] Review whole diff and run affected core/ROS/controller/DRL suites, sequential performance cases and source/boundary checks.
- [ ] Stage frozen runtime, native libraries and manifest; certify new runtime in isolated ROS domains.
- [ ] At naturally saved block-14 boundary verify checkpoint serialization and full replay count/hash; back up checkpoint and all identity/history records.
- [ ] Adopt new runtime with unchanged config/checkpoint, audited historical epochs and same run/ClearML task.
- [ ] Resume and prove sampled replay, incrementing updates, unchanged schedule continuity and active runtime hashes.
- [ ] Report exact verification limits; Orin/real vehicle remain NOT_RUN.

## Execution Ledger

- Task 1 common commit `254c0237`; training port `0516b839`. Task 2 common `ff68191f` + deadline follow-up `3760ba19`; training ports `531af85c` / `3634b137`. DRL diagnostics and captured-statistics regression `612d3015`.
- RED/GREEN evidence is stored outside Git in `/mnt/data/WS/lunar-navigation/analysis-artifacts/navigation-recovery-cutover-20261016/`. Common 19 CTest suites and final DRL 20 CTest suites pass. Final ROS 14 suites pass; identical controller source passes 154 tests. Failure diagnostics pass 67 targeted tests, plus 14 source/identity tests.
- Five actual failed maps preserve all fine-map classifications and recover 17 native start connections each; all 20 tested goal plans succeed. Original four goal coordinates succeed after physically replaying the original two initialization spins. Six matched canceled poses, including the original approximately 0.145 m stopped location, produce real constructable inputs.
- Final frozen Python package has 79 files: only `ros_env.py` and `ros_messages.py` differ from the previous deployed package, plus the new failure-only `navigation_failure.py`. No model/learner/sensor/reward/config source changes. Controller execution hashes are identical. Candidate runtime: `db923a3d24373e8f5d67a01901ae659c9b6997d40aa15993f2a5e8e69e6e47b1`.
- Final frozen environment executes five actual Actor decisions each on moon and cave without infrastructure errors. Formal service still runs the old environment; no live launcher/config/checkpoint has been modified.

Ruling: Keep the common regression independent of per-center terrain measurements — integration does not expose the DRL-only measurement API — the captured measurement fixture is additionally tested on the DRL development line; no DRL API is imported into common navigation.

Ruling: Retain the existing physical controller unchanged — exact original goals and stopped-pose replan recovery pass after the two core fixes — changing curvature limits or alignment behavior would add unneeded behavior changes.

Ruling: Replay original initialization rotations only in matched diagnostic trials — corrected start support allows the normal bounded initialization to finish earlier — direct old targets can otherwise be unknown solely because fewer directions have been observed; production initialization parameters remain untouched.

Ruling: Preserve cooperative cancellation in the shared geometry helper — short paths previously skipped this check and moving the helper removed per-cell polling — a separate RED/GREEN deadline regression covers the correction.

Ruling: Increase only the regression runner stack to 64 MiB — an existing snapshot test exhausts the default 8 MiB stack in its large local test arrays — production stack and navigation parameters are not changed.

Ruling: Rerun socket/DDS tests with host access and isolated domains — sandbox networking denies the local IPC transport and ROS initialization — sandbox failures remain recorded and are not counted as algorithm failures or passes.

Ruling: Establish a new explicit source manifest epoch for the intentional implementation change — the previous 78-file manifest correctly rejects modified files and the added diagnostic module — retain the old manifest in the cutover evidence and keep the active experiment identity authoritative until saved-boundary adoption.
