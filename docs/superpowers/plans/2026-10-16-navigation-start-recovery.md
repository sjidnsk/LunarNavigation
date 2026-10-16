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

- [x] Review whole diff and run affected core/ROS/controller/DRL suites, sequential performance cases and source/boundary checks.
- [x] Stage frozen runtime, native libraries and manifest; certify new runtime in isolated ROS domains.
- [x] At naturally saved block-14 boundary verify checkpoint serialization and full replay count/hash; back up checkpoint and all identity/history records.
- [x] Adopt new runtime with unchanged config/checkpoint, audited historical epochs and same run/ClearML task.
- [x] Resume and prove sampled replay, incrementing updates, unchanged schedule continuity and active runtime hashes.
- [x] Report exact verification limits; Orin/real vehicle remain NOT_RUN.

## Execution Ledger

- Task 1 common commit `254c0237`; training port `0516b839`. Task 2 common `ff68191f` + deadline follow-up `3760ba19`; training ports `531af85c` / `3634b137`. DRL diagnostics and captured-statistics regression `612d3015`.
- RED/GREEN evidence is stored outside Git in `/mnt/data/WS/lunar-navigation/analysis-artifacts/navigation-recovery-cutover-20261016/`. Common 19 CTest suites and final DRL 20 CTest suites pass. Final ROS 14 suites pass; identical controller source passes 154 tests. Reviewed failure diagnostics pass 68 targeted tests. The full final frozen DRL suite passes 1513 tests, with 2 skipped; the optimized-interpreter warning is retained in the log.
- Five actual failed maps preserve all fine-map classifications and recover 17 native start connections each; all 20 tested goal plans succeed. Original four goal coordinates succeed after physically replaying the original two initialization spins. Six matched canceled poses, including the original approximately 0.145 m stopped location, produce real constructable inputs.
- Final frozen Python package has 79 files: only `ros_env.py` and `ros_messages.py` differ from the previous deployed package, plus the new failure-only `navigation_failure.py`. No model/learner/sensor/reward/config source changes. Controller execution hashes are identical. Reviewed candidate runtime: `8470d67be1e6e5fc2dfa7e0a3351017bff35b07c630a9bd40246ba34c83b09da`. The first candidate is retained separately as evidence, not modified in place.
- Final frozen environment executes five actual Actor decisions each on moon and cave without infrastructure errors. Formal service still runs the old environment; no live launcher/config/checkpoint has been modified.

Ruling: Keep the common regression independent of per-center terrain measurements — integration does not expose the DRL-only measurement API — the captured measurement fixture is additionally tested on the DRL development line; no DRL API is imported into common navigation.

Ruling: Retain the existing physical controller unchanged — exact original goals and stopped-pose replan recovery pass after the two core fixes — changing curvature limits or alignment behavior would add unneeded behavior changes.

Ruling: Replay original initialization rotations only in matched diagnostic trials — corrected start support allows the normal bounded initialization to finish earlier — direct old targets can otherwise be unknown solely because fewer directions have been observed; production initialization parameters remain untouched.

Ruling: Preserve cooperative cancellation in the shared geometry helper — short paths previously skipped this check and moving the helper removed per-cell polling — a separate RED/GREEN deadline regression covers the correction.

Ruling: Increase only the regression runner stack to 64 MiB — an existing snapshot test exhausts the default 8 MiB stack in its large local test arrays — production stack and navigation parameters are not changed.

Ruling: Rerun socket/DDS tests with host access and isolated domains — sandbox networking denies the local IPC transport and ROS initialization — sandbox failures remain recorded and are not counted as algorithm failures or passes.

Ruling: Establish a new explicit source manifest epoch for the intentional implementation change — the previous 78-file manifest correctly rejects modified files and the added diagnostic module — retain the old manifest in the cutover evidence and keep the active experiment identity authoritative until saved-boundary adoption.

Final review: One independent whole-branch review found three Important issues and no Critical issues. All three entered one fix pass; no second reviewer was dispatched. The author regraded them as Important because they could prevent the promised cutover, prevent complete rollback, or lose current failure evidence.

- Final: fixed string/integer process-birth comparison — ownership tests RED→GREEN, including dead and reused PID cases.
- Final: fixed incomplete evaluation-move rollback — complete intent is persisted before moving anything; injected second-move failure and idempotent rollback tests RED→GREEN. Cutover safety suite: 6 passed.
- Final: fixed post-result stop failure losing the current action result — session tokens bind captured result and execution context to the current attempt, including failed-begin isolation; targeted diagnostics suite: 68 passed. Product commit `f5fa4023`.

Ruling: Repair only the signal-test fixture handshake — production `PYTHONOPTIMIZE=1` strips blocking I/O hidden inside the spawned fixture's asserts, reproducing the same failure on the old frozen runtime — explicit waits fix all 15 signal-boundary cases without changing training code. If this ruling were wrong, a production atomic-update issue could be masked; the unchanged old runtime passing the corrected real-signal cases and exact schedule/learner/replay checks distinguish that risk.

Ruling: Keep old experiment identity authoritative until activation — the reviewed candidate has its own frozen source epoch, launcher and binary hashes — a mistaken identity transition could invalidate old evaluations; immutable historical epochs and full-file hashes are required at adoption.

- Saved-boundary activation: after the user notified block completion, verified natural block-14 completion at 116605 transitions / 28672 updates, no trainer running. Paused evaluation only. Preserved checkpoint SHA `7a8979e855aee409ca64c299ac70915cf6909fdc6ec147d4be65e5264d5c51b0`, all 13540 replay entries and serialized replay SHA `c4b34a00c49315b1c619df814ec4fcefebaaece8cca4d1f3eb2324b9ef79f71c`. Independent backup and 1879 historical file hashes are retained.
- Activated reviewed runtime at 2026-10-16 18:15:22 Asia/Shanghai; same run and ClearML task. Eight live native workers load the final new libraries; old partial block-14 evaluation and live data are archived. The new runtime reruns block-14 quick evaluation before normal block-15 learning. The scoped watchdog HOLD was released only after live native runtime verification.

Ruling: Use the user-notified already-saved boundary rather than restarting the old observer — the observer was explicitly disarmed at the user request; trainer completion and absence were independently reverified before pausing evaluation — a mistaken boundary would risk interrupting new collection, so exact saved schedule, empty in-flight credit, trainer absence, evaluation phase and unchanged checkpoint stat were all checked.

Task 4: complete — natural saved block 14 was independently verified following user notification; full checkpoint and history preservation, runtime adoption and resumed sampling/CUDA updates passed. `post-resume-training.json` verifies restored replay 13540/13540 with SHA unchanged, restored Actor/Q checksum unchanged, actual 32/32 balanced sampling of 64 unique transitions from 64 episodes, and increments to 116637 transitions / 28680 updates. All 8 learner-owned native workers load the final new navigation libraries. Numeric guard version 1 has checked every update; losses and gradients are finite. The new-runtime block-14 quick evaluation completes 40/40 successes, zero collisions, infrastructure errors and excluded identities; this is operational/fixed-case evidence, not a convergence or generalization claim.

Historical preservation was rechecked after restart: 1879/1879 files match, including 1852 continued originals, 24 archived old evaluation rows and 3 independent before-copies for logs that have since continued. No missing historical file; all old branches/worktrees retained, no remote push. Host Humble verification is complete; native Orin, real vehicle and external TCP simulator remain NOT_RUN.
