# Course Two Rebalance Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan inline, with one independent review before launching trials. Steps use checkbox tracking.

**Goal:** Implement and qualify the approved scene/sampling redesign, then start recoverable C3 training from block31 with regression stops. User's latest instruction cancels C0/C1/C2 control runs.

**Architecture:** Keep the original course and evaluation readable, use the explicit C3 variant for new scene and replay behavior, and separate scenario roles from transition phases. The managed controller performs qualification, direct full-baseline fork, preparation, training, evaluation and stopping; it does not dispatch control runs.

**Tech Stack:** Python, NumPy, PyTorch, existing isolated ROS 2 Jazzy and native navigation/sensor backend.

**Spec:** `docs/superpowers/specs/2026-10-11-course-two-rebalance-design.md`.

## Global Constraints

- Full block31 checkpoint SHA `cf58f56b1aaf5411645123ffc76b027eaefa4146391cac6ef2e8ae1fc755f6bc`; preserve original artifacts and pre-change source archive.
- Actor/Critic/targets/optimizers/temperature, reward, 99% completion, navigation and A2 remain unchanged.
- C3 batch 64 = old four tasks 8 each + new cover 16 + approach 8 + finish 8. Finish reserves 4 from actual successful final-16-step windows; global episode cap 4 and no duplicate transitions.
- Old/new admission target 50/50; scene role is not learning phase. Sensor constant for the entire scene and after restore.
- C3 first block: 2,048 learning admissions / at most 512 updates; zero-update prefill at most 4,096 separately reported; no automatic extension after regression.
- Native scene qualification precedes training. Full predecessor observations only; no fabricated known cells or teleportation.
- User said “按此方案推进”; proceed through implementation and verified launch without another permission round.
- Existing isolated feature worktree is retained. No merge, push, PR, cleanup, or unrelated modification.

## Review Focus

- Rare or vanished replay phase on restore: fail or bounded prefill, never silently substitute old samples.
- Overlap between success and ordinary finish pools: deduplicate across both and apply episode cap globally.
- Initialization/prelude reveals or completes the ROI: preserve actual visibility and count only eligible tasks.
- Saved checkpoint from another variant/runtime: reject; cached evaluation binds actor/config/case/runtime.
- Regression or interrupt during evaluation: preserve the exact full checkpoint and resume unfinished evaluation before any new updates.

### Task 1: Preserve baseline and implement balanced replay

**Files:** new `lunar_drl_exploration/course_two_replay.py`, modify `replay.py`; test `test/test_course_two_replay.py`.

**Interfaces:** `PhaseIndex.add(index, transition, scenes)`, `remove(index)`, `status()`, `sample_indices(rng)`, `required_indices()`. Replay exposes opt-in index maintenance, balanced sampling and mandatory snapshot indices.

- [x] Archive pre-change package/scripts/tests and record hashes outside the repository; run current package tests.
- [x] Write/run failing tests for integer phase boundaries, real completion-window proof, batch quotas/cap/dedup, episode-length imbalance and snapshot preservation.
- [x] Implement incremental index, bounded exact quota allocation and snapshot guards; preserve old entry points.
- [x] Run `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ...run.sh python -m pytest -q ros2_ws/src/lunar_drl_exploration/test/test_course_two_replay.py`.
  Expected: all new behavior tests pass.

### Task 2: Build verifiable scene roles and legal preludes

**Files:** new `course_two_rebalanced_scene.py`, update `ros_env.py`; new scene tests and preflight script.

**Interfaces:** `make_case(seed, stage, ordinal, role, ...) -> dict`, `build_case(case, platform) -> (terrain, pose, task, sensor, descriptor)`, `qualify_policy_start(env, case)`; immutable specs and legal recorded predecessor actions.

- [x] Write/run failing tests for inside coverage starts, internal obstacles, required outside routes, bounded U/O approach and immutable sensor/version identity.
- [x] Implement role geometry and predecessor-state construction; native classification and costs recorded, no truth-mask injection.
- [x] Materialize 128 training and 64 development requirements and implement per-attempt rejection logs.
- [ ] Independently certify all native initialization/prelude states (managed qualification running).
- [x] Run scene unit tests and bounded native probes before marking scenes qualified.
  Expected: geometry and native classifications agree with frozen roles; failures remain visible.

### Task 3: Integrate versioned curriculum and full-state recovery

**Files:** `course_two.py`, `training.py`, `config.py`, `checkpoint.py`, `cli.py`; new integration tests.

**Interfaces:** `TrainingConfig.course_two_variant` defaults empty; C0–C3 explicitly fork `course-two-rebalance`; optional zero-update prefill mode; saved sampler rules and scene sequence.

- [x] Write/run failing tests for 16-slot allocation, actual admission/sample counts, variant mismatch and full-state fork preservation.
- [x] Wire role generation, replay readiness, exact C3 sampling and diagnostics; preserve default v1 behavior.
- [x] Make checkpoint save/reload verify required diversity and sampler identities; no carried prefill update debt.
- [x] Run course, checkpoint, replay and controlled training/stop/resume tests.
  Expected: parameters unchanged at fork; counters/RNG/scene sensors recover exactly; no updates when a required pool is absent.

### Task 4: Managed preparation, trial controller and terminal

**Files:** new `course_two_trial.py` and shell entry; reuse `course_two_parallel.py`, `course_two_eval.py`; update terminal diagnostics and tests.

**Interfaces:** `prepare/start/status/monitor/stop/resume` on a new root. Full checkpoint precedes every evaluation; repaired v2 plus frozen extended development cases.

- [x] Test regression stopping, identity-checked cache, interrupt/resume and missing-bucket reporting, reusing existing controller coverage where applicable.
- [x] Implement direct C3 training, 2,048-admission boundaries, full boundary-state save and original model protection; remove control dispatch per the user's latest instruction.
- [x] Display collection, replay, actual learning quotas, prelude cost and evaluation status/ETA in terminal.
- [x] Run control/panel and isolated recovery tests.
  Expected: regression cannot enqueue more training for that arm, and pause never loses the evaluated boundary.

### Task 5: Verify, review and launch

**Files:** new experiment `/home/kai/CodexDownloads/lunar_navigation/drl-course-two-redesign-20261011/`; validation/operations documents.

- [x] Run affected/full DRL Python tests and four native probe types; record pending full qualification and evaluation boundaries explicitly.
- [x] Obtain full initial learning-state identity proof against actual block31.
- [ ] Complete all native qualification, run block31 on frozen development cases and verify prefill supply (managed stages).
- [x] Obtain independent whole-change review, resolve important findings with tests, then freeze runtime identities.
- [x] Start managed C3 training preparation and open the terminal; verify PID, progress, output paths and stop/resume behavior.
- [x] Update design status, operations, validation and this ledger with exact results; do not claim learning improvement before completed evaluations.

## Execution ledger

- 2026-10-11: User approved the design and asked to proceed. Implement inline; all earlier uncommitted course-two work is preserved.
- Ruling: keep this same isolated feature worktree, archive its current contents before changes, and retain the ledger without committing unrelated existing files. This is continued work on the same unintegrated A2 feature; cost if wrong is additional later integration review, not loss of original code or artifacts.
- Pre-flight interfaces: scenes carry immutable curriculum role; replay independently derives action-before coverage phase; checkpoint restores both without relabeling historical transitions. Controller consumes exact full-state boundaries and frozen evaluation identity.
- User scope update: no C0/C1/C2 training or control-only qualification. Current qualification is 128 training + 64 development = 192. C3 forks block31 directly and performs its own prefill. Old control support remains compatible in lower-level modules but is unreachable from this managed run.
- Latest resource ruling: user reported two host crashes at 16 and requested 12. Qualification, evaluation and actual training config now all use 12. Preserve 128 completed qualification files under reviewed exact-hash resource migration; verified live progress 129/192 and 12 active workers. Do not restore the earlier 16-worker plan automatically.
