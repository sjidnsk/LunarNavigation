# DRL bounded training resume implementation plan

> **For agentic workers:** Use superpowers:subagent-driven-development and verify the integrated branch before training.

**Goal:** Resume learned exploration on small scenes while bounding storage and collecting enough reward evidence to diagnose loops.

**Architecture:** Reuse the existing checkpoint, rolling metrics, replay and collector. Add one shared artifact budget and fixed-size online reward summaries. Preserve the policy, reward, discount, optimizer and learning replay; training-only stagnation truncation remains enabled.

**Tech Stack:** Python, PyTorch, ROS 2 Jazzy, Linux file locks, pytest/colcon.

**Spec:** User-approved in-chat design on 2026-09-26; small-scene steering accepted during execution. Existing truncation semantics and validation are in `docs/validation/2026-09-26-drl-stagnation-budget.md`.

## Global constraints

- All new runs in this phase share 8 GiB including atomic-save temporary files; diagnostic files share 128 MiB.
- Rolling metrics <=8 MiB; reward summary <=1 MiB; <=16 cases with <=128 scalar steps each and <=16 MiB total; console logs <=64 MiB.
- Keep historical artifacts and source checkpoint unchanged. No runtime backtracking ban, blacklist, forced exit or loop penalty.
- First run: at most 2,048 new admitted transitions, 60 m moon/cave scenes, decision budget 512; preserve learned state and warmup. Existing `--probe --probe-extent 60 --probe-budget 512` explicitly overrides collection while still updating the learner.
- Integrate only into `dev/drl-exploration`; no push or integration/main changes.
- Statistics begin at this diagnostic fork; checkpoints restore cumulative counters. Cases are selected diagnostic examples, not population estimates. Evaluation disables stagnation truncation.

## Tasks and ownership

### Task 1: Shared storage and continuation boundary

Files: `artifacts.py`, `metrics.py`, `checkpoint.py`, `config.py`, `evaluation.py`, storage/fork tests.

Interface: `initialize_artifact_root`, `atomic_json`, `append_bounded`; `CheckpointManager(..., artifact_root='')`. Configuration adds `artifact_root` and `reward_diagnostics`; explicit fork allowances for resource/diagnostic fields retain strict learning semantics.

- [x] RED: multi-directory sum, old+temporary peak, diagnostic sum, path escape, incompatible budget, explicit fork allowances.
- [x] Implement using shared root manifest and flock, preserving existing no-root behavior.
- [x] GREEN: targeted storage/checkpoint/fork tests.

### Task 2: Fixed-capacity reward evidence

Files: `reward_diagnostics.py`, its tests; root integrates `training.py`.

Interface: `RewardDiagnostics(environments, completion_bonus, success_coverage, state=None)`, `observe`, `reset`, `summary`, `cases`, `state_dict`.

- [x] RED: reward component reconciliation, coverage bins, completed versus censored suffixes, recovery, fixed case/window capacity.
- [x] Implement bounded scalar aggregation with separate success/stagnation case slots.
- [x] Persist diagnostic counters only at valid checkpoint boundaries; publish bounded readable summaries.
- [x] GREEN: module tests and real learner/collector fixture train + resume test.

### Task 3: Bounded launch and logging

Files: `scripts/drl/run_bounded.py`, `ros_env.py` external log option, launcher tests.

- [x] RED: output cap, signal forwarding, child failure, single active wrapper, shared budget.
- [x] Implement one bounded console log and bounded lifecycle records under the common root. Disable Python ROS external logs consistently with native children.
- [x] GREEN: subprocess tests and installed native smoke.

### Task 4: Verify, integrate and resume

Files: training integration tests, operational profile, `docs/DRL开发入口.md`, `docs/操作指令.md`, validation report.

- [x] Enforce dispatch budget before granting goals, so 2,048 admissions cannot overshoot.
- [x] Review branch diff, run package source tests, build isolated Jazzy overlay, colcon tests and installed native smoke.
- [x] Fresh independent code review; resolve important findings and retest affected paths.
- [x] Commit exact paths and fast-forward the clean DRL development line after confirming ancestry/ownership.
- [x] Launch one bounded small-scene training run from the existing trusted checkpoint; verify module paths, source identity, persisted initial checkpoint and live admissions.
- [x] Observe the finite run, record evidence and limitations without claiming convergence from a single run.

## Review focus

- Concurrent checkpoint/log writers and atomic-save peak must share the same byte accounting.
- Resuming old checkpoints must not silently change reward, discount or model identity.
- Reset, recovery, truncation and unfinished episodes must not count as completed reward tails.
- Logging or quota failure must preserve the last good checkpoint and stop owned children.
- Shared worktrees and historical models must remain untouched outside the authorized DRL integration.

## Execution ledger

- Start: `fix/drl-stagnation-budget` at `452ca016`, 20 existing dirty paths from the preceding authorized stagnation implementation; `dev/drl-exploration` at same commit.
- Ruling: use existing small-scene probe overrides instead of changing curriculum semantics or resetting cumulative counters. The run remains genuine training with gradient updates.
- Ruling: preserve the original checkpoint as the immutable baseline; do not copy it into the phase root or prune historical outputs.

- Verification: final colcon package 435 passed / 2 skipped (88.07 s), installed ROS 2 passed (15.66 s), 40 installed module hashes match; independent review clean after two stop-path fixes.

- Integration: implementation commit `046ed473` fast-forwarded into clean `dev/drl-exploration`; tree identical. Started the bounded 60 m run at 2026-09-26T16:15:02Z, wrapper69666/learner69667; initial checkpoint and fork provenance present.

- Startup evidence: 12 new admissions, learner updates 24296→24300, all 8 scenes at 60 m, 2.36 GiB total/115441 B diagnostic. The finite run continues in background; final budget completion and policy-effect evaluation remain pending.

- Completion follow-up (2026-09-27): exit 0 at 2026-09-26T18:01:40Z; exactly 2,048 new admissions, saved transitions 100,260 and updates 24,808. Ten completed episodes (8 cave, 2 moon), eight unfinished episodes at the global budget stop; no episode truncation/collision. A retained cave trajectory still contains 73 zero-gain decisions almost entirely between two positions. The finite training requirement is complete; policy improvement is not inferred from training totals.
- User approved the next frozen screen: both Actor versions 24,296 and 24,808, four matched 60 m scenes (cave/moon × seeds 2026092701/2026092702), 512 decisions each, evaluation stagnation disabled. Separate domains 228/230, same committed implementation/configuration. At most one 16 MiB loop transition per arm, shared existing artifact budget. Detailed evidence and the conditional next decision belong to `docs/validation/2026-09-27-drl-frozen-screen.md`.

- Frozen screen completed: both arms exit 0, four matched cases each, Actor hashes unchanged; completion 2/4→3/4, no old-success regression, collision/error zero. Total decisions 1306→819. Difficult cave remains at 98.572% with an AB loop, so this is a development screen improvement, not loop resolution or generalization proof.
- Separate sampled-policy diagnostic: the same latest Actor in the difficult cave exhausted 512 decisions at 97.3795%, max zero-gain run 369. No model update or action override; excluded from the four-map comparison. All evaluation children closed before resuming.
- Conditional continuation gate passed; launched one additional <=2048-transition run at 2026-09-27T03:34:21Z, wrapper244581/learner244582. Resume from 100260 transitions / 24808 updates, same 60 m / 512-step collection and all learner/reward settings. Archive `evaluation/phase1-summary.json` preserves phase-one totals and exit evidence without another full checkpoint copy. Phase-two final outcome remains pending.
