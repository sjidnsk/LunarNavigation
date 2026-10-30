# Execution Safety Repair Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Eliminate false swept-cell collisions, align short references within curvature limits, and bound repeated executor rejection without altering model weights or public arrival tolerances.

**Architecture:** Match the native closed-segment supercover in the DRL plant; use a distance/curvature-aware alignment band in PathExecutor; distinguish stopped execution rejection in the session coordinator and stop repeated rejection at unchanged pose/evidence. Preserve baseline evaluation and run repaired evaluation under separate identity/output.

**Tech Stack:** Python/pytest, C++20/gtest, ROS 2 Humble/colcon.

**Spec:** /mnt/data/WS/lunar-navigation/analysis-artifacts/final-test-rootcause-20261017/ROOT_CAUSE.md

## Global Constraints

- Preserve 0.10m position and 0.05rad yaw public arrival tolerances, actor weights, optimizer and replay.
- No modified-runtime results may reuse or replace baseline evaluation records.
- Work initially remained uncommitted in isolated source/build/install/output directories. The user subsequently authorized commit, merge to the owning development branch, and push to sjidnsk/LunarNavigation.

## Review Focus

- Exact corner contact and travel along a blocked cell boundary must remain rejected.
- Swept tests must agree under reversal/subdivision and fractional/negative map origins.
- Forward/reverse short references must obey curvature limits and strict final arrival.
- Rejection budget must reset on new session or measured progress/new fine evidence.
- Healthy execution must not spend a rejection budget; transport failures stay distinguishable.

## Tasks

- [x] Add regression tests and observe baseline failures for supercover and short-reference controller.
- [x] Implement minimal plant and executor changes; run their affected suites.
- [x] Add stopped-execution rejection tests, implement explicit bounded recovery and ROS trigger mapping; build/test affected Humble packages.
- [x] Reproduce original collision-guard interval and terminal pose; run full frozen-policy moon051, moon053 and cave004 closed-loop smoke cases.
- [x] Preserve baseline identity/results; stage repaired immutable runtime and launch separately attributed complete test evaluation using frozen selected actor.
- [x] Verify active version/process identity and post-repair progress. Write delivery report including NOT_RUN boundaries.

## Verification ledger

- Python baseline: 8 failed / 1 passed; repaired affected suite: 104 passed.
- Reviewer found a feasible lateral-offset regression; extra forward/reverse tests failed before the correction and pass after it. Second read-only review has no blockers.
- Humble core / ROS / controller CTest: 20 / 14 / 7 targets passed. Snapshot test needs a 32 MiB test-process stack; original binary also crashes at the default 8 MiB.
- Exact saved 053 reference now completes at 3.65 s: position error 0.0347566 m, yaw error 0.0082636 rad; public limits unchanged.
- New runtime, output root, actor SHA, and unchanged manifests are recorded in analysis-artifacts/execution-safety-fix-20261018/.

- Three closed-loop smoke cases passed with zero collisions/errors: moon051 99.194%, moon053 99.290%, cave004 99.289%.
- Baseline finished all 200 cases before cutover. Actor/identity/config/resume.pt SHA and metadata unchanged.
- Separate repaired 8-worker 200-case regression evaluation is running; 8/8 workers advance and watchdog reports healthy. Full gate remains pending.

## Follow-up delivery

The initial repaired 200-case evaluation completed with 196 successes. Subsequent clearance/continuous-corner/controller fixes and their boundaries are documented in docs/drl/2026-10-18-navigation-execution-safety.md. Adopted frozen-policy acceptance uses a separate runtime identity/output; no old results are relabelled.
