# Navigation repair and cutover implementation plan

> **For agentic workers:** Use superpowers:executing-plans inline; request a fresh review before merging. The user has approved implementation and integration in this conversation; no further plan approval is needed.

**Goal:** Repair navigation lifecycle and partial-map compatibility, replace the formal RL navigation runtime without losing training state, and integrate the same algorithm in all three application branches.

**Architecture:** Preserve application PrepareCycle/ComputeCycle/ApplyCycle asynchronous planning. Fine-map safety notifications precede independent adaptive-map construction; only representations of the current fine snapshot are accepted. Full known routes take priority; partial known-space motion uses a certified prefix toward the unchanged navigation goal, followed by bounded map waiting.

**Tech Stack:** C++20, ROS 2 Humble native, CMake/CTest, Python training controller, Git worktrees.

**Spec:** ../specs/2026-10-15-adaptive-navigation-design.md and ../../validation/2026-10-15-adaptive-navigation-architecture-audit.md; this plan supersedes the original no-integration/no-training-restart restriction under the user's latest authorization.

## Global constraints

- No exploration gain/frontier selection or hidden-map access in navigation; no UNKNOWN executable cells.
- Preserve all existing terrain, platform, stationary-yaw, policy-map service and controller changes in consumers.
- Preserve model, optimizer, replay, records, ClearML identity and training hyperparameters; record navigation provenance and an explicit checkpoint boundary.
- Retain legacy_coarse and immutable pre-cutover runtime for rollback; no automatic algorithm fallback.
- Serial per-package CTest; Humble host is distinct from Orin and actual vehicle verification.
- Preserve unrelated dirty changes and staged state; no push or PR requested.

## Review focus

- Canceled/expired target construction through real ROS Instrument and asynchronous worker must not publish a candidate.
- New obstacle evidence must invalidate an active path even while representation building is blocked.
- Partial map must allow a safe prefix, wait without repeated stationary paths, resume on new evidence and remain cancelable.
- Full known detours must outrank incomplete prefixes; an unknown boundary does not prove the final goal reachable.
- Consumer merges must retain platform/terrain and training-policy services; replay and model state must survive cutover exactly.

## Task 1: Repair lifecycle and map publication

- [ ] Restore application's asynchronous cycle interfaces in feature core and ROS.
- [ ] Add and run failing tests for real controlled-target forwarding and fine safety notification before blocked representation build.
- [ ] Forward original deadline/stop and separate fine safety and navigation-ready events; reject stale built representations; retry incomplete builds.
- [ ] Build and test core and ROS.

## Task 2: Partial-map navigation and shared geometry

- [ ] Reproduce boundary-line safety inconsistency and unknown-goal early termination.
- [ ] Use shared closed supercover; preserve full known-route priority and add certified known-space prefix result without exploration scoring.
- [ ] Test unknown targets, known detours, waiting/resume/cancel, blocked endpoints and deadline behavior.
- [ ] Run all core and ROS tests serially and repeat frozen complex/large closed-loop cases.

## Task 3: Integrate consumers and review

- [ ] Back up dirty application/development files with SHA manifests; stage only navigation task changes.
- [ ] Review final feature independently; address important findings with regression tests.
- [ ] Commit and merge feature into integration, development and app branches, preserving pre-existing changes exactly.
- [ ] Build each distinct affected navigation variant and run package suites; document Orin/vehicle NOT_RUN.

## Task 4: Formal training cutover

- [ ] Build versioned Humble navigation runtime preserving GetPolicyMap and training-specific semantics.
- [ ] Finish/checkpoint current training block or use verified graceful checkpoint stop; protect watchdog with a scoped HOLD.
- [ ] Back up checkpoint/runtime identities and create an explicit audited navigation migration; do not relabel historical evaluations.
- [ ] Switch launcher to versioned runtime, resume same model/optimizer/replay and verify counts/checksums plus actual new executable identity.
- [ ] Remove task HOLD after verified resumed updates and save rollback instructions and final evidence.
