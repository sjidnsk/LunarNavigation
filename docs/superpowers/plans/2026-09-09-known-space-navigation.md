> 历史分支记录：`feat/drl-exploration-training` @ `c6473ae70eb4`。本文运行状态、接口和实验结论是当时快照；当前开发以 [统一入口](../../DRL开发入口.md) 为准。

# Known Space Navigation Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Remove independent frontier fallback from navigation; execute continuous known-space observation poses through platform-certified complete routes and bounded route prefixes.

**Architecture:** The exploration layer chooses the observation pose. The existing platform search kernel computes a complete route on the persistent fine map using sparse search storage; the local target selector only extracts a prefix of that route inside the configured local window. Local solvers retain their native wheel/legged constraints. Coarse guidance remains a map product, not an independent source of executable destinations.

**Tech Stack:** C++20, ROS 2 Jazzy/Humble, Python training environment.

**Spec:** [Known space navigation design](../specs/2026-09-09-known-space-navigation-design.md), approved by the user on 2026-09-09.

## Global Constraints

- Preserve 0.2 m/s motion, forward/reverse turning and in-place rotation; no lateral motion.
- Preserve 8 training environments, 10x simulation setting, 30-minute checkpoints and existing running process/artifacts.
- Keep fine local planning window at the configured 64 m; sparse route search must not allocate a dense task bounding rectangle.
- Unknown/blocked goals and disconnected known-space routes return explicit failures, not substitute goals.
- Keep terminal yaw, start-prefix evidence restrictions, directed legged certification and invalidated reference handling.
- No changes to the shared integration worktree; validation outputs live outside the repository.

## Tasks

- [x] Add failing regressions for unknown/blocked goals, missing-route fallback and two/three-point loop geometry.
- [x] Reuse native platform searches for complete known-space routes with sparse search storage; verify long corridors, unknown barriers, corner constraints and detours.
- [x] Replace independent candidate selection with route-prefix selection. Remove frontier heuristics.
- [x] Rewire navigation session/ROS ports and truthful planning diagnostics. Keep execution completion semantics.
- [x] Update training failure handling/tests and document the changed action contract.
- [x] Run affected Python tests, isolated Jazzy build/package tests and real closed-loop checks. Run Humble checks if the available container supports them; report Orin/vehicle as NOT_RUN.
- [x] Update README/operator instructions and validation evidence, explicitly distinguishing the running old training from validated new binaries.

Validation evidence: [known-space navigation](../../validation/2026-09-09-known-space-navigation.md).
