> 历史分支记录：`feat/drl-exploration-training` @ `c6473ae70eb4`。本文运行状态、接口和实验结论是当时快照；当前开发以 [统一入口](../../DRL开发入口.md) 为准。

# Path Tracking Executor Implementation Plan

> Execute inline using superpowers:executing-plans; each task uses a separate test cycle.

**Goal:** Replace the incremental path follower with a coherent stateful executor and close its navigation failure loop.
**Architecture:** Immutable polyline geometry, ROS-free execution state machine, thin ROS adapters, revision-bound feedback.
**Tech Stack:** Python3.10+, C++20, ROS2 Jazzy/Humble, existing pytest/gtest infrastructure.
**Spec:** [设计](../specs/2026-09-09-path-tracking-executor-design.md).

## Global Constraints

- vx positive/negative and yaw rate may coexist; vy=0; abs(vx)<=0.2 m/s.
- No second vehicle command publisher, no simulator truth in controller, no new planner in controller.
- Use existing platform parameters, preserve old training artifacts and other worktrees.
- Tests from repo root; CTest serial by package; explicit-path commits and UTF8 readback.

## Task 1: Geometry and execution core

Files: create controller python/path_geometry.py and execution.py, test/test_execution.py; update tracking.py.
Interfaces: Polyline(points).project(x,y,index,reach) -> Projection(index,s,x,y,error); point_at(s) -> (x,y).
PathExecutor(policy).set_path(points, identity=None, final=True); update(TrackingState,dt)->ExecutionResult;
clear(); results contain command, phase, progress_m, cross_track_m, heading_error_rad, direction.

- [x] Write tests and verify RED: long-line projection, behind-start turn, reverse, corner, speed bounds, stopped completion, stale revisions.
- [x] Implement finite geometry validation, bounded progression, direction lock, stop/align/follow/final stages and constrained commands.
- [x] Run existing tracking and new execution tests, validate positive/negative motion and E7 geometry with actual MotionPlant.

```python
executor = PathExecutor(TrackingPolicy(max_linear_mps=.2))
executor.set_path(((0,0,0),(3,0,0)))
result = executor.update(TrackingState(1.5,0,0), .05)
assert result.command.failure_reason is None
```

## Task 2: ROS identity and feedback

Files: TrackingStatus.msg and msgs CMakeLists; controller node.py/frame.py/tests/CMakeLists;
incremental_navigation_node.cpp and node tests.

- [x] Add failing identity/stale-input/feedback tests.
- [x] Adapt incremental_path and new incremental_reference modes to the same executor; preserve legacy trajectory mode.
- [x] Publish status only for current reference; navigator accepts only matching stopped FAILED status once per revision and replans.
- [x] Build messages and affected packages; run node tests and check no vehicle topics appear in isolated simulation parameters.

```text
FAILED(session A, revision 2) + active(A,2) -> original coordinator deviation/replan
FAILED(session A, revision 1) + active(A,2) -> ignored
```

## Task 3: Training execution progress

Files: DRL ros_env.py/config.py/config YAML/progress watchdog module/tests.

- [x] Test progress watchdog with forward progress, angular progress, stationary repeated updates and actual expiration.
- [x] Enable revision-bound reference/status endpoints in isolated env; replace entire-goal wall timeout with progress-aware watchdog.
- [x] Keep infrastructure faults separate from task success and sampling truncation; preserve old checkpoint identity rules.

## Task 4: Closed loop and delivery

- [x] Run Python suites, affected Jazzy build/package tests, real ROS original3 plus E7/reverse/corner coverage.
- [x] Run separate Humble build/tests; fix failures without weakening closed-loop assertions.
- [x] Update README, operator guide, interface YAML, prior audit status and new validation evidence.
- [x] Read UTF8/AST/YAML, diff-check and explicit-path commit; no active training/test processes remain.


## 执行记录

2026-09-09 已完成上述四项，具体计数及首次失败/复核结果见
[验证记录](../../validation/2026-09-09-path-tracking-executor.md)。
闭环过程中补全了两个接口契约：反馈启用时须等待停稳完成才能切段/报最终成功；
非最终段终点朝向不能跳过，已完成局部目标不允许无限重复发布。
E7实际ROS验收限定为原连续目标前4m，完整8m以上旧参考另以真实运动模型和岩石碰撞回放验证。
