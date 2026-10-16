# Formal DRL Version Consolidation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Commit the complete currently deployed candidate_geometry_v3 training implementation and integrate it into dev/drl-exploration without changing the running experiment.

**Architecture:** Treat the active frozen Python package as the behavioral authority. Preserve the already integrated public navigation implementation; import only corresponding DRL tests, launcher support, and documentation from V5 and scoped repair sources. Bind the resulting source to its deployed identity with a file-hash manifest and read-only verification tool.

**Tech Stack:** Python 3.10, PyTorch SAC, ROS 2 Humble, C++20 native extensions, Git worktrees.

**Spec:** User authorization: follow the audited ordering, completely commit the current formal version and merge it into dev, preserve models/replay/training records, do not affect running training, do not delete old branches/worktrees. Evidence: analysis-artifacts/training-worktree-audit-20261016/training-worktrees.md (outside repository).

## Global Constraints

- No write to active frozen source, launcher, service, configuration, model, replay, evaluation, or experiment metadata.
- No stop/restart or training migration; preserve all old worktrees, branches, and existing local changes.
- Integration destination is dev/drl-exploration only; no push, PR, integration/application merge.
- Diagnostics must avoid reserved training domains180..187 and evaluation216..223; use empty domains157..159 when needed.
- Record source provenance; import no checkpoint, model, cache, generated runtime artifact, credential, or machine-specific experiment state into Git.

## Review Focus

- A clean Git HEAD alone may omit untracked V5 architecture: verify all78 active Python files and exact file set.
- New frozen-only error-policy and budget code may be absent from tests: include their scoped regression tests.
- A launcher may silently import another overlay: verify imported package root and runtime digest.
- Imported historical scripts/tests may reference obsolete paths or APIs: run affected suite and CLI checks.
- Consolidation must not mutate production or source worktrees: compare before/after immutable file hashes and old worktree statuses; allow normal training state growth.

### Task 1: Capture authority and isolated baseline

**Files:** Create docs/drl/2026-10-16-formal-version-consolidation.md and config/drl_candidate_geometry_v3.yaml; external evidence directory analysis-artifacts/drl-formal-consolidation-20261016.
**Interfaces:** Consumes deployed package, runtime identity, user authorization; produces immutable before-manifest and scoped source inventory.

- [x] Record active source hashes, runtime/config identities, service/experiment paths, and old worktree statuses without writing to them.
- [x] Run baseline tests against dev in the new worktree, using the matching Humble/native install.
- [x] Document copied sources and exclusions; keep active machine paths in external evidence only where possible.

### Task 2: Restore complete committed training implementation

**Files:** Modify ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/*.py, corresponding test/*.py, necessary scripts/drl/*; create docs/drl/formal-runtime-source-manifest.json, scripts/drl/verify_formal_source.py and test/test_formal_source.py.
**Interfaces:** Consumes78-file authority and V5 support inventory; produces source-complete candidate package and read-only verifier returning mismatch details/nonzero on drift.

- [x] Write manifest verifier regression tests and observe missing verifier failure.
- [x] Copy exactly active package Python files; merge committed budget ancestry while retaining complete active ros_env implementation.
- [x] Copy relevant DRL tests/support from V5 and narrowly scoped error-policy/budget sources; preserve unrelated native/public files.
- [x] Implement read-only file-set/hash verification; test valid manifest, changed bytes, missing/extra source file, traversal rejection.
- [x] Check source set and hashes, runtime digest, explicit candidate configuration, CLI and affected package regressions; build affected native extensions in an independent external directory if necessary for package qualification.
- [x] Commit explicit source/test/support paths only, with recorded provenance and evidence.

### Task 3: Verify and integrate dev

**Files:** Update docs/DRL开发入口.md and validation summary; no active runtime files changed.
**Interfaces:** Consumes reviewed candidate commits and passing validation; produces dev at the exact validated commit plus before/after noninterference evidence.

- [x] Perform fresh read-only whole-branch review; resolve material findings without changing deployed behavior.
- [x] Confirm dev remains clean and unchanged since capture; merge --ff-only the validated branch into dev as explicitly authorized.
- [x] Reverify78-file manifest, package import/runtime identity, configuration and affected tests from dev.
- [x] Verify active service/source/config identities and old worktree changes remain intact; report any legitimate training progress separately.
- [x] Keep all old branches/worktrees; record remaining deployment/cwd/cache migration as future work.
