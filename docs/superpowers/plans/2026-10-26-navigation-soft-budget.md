# 局部软周期与计算效率 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 取消超 50 ms 即制动的机制，并减少距离场和认证的重复计算，用短复杂场景验证实际改善。

**Architecture:** 保持同步离线 tick、异步全局 worker 和独立认证；时间为统计，安全有效性决定动作。地图及认证分别管理自己的不可变计算缓存。

**Tech Stack:** Python / NumPy / SciPy、acados、已有 C++ 几何库。

**Spec:** docs/superpowers/specs/2026-10-26-navigation-soft-budget-design.md

## Global Constraints

- 同一 NAV-1CM-20261026 专题及工作树；只修改本实验包及相关测试/文档；不得提交、合并、推送、删除或接车辆。
- 局部窗口始终严格车体中心 5 m × 5 m、500 × 500、0.01 m，原始 UNKNOWN 保留，可配置是否允许未知。
- 原档与 .5 m/s / .5 rad/s 独立高档限值不变；所有实际动作通过当前地图、实测初态、连续碰撞、动力学与停车尾段认证。
- 50 ms 是软性能预算，超过不能单独触发制动；完整真实耗时、慢样本和失败均保留。
- 实现子任务串行，不改其他任务文件；Windows 使用任务 deps/windows，Orin 使用 P4_veri 内独立依赖。
- 不推断 ROS、上游 P3、真实执行器或车辆通过；原同步离线模型不宣称覆盖执行延迟。

## Review Focus

- 有效移动候选超过 50 ms 后是否仍被执行，是否有残留 49 ms 门禁。
- 时间统计命名是否还把允许的迟到移动当作策略错误。
- 缓存是否可能以旧 origin/epoch 或未知策略掩盖新危险。
- 认证上下文误传其他 grid/backend 是否明确拒绝。
- 性能改善是否来自减少重复计算，完整轨迹/动作安全检查是否仍实际执行。

### Task 1: 取消时间制动与更正证据语义

**Files:** scripts/navigation/one_cm/nav1cm/coordinator.py；scripts/navigation/one_cm/benchmark_navigation.py；tests/navigation/one_cm/test_coordinator.py。

**Interfaces:** NavigationCoordinator.tick / CycleResult 公共接口保持；deadline_miss 只为统计；benchmark 明确记录 late certified motion，不把其作为策略违例。

- [x] 确定性时钟失败测试：完成时间大于 50 ms 的有效 MPCC 候选与动作仍有效执行；49 ms 也无额外门禁。
- [x] 删除 dispatch_reserve_s / MOTION_BUDGET_EXCEEDED 对动作选择的作用。仅 candidate/action 无效等真实原因才制动；保留耗时与 LOCAL_DEADLINE_MISS 观测事件。
- [x] 更新基准软预算策略标识、迟到且认证有效动作计数和测试；不伪造 strict_50ms_pass。
- [x] 运行定向单测，保存 RED/GREEN 和变更报告。无需 Orin 批次，由主 agent 统一执行。

### Task 2: 距离场与认证上下文的精确复用

**Files:** scripts/navigation/one_cm/nav1cm/maps.py、certification.py、coordinator.py；tests/navigation/one_cm/test_contracts_maps_reference.py、test_certification.py、test_coordinator.py；必要时新增 focused tests 文件。

**Interfaces:** 保持现有 build_distance_field / certify 调用兼容；可新增 DistanceFieldCache、prepare_certification(grid,backend=...)，及 certify(...,context=...) 可选参数。coordinator 只组合，不实现地图或碰撞算法。

- [x] 写失败测试证明位图内容相同而 origin/epoch 不同可正确重绑定复用，位图/分辨率/shape/未知策略变化正确失效，距离及梯度与无缓存结果一致。
- [x] 在地图模块实现有界单项缓存；不可变几何数组允许共享，新场身份必须为当前图；窗口保持严格居中。
- [x] 写认证上下文测试：同一 tick 多次认证仅构建一个危险格索引，错误 grid/backend 明确拒绝；前缀仍独立认证、证书仍绑定完整身份。
- [x] 在认证模块实现 prepared context，在 coordinator 复用；不削弱 sweep/制动/动力学检查。
- [x] 运行定向与完整导航测试，保存计算微基准/对照数据及变更报告。若还有明显热点，先报告主 agent，再扩展范围。

主 agent 在两个任务审查通过后统一上传受控源码、执行真实 Orin 测试与短闭环，更新任务登记和结果。每个任务独立审查，最后检查跨模块一致性。源码保持未提交。

### Task 3: 测量驱动的可选原生距离场

**Files:** 新增 nav1cm/distance_field_native.cpp、nav1cm/field_native.py；修改 maps.py、coordinator.py、benchmark_navigation.py；新增 tests/navigation/one_cm/test_native_distance_field.py。路径均在 scripts/navigation/one_cm 下（测试除外）。

**Interfaces:** NativeDistanceField(library_path).build(grid)->DistanceField；build_distance_field(grid, *, backend=None) 保持默认 SciPy；DistanceFieldCache(backend=None)；coordinator 新可选 field_backend=None；benchmark 新可选 --distance-field-lib，显式记录库身份。认证不改变。

- [x] 写 RED/GREEN：native 与 SciPy 在随机稠密/稀疏、细墙、全部阻断/自由、UNKNOWN两策略、非方形和500²图上，距离及梯度逐点一致到1e-10；epoch/origin和不可变性正确。库缺失时显式失败，不冒称原生通过。
- [x] 独立 C++ 库使用可分离精确平方距离变换，O(HW)；继续原有保守减项、边界和gradient定义。无新依赖；Python仅负责显式加载、shape/dtype/finite校验及返回不可变场。
- [x] 显式后端注入地图缓存与coordinator；benchmark默认仍可用SciPy，提供原生库时记录SHA，所有慢样本仍统计。
- [x] 本机编译可用则跑native对照，否则纯接口测试并明确跳过；主agent统一Orin编译、逐点测试、同输入回放和必要短闭环。不得自行启动远端批次。

这是前两任务实际测量后的追加优化，前两任务及原始失败证据保留。新实现完成后定向审查与统一终审，保持相同任务与分支。
