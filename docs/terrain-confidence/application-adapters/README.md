# 应用配置适配补丁

这些补丁记录 2026-10-05 迁入时，各应用工作树已有的可信度配置增量。每份补丁均由来源
`git diff HEAD -- config tools/tcp_deployment/templates` 精确导出，只包含新增的
`confidence_weight: 2.0`，不携带其他部署配置。

| 补丁 | 来源基线 | 涉及入口 |
|---|---|---|
| `orin-legged-vehicle.patch` | `0209ca23` | navigation 和 Orin hardware |
| `orin-real-vehicle.patch` | `c4866408` | navigation 和 Orin hardware |
| `orin-unreal-simulation.patch` | `17ebae41` | navigation、Orin hardware、TCP P3 联调模板 |

开发分支通用 navigation 配置已经启用相同权重。开发分支没有引入应用专有的 hardware/TCP
部署入口；补丁在此保存，供同步时核对。来源工作树目前已经包含这些行，不能重复应用。

向目标应用同步前，先检查实际文件和已同步内容，确认 patch 适用性。这些补丁只用于配置适配，
算法来自开发分支的已验证实现。轮式应用当前保留的未启用 legacy 足式实现不作为开发分支的
公共足式实现；开发分支保留足式来源的新评价逻辑与其测试覆盖。
