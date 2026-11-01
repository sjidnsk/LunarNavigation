# 2026-10-29 全分支盘点与处置

盘点基准main@fd4e0e4，31个本地分支、8个实时远程分支、24个工作树。此表是续办入口快照；随后仅治理提交使main与治理分支前移，最终落地身份在仓库外final-receipt.json及Git实际HEAD核对。产品采用与问题挂起见[续办记录](20261029-resume.md)。

创建来源取本地reflog或已登记保存记录，不能把最近提交日期当创建日期。提交保存时间、文件mtime、独有提交数量均不替代功能验收。下表“主/本独有”表示相对主线的对称差，不能解释为本支功能领先。

## 全部本地分支

|分支|HEAD / 最近提交日期|创建来源证据|长期owner|主/本独有|用途及处置|
|---|---|---|---|---:|---|
|`app/orin-legged-vehicle`|`ff75f527` / 2026-10-18|origin/app/orin-legged-vehicle @ c14655ce|`app/orin-legged-vehicle`|161/39|足式应用；保留足式默认能力与入口|
|`app/orin-real-vehicle`|`a86b9cbf` / 2026-10-29|origin/app/orin-real-vehicle @ 02244917|`app/orin-real-vehicle`|161/59|轮式应用；与轮式专题同指，待兼容审查|
|`app/orin-unreal-simulation`|`2d9a8ce1` / 2026-10-18|origin/app/orin-unreal-simulation @ 02aa1b80|`app/orin-unreal-simulation`|161/34|TCP/Unreal应用；保留平台适配|
|`dev/drl-exploration`|`dfd422b7` / 2026-10-19|origin/dev/drl-exploration @ 68465efc|`dev/drl-exploration`|145/0|唯一DRL开发线；导航候选门槛未过，保持指针|
|`dev/terrain-confidence`|`44acb7bf` / 2026-10-18|origin/dev/terrain-confidence @ aa04d99a|`dev/terrain-confidence`|160/18|地形可信度开发；专属机制待独立晋级|
|`docs/paper-pre-results-20261023`|`946501c0` / 2026-10-29|dev/drl-exploration @ dfd422b7|`integration/pure-planner-orin`|145/1|论文预结果初稿WIP；正文与证据单独审查|
|`feat/paper-p4-baselines-20261022`|`cb8b0aa0` / 2026-10-29|origin/dev/drl-exploration @ a094b902|`dev/drl-exploration`|144/1|历史专题/WIP保存；逐文件对照6803781，禁止整支覆盖|
|`feat/paper-p4-external-training-20261022`|`d318b97a` / 2026-10-29|a094b90206db9c9bef2b6cf075db55cf210ccc22 @ a094b902|`dev/drl-exploration`|144/1|历史专题/WIP保存；逐文件对照6803781，禁止整支覆盖|
|`feat/paper-p4-graph-baselines-20261022`|`1b5bfc69` / 2026-10-29|origin/dev/drl-exploration @ a094b902|`dev/drl-exploration`|144/1|历史专题/WIP保存；逐文件对照6803781，禁止整支覆盖|
|`feat/paper-p4-marvel-baseline-20261022`|`7ae3f9c7` / 2026-10-29|origin/dev/drl-exploration @ a094b902|`dev/drl-exploration`|144/1|历史专题/WIP保存；逐文件对照6803781，禁止整支覆盖|
|`feat/paper-p4-tare-adapter-20261022`|`e76745a6` / 2026-10-29|a094b90206db9c9bef2b6cf075db55cf210ccc22 @ a094b902|`dev/drl-exploration`|144/1|历史专题/WIP保存；逐文件对照6803781，禁止整支覆盖|
|`feat/paper-p4-training-20261022`|`fb6434ef` / 2026-10-29|a094b90206db9c9bef2b6cf075db55cf210ccc22 @ a094b902|`dev/drl-exploration`|144/1|历史专题/WIP保存；逐文件对照6803781，禁止整支覆盖|
|`feat/paper-p4-training-campaign-20261022`|`0d586c46` / 2026-10-29|a094b90206db9c9bef2b6cf075db55cf210ccc22 @ a094b902|`dev/drl-exploration`|144/1|历史专题/WIP保存；逐文件对照6803781，禁止整支覆盖|
|`feat/paper-p4-training-qualification-20261022`|`86050081` / 2026-10-29|a094b90206db9c9bef2b6cf075db55cf210ccc22 @ a094b902|`dev/drl-exploration`|144/1|历史专题/WIP保存；逐文件对照6803781，禁止整支覆盖|
|`feat/paper-single-seed-half-budget-20261024`|`b070a17c` / 2026-10-29|dev/drl-exploration @ dfd422b7|`dev/drl-exploration`|145/1|457文件历史保全；冻结镜像/恢复工具不重放|
|`hk/drl-integration-20261029`|`74328282` / 2026-10-29|dev/drl-exploration @ dfd422b7|`dev/drl-exploration`|2/27|DRL/公共修复候选；7432828已保存，闭环FAIL暂缓|
|`hk/drl-nav1cm-adapter-20261029`|`5c895199` / 2026-10-29|HEAD @ a094b902|`dev/drl-exploration`|144/13|5c89519适配来源已进入候选，owner未采用|
|`hk/follower-swept-map-20261024`|`83d6d10a` / 2026-10-28|integration/pure-planner-orin @ 439178f2|`integration/pure-planner-orin`|133/0|源码已合入；资源待审计回收|
|`hk/git-preservation-20261029`|`fd4e0e4f` / 2026-10-29|1532cf0981a2665b70d8c0f2a8ff2eb420d54e73 @ 1532cf09|`integration/pure-planner-orin`|0/0|本任务治理线；与主线快进后继续整理|
|`hk/isaac-polar-midterm-20261028`|`c18fddd1` / 2026-10-29|HEAD @ 439178f2|`integration/pure-planner-orin`|132/0|场景源码已整体合入；继承脏资料保留|
|`hk/isaac-viper-engineering-20261028`|`accf75ba` / 2026-10-29|HEAD @ 2d9a8ce1|`app/orin-unreal-simulation`|161/54|19文件精确采用；54独有提交不整包上送|
|`hk/isaac-viper-showcase-20261029`|`0d19d320` / 2026-10-29|HEAD @ 1a60611a|`integration/pure-planner-orin`|105/3|默认MPPI已撤回；五文件停车WIP暂缓|
|`hk/nav-1cm-feasibility-20261026`|`eb10ff26` / 2026-10-29|439178f2f09399470696b43e1f47f089c44f41cb @ 439178f2|`integration/pure-planner-orin`|35/0|eb10ff2已由fe5cef6合入|
|`hk/nav1cm-wheel-app-integration-20261028`|`a86b9cbf` / 2026-10-29|81ed242c496d1d6b40aba17ccc69c46ef9b451e5 @ 81ed242c|`app/orin-real-vehicle`|161/59|与app同指a86b9cb；避免重复合并|
|`hk/paper-dashboard-20261020`|`90a136d9` / 2026-10-20|7e7cbf19b43e8f5e3710d2cc0f340af71f2ddee7 @ 7e7cbf19|`integration/pure-planner-orin`|136/0|90a136d已在主线祖先；不重复合并|
|`hk/paper-five-methods-20261028`|`6803781c` / 2026-10-29|dev/drl-exploration @ dfd422b7|`dev/drl-exploration`|145/41|现行五方法来源；资源门BLOCKED_RESOURCE_PREFLIGHT|
|`hk/paper-safe-start-2m-20261021`|`ab563ca9` / 2026-10-29|a094b90206db9c9bef2b6cf075db55cf210ccc22 @ a094b902|`dev/drl-exploration`|144/1|a094b90功能已在main；后续41文件WIP另审|
|`hk/preserve-paper-main-20261029`|`e15f3bec` / 2026-10-29|主目录1532cf0单父保存记录→dd35b7e→e15f3be|`integration/pure-planner-orin`|105/2|主目录论文资料保存源；非运行/训练分支|
|`hk/preserve-paper-polar-20261029`|`c791fb6a` / 2026-10-29|polar c18fddd单父保存记录→c791fb6|`integration/pure-planner-orin`|132/1|四份不同历史资料保存源；不按mtime替换主源|
|`hk/protected-main-working-20261029`|`1532cf09` / 2026-10-29|HEAD @ 1532cf09|`integration/pure-planner-orin`|105/0|受保护原目录身份；不作为待功能合入分支|
|`integration/pure-planner-orin`|`fd4e0e4f` / 2026-10-29|origin/integration/pure-planner-orin @ 26d7a86e|`integration/pure-planner-orin`|0/0|完整架构主线；本轮只接治理记录|

## 全部实时远程分支

直接git ls-remote读取，不使用可能陈旧的origin缓存冒充实时状态；没有fetch/prune/push，也没有新建本地对应分支。主线缓存指向a0f58e2，而远端实际为b206870，本表按后者计算。

|远程分支|实际HEAD|同名本地相对远程|说明|
|---|---|---|---|
|`origin/app/orin-legged-vehicle`|`ff75f527d4a45c424f9b1e624f143e3668000ec6`|本地独有0 / 远端独有0|本轮不推送、不自动移动owner|
|`origin/app/orin-real-vehicle`|`81ed242c496d1d6b40aba17ccc69c46ef9b451e5`|本地独有26 / 远端独有0|本轮不推送、不自动移动owner|
|`origin/app/orin-unreal-simulation`|`2d9a8ce1620d951f110c36ab13e59d3bb478df16`|本地独有0 / 远端独有0|本轮不推送、不自动移动owner|
|`origin/dev/drl-exploration`|`a094b90206db9c9bef2b6cf075db55cf210ccc22`|本地独有0 / 远端独有1|本轮不推送、不自动移动owner|
|`origin/dev/terrain-confidence`|`44acb7bff4aaca6a18b3ba8b686226a56b00e743`|本地独有0 / 远端独有0|本轮不推送、不自动移动owner|
|`origin/feat/mppi-controller-20261018`|`a094b90206db9c9bef2b6cf075db55cf210ccc22`|无同名本地分支|仅远程历史来源；不为对称新增分支|
|`origin/fix/execution-safety-20261018`|`4ea7f8c75912abb6abfdf8033bc82912cc847b41`|无同名本地分支|仅远程历史来源；不为对称新增分支|
|`origin/integration/pure-planner-orin`|`b2068706203de5987eb0cdd7c4a968fdba2f5338`|本地独有133 / 远端独有0|本轮不推送、不自动移动owner|

## 论文保存版本对照现行五方法

目标为hk/paper-five-methods-20261028@6803781。只比较各保存提交明确变更路径，共772条文件版本记录（未去重），不是仓库全部文件。相同mode/blob证明精确版本已存在，不证明对应功能验收；不同和缺失也不能直接认定功能未继承，需结合路径职责与现行实现。

|保存专题|记录数|相同路径mode/blob|同路径不同版本|当前无该路径|处置|
|---|---:|---:|---:|---:|---|
|`feat/paper-p4-baselines-20261022`|40|22|18|0|保存来源，按模块审查；不整支覆盖|
|`feat/paper-p4-graph-baselines-20261022`|35|19|16|0|保存来源，按模块审查；不整支覆盖|
|`feat/paper-p4-marvel-baseline-20261022`|34|18|16|0|保存来源，按模块审查；不整支覆盖|
|`feat/paper-p4-tare-adapter-20261022`|16|0|0|16|保存来源，按模块审查；不整支覆盖|
|`feat/paper-p4-external-training-20261022`|44|22|22|0|保存来源，按模块审查；不整支覆盖|
|`feat/paper-p4-training-qualification-20261022`|42|22|20|0|保存来源，按模块审查；不整支覆盖|
|`feat/paper-p4-training-campaign-20261022`|1|0|1|0|保存来源，按模块审查；不整支覆盖|
|`feat/paper-p4-training-20261022`|57|31|22|4|保存来源，按模块审查；不整支覆盖|
|`feat/paper-single-seed-half-budget-20261024`|457|45|18|394|保存来源，按模块审查；不整支覆盖|
|`hk/paper-safe-start-2m-20261021`|41|18|17|6|保存来源，按模块审查；不整支覆盖|
|`docs/paper-pre-results-20261023`|5|0|1|4|保存来源，按模块审查；不整支覆盖|

合计197条完全相同、151条同路径不同、424条当前无该路径。single-seed的394条缺失大多属于历史冻结镜像/恢复工具等，保存在b070a17，不因当前缺路径就补入运行源码。TARE上游31文件不在本表16个wrapper保存路径中，仍以外部保全和条款缺口记录为准。

现行五方法不是旧P4专题依次整合的Git后代；与其merge-base为dfd422b。cherry显示旧WIP不整提交等价，不能代替上述逐文件证据。没有采用旧模型、训练配置或账本，没有重放恢复。

## 核对与恢复证据

仓库外D:/CodexDownloads/lunar_navigation/git-preservation-20261029/branch-continuation-20261029保存inventory.json、remote-heads.txt、四线完整merge-tree输出和paper-source-matrix.json（含772条path/mode/blob/现行路径对照）。

已合入来源可登记待回收，但24个工作树全部保留；保护根目录、polar与TARE存在继承未提交内容，ignored依赖未审计。没有删除、强制重置、推送或设置upstream。
