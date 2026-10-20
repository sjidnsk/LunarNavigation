# 图强化学习探索策略论文工作区

本目录管理本篇论文从研究问题、检查点选优到投稿修回的全过程。当前起点为用户提供的“训练已完成，正在评测选出最佳检查点”；模型、运行与结果身份由后续任务核对。

## 从这里开始

1. 打开 [研究进度总览](02-research-design/PROGRESS.md)，查看当前状态、可推进任务及未关闭依赖。
2. 阅读 [步骤安排与执行章程](02-research-design/WORKPLAN.md)，了解阶段放行条件和返回修订规则。
3. 按 [详细工作清单](02-research-design/CHECKLIST.md) 执行；每项都列出操作、产物和验收条件。
4. 工作后更新 [任务账本](02-research-design/work-items.json)，或使用下列命令。进度只维护这一份账本，另外两份文档由工具生成。

已有历史训练和实验先核对来源再认定适用性，避免重复劳动。账本初始的0/61表示“本次安排的论文流程尚未验收”，不表示此前项目没有成果。

## 网页工作台

打开[本机研究工作台](http://127.0.0.1:8765/)，可按阶段、状态和可推进条件筛选任务，填写负责人、目标日期、说明与证据，并阅读章程、总览和清单。保存会更新本目录的原任务账本及两份生成文档；完成任务仍需人工核对证据。

服务未启动时，双击[启动研究工作台](web/Start-ResearchDashboard.cmd)。首次启动保留服务窗口；重复启动会复用同论文的已有服务。此地址仅供本机访问。

网页源码、启动器与配置均位于本论文的 `web/` 文件夹，运行备份与日志位于被Git忽略的 `web/.local/`。详见[网页使用说明](web/README.md)。整个工作台与论文资料一同管理。

## 日常使用

在本目录打开PowerShell：

~~~powershell
$tracker = ".\02-research-design\progress.py"
python $tracker next
python $tracker show P0-01
~~~

开始工作并填写实际负责人、目标日期：

~~~powershell
python $tracker update P0-01 --status doing --owner "实际负责人"
~~~

产物形成后可先提交待验收，再经人工核对关闭；下面的完成命令仅在真实盘点完成后执行：

~~~powershell
python $tracker update P0-01 --status review --note "盘点材料已整理，等待核对"
python $tracker update P0-01 --status done --evidence "04-experiments/records/baseline-inventory.md" --note "已逐项核对来源、预算和未决问题"
~~~

update和reopen会自动刷新进度总览、详细清单并保留状态变更历史。--evidence可以重复指定，填写实际文件、外部运行目录或回执定位；工具只校验流程和记录完整性，证据内容仍需人核实。

受阻时记录原因及解除条件：

~~~powershell
python $tracker update P0-01 --status blocked --note "缺少原训练完整配置；待从原运行目录取得"
~~~

上游内容发生实质变化时，先记录修订，再重开已完成的对应任务：

~~~powershell
python $tracker reopen P2-07 --reason "正式协议的指标定义发生变化"
~~~

reopen会使已关闭的下游任务重新待验收，并保留旧证据；它不会改模型或停止运行。重新关闭任务时应重新核对证据。再次使用过的测试集须按章程判断是否需要新的留出数据。

更多命令：

~~~powershell
python $tracker summary
python $tracker validate
python $tracker render
~~~

排期时在update命令后增加--due，填写真实目标日期，格式为YYYY-MM-DD。任何update/reopen命令增加--dry-run可预览校验，不写账本。

若直接编辑JSON，请保持UTF-8，执行validate及render。不要直接修改生成文档里的勾选来登记进度。此工具没有后台监控；完成工作后主动登记，之后打开总览即可看到最新记录。也可以要求后续会话“核对P0-01的实际产物并更新论文账本”。

## 资料与版本边界

方法文档进入03-method，协议/配置/记录进入04-experiments，分析进入05-results，正文进入06-manuscript，图表进入07-figures，投稿与修回进入08-submission。通用记录模板见 [研究记录模板](../shared/templates/research-records.md)。

大数据、权重、日志、bag和设备运行产物存放在仓库外的D盘受控目录或实际运行机器的受控存储，记录路径及校验值。不要把示例路径写成真实证据。

本工具只管理论文工作记录；真实训练、实验运行、平台验证、投稿及向他人发送材料均由对应任务单独执行。
