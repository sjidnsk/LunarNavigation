# 公共导航与DRL适配顺序落地

任务GIT-PRESERVE-20261029；2026-10-30用户批准按推荐处理公共导航和DRL适配。复用治理、主线及DRL专题三个已登记工作树，主agent单写入者，最后一次独立只读整合审查。

## 固定来源与边界

- 原main：a740deeda271d4cfec3809e88b4d13f8a8be32e8；治理记录：87119df；候选：04f6b129c5815d69093f986dc73b06b7125d8382；原DRL owner：dfd422b7536173a96bd61de92fa4cb0d769014c4。
- ①公共批次按机器清单的46个公共路径精确取04f6b12最终内容，携22份说明/设计/历史证据；开发源码清单留给②。不整提交挑选混合路径，不改算法。
- ②DRL在公共main之后接收候选完整谱系，开发源码清单跟随DRL产品一起采用；再将已验收适配晋级main。
- 用户要求不追加验证：本轮不运行测试、构建、Actor或设备验证；只核对Git来源、路径/字节、差异、合并和分支状态。沿用r17/r18/r20及既有包测，不能写成合并后新测试。
- 不移动应用线、terrain、五方法、现役运行、formal清单/模型/原配置，不push/PR/delete/upstream，不回收工作树。

## 执行记录

1. 进入治理main/DRL三工作树均干净；原保护目录只读保留。
2. 公共路径来源固定为04f6b12；最终合入SHA和DRL晋级SHA随实际落地追加。
3. 公共批次773828f9ad0271436a97fc741ffdc3792a3dce75已正常ff进入main；68个来源文件Git blob与候选相同，公共只读审查无阻断。
4. 原DRL专题工作树从hk/drl-integration-20261029切到dev/drl-exploration用于owner接收；候选指针04f6b12保留。owner先ff公共main，再合04f6b12，唯一任务登记冲突完整保留双方追加。98个非任务清单文件、全部非docs源码树均与候选一致；没有改写算法或启动运行。
5. 公共与最终DRL待提交树分别独立只读审查，无阻断；开发清单全部文件SHA吻合源码，仍not_deployed；默认DRL后端incremental_v2保持，nav1cm显式选择。主线治理资料及任务登记双方记录保留，formal清单不变。
6. DRL owner合并提交f5e8e9a68b395a30810fd7f2df0872715dba9316（双亲773828f、04f6b12）；main已从773828f正常ff至该提交，治理树随后ff接收。①②产品顺序落地完成。
7. 七条受保护来源/应用/地形/五方法引用保持；原保护工作树status摘要与接收前相同。全部非docs产品树与04f6b12相同。实际回执见[机器记录](20261030-integration-receipt.json)。

Ruling：将初步机器清单中按docs目录归类的development-source-manifest.json作为DRL源码身份元数据处理，放入②而非公共证据。其余历史报告可在①保存，不表示提前接收DRL产品或训练资格。

## 最终处置

①公共导航已合入main，②DRL适配已合入dev/drl-exploration并晋级main。原候选hk/drl-integration-20261029@04f6b12及适配来源5c89519已在采用谱系中，保留为恢复来源，标记待资源回收审计；本轮没有删除任何分支/工作树。

DRL接收工作树当前绑定dev/drl-exploration，路径仍为D:/CodexDownloads/lunar_navigation/git-preservation-20261029/drl-worktree；main与治理使用原各自工作树。收尾文档提交由main及DRL owner正常ff同步，具体最终HEAD以Git和用户交付记录为准，不自引用文档所属提交号。

本轮没有运行测试/构建/Actor或设备验证。已有r17/r18/r20及包测试只作固定来源的历史证据；r21主动停止不算通过。未推送、未部署、未切正式训练身份/默认后端，应用线、terrain和五方法均未移动。Orin/实车/真实坡面/Isaac在线仍NOT_RUN。
