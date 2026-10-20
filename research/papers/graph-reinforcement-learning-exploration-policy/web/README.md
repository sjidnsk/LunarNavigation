# 论文研究网页工作台

本文件夹包含本篇论文工作台的网页源码、Python服务、启动入口和配置。与论文资料放在一起，运行不依赖外部临时工作树。

## 打开与停止

双击本文件夹中的 `Start-ResearchDashboard.cmd`，打开 [研究工作台](http://127.0.0.1:8765/)。首次双击启动的服务运行在窗口中，保留该窗口；按 Ctrl+C 可停止。重复双击会复用同一论文已有的服务，遇到其他服务占用端口会报错。

本次交付为隐藏运行的后台实例，关闭浏览器不会停止它。要停止后台实例，请在本 `web` 文件夹打开PowerShell，先保存页面修改，再执行下列代码；它核对唯一监听、脚本和论文目录，确认后只结束对应PID。

```powershell
$webRoot = (Get-Location).Path
$paperRoot = [IO.Path]::GetFullPath((Join-Path $webRoot '..')).TrimEnd('\', '/')
$serverPath = Join-Path $webRoot 'server.py'
$config = Get-Content -LiteralPath (Join-Path $webRoot 'runtime.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$port = [int]$config.port
$listeners = @(Get-NetTCPConnection -LocalAddress '127.0.0.1' -LocalPort $port -State Listen -ErrorAction Stop)
if ($listeners.Count -ne 1) { throw '监听归属不唯一，拒绝停止。' }
$servicePid = [int]$listeners[0].OwningProcess
$processInfo = Get-CimInstance Win32_Process -Filter "ProcessId = $servicePid"
$command = [string]$processInfo.CommandLine
Write-Host $command
if (-not $command.Contains($serverPath) -or -not $command.Contains($paperRoot) -or
    $command -notmatch ('--port\s+' + $port + '(?:\s|$)')) { throw '脚本、论文或端口不匹配，拒绝停止。' }
if ((Read-Host "核对命令行后输入当前 PID $servicePid 才停止") -cne [string]$servicePid) { throw '未确认。' }
$current = Get-CimInstance Win32_Process -Filter "ProcessId = $servicePid"
if (-not $current -or $current.CreationDate -ne $processInfo.CreationDate -or
    $current.CommandLine -cne $command) { throw '进程身份已变化，拒绝停止。' }
Stop-Process -Id $servicePid -ErrorAction Stop
```

上面的停止示例未实际执行。停止后可再次双击启动器，以前台窗口方式启动。

## 目录与配置

- `index.html`、`styles.css`、`app.js`：页面结构、配色与交互。
- `server.py`：本机HTTP服务，复用上一层论文的 `02-research-design/progress.py`。
- `runtime.json`：Python路径、论文相对路径与端口。`paper_root` 的 `..` 相对于本文件夹，移动整个论文目录后仍能定位账本；新机器需填写当地可用的Python路径。
- `Start-ResearchDashboard.cmd` / `.ps1`：启动入口。
- `.local/backups/`：每次保存前的三文件备份；`.local/`同时存放本机运行日志，已在Git中忽略。
- `test_server.py`：既有后端隔离测试源码。本次位置调整未运行测试，也不对真实论文进度写入测试状态。

## 使用与保存规则

研究总览显示账本中的真实进度；任务可组合搜索、阶段、状态及可推进筛选。点击任务查看操作、产物、验收、依赖和历史，更新负责人、日期、说明与证据。证据按每行一条保存完整列表。

成功保存才更新页面，并同步上一层论文中的 `work-items.json`、`PROGRESS.md`、`CHECKLIST.md`。规则拒绝会保留输入；版本冲突会展示最新记录与草稿，明确选择后才可重试。完成和不适用需真实说明及证据，工具不能替代人工验收。

重开已关闭任务会保留历史证据并要求下游复核。研究章程集中阅读工作计划、研究进度、清单和论文说明；有限Markdown阅读器不执行原始HTML，未知本地链接显示为文件参考。

草稿仅保留在当前页面内存；页面刷新不等于保存。外部CLI与网页保持单写入者，版本检查不是跨进程锁。每次保存保留原三文件备份；异常中断需停止服务并按明确备份来源人工恢复，不保证断电时多文件事务原子性。迁移前的历史备份仍保留于原D盘任务目录，不作为当前运行依赖。

服务仅监听 `127.0.0.1`，使用同源、会话令牌与版本校验。它记录论文进展，不自动执行训练、评测或投稿。界面采用Claude风格的米白、暖灰、陶土色，信息布局参考 [shadcn/ui Tasks](https://ui.shadcn.com/examples/tasks)。
