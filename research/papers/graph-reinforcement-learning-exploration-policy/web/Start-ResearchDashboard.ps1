$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$configPath = Join-Path $taskRoot 'runtime.json'
$serverPath = Join-Path $taskRoot 'server.py'
$backupPath = Join-Path $taskRoot '.local/backups'

try {
    $config = Get-Content -LiteralPath $configPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $pythonPath = [string]$config.python_executable
    $paperRoot = [IO.Path]::GetFullPath((Join-Path $taskRoot ([string]$config.paper_root))).TrimEnd('\', '/')
    $port = [int]$config.port
    if ($port -lt 1 -or $port -gt 65535) { throw 'runtime.json port must be between 1 and 65535.' }
    if (-not (Test-Path -LiteralPath $pythonPath -PathType Leaf)) { throw "Python not found: $pythonPath" }
    if (-not (Test-Path -LiteralPath $serverPath -PathType Leaf)) { throw "Server not found: $serverPath" }
    if (-not (Test-Path -LiteralPath $paperRoot -PathType Container)) { throw "Paper directory not found: $paperRoot" }
    if ([IO.Path]::GetPathRoot([IO.Path]::GetFullPath($backupPath)) -ne 'D:\') { throw 'Backups must stay on drive D.' }

    $url = "http://127.0.0.1:$port/"
    $occupied = $false
    $client = New-Object Net.Sockets.TcpClient
    try {
        $connect = $client.ConnectAsync('127.0.0.1', $port)
        $occupied = $connect.Wait(800) -and $client.Connected
    } catch {
        $occupied = $false
    } finally {
        $client.Dispose()
    }
    if ($occupied) {
        try {
            $health = Invoke-RestMethod -Uri ($url + 'health') -TimeoutSec 3
            $workspace = Invoke-RestMethod -Uri ($url + 'api/workspace') -TimeoutSec 3
            $runningRoot = [IO.Path]::GetFullPath([string]$workspace.paper_root).TrimEnd('\', '/')
            $sameApp = $health.status -eq 'ok' -and $health.study_id -eq $workspace.ledger.study_id -and $workspace.csrf_token -and $workspace.revision
            if (-not $sameApp -or -not [string]::Equals($paperRoot, $runningRoot, [StringComparison]::OrdinalIgnoreCase)) {
                throw 'The running service is not this configured research workspace.'
            }
        } catch {
            throw "Port $port is already in use by another service or paper. Nothing was stopped. Choose another port in runtime.json. Details: $($_.Exception.Message)"
        }
        Write-Host "Opening the existing research workspace: $url"
        & $pythonPath -B -c 'import sys, webbrowser; webbrowser.open(sys.argv[1])' $url
        exit 0
    }

    Write-Host "Starting research workspace: $url"
    Write-Host "Paper: $paperRoot"
    Write-Host "Backups: $backupPath"
    Write-Host 'Keep this window open. Press Ctrl+C to stop the local service.'
    & $pythonPath -B $serverPath --paper-root $paperRoot --port $port --open --backup-dir $backupPath
    if ($LASTEXITCODE -ne 0) { throw "Server stopped with exit code $LASTEXITCODE." }
} catch {
    Write-Host ("Launch failed: " + $_.Exception.Message) -ForegroundColor Red
    exit 1
}
