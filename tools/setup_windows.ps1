$projectRoot = Split-Path -Parent $PSScriptRoot
$localCache = Join-Path $projectRoot '.uv-cache'
$bundledPython = 'C:\Users\dingr\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'

if (-not (Test-Path -LiteralPath $bundledPython)) {
    throw "未找到项目验证过的 Python 3.12：$bundledPython"
}

Push-Location $projectRoot
try {
    & uv --cache-dir $localCache --no-managed-python sync --python $bundledPython --extra all
    if ($LASTEXITCODE -ne 0) {
        throw "uv 安装失败，退出码：$LASTEXITCODE"
    }
}
finally {
    Pop-Location
}

Write-Host '环境安装完成。运行 tools/start_preview.ps1 即可启动。' -ForegroundColor Green
