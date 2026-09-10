$projectRoot = Split-Path -Parent $PSScriptRoot
$launcher = Join-Path $projectRoot '.venv\Scripts\uav-preview.exe'
$config = Join-Path $projectRoot 'config.toml'

if (-not (Test-Path -LiteralPath $launcher)) {
    throw 'Environment is not installed. Run tools/setup_windows.ps1 first.'
}

Push-Location $projectRoot
try {
    & $launcher --config $config
}
finally {
    Pop-Location
}
