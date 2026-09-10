$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$scriptPath = Join-Path $PSScriptRoot "configure_d7_camera_network.ps1"
$logPath = Join-Path $projectRoot ".runtime\d7-network-admin.log"

try {
    & $scriptPath -InterfaceIndex 2 *>&1 | Out-File -LiteralPath $logPath -Encoding utf8
    exit 0
}
catch {
    $_ | Format-List * -Force | Out-File -LiteralPath $logPath -Encoding utf8
    exit 1
}
