param(
    [int]$InterfaceIndex = 2,
    [switch]$Remove
)

$ErrorActionPreference = "Stop"
$cameraAddress = "192.168.111.11"
$localAddress = "192.168.111.249"
$prefixLength = 24

$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = [Security.Principal.WindowsPrincipal]::new($identity)
$isAdministrator = $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdministrator) {
    throw "Run this script from an elevated Administrator PowerShell window."
}

$adapter = Get-NetAdapter -ErrorAction Stop | Where-Object ifIndex -eq $InterfaceIndex
if ($null -eq $adapter) {
    throw "Network adapter index $InterfaceIndex was not found."
}
if ($adapter.Status -ne "Up") {
    throw "Network adapter index $InterfaceIndex is not connected."
}

$existing = Get-NetIPAddress `
    -InterfaceIndex $InterfaceIndex `
    -AddressFamily IPv4 `
    -ErrorAction SilentlyContinue | Where-Object IPAddress -eq $localAddress

if ($Remove) {
    if ($existing) {
        $existing | Remove-NetIPAddress -Confirm:$false
        Write-Host "Removed temporary local address $localAddress/$prefixLength."
    }
    else {
        Write-Host "Address $localAddress is not configured; nothing to remove."
    }
    exit 0
}

if (-not $existing) {
    New-NetIPAddress `
        -InterfaceIndex $InterfaceIndex `
        -IPAddress $localAddress `
        -PrefixLength $prefixLength `
        -AddressFamily IPv4 `
        -SkipAsSource $false `
        -PolicyStore ActiveStore | Out-Null
}
elseif ($existing.SkipAsSource) {
    Set-NetIPAddress `
        -InterfaceIndex $InterfaceIndex `
        -IPAddress $localAddress `
        -SkipAsSource $false
}

Write-Host "Temporary local address $localAddress/$prefixLength is configured."
Write-Host "No gateway, vehicle, flight-controller, or PX4 parameter was changed."
Write-Host "Testing D7 camera $cameraAddress ..."
Test-NetConnection -ComputerName $cameraAddress -Port 554 -InformationLevel Detailed
