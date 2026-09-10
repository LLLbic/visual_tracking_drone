param(
    [string[]]$Addresses = @(
        '192.168.1.10',
        '192.168.1.11',
        '192.168.1.12',
        '192.168.1.175'
    ),
    [int]$TimeoutMilliseconds = 3000
)

$ffprobe = Get-Command ffprobe.exe -ErrorAction Stop
$timeoutMicroseconds = $TimeoutMilliseconds * 1000
$found = @()

foreach ($address in $Addresses) {
    foreach ($stream in 0, 1) {
        $url = "rtsp://${address}:554/user=admin&password=&channel=1&stream=${stream}.sdp?"
        $details = & $ffprobe.Source `
            -v error `
            -rtsp_transport tcp `
            -timeout $timeoutMicroseconds `
            -analyzeduration 1000000 `
            -probesize 1000000 `
            -select_streams 'v:0' `
            -show_entries 'stream=codec_name,width,height,r_frame_rate' `
            -of 'default=noprint_wrappers=1' `
            $url 2>&1

        if ($LASTEXITCODE -eq 0) {
            $found += $url
            Write-Host "[VIDEO OK] $url" -ForegroundColor Green
            $details | ForEach-Object { Write-Host "  $_" }
        } else {
            Write-Host "[NO VIDEO] $url" -ForegroundColor DarkGray
        }
    }
}

if ($found.Count -eq 0) {
    Write-Warning '没有发现可解码的原生 RTSP。请检查摄像头供电、天空端网线、MiniHomer 视频链路和实际相机 IP。'
    exit 2
}

Write-Host '可用视频地址：' -ForegroundColor Cyan
$found | ForEach-Object { Write-Host "  $_" }
