# NVIDIA Optical Flow SDK / FRUC 实验

本分支在 OpenCV 解码与 YOLO 之间接入 NVIDIA FRUC。相邻两张真实帧之间生成一张中间帧，目标是把约 5 FPS 的预览提升为约 10 FPS，并独立测量合成帧对 YOLO 的影响。

## 边界

- FRUC 依赖前一张和当前真实帧，因此不能降低摄像头编码、RTSP 网络或解码造成的原始延迟。
- 实时工作站只在网页预览中显示合成帧。YOLO、目标锁定、PID 和 MAVLink 控制只使用真实帧。
- A/B 工具才会在合成帧上运行 YOLO，用于统计置信度、检测保持率、框一致性和推理耗时。
- SDK 的 DLL 保持在 `D:\NVToolKits`，不会复制或提交到仓库。

## 构建本机桥接层

在仓库根目录运行：

```powershell
.\tools\build_nvof_fruc_bridge.ps1
```

生成物位于 `.runtime\nvof-fruc\nvof_fruc_bridge.dll`，已经由 `.gitignore` 排除。

## A/B 测试

先关闭当前占用 RTSP 的工作站，然后采集 30 张真实帧并测试：

```powershell
$env:UV_CACHE_DIR = Join-Path (Get-Location) ".uv-cache"
uv run python tools\benchmark_nvof_fruc.py --frames 30 --profile red_barrel
```

报告输出到 `.runtime\nvof-fruc\ab-report.json`。其中合成帧没有人工标注真值，因此一致性结果不能替代 mAP；它只回答“插帧是否明显破坏当前 YOLO 输出”。

## 回退

把 `config.toml` 中 `[frame_interpolation].enabled` 改为 `false` 即可完全关闭FRUC，不影响其他功能。默认的 `video.low_latency_latest_frame = true` 会优先立即显示最新真实帧：FRUC合成帧仅计入实验指标，不进入网页顺序播放，因为中间帧只有在后一张真实帧到达后才能生成，强行显示会增加端到端延迟。低延迟路径使用FFmpeg/NVDEC输入和浏览器单帧长轮询；`/video.mjpg`仅保留为诊断兼容接口。将低延迟配置改为 `false` 才会恢复5 FPS到10 FPS的顺序插帧对比模式。
