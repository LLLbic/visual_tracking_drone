# 无人机人员跟踪工作台（地面联调版）

这是第一阶段地面电脑程序：拉取无人机原生视频、被动接收 PX4/MAVLink 遥测、识别并锁定所选目标、计算跟随控制建议。视觉跟踪控制仍然**绝不向无人机发送**；另有默认关闭、必须由页面手动开启的地面 Offboard 零速度测试流，以及以启动瞬间本地位置为高度零点的受保护 Offboard 起飞/悬停/位置式键盘交接状态机。

当前默认视觉目标已改为红色铁桶。使用官方 `YOLOE-26s-seg` 的开放词汇提示 `red metal barrel / red oil drum / red barrel` 找出候选框，再要求框内 HSV 红色像素占比至少为 8%。页面“视觉模型切换”可在红桶准确版（26s）、红桶快速版（26n）、人员和 COCO 通用模型之间即时切换；切换时自动取消旧目标锁定。模型列表由 `config.toml` 的 `[vision_models.*]` 表维护，网页不能提交任意本地路径或网络地址。

当前 `experiment/nvidia-optical-flow-sdk` 分支包含 NVIDIA Optical Flow SDK 的 FRUC 2× 插帧实验及低延迟视频路径。默认由独立 FFmpeg 进程通过 UDP 拉流，并用 RTX 4070 的 `h264_cuvid`/NVDEC 解码；OpenCV不再管理RTSP连接。采集线程与YOLO/绘制线程分离，处理来不及时直接丢弃旧帧。浏览器通过单帧长轮询获取最新JPEG。为优先降低延迟，FRUC当前默认关闭；需要A/B实验时可重新开启，低延迟模式仍不会把时间上更旧的中间帧排到真实帧前面。YOLO、目标锁定、PID 与飞控链路始终只接受真实帧。相机已实测 H.264 / 15 FPS / 15帧关键帧间隔；低于1秒的端到端目标仍需现场计时验收，不能由网页FPS推算。调整记录见 [docs/CAMERA_LOW_LATENCY.md](docs/CAMERA_LOW_LATENCY.md)，插帧实验见 [docs/NVIDIA_FRUC_EXPERIMENT.md](docs/NVIDIA_FRUC_EXPERIMENT.md)。

## 已固化的安全要求

2026-09-15：在纯位置起飞/悬停基线上实现位置式键盘交接，参见
[平滑交接说明](docs/KEYBOARD_HANDOFF.md)。起飞到键盘移动全程复用同一 LOCAL_NED 位置发送器，
旧的独立 BODY_NED 速度发送器已关闭。交接必须由当前网页手动授权，不会自动倒计时开启；本版本仅完成离线验证，尚未部署或实飞验收。

- 不修改任何 PX4 参数。
- 不使用 RC 通道覆盖，不模拟遥控器 PWM/摇杆。
- 程序启动时不自动解锁、起飞或切换 Offboard。页面提供用户明确确认后才运行的 PX4 原生起飞和本地 Offboard 起飞两种独立状态机，高度均硬限制为 `1.0–3.0 m`；两者互斥且不会随开机自动运行。
- 仅观察飞控模式；如果飞手切到 Land、Return/RTL、Position 等非 Offboard 模式，本地控制安全门立即关闭。
- 当前构建没有自动视觉跟踪控制发送器。电脑上的本地 MAVLink 路由占用外部 UDP `8080`，把飞控遥测分别复制给 QGC（`127.0.0.1:14551`）和网页工作站（`127.0.0.1:14550`）。配置中的视觉跟踪 `transmit_enabled` 必须为 `false`，改成 `true` 程序会拒绝启动。
- 本地路由强制实体 RC 摇杆最高优先级：丢弃电脑侧 `MANUAL_CONTROL`、`RC_CHANNELS_OVERRIDE`、非本程序固定身份发出的连续飞行设定值，以及被 QGC 回送的飞控遥测或应用设定值。这些检查切断 `飞控/应用 → QGC → 本地路由` 的放大回环。QGC仍可接收遥测，并保留显式模式切换、解锁、起飞、着陆、返航和暂停/制动等高层操作，但不能再用虚拟摇杆抢占飞控手动输入源。
- 网页程序允许受保护的飞控上行：手动开启的“地面 Offboard 心跳 / 零速度设定值”、本地起飞状态机内显式授权的位置式键盘目标、每次均需现场确认的 ARM/DISARM、PX4 Pause、Land、RTL 单次命令，以及用户明确启动的 PX4 原生起飞或本地 Offboard 起飞状态机。它们均经本地路由入口 `127.0.0.1:14560` 转发给 `192.168.1.201:8080`；不发送 RC 覆盖或 `MANUAL_CONTROL`。视觉跟踪控制始终保持 TX=0。
- 地面预备流每次启动程序均为关闭状态；只有遥测在线且飞控明确为未解锁时才允许手动开启。遥测过期、解锁状态未知、检测到已解锁或本地急停时自动关闭。
- “跟踪与控制预览”区域只保留目标锁定和电脑端安全门操作；“本地急停”只把电脑端 PID 预览锁为零，不向飞控发送制动命令。
- 视频画面下方显示位置式键盘输入：`W/S`=机体朝向前后，`A/D`=左右，左 `Shift/Alt`=上升/下降，`Q/E`=左/右偏航。按键只改变同一控制器内部的连续位置/航向轨迹，不模拟油门或遥控器通道。完成本地指定高度悬停并满足稳定条件后仍需人工授权；失焦、输入超时或 CH8 离开 Offboard 会撤权或停止电脑发送，且不会自动恢复。
- 画面下方的 Land/RTL 按钮发送真实高层命令，但只在 `ARMED + IN_AIR` 时可用，并要求现场安全确认；PX4 Pause 是独立的空中暂停动作，不当作停桨急停。激光高度只接受向下安装的 `DISTANCE_SENSOR`（或旧版 `RANGEFINDER`）遥测，横向避障测距不会被误当成离地高度。
- “真实飞控动作”区域提供受保护的地面 ARM 开关和“安全处置锁存”。网页 ARM 只允许 `ON_GROUND` 的 Position/手动辅助模式，且要求 CH6 实体急停已解除、CH8 已离开 Offboard、所有电脑设定值发送器已关闭。安全处置打开时先原子锁住危险命令，再关闭电脑 TX 与起飞状态机：`ON_GROUND` 时发送正常 DISARM，`ARMED + IN_AIR` 时发送 LAND，绝不空中强制停桨。锁存后网页 ARM、Takeoff、Pause、RTL 与连续设定值均被拒绝，受控 LAND 和地面 DISARM 仍可执行。解除采用失效安全策略：必须有实时遥测确认 `DISARMED + ON_GROUND`、实体 CH6 Kill Switch 保持触发高位、CH8 已离开 Offboard，且所有电脑发送器/起飞流程均已停止；解除本身不会自动 ARM。该锁存不能禁止实体遥控器或 QGC 再次解锁，不能替代实体 Kill Switch。所有命令均不使用强制解锁/强制停桨参数，COMMAND_ACK 拒绝会在页面标红。
- 指定高度起飞状态机要求遥测、本地位置、全球位置和姿态数据均实时有效，连续稳定检查通过后只发送一次 ARM 和一次 `MAV_CMD_NAV_TAKEOFF`。目标高度通过当前地面 AMSL 海拔计算，不写 `MIS_TAKEOFF_ALT` 等 PX4 参数。飞控拒绝、离地/爬升超时、倾斜超过20°、水平漂移超过1米或高度超调超过0.5米时，状态机会在地面请求正常 DISARM、空中请求 LAND；实体CH6/CH8变化则立即停止流程并让飞手接管，不与遥控器竞争。
- 本地 Offboard 起飞不需要全球坐标：程序把启动瞬间的 `LOCAL_POSITION_NED x0/y0/z0/yaw0` 作为定点与高度基准，以10 Hz预发送当前位置；只有页面提示就绪且飞手用实体CH8手动切入Offboard后，才发送一次正常ARM，再以0.30 m/s斜坡把目标设为 `z0 - 目标高度`。达到高度后同一发送器继续定点，稳定检查通过且人工授权后才接受键盘轨迹。所有阶段保持 `LOCAL_NED + type_mask 2552`；CH6触发、CH8/模式退出、遥测过期、坐标突变或目标越界会撤权或停止任务。它不改PX4参数、不自动切模式、不发送RC覆盖；没有有效全球定位时RTL不可作为唯一安全手段。
- 程序在真实飞控明确报告 `DISARMED + ON_GROUND` 后，会用 `MAV_CMD_SET_MESSAGE_INTERVAL` 临时请求 `LOCAL_POSITION_NED`、`GLOBAL_POSITION_INT` 和 `EXTENDED_SYS_STATE` 的更新频率。它不写PX4参数、不切换模式、不发送设定值，并且飞行中绝不重发。页面显示实测频率与新鲜度；飞控心跳、姿态、落地状态、RC或位置任一过期都会关闭起飞条件。
- RTSP默认使用独立FFmpeg进程进行UDP低延迟传输：1 MB套接字缓冲、32包乱序队列和100 ms最大乱序等待，并用 `h264_cuvid` 调用RTX 4070的NVDEC。采集与YOLO处理分线程，网页通过 `/video/latest.jpg` 长轮询且只接收最新完成帧。rawvideo输出固定单线程，避免编码器额外帧队列；RTSP使用协议自身的socket超时。瞬时中断期间保留最后一张有效画面。需要优先稳定性时仍可把 `video.transport` 改回 `tcp`；已识别的NVDEC初始化错误会回退CPU解码。
- 遥控杆显示优先读取 `MANUAL_CONTROL`；若飞控只广播 `RC_CHANNELS`，则按常见的通道 1/2/3/4 = Roll/Pitch/Throttle/Yaw 做只读近似显示。程序不会读取或修改 PX4 的实际通道映射参数，飞行前必须以 QGC 校准界面核对映射。

## 当前现场链路结论

- 电脑有线口：`192.168.1.123/24`。
- 已发现 MiniHomer 设备：`192.168.1.101`、`.102`、`.201`、`.202`。
- 已实测 `192.168.1.201:8080` 持续向电脑发送 MAVLink 2 数据包。本地 MAVLink 路由监听 UDP `8080`，QGC 改为监听 `14551` 并把上行发往 `127.0.0.1:14560`。
- MiniHomer 官方摄像头默认 IP 为 `192.168.1.10/11/12`，RTSP 主码流格式为：

  ```text
  rtsp://192.168.1.10:554/user=admin&password=&channel=1&stream=0.sdp?
  ```

- 本次扫描中 `.10/.11/.12` 均未实际应答，说明当前摄像头或视频天空端链路尚未在线。程序会显示等待页并自动重连，不会因此触发控制。

## 数据流

```text
MiniHomer 原生 RTSP
  -> YOLO 人体检测
  -> ByteTrack 多目标 ID
  -> 点击选定目标
  -> CSRT / DaSiamRPN 高频单目标跟踪
  -> 框中心偏差 + 框高占比
  -> PID 前后速度/Yaw 角速度建议
  -> 安全门检查
  -> 页面预览（视觉跟踪控制实际发送恒为 0）

MiniHomer UDP 8080 MAVLink 2
  -> 本地 MAVLink 路由监听 0.0.0.0:8080
  -> 原样复制给 QGC 127.0.0.1:14551
  -> 原样复制给网页工作站 127.0.0.1:14550
  -> 模式、解锁、姿态、杆量、NED 速度、高度、油门显示
  -> 正常状态下网页不发送飞控数据

QGC 上行
  -> QGC 发送到本地路由 127.0.0.1:14560
  -> 路由拦截虚拟摇杆、RC覆盖和未授权连续设定值
  -> 模式、解锁、起降、返航、暂停/制动及其他QGC通信继续转发
  -> 从电脑外部 UDP 8080 发往 MiniHomer 192.168.1.201:8080

页面手动打开“地面 Offboard 心跳 / 零速度设定值”
  -> 发送到本地路由 127.0.0.1:14560
  -> 路由从电脑外部 UDP 8080 发往 MiniHomer 192.168.1.201:8080
  -> 5 Hz SET_POSITION_TARGET_LOCAL_NED（vx=vy=vz=0）
  -> 只建立 MAVLink Offboard proof-of-life，不切模式、不解锁
  -> 遥测异常、已解锁或本地急停时自动停止

本地起飞悬停后手动授权“位置式键盘交接”
  -> 继续复用10 Hz LOCAL_NED位置发送器（type_mask 2552）
  -> W/S/A/D/Shift/Alt/Q/E只改变内部受限轨迹，报文速度字段保持零并被忽略
  -> 授权不发额外控制包；切回其他模式立即停止电脑设定值
  -> 松键受限减速并定点，页面失联撤权且不自动恢复
```

## 安装和启动

建议用 `uv` 创建独立的 Python 3.12 环境。在本目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File tools/setup_windows.ps1
powershell -ExecutionPolicy Bypass -File tools/start_preview.ps1
```

浏览器打开：

```text
http://127.0.0.1:8765
```

当前 D7 本地视觉链路为：电脑从机内摄像头
`192.168.111.11:554` 拉取 RTSP 原始码流，使用 YOLO26n + ByteTrack
检测并跟踪 `person`，随后通过网页的 `/video.mjpg` 显示带检测框的视频。
该摄像头位于 D7 内部 `192.168.111.0/24` 网络；电脑的有线网卡必须具有
此网段的直连地址或路由，RTSP 拉流才会成功。

Windows 视觉环境固定使用 PyTorch 2.14 的 CUDA 13.0 构建，并由 RTX 4070
通过 `cuda:0` 执行 YOLO 推理。运行 `uv sync --extra all` 会从项目声明的
PyTorch CUDA 专用索引恢复正确的 GPU 版本，而不会退回 CPU 包。

低延迟采集还要求系统PATH中存在FFmpeg，并且构建包含 `h264_cuvid`。程序默认
不依赖OpenCV的GStreamer或CUDA VideoReader；FFmpeg把解码后的固定尺寸BGR帧
写入管道，OpenCV只负责后续视觉处理。当前子码流尺寸固定为640×360，修改摄像头
分辨率后必须同步修改 `video.frame_width` 和 `video.frame_height`。
当前相机已切换H.264，`video.ffmpeg_decoder` 同步设为 `h264_cuvid`。
如恢复H.265原配置，必须同步改回 `hevc_cuvid`，不能仅改变解码器来转换相机编码格式。

如果电脑通过 D7 数传的有线网口能够二层访问该内部网段，可在“管理员
PowerShell”中执行以下脚本，为电脑添加一个仅当前开机有效的辅助地址：

```powershell
powershell -ExecutionPolicy Bypass -File tools/configure_d7_camera_network.ps1
```

测试结束后可撤销：

```powershell
powershell -ExecutionPolicy Bypass -File tools/configure_d7_camera_network.ps1 -Remove
```

如摄像头地址不确定，可先运行只读探测脚本：

```powershell
powershell -ExecutionPolicy Bypass -File tools/discover_native_video.ps1
```

脚本只尝试解码 `.10/.11/.12/.175` 的主、辅码流，不发送飞控数据。找到地址后写入 `config.toml` 的 `video.source`。

首次使用 Ultralytics 官方模型名时会下载权重。也可以把自己的 `best.pt` 放到本机，然后在 `config.toml` 中修改 `model_path`。自定义模型只要把人体类别名写入 `target_class_names`，或把类别编号写入 `target_class_ids`，后续 ByteTrack 逻辑无需修改。

### 与 QGC 同时显示遥测

当前使用 **QGC 主链路 + 单向本机转发**：

1. QGC 的自定义 UDP 链路直接监听 `8080`。
2. 关闭 QGC 默认的 UDP 自动连接，防止它额外占用 `14550`。
3. 在 QGC 的 MAVLink 设置中启用转发，目标为 `127.0.0.1:14550`。
4. 网页程序只监听 `127.0.0.1:14550`。

QGC 4.4.4 的 MAVLink 转发是单向的：只把已连接车辆的消息发给指定 UDP 端点，从该端点收到的消息会被忽略。因此网页无法经由 `14550` 反向控制飞控；QGC 自身仍保持原有 `8080` 主链路能力。

## 使用方法

1. 确认桨叶已拆除或无人机处于安全测试状态。
2. 打开程序后先确认“遥测在线”和视频画面。
3. 在画面中点击希望跟踪的人。ByteTrack 提供多目标 ID，锁定后由 CSRT 在两次 YOLO 推理之间更新框。
4. 页面只显示 PID 的“建议前后速度”和“建议 Yaw 角速度”。自动视觉跟踪控制的“实际已发送”必须始终为 `0 / 0`。
5. 如需验证上行链路，只能在拆桨或实体急停触发、`DISARMED + ON_GROUND` 时打开“Offboard 地面链路测试”，并在 QGC Inspector 中确认零速度包；验证后关闭。本测试不参与 ARM 或起飞。
6. 确认飞控为 `DISARMED + ON_GROUND + POSCTL`、实体 CH6 Kill Switch 已解除、CH8 已离开 Offboard、定位有效且遥控器可立即接管。可使用单独 ARM 开关做地面测试，也可在 `1.0–3.0 m` 内设置高度并明确确认启动状态机；两种流程不能同时运行。
7. 本地状态机完成并稳定悬停后，保持实体 CH8 在 `OFFBOARD`；确认全部按键松开并由当前网页手动授权位置式键盘交接。授权本身不改变悬停目标，不能使用已停用的旧速度 TX。
8. 任意异常应立即松键，并由飞手切回 Position/返航/降落或使用实体急停。页面“取消/安全退出”会按落地状态请求地面 DISARM 或空中 LAND；网页安全处置锁存同样会关闭电脑发送并阻止后续网页 ARM，但二者都不替代实体遥控器。

## 重要技术边界

- ByteTrack 依靠运动和框匹配保持 ID，并不是真正的外观 ReID。当前版本的“重新找到”是在短时丢失后按中心距离和框高变化选候选人。多人交叉、长时间遮挡时可能认错人。
- 无人机相机本身在运动，后续可将 `multi_object_tracker` 改为带相机运动补偿/外观 ReID 的跟踪器，并做对比测试。
- 人体框高只能得到相对距离。第一版用“目标框高占比”控制远近；在镜头俯仰变化、人物蹲下、遮挡时会产生误差。真机控制前应加入测距/GPS/深度或至少相机标定与高度补偿。
- 数秒级视频延迟不能直接用于闭环飞行。必须先把端到端延迟降到稳定的亚秒级，并在控制层加入时间戳、丢帧检测、预测与速度限制。
- 后续 MAVSDK 阶段的约束见 [docs/CONTROL_PHASE_DESIGN.md](docs/CONTROL_PHASE_DESIGN.md)。

## 运行纯逻辑测试

使用已安装项目遥测及开发依赖的Python环境；测试使用模拟数据及本地回环测试端口，不连接真实摄像头或无人机：

```powershell
python -m unittest discover -s tests -v
```
# MAVLink 本地转发服务器启动指令（管理员）
powershell -ExecutionPolicy Bypass -File tools/start_preview.ps1

# 启动摄像头（管理员）
cd C:\Users\dingr\source\repos\ZJ\uav_tracking_preview
powershell -ExecutionPolicy Bypass -File tools\configure_d7_camera_network.ps1
