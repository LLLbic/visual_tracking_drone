# 无人机人员跟踪工作台（地面联调版）

这是第一阶段地面电脑程序：拉取无人机原生视频、被动接收 PX4/MAVLink 遥测、识别并锁定所选目标、计算跟随控制建议。视觉跟踪控制仍然**绝不向无人机发送**；另有默认关闭、必须由页面手动开启的地面 Offboard 零速度预备流与键盘 Offboard 速度发送器，用于拆桨或实体急停已锁定时验证上行链路。

当前默认视觉目标已改为红色铁桶。使用官方 `YOLOE-26s-seg` 的开放词汇提示 `red metal barrel / red oil drum / red barrel` 找出候选框，再要求框内 HSV 红色像素占比至少为 8%。页面“视觉模型切换”可在红桶准确版（26s）、红桶快速版（26n）、人员和 COCO 通用模型之间即时切换；切换时自动取消旧目标锁定。模型列表由 `config.toml` 的 `[vision_models.*]` 表维护，网页不能提交任意本地路径或网络地址。

## 已固化的安全要求

- 不修改任何 PX4 参数。
- 不使用 RC 通道覆盖，不模拟遥控器 PWM/摇杆。
- 不自动解锁、起飞或切换 Offboard。真实ARM/DISARM只能由页面明确操作并通过现场确认；当前不提供真实起飞按钮。
- 仅观察飞控模式；如果飞手切到 Land、Return/RTL、Position 等非 Offboard 模式，本地控制安全门立即关闭。
- 当前构建没有自动视觉跟踪控制发送器。电脑上的本地 MAVLink 路由占用外部 UDP `8080`，把飞控遥测分别复制给 QGC（`127.0.0.1:14551`）和网页工作站（`127.0.0.1:14550`）。配置中的视觉跟踪 `transmit_enabled` 必须为 `false`，改成 `true` 程序会拒绝启动。
- 本地路由强制实体 RC 摇杆最高优先级：丢弃电脑侧 `MANUAL_CONTROL`、`RC_CHANNELS_OVERRIDE`、非本程序固定身份发出的连续飞行设定值，以及被 QGC 回送的飞控遥测或应用设定值。这些检查切断 `飞控/应用 → QGC → 本地路由` 的放大回环。QGC仍可接收遥测，并保留显式模式切换、解锁、起飞、着陆、返航和暂停/制动等高层操作，但不能再用虚拟摇杆抢占飞控手动输入源。
- 网页程序允许三类受保护的飞控上行：手动开启的“地面 Offboard 心跳 / 零速度设定值”、手动开启的键盘 Offboard 速度设定值，以及每次均需现场确认的 ARM/DISARM、PX4 Pause、Land、RTL 单次命令。它们均经本地路由入口 `127.0.0.1:14560` 转发给 `192.168.1.201:8080`；不发送自动解锁、自动起飞、RC 覆盖或 `MANUAL_CONTROL`。视觉跟踪控制始终保持 TX=0。
- 地面预备流每次启动程序均为关闭状态；只有遥测在线且飞控明确为未解锁时才允许手动开启。遥测过期、解锁状态未知、检测到已解锁或本地急停时自动关闭。
- “跟踪与控制预览”区域只保留目标锁定和电脑端安全门操作；“本地急停”只把电脑端 PID 预览锁为零，不向飞控发送制动命令。
- 视频画面下方提供键盘 Offboard 速度控制：`W/S`=机体系前后、`A/D`=机体系左右、左 `Shift/Alt`=上升/下降、`Q/E`=左/右偏航。必须由页面明确开启“真实 TX”；地面仅在 `DISARMED + ON_GROUND + 实体急停已锁定` 时允许，空中仅在 `ARMED + IN_AIR + OFFBOARD` 时允许。松键或窗口失焦立即归零，浏览器输入中断超过 1.5 秒会自动停止发送。
- 画面下方的 Land/RTL 按钮发送真实高层命令，但只在 `ARMED + IN_AIR` 时可用，并要求现场安全确认；真实制动统一由视频下方受保护的 PX4 Pause 按钮执行。激光高度只接受向下安装的 `DISTANCE_SENSOR`（或旧版 `RANGEFINDER`）遥测，横向避障测距不会被误当成离地高度。
- “真实飞控动作”区域提供受保护的ARM开关和锁存式紧急制动。锁存一旦打开便立即禁止后续网页ARM：`ON_GROUND`时发送正常DISARM（包括取消刚发出的ARM），`ARMED + IN_AIR`时发送PX4 Pause而不直接停桨；解除锁存不会自动ARM。该锁存只约束网页，不能禁止实体遥控器或QGC解锁。所有命令均不使用强制解锁/强制停桨参数。
- RTSP使用TCP传输，读取超时为6秒；瞬时重连期间保留最后一帧，避免页面反复被错误占位图覆盖。网页状态栏会显示视频重连次数与当前捕获错误。
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

页面手动打开“键盘 Offboard 速度 TX”
  -> 10 Hz SET_POSITION_TARGET_LOCAL_NED（MAV_FRAME_BODY_NED）
  -> W/S 控制 vx，A/D 控制 vy，Shift/Alt 控制 vz，Q/E 控制 yaw_rate
  -> 地面急停锁定时可用于 QGC MAVLink Inspector 验证；不会解锁或启动旋翼
  -> 空中只有飞手已切入 Offboard 后才允许；切回其他模式立即停止
  -> 松键归零，页面失联或遥测安全条件失效时自动停止
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
5. 电脑起飞前，先完成页面现场安全确认并打开“Offboard 起飞预备”。等待页面显示“ARM预备已就绪”，再由飞手通过实体遥控器或 QGC 手动切入 `OFFBOARD`。程序不会自动切换模式。
6. 确认飞控仍为 `ON_GROUND`、实体 Kill Switch 已解除且遥控器可立即接管，然后使用网页 ARM 开关发送一次真实 ARM。ARM 后电机是否怠速旋转由 PX4 当前设置决定。
7. 飞控心跳确认 `ARMED` 后打开键盘真实 TX。地面阶段只接受左 Shift 的上升速度，W/S/A/D/左 Alt/Q/E 均被强制为零；PX4 报告 `IN_AIR` 后才开放全部方向键。松开按键后速度立即回零。
8. 任意异常应立即松键，并由飞手切回 Position/返航/降落或使用实体急停。网页急停会关闭两个设定值发送器并阻止后续网页 ARM，但不替代实体遥控器。

## 重要技术边界

- ByteTrack 依靠运动和框匹配保持 ID，并不是真正的外观 ReID。当前版本的“重新找到”是在短时丢失后按中心距离和框高变化选候选人。多人交叉、长时间遮挡时可能认错人。
- 无人机相机本身在运动，后续可将 `multi_object_tracker` 改为带相机运动补偿/外观 ReID 的跟踪器，并做对比测试。
- 人体框高只能得到相对距离。第一版用“目标框高占比”控制远近；在镜头俯仰变化、人物蹲下、遮挡时会产生误差。真机控制前应加入测距/GPS/深度或至少相机标定与高度补偿。
- 数秒级视频延迟不能直接用于闭环飞行。必须先把端到端延迟降到稳定的亚秒级，并在控制层加入时间戳、丢帧检测、预测与速度限制。
- 后续 MAVSDK 阶段的约束见 [docs/CONTROL_PHASE_DESIGN.md](docs/CONTROL_PHASE_DESIGN.md)。

## 运行纯逻辑测试

测试不需要摄像头、MAVLink 或第三方包：

```powershell
python -m unittest discover -s tests -v
```
# MAVLink 本地转发服务器启动指令（管理员）
powershell -ExecutionPolicy Bypass -File tools/start_preview.ps1

# 启动摄像头（管理员）
cd C:\Users\dingr\source\repos\ZJ\uav_tracking_preview
powershell -ExecutionPolicy Bypass -File tools\configure_d7_camera_network.ps1
