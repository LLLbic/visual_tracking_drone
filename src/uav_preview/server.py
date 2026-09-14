from __future__ import annotations

from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from pydantic import BaseModel, Field, StrictBool, StrictInt

from .config import AppConfig
from .runtime import Runtime


class TargetClick(BaseModel):
    x: float = Field(ge=0.0, le=1.0)
    y: float = Field(ge=0.0, le=1.0)


class TuningUpdate(BaseModel):
    target_bbox_height_ratio: float | None = None
    yaw_deadband: float | None = None
    distance_deadband: float | None = None
    yaw_kp: float | None = None
    yaw_ki: float | None = None
    yaw_kd: float | None = None
    distance_kp: float | None = None
    distance_ki: float | None = None
    distance_kd: float | None = None
    max_yaw_rate_deg_s: float | None = None
    max_forward_speed_m_s: float | None = None


class ModelSelection(BaseModel):
    model_id: str = Field(min_length=1, max_length=80)


class ArmRequest(BaseModel):
    armed: bool
    confirmation: str = Field(min_length=3, max_length=12)


class EmergencyLatchRequest(BaseModel):
    enabled: bool
    confirmation: str = Field(min_length=6, max_length=12)


class KeyboardAxes(BaseModel):
    pitch: float = Field(ge=-1.0, le=1.0)
    roll: float = Field(ge=-1.0, le=1.0)
    throttle: float = Field(ge=-1.0, le=1.0)
    yaw: float = Field(ge=-1.0, le=1.0)


class ExplicitConfirmation(BaseModel):
    confirmation: str = Field(min_length=3, max_length=12)


class HandoffIdentity(BaseModel):
    run_id: str = Field(min_length=16, max_length=80)
    client_id: str = Field(min_length=16, max_length=80)


class HandoffInput(HandoffIdentity, KeyboardAxes):
    sequence: StrictInt = Field(ge=0)
    foreground: StrictBool
    confirmed: StrictBool
    keys_released: StrictBool
    token: str | None = Field(default=None, min_length=16, max_length=128)


class HandoffAuthorization(HandoffIdentity):
    confirmation: str


class HandoffRevocation(HandoffIdentity):
    token: str = Field(min_length=16, max_length=128)


class TakeoffRequest(BaseModel):
    target_height_m: float = Field(ge=1.0, le=3.0)
    confirmation: str = Field(min_length=7, max_length=12)


def create_app(config: AppConfig) -> FastAPI:
    runtime = Runtime(config)

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        runtime.start()
        try:
            yield
        finally:
            runtime.stop()

    app = FastAPI(title="UAV Tracking Preview", lifespan=lifespan)
    html_path = Path(__file__).with_name("web") / "index.html"

    @app.get("/", response_class=HTMLResponse)
    async def index() -> str:
        return html_path.read_text(encoding="utf-8")

    @app.get("/video.mjpg")
    async def video_stream() -> StreamingResponse:
        return StreamingResponse(
            runtime.video.mjpeg(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={
                "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
                "Pragma": "no-cache",
                "Expires": "0",
                "X-Accel-Buffering": "no",
            },
        )

    @app.get("/video/latest.jpg")
    def latest_video_frame(after: int = -1) -> Response:
        serial, jpeg = runtime.video.wait_for_jpeg(after_serial=after, timeout=1.0)
        headers = {
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
            "Expires": "0",
            "X-Frame-Serial": str(serial),
        }
        if jpeg is None:
            return Response(status_code=204, headers=headers)
        return Response(content=jpeg, media_type="image/jpeg", headers=headers)

    @app.get("/api/state")
    async def state() -> dict[str, object]:
        return runtime.state()

    @app.post("/api/target/select")
    async def select_target(click: TargetClick) -> dict[str, object]:
        runtime.video.select_target(click.x, click.y)
        return {"ok": True, "message": "已请求锁定点击位置附近的人体目标"}

    @app.post("/api/target/clear")
    async def clear_target() -> dict[str, object]:
        runtime.video.clear_target()
        return {"ok": True}

    @app.post("/api/models/select")
    async def select_model(selection: ModelSelection) -> dict[str, object]:
        try:
            model_state = runtime.video.select_model(selection.model_id)
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        return {
            "ok": True,
            "message": "已请求切换视觉模型；新模型就绪前不会产生目标锁定",
            "models": model_state,
        }

    @app.post("/api/safety/local-estop")
    async def local_estop() -> dict[str, object]:
        state = runtime.latch_local_control_only("兼容接口触发本地安全锁存")
        return {"ok": True, **state}

    @app.post("/api/safety/reset-local-estop")
    async def reset_local_estop(request: EmergencyLatchRequest) -> dict[str, object]:
        if request.enabled or request.confirmation != "RELEASE":
            raise HTTPException(status_code=409, detail="必须明确确认RELEASE")
        try:
            state = runtime.set_emergency_latch(False)
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, **state}

    @app.post("/api/offboard-test/enable")
    async def enable_ground_offboard_test(request: ExplicitConfirmation) -> dict[str, object]:
        if request.confirmation != "PREPARE":
            raise HTTPException(status_code=409, detail="必须明确确认PREPARE")
        try:
            state = runtime.enable_ground_offboard_test()
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {
            "ok": True,
            "message": "地面 Offboard 零速度预备流已开启（5 Hz）；未切模式、未解锁",
            "state": state,
        }

    @app.post("/api/offboard-test/disable")
    async def disable_ground_offboard_test() -> dict[str, object]:
        state = runtime.disable_ground_offboard_test()
        return {
            "ok": True,
            "message": "地面 Offboard 零速度预备流已关闭",
            "state": state,
        }

    @app.post("/api/keyboard-control/presence")
    async def keyboard_presence() -> dict[str, object]:
        runtime.keyboard_control.browser_presence()
        return {"ok": True}

    @app.post("/api/keyboard-control/enable")
    async def enable_keyboard_control(request: ExplicitConfirmation) -> dict[str, object]:
        if request.confirmation != "ENABLE":
            raise HTTPException(status_code=409, detail="必须明确确认ENABLE")
        try:
            state = runtime.enable_keyboard_control()
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {
            "ok": True,
            "message": "空中键盘Offboard速度发送已开启；松键归零，输入中断或实体CH8离开Offboard将自动停止",
            "state": state,
        }

    @app.post("/api/keyboard-control/disable")
    async def disable_keyboard_control() -> dict[str, object]:
        return {
            "ok": True,
            "message": "键盘真实设定值发送已关闭并归零",
            "state": runtime.disable_keyboard_control(),
        }

    @app.post("/api/keyboard-control/axes")
    async def update_keyboard_axes(axes: KeyboardAxes) -> dict[str, object]:
        try:
            state = runtime.update_keyboard_control(
                axes.pitch, axes.roll, axes.throttle, axes.yaw
            )
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, "state": state}

    @app.post("/api/flight/arm")
    async def set_armed(request: ArmRequest) -> dict[str, object]:
        expected = "ARM" if request.armed else "DISARM"
        if request.confirmation != expected:
            raise HTTPException(status_code=409, detail=f"必须明确确认{expected}")
        try:
            command_state = runtime.set_armed(request.armed)
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {
            "ok": True,
            "message": f"真实{expected}命令已发送，等待飞控COMMAND_ACK与心跳确认",
            "state": command_state,
        }

    @app.post("/api/takeoff/start")
    async def start_takeoff(request: TakeoffRequest) -> dict[str, object]:
        if request.confirmation != "TAKEOFF":
            raise HTTPException(status_code=409, detail="必须明确确认TAKEOFF")
        try:
            state = runtime.begin_takeoff(request.target_height_m)
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {
            "ok": True,
            "message": (
                f"指定高度起飞状态机已启动：{request.target_height_m:.1f}米。"
                "先进行稳定地面检查，再依次发送ARM和PX4原生Takeoff。"
            ),
            "state": state,
        }

    @app.post("/api/takeoff/abort")
    async def abort_takeoff(request: ExplicitConfirmation) -> dict[str, object]:
        if request.confirmation != "ABORT":
            raise HTTPException(status_code=409, detail="必须明确确认ABORT")
        try:
            state = runtime.abort_takeoff()
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {
            "ok": True,
            "message": "指定高度起飞流程已取消；地面请求正常DISARM，空中请求LAND",
            "state": state,
        }

    @app.post("/api/local-takeoff/start")
    async def start_local_takeoff(request: TakeoffRequest) -> dict[str, object]:
        if request.confirmation != "TAKEOFF":
            raise HTTPException(status_code=409, detail="必须明确确认TAKEOFF")
        try:
            state = runtime.begin_local_takeoff(request.target_height_m)
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {
            "ok": True,
            "message": (
                f"本地Offboard起飞预发送已启动：相对当前高度{request.target_height_m:.1f}米。"
                "请等待页面提示后，再用实体遥控器CH8手动切入Offboard；程序不会自动切模式。"
            ),
            "state": state,
        }

    @app.post("/api/local-takeoff/abort")
    async def abort_local_takeoff(request: ExplicitConfirmation) -> dict[str, object]:
        if request.confirmation != "ABORT":
            raise HTTPException(status_code=409, detail="必须明确确认ABORT")
        try:
            state = runtime.abort_local_takeoff()
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {
            "ok": True,
            "message": "本地Offboard起飞已受控退出：地面正常DISARM，空中请求LAND",
            "state": state,
        }

    @app.post("/api/local-takeoff/keyboard/input")
    async def handoff_input(request: HandoffInput) -> dict[str, object]:
        try:
            state = runtime.report_handoff_input(run_id=request.run_id, client_id=request.client_id,
                sequence=request.sequence, axes=(request.pitch,request.roll,request.throttle,request.yaw),
                foreground=request.foreground,confirmed=request.confirmed,keys_released=request.keys_released,token=request.token)
            return {"ok":True,"state":state}
        except ValueError as exc:
            raise HTTPException(status_code=409,detail=str(exc)) from exc

    @app.post("/api/local-takeoff/keyboard/authorize")
    async def authorize_handoff(request: HandoffAuthorization) -> dict[str, object]:
        if request.confirmation != "HANDOFF":
            raise HTTPException(status_code=409,detail="必须明确确认HANDOFF")
        try:
            result = runtime.authorize_handoff(request.run_id,request.client_id)
            return {"ok":True,"message":"键盘待命：原位置目标不变，同一个发送器继续运行",**result}
        except ValueError as exc:
            raise HTTPException(status_code=409,detail=str(exc)) from exc

    @app.post("/api/local-takeoff/keyboard/revoke")
    async def revoke_handoff(request: HandoffRevocation) -> dict[str, object]:
        try:
            state=runtime.revoke_handoff(request.run_id,request.client_id,request.token)
            return {"ok":True,"state":state,"message":"已撤销键盘权限；原轨迹减速并保持，不发送LAND"}
        except ValueError as exc:
            raise HTTPException(status_code=409,detail=str(exc)) from exc

    @app.post("/api/flight/brake")
    async def brake(request: ExplicitConfirmation) -> dict[str, object]:
        if request.confirmation != "BRAKE":
            raise HTTPException(status_code=409, detail="必须明确确认BRAKE")
        try:
            command_state = runtime.brake()
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {
            "ok": True,
            "message": "真实PX4 Pause命令已发送，等待飞控COMMAND_ACK确认；此接口不是停桨急停",
            "state": command_state,
        }

    @app.post("/api/flight/emergency-latch")
    async def emergency_latch(request: EmergencyLatchRequest) -> dict[str, object]:
        expected = "ENGAGE" if request.enabled else "RELEASE"
        if request.confirmation != expected:
            raise HTTPException(status_code=409, detail=f"必须明确确认{expected}")
        try:
            state = runtime.set_emergency_latch(request.enabled)
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, **state}

    @app.post("/api/flight/land")
    async def land(request: ExplicitConfirmation) -> dict[str, object]:
        if request.confirmation != "LAND":
            raise HTTPException(status_code=409, detail="必须明确确认LAND")
        try:
            state = runtime.land()
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, "message": "真实降落命令已发送", "state": state}

    @app.post("/api/flight/rtl")
    async def rtl(request: ExplicitConfirmation) -> dict[str, object]:
        if request.confirmation != "RTL":
            raise HTTPException(status_code=409, detail="必须明确确认RTL")
        try:
            state = runtime.rtl()
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, "message": "真实返航命令已发送", "state": state}

    @app.post("/api/tuning")
    async def update_tuning(update: TuningUpdate) -> dict[str, object]:
        values = {key: value for key, value in update.model_dump().items() if value is not None}
        try:
            runtime.controller.update_tuning(values)
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        return {"ok": True, "tuning": runtime.controller.tuning()}

    @app.post("/api/action/{action}")
    async def disabled_aircraft_action(action: str) -> JSONResponse:
        return JSONResponse(
            status_code=423,
            content={
                "ok": False,
                "action": action,
                "message": "该通用动作接口保持禁用。ARM/DISARM、PX4 Pause、Land、RTL和空中键盘速度只通过各自的受保护接口执行。",
            },
        )

    return app
