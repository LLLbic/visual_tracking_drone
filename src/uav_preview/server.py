from __future__ import annotations

from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel, Field

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
        return StreamingResponse(runtime.video.mjpeg(), media_type="multipart/x-mixed-replace; boundary=frame")

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
        runtime.disable_ground_offboard_test("本地急停触发")
        runtime.gate.latch_estop()
        return {"ok": True, "message": "本地控制已锁存为停止；地面 Offboard 预备流已关闭"}

    @app.post("/api/safety/reset-local-estop")
    async def reset_local_estop() -> dict[str, object]:
        runtime.gate.reset_estop()
        return {"ok": True, "message": "仅解除本地预览锁存；飞控状态未改变"}

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
            "message": "键盘真实设定值发送已开启；松键归零，输入中断将自动停止",
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

    @app.post("/api/flight/brake")
    async def brake() -> dict[str, object]:
        try:
            command_state = runtime.brake()
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {
            "ok": True,
            "message": "真实PX4紧急制动/暂停命令已发送，等待飞控COMMAND_ACK确认",
            "state": command_state,
        }

    @app.post("/api/flight/emergency-latch")
    async def emergency_latch(request: EmergencyLatchRequest) -> dict[str, object]:
        expected = "ENGAGE" if request.enabled else "RELEASE"
        if request.confirmation != expected:
            raise HTTPException(status_code=409, detail=f"必须明确确认{expected}")
        state = runtime.set_emergency_latch(request.enabled)
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
                "message": "该通用动作接口保持禁用：降落、返航和速度控制未接入。真实ARM/DISARM与PX4 Pause只通过专用受保护接口执行。",
            },
        )

    return app
