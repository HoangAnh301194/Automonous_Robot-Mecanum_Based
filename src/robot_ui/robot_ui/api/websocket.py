import asyncio

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from robot_ui.api.dependencies import runtime_from_websocket


router = APIRouter(tags=["telemetry"])


@router.websocket("/ws/telemetry")
async def telemetry(websocket: WebSocket):
    await websocket.accept()
    runtime = runtime_from_websocket(websocket)
    interval = max(
        0.1,
        runtime.config.server.get("telemetry_interval_ms", 500) / 1000.0,
    )

    try:
        while True:
            await websocket.send_json(runtime.store.envelope())
            await asyncio.sleep(interval)
    except WebSocketDisconnect:
        return
