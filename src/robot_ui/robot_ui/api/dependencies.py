from fastapi import Request, WebSocket

from robot_ui.runtime import Runtime


def runtime_from_request(request: Request) -> Runtime:
    return request.app.state.runtime


def runtime_from_websocket(websocket: WebSocket) -> Runtime:
    return websocket.app.state.runtime
