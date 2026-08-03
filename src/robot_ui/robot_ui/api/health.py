from fastapi import APIRouter, Depends

from robot_ui.api.dependencies import runtime_from_request
from robot_ui.runtime import Runtime


router = APIRouter(prefix="/api/v1", tags=["health"])


@router.get("/health")
def health(runtime: Runtime = Depends(runtime_from_request)):
    state = runtime.store.snapshot()
    return {
        "status": "ok",
        "robot_state": state["robot"].get("state", "UNKNOWN"),
        "ros_connected": state["robot"].get("ros_connected", False),
        "read_only": runtime.config.features.get("read_only", True),
    }


@router.get("/state")
def state(runtime: Runtime = Depends(runtime_from_request)):
    return runtime.store.envelope()


@router.get("/config")
def config(runtime: Runtime = Depends(runtime_from_request)):
    return {
        "source": str(runtime.config.source),
        "server": runtime.config.server,
        "ros": runtime.config.ros,
        "features": runtime.config.features,
    }
