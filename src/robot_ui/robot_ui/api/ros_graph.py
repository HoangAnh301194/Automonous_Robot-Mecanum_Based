from fastapi import APIRouter, Depends

from robot_ui.api.dependencies import runtime_from_request
from robot_ui.runtime import Runtime


router = APIRouter(prefix="/api/v1/ros", tags=["ros-graph"])


@router.get("/graph")
def ros_graph(runtime: Runtime = Depends(runtime_from_request)):
    return runtime.store.snapshot().get("ros_graph", {})
