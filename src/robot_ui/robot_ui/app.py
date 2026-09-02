from __future__ import annotations

from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

from robot_ui.api.health import router as health_router
from robot_ui.api.ros_graph import router as ros_graph_router
from robot_ui.api.websocket import router as websocket_router
from robot_ui.config import AppConfig, default_web_dir
from robot_ui.runtime import Runtime


def create_app(config: AppConfig) -> FastAPI:
    runtime = Runtime(config)

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        runtime.start()
        yield
        runtime.stop()

    app = FastAPI(
        title="Robot Admin Developer API",
        version="0.1.0",
        lifespan=lifespan,
    )
    app.state.runtime = runtime

    origins = config.server.get("cors_origins", [])
    if origins:
        app.add_middleware(
            CORSMiddleware,
            allow_origins=origins,
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

    app.include_router(health_router)
    app.include_router(ros_graph_router)
    app.include_router(websocket_router)

    web_dir = default_web_dir()
    if (web_dir / "index.html").exists():
        app.mount("/", StaticFiles(directory=web_dir, html=True), name="frontend")
    else:

        @app.get("/", include_in_schema=False)
        def frontend_missing():
            return JSONResponse(
                {
                    "service": "robot_ui",
                    "message": "Frontend ch?a build. Ch?y npm install && npm run build trong frontend/.",
                    "api_docs": "/docs",
                    "health": "/api/v1/health",
                }
            )

    return app
