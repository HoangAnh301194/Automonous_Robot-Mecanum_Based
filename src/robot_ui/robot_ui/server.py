from __future__ import annotations

import argparse

import uvicorn

from robot_ui.app import create_app
from robot_ui.config import load_config


def parse_args():
    parser = argparse.ArgumentParser(description="Robot admin developer web server")
    parser.add_argument("--config", default=None, help="Path to robot_ui.yaml")
    parser.add_argument("--host", default=None, help="Override bind host")
    parser.add_argument("--port", type=int, default=None, help="Override bind port")
    return parser.parse_args()


def main():
    args = parse_args()
    config = load_config(args.config)
    host = args.host or config.server["host"]
    port = args.port or int(config.server["port"])
    uvicorn.run(create_app(config), host=host, port=port, log_level="info")


if __name__ == "__main__":
    main()
