"""Start a locally installed 9router server when Kiosk needs it.

No package installation, shell evaluation or remote-host management is attempted.
"""
from __future__ import annotations

import os
import shlex
import shutil
import signal
import socket
import subprocess
import time
from pathlib import Path
from urllib.parse import urlsplit


def _port_open(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.3):
            return True
    except OSError:
        return False


def _local_endpoint(base_url: str) -> tuple[str, int] | None:
    try:
        parsed = urlsplit(base_url.strip())
        if parsed.scheme not in {"http", "https"}:
            return None
        if parsed.hostname not in {"localhost", "127.0.0.1", "::1"}:
            return None
        return parsed.hostname, parsed.port or (443 if parsed.scheme == "https" else 80)
    except ValueError:
        return None


def start_local_9router(
    base_url: str, *, enabled: bool = True, command: str = "", wait_seconds: float = 15.0
) -> subprocess.Popen | None:
    """Return an owned CLI process or None for existing/Docker/remote servers.

    Docker containers deliberately remain running on Kiosk exit. Only a CLI
    process started by this Kiosk is terminated by stop_local_9router().
    """
    if not enabled:
        print("[robot_ui] 9router autostart disabled.", flush=True)
        return None

    endpoint = _local_endpoint(base_url)
    if endpoint is None:
        print("[robot_ui] 9router autostart skipped: non-local/invalid LLM_BASE_URL.", flush=True)
        return None
    host, port = endpoint
    if _port_open(host, port):
        print(f"[robot_ui] 9router endpoint already listening on {host}:{port}; reusing.", flush=True)
        return None

    process = None
    docker_started = False
    if command.strip():
        try:
            argv = shlex.split(command)
        except ValueError as exc:
            print(f"[robot_ui] Warning: invalid 9router command: {exc}", flush=True)
            return None
        if not argv:
            print("[robot_ui] Warning: 9router command is empty.", flush=True)
            return None
    else:
        exe = shutil.which("9router")
        argv = [exe, "--no-browser", "--port", str(port)] if exe else []
        if not argv:
            docker = shutil.which("docker")
            if docker:
                try:
                    inspect = subprocess.run(
                        [docker, "container", "inspect", "9router", "--format",
                         "{{.State.Running}}"],
                        capture_output=True, text=True, timeout=4, check=False,
                    )
                    if inspect.returncode == 0:
                        if inspect.stdout.strip() == "false":
                            subprocess.run(
                                [docker, "start", "9router"],
                                capture_output=True, text=True, timeout=10, check=True,
                            )
                            docker_started = True
                            print("[robot_ui] Started existing Docker container '9router'.", flush=True)
                        else:
                            print("[robot_ui] Docker container '9router' is already running.", flush=True)
                    else:
                        print("[robot_ui] Warning: no installed 9router CLI or existing Docker container.", flush=True)
                        return None
                except (OSError, subprocess.SubprocessError) as exc:
                    print(f"[robot_ui] Warning: cannot start Docker 9router: {exc}", flush=True)
                    return None
            else:
                print(
                    "[robot_ui] Warning: 9router CLI/Docker not found. "
                    "Install 9router locally or set 9router_command for Kiosk.",
                    flush=True,
                )
                return None

    if argv:
        try:
            log_file = Path.home() / ".ros" / "9router-kiosk.log"
            log_file.parent.mkdir(parents=True, exist_ok=True)
            with log_file.open("a", encoding="utf-8") as output:
                os.chmod(log_file, 0o600)
                process = subprocess.Popen(
                    argv, stdin=subprocess.DEVNULL, stdout=output,
                    stderr=subprocess.STDOUT, start_new_session=True,
                )
            print(f"[robot_ui] Starting local 9router; log: {log_file}", flush=True)
        except (OSError, subprocess.SubprocessError) as exc:
            print(f"[robot_ui] Warning: cannot launch 9router: {exc}", flush=True)
            return None

    deadline = time.monotonic() + max(0.0, wait_seconds)
    while time.monotonic() < deadline:
        if _port_open(host, port):
            print(f"[robot_ui] 9router endpoint ready on {host}:{port}.", flush=True)
            return process
        if process is not None and process.poll() is not None:
            print(
                f"[robot_ui] Warning: 9router exited with code {process.returncode}.",
                flush=True,
            )
            return None
        time.sleep(0.25)

    if _port_open(host, port):
        print(f"[robot_ui] 9router endpoint ready on {host}:{port}.", flush=True)
    else:
        print("[robot_ui] Warning: 9router not ready; Kiosk will open anyway.", flush=True)
    return process


def stop_local_9router(process: subprocess.Popen | None) -> None:
    """Stop only the local CLI process this Kiosk launched."""
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=3)
    except ProcessLookupError:
        pass
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    except OSError:
        process.terminate()
