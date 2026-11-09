"""Read untracked 9router settings without sourcing or executing shell code."""
from __future__ import annotations

import os
import re
import shlex
from pathlib import Path
from typing import MutableMapping

LLM_KEYS = frozenset(("LLM_BASE_URL", "LLM_API_KEY", "LLM_MODEL"))
_ASSIGNMENT = re.compile(r"^([A-Za-z_][A-Za-z_0-9]*)\s*=\s*(.*)$")


def load_llm_env_file(
    path: str | Path,
    *,
    environ: MutableMapping[str, str] | None = None,
) -> dict[str, str]:
    """Load settings from a local .env file; file values override the environment.

    Only LLM_BASE_URL, LLM_API_KEY and LLM_MODEL are accepted.
    Supports KEY=value, export KEY=value, quotes, and # comments. Values
    containing shell commands or expansions are rejected, never executed.
    No API keys or values are logged by this module.
    """
    file_path = Path(path).expanduser()
    if not file_path.is_file():
        raise FileNotFoundError(f"9router configuration file not found: {file_path}")
    try:
        raw = file_path.read_text(encoding="utf-8-sig")
    except (OSError, UnicodeError) as exc:
        raise OSError(f"Cannot read 9router configuration file: {file_path}") from exc

    parsed: dict[str, str] = {}
    for number, original_line in enumerate(raw.splitlines(), start=1):
        line = original_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].strip()
        match = _ASSIGNMENT.fullmatch(line)
        if match is None:
            raise ValueError(f"Invalid 9router configuration syntax at line {number}")
        key, raw_value = match.groups()
        if key not in LLM_KEYS:
            # Keep this loader limited to LLM settings; never set arbitrary vars.
            continue
        try:
            values = shlex.split(raw_value, comments=True, posix=True)
        except ValueError as exc:
            raise ValueError(f"Invalid quoting at line {number} ({key})") from exc
        if len(values) > 1:
            raise ValueError(f"Multiple values at line {number} ({key})")
        value = values[0] if values else ""
        if "$" in value or chr(96) in value:
            raise ValueError(f"Shell expansion not supported at line {number} ({key})")
        parsed[key] = value

    target = os.environ if environ is None else environ
    # Update only after all lines are validated, to avoid partial configuration.
    target.update(parsed)
    return parsed
