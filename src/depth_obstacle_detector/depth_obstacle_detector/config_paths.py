"""Portable ground-reference paths for camera configuration files."""

import os


def resolve_ground_path(config_path, ground_path):
    """Resolve a ground reference relative to its YAML file, not the process cwd."""
    config_dir = os.path.dirname(os.path.abspath(config_path))
    ground_path = os.path.expanduser(os.path.expandvars(ground_path))
    return os.path.abspath(os.path.join(config_dir, ground_path))


def relative_ground_path(config_path, ground_path):
    """Store a loaded ground reference relative to the destination YAML file."""
    config_dir = os.path.dirname(os.path.abspath(config_path))
    ground_path = os.path.expanduser(os.path.expandvars(ground_path))
    return os.path.relpath(ground_path, config_dir)
