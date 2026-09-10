"""Check portable camera reference paths without starting ROS or a camera."""

from pathlib import Path
import shutil

import pytest
import yaml

from depth_obstacle_detector.config_paths import relative_ground_path, resolve_ground_path


@pytest.mark.parametrize('reference', ['cam_nen.npy', 'my_map/cam_nen.npy'])
def test_relative_path_uses_config_directory(tmp_path, monkeypatch, reference):
    """Ignore same-named files in the process working directory."""
    config_path = tmp_path / 'workspace with spaces' / 'camera.yaml'
    monkeypatch.chdir(tmp_path)
    assert resolve_ground_path(str(config_path), reference) == str(
        config_path.parent / reference
    )


def test_absolute_reference_is_preserved(tmp_path):
    """Keep existing absolute ground references usable."""
    reference = tmp_path / 'external' / 'ground.npy'
    assert resolve_ground_path('camera.yaml', str(reference)) == str(reference)


def test_environment_reference(tmp_path, monkeypatch):
    """Expand a configurable data directory before resolving the reference."""
    monkeypatch.setenv('GROUND_DATA', str(tmp_path))
    assert resolve_ground_path('camera.yaml', '$GROUND_DATA/ground.npy') == str(
        tmp_path / 'ground.npy'
    )


def test_saved_reference_survives_workspace_move(tmp_path, monkeypatch):
    """Keep references valid after copying a workspace to another directory."""
    original = tmp_path / 'original workspace'
    ground = original / 'my_map' / 'ground.npy'
    ground.parent.mkdir(parents=True)
    ground.write_bytes(b'ground reference')
    config_path = original / 'config' / 'camera.yaml'
    config_path.parent.mkdir()
    reference = relative_ground_path(str(config_path), str(ground))
    assert reference == '../my_map/ground.npy'
    config_path.write_text(yaml.safe_dump({'ground_file_path': reference}))
    relocated = tmp_path / 'new disk' / 'robot'
    shutil.copytree(original, relocated)
    monkeypatch.chdir(tmp_path)
    moved_config = relocated / 'config' / 'camera.yaml'
    loaded = yaml.safe_load(moved_config.read_text())
    resolved = Path(resolve_ground_path(str(moved_config), loaded['ground_file_path']))
    assert resolved == relocated / 'my_map' / 'ground.npy'
    assert resolved.read_bytes() == b'ground reference'


@pytest.mark.parametrize('config_name', ['my_map/cam.yaml', 'onll.yaml'])
def test_repository_camera_configs(config_name):
    """Both supplied camera configurations refer to the bundled ground file."""
    workspace = Path(__file__).resolve().parents[3]
    config_path = workspace / config_name
    config = yaml.safe_load(config_path.read_text())
    reference = config['ground_file_path']
    assert not Path(reference).is_absolute()
    resolved = Path(resolve_ground_path(str(config_path), reference))
    assert resolved == workspace / 'my_map' / 'cam_nen.npy'
    assert resolved.is_file()
