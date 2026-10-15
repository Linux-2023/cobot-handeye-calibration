"""Regression tests for the three-camera color launch orientation."""

from __future__ import annotations

import importlib.util
from pathlib import Path


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "multi_camera_3cam_color.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("multi_camera_3cam_color", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_three_color_cameras_do_not_add_rotation_filter():
    module = _load_launch_module()
    description = module.generate_launch_description()

    includes = [entity for entity in description.entities if hasattr(entity, "launch_arguments")]
    assert len(includes) == 3
    for include in includes:
        arguments = dict(include.launch_arguments)
        assert "rotation_filter.enable" not in arguments
        assert "rotation_filter.rotation" not in arguments
        assert arguments["enable_color"] == "true"
        assert arguments["enable_depth"] == "false"
