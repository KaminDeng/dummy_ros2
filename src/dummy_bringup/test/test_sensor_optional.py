import importlib.util
from pathlib import Path

import pytest


LAUNCH = Path(__file__).parents[1] / "launch" / "demo_sensor_input.launch.py"


def load_launch_module():
    specification = importlib.util.spec_from_file_location("demo_sensor_input", LAUNCH)
    module = importlib.util.module_from_spec(specification)
    assert specification.loader is not None
    specification.loader.exec_module(module)
    return module


def test_camera_is_disabled_by_default_and_has_no_control_surface() -> None:
    module = load_launch_module()
    description = module.generate_launch_description()
    assert module._enabled("false") is False
    assert len(description.entities) == 7
    text = LAUNCH.read_text().lower()
    for forbidden in ("joint_trajectory", "/enable", "follow_joint_trajectory",
                      "dummy_motion_client", "dpsdk"):
        assert forbidden not in text


def test_camera_boolean_rejects_ambiguous_values() -> None:
    module = load_launch_module()
    with pytest.raises(ValueError, match="true or false"):
        module._enabled("1")


def test_replay_and_runtime_identity_arguments_are_declared() -> None:
    module = load_launch_module()
    names = {
        entity.name for entity in module.generate_launch_description().entities
        if hasattr(entity, "name")
    }
    assert {
        "camera_source", "allow_camera_hardware", "camera_serial_number",
        "camera_bag_path", "camera_capture_manifest",
    }.issubset(names)
