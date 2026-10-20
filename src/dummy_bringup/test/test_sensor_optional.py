import importlib.util
##
# @file test_sensor_optional.py
# @brief 可选相机launch的默认关闭与声明面静态回归。
#
# 仅动态导入launch、检查entities/布尔解析和源码禁用词，不执行LaunchService或打开相机/回放。缺禁用词只是回归约束，不能推导整个依赖图绝不提供控制能力。
from pathlib import Path

import pytest


LAUNCH = Path(__file__).parents[1] / "launch" / "demo_sensor_input.launch.py"


##
# @brief 按源码路径加载传感器launch模块，先断言loader存在，不执行launch动作。
# @return 已导入的demo_sensor_input模块。
def load_launch_module():
    specification = importlib.util.spec_from_file_location("demo_sensor_input", LAUNCH)
    module = importlib.util.module_from_spec(specification)
    assert specification.loader is not None
    specification.loader.exec_module(module)
    return module


##
# @brief 核false解析、七实体及源码不含五类控制词；不验证真实运行图。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_camera_is_disabled_by_default_and_has_no_control_surface() -> None:
    module = load_launch_module()
    description = module.generate_launch_description()
    assert module._enabled("false") is False
    assert len(description.entities) == 7
    text = LAUNCH.read_text().lower()
    for forbidden in ("joint_trajectory", "/enable", "follow_joint_trajectory",
                      "dummy_motion_client", "dpsdk"):
        assert forbidden not in text


##
# @brief 字符串1不能隐式代表true，必须报true or false的ValueError。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_camera_boolean_rejects_ambiguous_values() -> None:
    module = load_launch_module()
    with pytest.raises(ValueError, match="true or false"):
        module._enabled("1")


##
# @brief 仅核source/hardware许可/serial/bag/manifest五参数已声明，不启动硬件或验证manifest内容。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
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
