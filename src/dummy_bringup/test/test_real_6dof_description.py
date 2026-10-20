"""离线验证真实六轴几何与受限 ros2_control 描述。"""
##
# @file test_real_6dof_description.py
# @brief 真实后端URDF配置的离线xacro/安装布局/边界映射回归。
#
# 只展开XML并调用限制生成/转换函数写临时YAML，不启动hardware plugin或ROS控制器。PARAMETERS的serial/device/hash全为输入fixture，不代表当前设备匹配或已授权；设备绝对范围与BENCH参考相对±0.6rad分开检查。

import os
import importlib.util
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml


ROOT = Path(__file__).resolve().parents[3]
DESCRIPTION = ROOT / "src/dummy_bringup/config/dummy_ref_real_6dof.urdf.xacro"
SOURCE = ROOT / "src/dummy-ros2_description/urdf"
PARAMETERS = {
    "endpoint": "serial//dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_DEMO00000001-if00#baudrate=115200",
    "device_id": "01" + "00" * 15, "client_instance_id": "02" + "00" * 15,
    "profile_id": "201", "configured_mask": "63",
    "contract_sha256": "1" * 64, "model_sha256": "2" * 64,
    "profile_sha256": "3" * 64, "firmware_sha256": "4" * 64,
    "compiled_policy_sha256": "5" * 64,
    "position_min_rad": "-2.9670597283903604,-1.2740903539558606,0.6108652381980153,-3.141592653589793,-2.0943951023931953,-12.566370614359172",
    "position_max_rad": "2.9670597283903604,1.5707963267948966,3.141592653589793,3.141592653589793,2.0943951023931953,12.566370614359172",
    "velocity_max_rad_s": ",".join(["0.15079644737231007"] * 6),
    "acceleration_max_rad_s": ",".join(["1.0"] * 6),
    "control_period_us": "20000", "state_publish_period_us": "20000",
    "max_state_age_us": "250000",
    "heartbeat_stop_budget_us": "250000",
    "stop_confirmation_timeout_us": "0",
}


##
# @brief 自动fixture创建单独ament前缀，描述urdf目录symlink指向源码，并临时扩展AMENT_PREFIX_PATH。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @return None；fixture仅装配临时ament索引和进程环境。
@pytest.fixture(autouse=True)
def description_package(tmp_path, monkeypatch):
    prefix = tmp_path / "prefix"
    index = prefix / "share/ament_index/resource_index/packages"
    index.mkdir(parents=True)
    (index / "dummy-ros2_description").write_text("")
    package = prefix / "share/dummy-ros2_description/urdf"
    package.parent.mkdir(parents=True)
    package.symlink_to(SOURCE, target_is_directory=True)
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix) + os.pathsep +
                       os.environ.get("AMENT_PREFIX_PATH", ""))


##
# @brief 将指定参数映射交xacro展开并解析为ElementTree根；不加载hardware插件。
# @param values xacro参数映射；默认PARAMETERS含输入身份/设备范围/时序。
# @param path 待展开xacro路径，默认真实六轴描述源码。
# @return 展开XML根Element；展开失败异常向外传播。
def expand(values=PARAMETERS, path=DESCRIPTION):
    return ET.fromstring(xacro.process_file(str(path), mappings=dict(values)).toxml())


##
# @brief 复制bringup xacro到另一安装前缀，依靠ament查找描述，仍展开七个visual mesh。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_installed_bringup_finds_description_in_separate_ament_prefix(tmp_path):
    installed = tmp_path / "bringup_prefix/share/dummy_bringup/config/dummy_ref_real_6dof.urdf.xacro"
    installed.parent.mkdir(parents=True)
    shutil.copyfile(DESCRIPTION, installed)
    root = expand(path=installed)
    assert len(root.findall("link/visual/geometry/mesh")) == 7


##
# @brief 核七link/mesh、六关节及SRDF同名；默认绝对J3范围35..180°，不是零中心模型。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_real_description_keeps_mesh_geometry_and_absolute_joint3():
    root = expand()
    srdf = ET.parse(ROOT / "src/dummy_moveit_config/config/dummy-ros2.srdf").getroot()
    assert root.attrib["name"] == srdf.attrib["name"]
    assert len(root.findall("link")) == 7
    assert len(root.findall("link/visual/geometry/mesh")) == 7
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    assert list(joints) == [f"Joint{i}" for i in range(1, 7)]
    joint3 = joints["Joint3"].find("limit")
    assert float(joint3.attrib["lower"]) == pytest.approx(0.6108652381980153)
    assert float(joint3.attrib["upper"]) == pytest.approx(3.141592653589793)


##
# @brief 显式覆盖J3规划边界±0.6rad，允许BENCH参考相对坐标，与默认机械绝对限位区分。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_no_load_description_uses_reference_relative_joint3():
    values = {**PARAMETERS, "Joint3_lower": "-0.6", "Joint3_upper": "0.6"}
    joint3 = {joint.attrib["name"]: joint for joint in expand(values).findall("joint")}["Joint3"]
    assert float(joint3.find("limit").attrib["lower"]) == -0.6
    assert float(joint3.find("limit").attrib["upper"]) == 0.6


##
# @brief 调用real_6dof.planning_mappings，把六轴规划边界都映射到URDF±0.6rad。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_no_load_description_uses_all_six_planning_bounds():
    launch_path = ROOT / "src/dummy_bringup/launch/real_6dof.launch.py"
    spec = importlib.util.spec_from_file_location("real_6dof_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    joints = [{"name": f"Joint{index}", "planning_position_min_rad": -0.6,
               "planning_position_max_rad": 0.6} for index in range(1, 7)]
    root = expand({**PARAMETERS, **module.planning_mappings(joints)})
    for joint in root.findall("joint"):
        bounds = joint.find("limit")
        assert float(bounds.attrib["lower"]) == -0.6
        assert float(bounds.attrib["upper"]) == 0.6


##
# @brief 从当前默认deny候选生成/转换限制，核MoveIt与URDF J3同±0.6rad且含零；不激活候选或授实机许可。
# @param tmp_path pytest临时目录Path；所有本用例产物限制在该目录。
# @param monkeypatch pytest替换/环境fixture，作用域结束自动恢复。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_frozen_candidate_generates_matching_moveit_and_urdf_bench_bounds(
        tmp_path, monkeypatch):
    dpsdk = ROOT.parent / "dpsdk"
    # 使用当前默认deny源码候选，历史7d89/model50产物不作为新model1测试输入。
    candidate = ROOT / "contracts/dummy_robot_v2_application_hold_no_load_candidate/manifest.json"
    monkeypatch.syspath_prepend(str(dpsdk))
    from tools.dummy.generate_application_ros_limits import generate

    converter_path = ROOT / "scripts/dummy/convert_application_ros_limits.py"
    spec = importlib.util.spec_from_file_location("convert_application_ros_limits", converter_path)
    converter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(converter)
    limits = generate(candidate)
    limits_path = tmp_path / "limits.json"
    limits_path.write_text(json.dumps(limits), encoding="utf-8")
    output = tmp_path / "converted"
    converter.convert(limits_path, candidate, output)
    moveit = yaml.safe_load((output / "joint_limits.yaml").read_text())
    assert moveit["joint_limits"]["Joint3"]["min_position"] == -0.6
    assert moveit["joint_limits"]["Joint3"]["max_position"] == 0.6

    launch_path = ROOT / "src/dummy_bringup/launch/real_6dof.launch.py"
    launch_spec = importlib.util.spec_from_file_location("real_6dof_launch", launch_path)
    launch = importlib.util.module_from_spec(launch_spec)
    launch_spec.loader.exec_module(launch)
    joints = {joint.attrib["name"]: joint for joint in
              expand({**PARAMETERS, **launch.planning_mappings(limits["joints"])}).findall("joint")}
    joint3 = joints["Joint3"].find("limit")
    assert float(joint3.attrib["lower"]) <= 0 <= float(joint3.attrib["upper"])
    assert float(joint3.attrib["lower"]) == moveit["joint_limits"]["Joint3"]["min_position"]
    assert float(joint3.attrib["upper"]) == moveit["joint_limits"]["Joint3"]["max_position"]


##
# @brief 核GuardedZenbeddedSystem插件名、参数逐字相等及六轴position命令/position+velocity反馈接口；不实例化插件。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_hardware_identity_limits_and_interfaces_are_explicit():
    root = expand()
    assert root.find("ros2_control").attrib["name"] == "dummy_ref_real_6dof"
    hardware = root.find("ros2_control/hardware")
    assert hardware.findtext("plugin") == "dummy_ros_control/GuardedZenbeddedSystem"
    params = {node.attrib["name"]: node.text for node in hardware.findall("param")}
    assert params == PARAMETERS
    joints = root.findall("ros2_control/joint")
    assert [joint.attrib["name"] for joint in joints] == [f"Joint{i}" for i in range(1, 7)]
    for joint in joints:
        assert [node.attrib["name"] for node in joint.findall("command_interface")] == ["position"]
        assert [node.attrib["name"] for node in joint.findall("state_interface")] == ["position", "velocity"]


##
# @brief 以设备限位/activation夹具构造hardware mappings，核freshness及heartbeat均250000µs；只做字符串/展开验证。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_launch_mappings_expand_into_hardware_parameters():
    launch_path = ROOT / "src/dummy_bringup/launch/real_6dof.launch.py"
    spec = importlib.util.spec_from_file_location("real_6dof_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    fields = {key: [float(value) for value in PARAMETERS[key].split(",")]
              for key in ("position_min_rad", "position_max_rad", "velocity_max_rad_s",
                          "acceleration_max_rad_s")}
    joints = [{"name": f"Joint{index+1}",
               "device_position_min_rad": fields["position_min_rad"][index],
               "device_position_max_rad": fields["position_max_rad"][index],
               "device_velocity_max_rad_s": fields["velocity_max_rad_s"][index],
               "device_acceleration_max_rad_s2": fields["acceleration_max_rad_s"][index]}
              for index in range(6)]
    activation = {"source_contract_sha256": PARAMETERS["contract_sha256"],
                  "model_sha256": PARAMETERS["model_sha256"],
                  "live_profile_sha256": PARAMETERS["profile_sha256"],
                  "firmware_sha256": PARAMETERS["firmware_sha256"],
                  "compiled_policy_sha256": PARAMETERS["compiled_policy_sha256"]}
    mappings = module.hardware_mappings(
        joints, activation, PARAMETERS["endpoint"], PARAMETERS["device_id"],
        PARAMETERS["client_instance_id"], 20000, 20000)
    hardware = expand(mappings).find("ros2_control/hardware")
    values = {node.attrib["name"]: node.text for node in hardware.findall("param")}
    assert values["max_state_age_us"] == values["heartbeat_stop_budget_us"] == "250000"


##
# @brief 默认无raw_evidence_path，显式带空格/引号路径原样展开且endpoint不重复；不创建raw文件。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_raw_capture_mapping_is_optional_hardware_parameter():
    default = expand().find("ros2_control/hardware")
    assert default.find("param[@name='raw_evidence_path']") is None
    path = "/report/operator's transport.raw"
    enabled = expand({**PARAMETERS, "raw_evidence_path": path})
    hardware = enabled.find("ros2_control/hardware")
    assert hardware.findtext("param[@name='raw_evidence_path']") == path
    assert len(hardware.findall("param[@name='endpoint']")) == 1


##
# @brief firmware SHA为空必须在xacro展开时报错，拒绝缺身份声明。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_missing_parameter_fails_expansion():
    values = {**PARAMETERS, "firmware_sha256": ""}
    with pytest.raises(Exception):
        expand(values)


##
# @brief 64字符摘要前导0必须按字符串保留，不能被YAML/表达式当数值丢失。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_leading_zero_policy_hash_is_preserved():
    values = {**PARAMETERS, "compiled_policy_sha256": "0" + "1" * 63}
    root = expand(values)
    params = {node.attrib["name"]: node.text
              for node in root.findall("ros2_control/hardware/param")}
    assert params["compiled_policy_sha256"] == values["compiled_policy_sha256"]


##
# @brief 独立stop_confirmation_timeout设300000µs时，State/heartbeat250000µs不变，避免混淆停止确认与新鲜度。
# @return 无；通过assert/unittest断言表达结果，失败抛异常，不返回设备执行凭据。
def test_independent_stop_budget_expands_without_changing_freshness():
    mappings = dict(PARAMETERS, stop_confirmation_timeout_us="300000")
    root = xacro.process_file(str(DESCRIPTION), mappings=mappings).toxml()
    hardware = ET.fromstring(root).find("ros2_control/hardware")
    params = {node.attrib["name"]: node.text for node in hardware.findall("param")}
    assert params["stop_confirmation_timeout_us"] == "300000"
    assert params["max_state_age_us"] == params["heartbeat_stop_budget_us"] == "250000"
