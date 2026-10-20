##
# @file servo_cpp_interface_demo.launch.py
# @brief 历史Servo C++接口演示启动草稿，与当前面板/真机守卫入口分开。
# @details 它引用dummy_servo_cpp_interface_demo可执行文件，但当前CMake只构建keyboard_input。
# 控制器名仍用dummy_arm_controller，需核对实际安装产物和配置，不能据此宣称演示可直接运行。
# adapt to Dummy by Muzhxiaowen, check out the details on bilibili.com

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

##
# @brief 构建示例的mock硬件、Servo/RSP/RViz/TF和controller spawner动作树。
# @return LaunchDescription；缺失可执行文件或控制器名不匹配会在实际启动时失败，此函数不修正它们。
def generate_launch_description():
    # moveit_config = MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config").to_moveit_configs()
    ##
    # 显式选择mock URDF、SRDF和执行配置，保持示例参数来源可见；不载入DPSDK实机部署身份。
    moveit_config = (
        MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
        .robot_description(file_path="config/dummy-ros2.urdf.xacro")
        .robot_description_semantic(file_path="config/dummy-ros2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    ##
    # 将历史Servo YAML放入moveit_servo参数命名空间；速度、话题、碰撞与平滑参数详见该YAML。
    servo_params = (
        ParameterBuilder("dummy_moveit_config")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/dummy_simulated_config.yaml",
        )
        .to_dict()
    )

    # A node to publish world -> base_link transform
    ##
    # 本演示独立发布world到base_link零静态变换，不能再叠加同一TF边的另一发布者。
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # The servo cpp interface demo
    # Creates the Servo node and publishes commands to it
    ##
    # 此可执行文件名须在安装区实际存在；当前本包CMake未声明同名target，示例源码存在也不等于已编译安装。
    servo_node = Node(
        package="dummy_moveit_config",
        executable="dummy_servo_cpp_interface_demo",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("dummy_moveit_config"), "config"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit_empty.rviz")

    # rviz_config_file = (
    #     get_package_share_directory("moveit2_tutorials")
    #     + "/config/demo_rviz_config.rviz"
    # )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ##
    # 示例使用FakeSystem描述装配controller_manager；这不建立设备session/permit。
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                get_package_share_directory("dummy_moveit_config"),
                "config",
                "ros2_controllers.yaml",
            ),
        ],
        output="both",
    )

    # Load controllers
    ##
    # 旧演示用shell方式逐个启动spawner；其dummy_arm_controller名与当前通用YAML内部名不同，保留原行为并说明限制。
    load_controllers = []
    for controller in ["dummy_arm_controller", "joint_state_broadcaster"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [rviz_node, static_tf, servo_node, ros2_control_node, robot_state_publisher]
        + load_controllers
    )
