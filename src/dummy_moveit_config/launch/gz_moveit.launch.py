##
# @file gz_moveit.launch.py
# @brief 组合Gazebo Sim与MoveIt规划，使用Gazebo内的控制系统，避免再启动一份FakeSystem。
"""Launch Gazebo Sim and the MoveIt planning stack without a duplicate FakeSystem."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_static_virtual_joint_tfs_launch,
)


##
# @brief 声明gui/rviz选项并组合仿真、虚拟根TF、move_group和可选RViz。
# @return LaunchDescription；gui/rviz默认均false，此处不直接执行仿真或开动机械臂。
def generate_launch_description():
    ##
    # MoveIt仍复用本包共享几何/语义配置；Gazebo具体硬件后端在gz_sim子launch中选择。
    moveit_config = MoveItConfigsBuilder(
        "dummy-ros2", package_name="dummy_moveit_config"
    ).to_moveit_configs()
    share_dir = get_package_share_directory("dummy_moveit_config")
    ##
    # 只把gui参数交给gz_sim；rviz由本文件独立控制，不为仿真再创建ros2_control_node。
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_dir, "launch", "gz_sim.launch.py")),
        launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
    )
    ##
    # 通用生成器提供规划/执行能力，实际执行需匹配公开Action入口；本组合不额外启动Native守卫。
    move_group_launch = generate_move_group_launch(moveit_config)
    rviz_launch = generate_moveit_rviz_launch(moveit_config)
    static_tf_launch = generate_static_virtual_joint_tfs_launch(moveit_config)

    ##
    # 将生成器动作展开到同一启动树；RViz实体放进IfCondition组。动作被列出不等于模型、控制器和服务已完成就绪。
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            simulation_launch,
            *static_tf_launch.entities,
            *move_group_launch.entities,
            GroupAction(
                actions=list(rviz_launch.entities),
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
