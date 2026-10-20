##
# @file move_group.launch.py
# @brief 按MoveIt通用生成器启动move_group，配置来自本包与描述包。
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


##
# @brief 构建并返回该入口的LaunchDescription，具体节点动作由上游生成器提供。
# 上游生成器默认允许轨迹执行，调用方可用allow_trajectory_execution覆盖；这与面板专用只读规划launch不同。
# 本wrapper不启动机器人硬件或创建会话/permit，实际执行还依赖匹配Action入口。
# @return launch执行器后续消费的动作树；调用本函数不是节点已就绪的证明。
def generate_launch_description():
    ##
    # 解析本包robot配置，统一取得URDF/SRDF、运动学、规划流水线与控制器数据；缺依赖/配置时由builder抛错。
    moveit_config = MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config").to_moveit_configs()
    return generate_move_group_launch(moveit_config)
