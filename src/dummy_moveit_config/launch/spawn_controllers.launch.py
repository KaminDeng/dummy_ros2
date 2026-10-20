##
# @file spawn_controllers.launch.py
# @brief 按MoveIt执行配置的controller_names加joint_state_broadcaster启动spawner。
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


##
# @brief 构建并返回该入口的LaunchDescription，具体节点动作由上游生成器提供。
# 名单来自moveit_controllers.yaml，不是扫描controller_manager中全部类型。
# 本文件不会创建公开Native Action守卫，也不修正arm_controller与内部JTC注册名之间的差异。
# @return launch执行器后续消费的动作树；调用本函数不是节点已就绪的证明。
def generate_launch_description():
    ##
    # 解析本包robot配置，统一取得URDF/SRDF、运动学、规划流水线与控制器数据；缺依赖/配置时由builder抛错。
    moveit_config = MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
