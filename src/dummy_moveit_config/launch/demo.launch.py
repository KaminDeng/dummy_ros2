##
# @file demo.launch.py
# @brief 通用MoveIt示例组合，加载RSP、move_group、可选RViz/warehouse及ros2_control/spawner。
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


##
# @brief 构建并返回该入口的LaunchDescription，具体节点动作由上游生成器提供。
# 该通用组合使用本包配置，不是Qt面板的唯一只读规划入口，也不创建DPSDK授权。
# 当前通用控制器名的注册与MoveIt名单仍需匹配；wrapper不会自动补Native公开Action守卫。
# @return launch执行器后续消费的动作树；调用本函数不是节点已就绪的证明。
def generate_launch_description():
    ##
    # 解析本包robot配置，统一取得URDF/SRDF、运动学、规划流水线与控制器数据；缺依赖/配置时由builder抛错。
    moveit_config = MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
