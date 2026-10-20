##
# @file rsp.launch.py
# @brief 使用MoveIt配置中的robot_description启动robot_state_publisher。
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


##
# @brief 构建并返回该入口的LaunchDescription，具体节点动作由上游生成器提供。
# 上游支持publish_frequency，收到关节状态后发布连杆TF；本launch不产生六轴反馈、不发运动命令。
# @return launch执行器后续消费的动作树；调用本函数不是节点已就绪的证明。
def generate_launch_description():
    ##
    # 解析本包robot配置，统一取得URDF/SRDF、运动学、规划流水线与控制器数据；缺依赖/配置时由builder抛错。
    moveit_config = MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
