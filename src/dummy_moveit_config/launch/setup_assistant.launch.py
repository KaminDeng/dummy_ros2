##
# @file setup_assistant.launch.py
# @brief 启动MoveIt Setup Assistant并指定当前配置包路径。
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


##
# @brief 构建并返回该入口的LaunchDescription，具体节点动作由上游生成器提供。
# 该GUI用于查看/编辑配置，保存操作由使用者在GUI中触发；它不是部署、标定或实机控制入口。
# @return launch执行器后续消费的动作树；调用本函数不是节点已就绪的证明。
def generate_launch_description():
    ##
    # 解析本包robot配置，统一取得URDF/SRDF、运动学、规划流水线与控制器数据；缺依赖/配置时由builder抛错。
    moveit_config = MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
