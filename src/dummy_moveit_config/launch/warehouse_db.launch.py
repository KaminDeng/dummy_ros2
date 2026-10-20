##
# @file warehouse_db.launch.py
# @brief 生成MoveIt warehouse MongoDB节点及可选初始化动作。
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch


##
# @brief 构建并返回该入口的LaunchDescription，具体节点动作由上游生成器提供。
# 上游提供数据库路径/host/port与reset参数，reset默认false；仅在实际执行launch时才启动数据库进程。
# 数据库保存规划场景/约束等资料，不保存设备控制会话或建立运动许可。
# @return launch执行器后续消费的动作树；调用本函数不是节点已就绪的证明。
def generate_launch_description():
    ##
    # 解析本包robot配置，统一取得URDF/SRDF、运动学、规划流水线与控制器数据；缺依赖/配置时由builder抛错。
    moveit_config = MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config").to_moveit_configs()
    return generate_warehouse_db_launch(moveit_config)
