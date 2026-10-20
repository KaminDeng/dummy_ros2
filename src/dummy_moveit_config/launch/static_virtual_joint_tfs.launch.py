##
# @file static_virtual_joint_tfs.launch.py
# @brief 从SRDF虚拟关节生成父frame到child_link的静态TF发布节点。
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


##
# @brief 构建并返回该入口的LaunchDescription，具体节点动作由上游生成器提供。
# 当前SRDF为固定world到base_link；生成器按虚拟关节列表装配节点，不代表读取现场根位姿。
# 同一TF边应只有所选装配owner发布，不能用重复发布解决模型不一致。
# @return launch执行器后续消费的动作树；调用本函数不是节点已就绪的证明。
def generate_launch_description():
    ##
    # 解析本包robot配置，统一取得URDF/SRDF、运动学、规划流水线与控制器数据；缺依赖/配置时由builder抛错。
    moveit_config = MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config").to_moveit_configs()
    return generate_static_virtual_joint_tfs_launch(moveit_config)
