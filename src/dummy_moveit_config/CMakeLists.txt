# @file CMakeLists.txt
# @brief MoveIt配置资源、键盘演示及模型回归的构建/安装入口。
cmake_minimum_required(VERSION 3.22)
project(dummy_moveit_config)

# 寻找ROS/MoveIt开发依赖；这些find_package不表示已启动move_group、Servo或Gazebo。
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_servo REQUIRED)

# 启用时注册模型、FK、装配、保护体和历史仿真/Action配置检查；CTest通过仅支持各自覆盖的范围。
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_jazzy_simulation_contract test/test_jazzy_simulation_contract.py)
  ament_add_pytest_test(test_model_alignment test/test_model_alignment.py)
# 参考FK只核旧数学链等价，不用它冒称完整CAD或实物碰撞精度。
  ament_add_pytest_test(test_reference_chain_fk test/test_reference_chain_fk.py)
  ament_add_pytest_test(test_j6_reducer test/test_j6_reducer.py)
  ament_add_pytest_test(test_full_step_model test/test_full_step_model.py)
  ament_add_pytest_test(test_ground_origin test/test_ground_origin.py)
  ament_add_pytest_test(test_clearance_guards test/test_clearance_guards.py)
# REST几何回归检查凹支架分区包络覆盖及模型接触，不执行实体轨迹。
  ament_add_pytest_test(test_rest_collision_geometry test/test_rest_collision_geometry.py)
  ament_add_pytest_test(test_r02_gazebo_contract test/test_r02_gazebo_contract.py)
  ament_add_pytest_test(test_r05_scene_contract test/test_r05_scene_contract.py)
  ament_add_pytest_test(test_r05_action_gate_contract test/test_r05_action_gate_contract.py)
endif()

# 当前唯一显式构建的示例可执行文件是keyboard_input；servo_cpp_interface_demo.cpp虽保留源码，当前未建同名target。
add_executable(dummy_servo_keyboard_input src/servo_keyboard_input.cpp)
target_include_directories(dummy_servo_keyboard_input PUBLIC include)
ament_target_dependencies(dummy_servo_keyboard_input std_msgs control_msgs rclcpp)

# 可执行文件放入lib/包名供ros2 run查找，依赖按目标声明链接；资源安装在后面的share目录。
install(
  TARGETS
    dummy_servo_keyboard_input
  DESTINATION
    lib/${PROJECT_NAME}
)

# 生成ament配置和索引；本包后续安装规则继续声明资源目录，不在配置阶段运行机器人。
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()

# 安装launch目录；当前EXCLUDE模式为setup_assistant.launch，实际源文件名带.py，不能据该模式声称已经排除了setup_assistant.launch.py。
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
# 配置与worlds随包安装，使launch通过包索引定位资源，不依赖开发目录。
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
# 保存设置助手的生成来源与接口元数据；它不替代真实部署配置或运行状态。
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
