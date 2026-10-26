# @file CMakeLists.txt
# @brief 装配 dummy_bringup 的启动配置、面板运行时与离线/运行时测试。
# 本包使用 ament_cmake 安装 Python 脚本与资源；配置/安装阶段不启动 ROS 图，不部署固件或授权设备。
cmake_minimum_required(VERSION 3.8)
project(dummy_bringup)

# ament_cmake 提供包索引、安装导出与测试集成；实际进程由用户选择 launch 后才创建。
find_package(ament_cmake REQUIRED)
find_package(DPSDKHostSDK CONFIG REQUIRED)
get_target_property(_dummy_protocol_include dp_protocol::selected INTERFACE_INCLUDE_DIRECTORIES)
get_filename_component(_dummy_protocol_release "${_dummy_protocol_include}" DIRECTORY)

# 仅 BUILD_TESTING 开启时注册测试依赖与用例；注册不是运行，colcon/CTest 后续才调度。
# pytest 主要验证配置/状态机及假服务；下方 add_launch_test 会建立独立运行图，应与纯离线检查区别。
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(manual_ros_control_launch test/test_manual_ros_control_launch.py)
  foreach(test_name IN ITEMS manual_panel_ros_backend manual_panel_ros_runtime manual_panel_ros_qt manual_panel_ros_execution manual_panel_ros_event_flow)
    ament_add_pytest_test(${test_name} test/test_${test_name}.py
      ENV QT_QPA_PLATFORM=offscreen)
  endforeach()
# 部署/安装/启动参数回归核对固定配置入口与包共享布局，防止从任意构建目录回退。
  ament_add_pytest_test(manual_panel_deployment_config test/test_manual_panel_deployment_config.py)
  ament_add_pytest_test(manual_panel_runtime_config test/test_manual_panel_runtime_config.py)
  ament_add_pytest_test(manual_panel_launch test/test_manual_panel_launch.py)
  ament_add_pytest_test(manual_panel_measured_publisher test/test_manual_panel_measured_publisher.py)
  ament_add_pytest_test(manual_panel_ros_joint_control test/test_manual_panel_ros_joint_control.py
    ENV QT_QPA_PLATFORM=offscreen)
  ament_add_pytest_test(manual_panel_install_deployment test/test_manual_panel_install_deployment.py)
# 可选相机关闭时不依赖传感器overlay；真实六轴测试用固定输入/替身检查身份预检和模型配置边界。
  ament_add_pytest_test(sensor_optional test/test_sensor_optional.py)
  ament_add_pytest_test(real_6dof_description test/test_real_6dof_description.py)
  ament_add_pytest_test(real_six_axis_preflight ../../scripts/dummy/tests/test_real_six_axis_preflight.py)
  ament_add_pytest_test(real_6dof_launch test/test_real_6dof_launch.py)
# 面板规划与执行回归覆盖完整路径、连续重采样、预规划、设备收据和只读参数事务；不是实机资格测试。
  ament_add_pytest_test(manual_moveit_planner test/test_manual_moveit_planner.py)
  ament_add_pytest_test(manual_moveit_execution test/test_manual_moveit_execution.py)
  ament_add_pytest_test(manual_moveit_continuous test/test_manual_moveit_continuous.py)
  ament_add_pytest_test(manual_moveit_autotest test/test_manual_moveit_autotest.py)
  ament_add_pytest_test(manual_ros_rolling_pipeline test/test_manual_ros_rolling_pipeline.py)
  ament_add_pytest_test(manual_moveit_tab test/test_manual_moveit_tab.py)
  ament_add_pytest_test(manual_motor_parameters test/test_manual_motor_parameters.py)
  ament_add_pytest_test(manual_moveit_target test/test_manual_moveit_target.py)
  ament_add_pytest_test(manual_scene_obstacles test/test_manual_scene_obstacles.py)
  ament_add_pytest_test(manual_moveit_service_topology test/test_manual_moveit_service_topology.py)
  ament_add_pytest_test(manual_panel_messages test/test_manual_panel_messages.py)
# 这些入口引用仓库 scripts/dummy/tests 中的应用集成回归；相对路径按本包源码目录解析。
  ament_add_pytest_test(ros_six_axis_movej ../../scripts/dummy/tests/test_ros_six_axis_movej.py)
  ament_add_pytest_test(application_e2e ../../scripts/dummy/tests/test_application_e2e.py)
  ament_add_pytest_test(activate_real_no_load_stack ../../scripts/dummy/tests/test_activate_real_no_load_stack.py)
# 运行时 launch 测试分别给 Gazebo 120s、规划执行150s、后端契约45s总预算；不会因构建本包自动启动。
  add_launch_test(test/test_gazebo_runtime.launch.py TIMEOUT 120)
  add_launch_test(test/test_plan_execute.launch.py TIMEOUT 150)
  add_launch_test(test/test_backend_contract.launch.py TIMEOUT 45)
endif()

# launch 与 config 安装到 share/dummy_bringup，供 ament 查询与 ros2 launch 定位；不会改写源配置内容。
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
# 真实开发入口动态导入的预检器和限制转换器安装为共享脚本，不在安装阶段执行；模型文件提供绑定校验输入。
install(FILES
  ../../scripts/dummy/real_six_axis_preflight.py
  ../../scripts/dummy/convert_application_ros_limits.py
  DESTINATION share/${PROJECT_NAME}/scripts)
# 业务模型与解码发布均来自配置时选定的安装SDK。
install(FILES "${DPSDKHostSDK_ROBOT_BUNDLE_DIR}/model/model.yaml"
  DESTINATION share/${PROJECT_NAME}/model)
install(FILES ../../scripts/dummy/join_real_6dof_evidence.py
  ../../scripts/dummy/selected_protocol.py
  DESTINATION share/${PROJECT_NAME}/panel_runtime/scripts/dummy)
install(DIRECTORY "${_dummy_protocol_release}/"
  DESTINATION share/${PROJECT_NAME}/panel_runtime/scripts/dummy/protocol)
# 可执行脚本安装至 lib/dummy_bringup，供 ros2 run/launch 查找；辅助模块同目录支持面板进程内导入。
# 配置/部署工具只处理明确本机产物；手动面板独占Native通信，MoveIt/场景/探针通过既定ROS或面板接口协作。
install(PROGRAMS
  scripts/manual_panel_config.py
  scripts/manual_panel_ros_backend.py
  scripts/manual_panel_runtime.py
  scripts/deploy_manual_panel.py
  scripts/render_manual_panel_config.py
  scripts/manual_joint_panel.py
  scripts/manual_panel_ui.py
  scripts/manual_panel_messages.py
  scripts/manual_panel_simulator.py
  scripts/manual_motor_parameters.py
  scripts/manual_target_presets.py
  scripts/manual_moveit_planner.py
  scripts/manual_moveit_execution.py
  scripts/manual_moveit_continuous.py
  scripts/manual_moveit_tab.py
  scripts/manual_moveit_autotest.py
  scripts/manual_moveit_target.py
  scripts/manual_scene_obstacles.py
  scripts/action_gate_probe.py
  scripts/gazebo_runtime_probe.py
  scripts/planning_probe.py
  scripts/scene_publisher.py
  DESTINATION lib/${PROJECT_NAME})

# 最后生成 ament 包导出/索引信息；运行依赖在 package.xml 声明，启动选择由各 launch 文件明确区分。
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
