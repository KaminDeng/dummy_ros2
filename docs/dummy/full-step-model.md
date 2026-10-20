# 完整 STEP 模型与真机界面

## 先区分显示、碰撞与历史参考

彩色 OBJ/MTL 用于显示，保守 STL 用于碰撞，joint origin/axis 用于运动学；三者职责不同。
包内 manifest 绑定转换输入和产物，历史 `dummy_reference_chain` 只对照旧求解器，不能替代当前 CAD。
文中的105项、重启实读及距离是对应日期的证据，不表示每次打开此文都已重新验证运行进程。
几何或安装资源变化后，应同时核对实际模型/目标虚影/规划器所加载的描述，并作新规划。
当前 REST 修复、地面原点和最小间隙分别见本页后续链接；三者需一起理解。

2026-10-20，用户人工核对彩色RViz模型正常后，按明确指令将完整STEP模型
安装到描述包，并重启真机面板、MoveIt和RViz。启动为只读，未执行实机运动。

共享入口仍为 `src/dummy-ros2_description/urdf/dummy-ros2.xacro`。
显示资源为 `meshes/full_step_*-colors.obj` 及同目录MTL，碰撞资源为
`meshes/full_step_*-collision-local.stl`；所有资源随描述包安装，不依赖下载目录
或DPSDK报告目录。转换身份与安装差异保存在 `meshes/full_step_manifest.json`。

实际显示、目标虚影、MoveIt的FK/IK和路径碰撞检查使用同一描述。
源材质保留于实际模型；目标虚影仍按面板规则显示预览色。
电机额外具有固定碰撞子连杆，避免相邻连杆整体豁免漏检。
原有50:1控制映射、Native/固件身份不变。

减速器按完整STEP的末端支架安装平面和孔组中心定位，
J5局部安装点为 `[-0.0168,0.0625,0]` m，输出端再延伸28mm。
根据完整CAD校正了原简化模型的J2安装高度与腕部横向安装偏移；
历史 `dummy_reference_chain.*` 仍是原求解器对照证据，不代表当前完整CAD。

## 结果和限制

- 新模型资源、材质引用、电机碰撞归属、减速器安装及相关回归共105项通过；
  描述、MoveIt配置和面板三个包构建通过。
- 重启后实读三个模型消费者的 `robot_description`，确认使用包内完整模型；
  实读允许碰撞矩阵确认上臂对腕部电机检查开启。
- 只读调用真机域中的MoveIt服务，以已记录干涉关节姿态核验，
  检出 `link2_1_1` 对 `link5_1_1` 约0.479mm及对
  `step_link5_motor_1` 约0.713mm的模型接触，拒绝该姿态。
- 首次独立域验证中 Home 有效，原 REST 曾检出旋转底座对 J2 电机的误碰撞。
  现已定位为凹支架被整件凸包填实，并通过分区保守包络修正；
  REST 双向规划的当前验证与重建入口见 [REST 规划修复](rest-planning.md)。
  规划服务验证不等同于实机路径执行资格。
- 未执行完整STEP模型的实机运动，未授予连续20分钟测试通过结论。

可复用流程见[STEP装配转换工程说明](model-conversion/README.md)。
历史记录的原件位于DPSDK的 `build_dummy/reports/full-step-model-20261019/`，
包含 `real-runtime-model-verification.json`、`real-panel-restart.log` 和候选转换记录。

2026-10-20加入近端到腕部至少5mm的规划间隙，保留源视觉和运动学。适用范围、距离证据和重建方法见[collision-clearance.md](collision-clearance.md)。

同日将用户根坐标 `base_link` 调整为 J1 轴心 X/Y、基座底面 Z=0；
仅统一平移根部几何，完整 CAD 装配关系不变。数值依据与验证见[ground-origin.md](ground-origin.md)。
