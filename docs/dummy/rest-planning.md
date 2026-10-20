# REST 双向规划与旋转底座包络修复

当前工作 REST 来自 `manual_joint_panel.json`：六轴为
`[0,-60,163.97260273972603,0,0,0]` 度。
完整 STEP 导入后的旋转底座将单个凹形支架 `J1/Dummy-0001` 替换为整件凸包，
支架侧板与底部结构之间的让位空间被填实。真实 MoveIt 的有效性服务因此报告
`link1_1_1` 对 `step_link2_motor_2` 接触约 7.93mm，REST 作为起点或终点均无法规划。
去除地面保护后仍复现；原始 CAD 三角表面在同姿态无接触，排除了地面或关节范围原因。

修复在支架局部 Z=24mm 处裁剪全部三角形（含跨面交点），分别建立下部和侧板的
保守凸外包络。每一侧相对于裁剪后源几何保留至少 0.3mm 外扩，12 个紧固件
沿用原碰撞包络。没有放宽 SRDF 接触对、关节界限或地面保护，视觉和关节变换不变。
近端对腕部的独立 5mm 保护体随后从新包络重建，序列化后最小外扩约 5.009998mm。
原始支架、紧固件输入与输出 SHA 记录在 `meshes/rest-collision-refinement.json`，
源 STEP SHA 与完整模型清单相同。

重建只操作描述包资源，不连接设备。需要 numpy、scipy、trimesh、shapely：

```bash
python3 scripts/dummy/refine_j1_collision.py \
  --meshes src/dummy-ros2_description/meshes
```

几何测试的依赖与实际验证版本见 `src/dummy_moveit_config/test/requirements-geometry.txt`。
trimesh/python-fcl 使用该 pip 清单安装，不把它们伪报为系统 apt/rosdep 依赖。
例如先创建 `python3 -m venv --system-site-packages /tmp/dummy-rest-tests`，再用该环境的
`python -m pip install -r src/dummy_moveit_config/test/requirements-geometry.txt` 安装；
source ROS 环境后使用同一虚拟环境的 Python 运行下述测试。

几何回归在旧模型下已实际失败；修复后检查 REST 到 Home 的 122 个 J2 采样、
全部裁剪后源三角形的包络包含关系及紧固件覆盖。可与其余模型回归一起运行：

```bash
python3 -m pytest src/dummy_moveit_config/test -q
```

运行真实服务回归时 source 当前安装区，使用只有一个 MoveIt 服务端的 ROS 域。
该脚本只调用规划/状态服务，不创建轨迹执行或电机控制连接：

```bash
ROS_DOMAIN_ID=187 ros2 launch dummy_bringup manual_moveit_planning.launch.py backend:=real
# 另一终端，source 相同环境：
ROS_DOMAIN_ID=187 python3 scripts/dummy/verify_rest_planning.py --output /tmp/rest-planning.json
```

验证要求 REST/Home 均有效，双向规划成功，规划路点和插值采样全程有效，
并保留已记录干涉目标的拒绝。规划服务不会执行物理运动。
模型资源更新后，已有 MoveIt 的内存 BVH 不会自动刷新，必须重启规划节点；
确认其 `robot_description` 与当前包相同，并重新运行服务回归。

实际验证摘要保存在 [rest-planning-evidence.json](rest-planning-evidence.json)，
包含两域服务端实际 URDF/SRDF 规范摘要、安装资源 SHA、双向路径数量和负例接触对。
网格 SHA 对应服务端 URI 指向的当前安装文件；服务进程内缓存是否生效还由完整行为回归核验，不能只以磁盘摘要代替。
