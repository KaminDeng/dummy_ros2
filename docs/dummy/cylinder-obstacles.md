# 圆柱障碍物

启用MoveIt的桌面面板新增“圆柱障碍物”页，可选择一行后修改或删除，也可输入位置及尺寸添加。X、Y、直径、高度全部使用毫米。默认坐标基准`base_link`是地面上的J1轴心：Z=0为实际基座底面。圆柱底面始终贴Z=0，轴线沿地面法向向上；直径在ROS中转换为半径，几何中心位于高度的一半。

如需以已测量的安装螺纹孔为原点，在同页“已测量安装孔基准”输入名称及相对J1的XY毫米偏移，点击“新增 / 修改孔基准”即可选用。基准与障碍物一并持久保存并在重启时恢复；base_link不可修改或删除。修改孔偏移会同步移动所有引用该孔的圆柱，并走相同的作废旧规划、停车确认及Apply事务。删除孔基准则把其圆柱转换为base_link坐标以保留世界位置，也需相同事务。也可在面板配置增加`obstacle_reference_origins_mm`，每个名称映射至该原点相对`base_link`的`[X_mm, Y_mm]`以设置初始基准。例如`{"安装孔A": [40.0, -25.0]}`仅代表人为测量数据，不是现有CAD自动识别孔位。基准仅平移XY，不接受任意倾斜TF，防止圆柱倾斜或离地。

默认文件`~/.config/dummy_manual_panel/obstacles.json`，可用配置`obstacles_file`覆盖。文件包含版本、障碍物列表及曾使用的ID；先获`/apply_planning_scene`正向回执，再原子保存。首次启动即加载并同步，尚未成功时规划和运动入口保持封锁。服务拒绝、超时、文件损坏、保存失败均明确显示失败；不得把本地模型当作实际MoveIt场景。服务调用超时后不发并行重试，避免迟到旧请求覆盖新请求。

障碍物以`manual_cylinder_`前缀的真实`CollisionObject`差量提交。差量只添加/移除本功能自己的对象，不覆盖地面、安装接触例外或5mm guards。规划器的目标碰撞检查和整条执行路径检查读取该实际规划场景。RViz通过`/manual_obstacle_markers`显示已获MoveIt回执的相同圆柱，橙色显示；更新失败不提前展示未应用的新几何。删除通过差量REMOVE与标记全量刷新同步。

每次更改立即递增场景版本、作废旧规划和连续链，并封锁新请求。运动中先清除自动测试/待接曲线，发原有单owner的`CANCEL_HOLD`；必须接收到同boot/reference/session的`MOVE_CANCELED_HOLD`，且随后更新的实测状态满足同motion、HoldReady、六轴位置/速度/保持有效、实测距Hold小于0.003rad及速度小于0.01rad/s，才提交场景。确认失败时保持封锁，不自动继续旧场景、旧轨迹或自动测试。

离线验证：

```sh
source /opt/ros/jazzy/setup.bash
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m pytest -q \
  src/dummy_bringup/test/test_manual_scene_obstacles.py \
  src/dummy_bringup/test/test_manual_panel_backend_qt.py \
  src/dummy_bringup/test/test_manual_panel_connection_recovery.py
```

## 附着安全地面与圆柱的碰撞矩阵

规划请求自带的`manual_base_floor`是保持55mm禁入高度的附着安全体，不是真实机械臂连杆。贴地圆柱不可避免与它重叠，因此每次事务在Apply前先通过`/get_planning_scene`读取当前允许碰撞矩阵（ACM），完整保留已有SRDF/guard条目、默认条目，仅新增`manual_base_floor`与各`manual_cylinder_<id>`的双向明确允许对。圆柱与真实机器人连杆、末端、5mm guards的新增矩阵值均为false，不做任何机器人对圆柱豁免。已有地面与机器人间的安装接触规则不变。

Get回执未成功、ACM格式错误、服务未就绪或超时，均不会进入Apply，也不会开放规划。Get与Apply各自等待服务回执，事务期间不能再提交重叠的场景修改。

世界删除操作同时读取`WORLD_OBJECT_NAMES`：持久化的`known_ids`只代表本功能曾管理的ID，不代表物体当前仍存在。每次仅对Get实际世界中存在、且本次已删除的已知圆柱发送REMOVE，避免MoveIt因“移除不存在物体”拒绝整个事务。删除后的基准编辑及重启空场景同步不会重复REMOVE；外部对象不在本模块清理范围内。
