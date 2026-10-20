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

## 重复MoveIt会产生两个不同的碰撞世界

同一`ROS_DOMAIN_ID`中若同时启动两套MoveIt，RViz圆柱和一次Apply成功不能证明随后规划
请求用了相同场景。实际复现中六类服务各有2个实例，同名Get请求分别返回两个圆柱和空世界。
原先已通过检查的一条路径，在唯一正确世界重新检查时，腕部与圆柱接触约3.65mm，规划被拒绝。

面板现在要求Apply/Get/Plan/Validity/IK/FK每类服务只有一个实例；公共服务必须属于同一节点，
Get允许Jazzy同节点组的`<move_group>_private_<数字>`场景监视器。数量检查在节点名去重之前，
不会把两个完全同名的进程当成一个。发现尚未完整时在启动预算内等待；重复、来源不符或查询失败
保持封锁。每次场景tick、规划请求和执行确认前后都复核；失效立即作废旧路径/未来链，运动中请求
受控停车。恢复单实例后须重新同步场景并重新规划，旧路径不能自动恢复。

查看与关闭旧实例（按实际域设置环境）：

```zsh
source /opt/ros/jazzy/setup.zsh
export ROS_DOMAIN_ID=83
ros2 node list --no-daemon
ps -eo pid,ppid,args | rg 'move_group|ros2 launch.*manual'
ros2 service info /plan_kinematic_path
ros2 service info /get_planning_scene
```

优先在旧实例的启动终端按Ctrl+C；找不到原终端时，先用`ps -p <PID> -o pid,ppid,args`
复核对应`ros2 launch`进程，再执行`kill -INT <PID>`。保留当前使用的面板，不使用killall/pkill
批量结束所有ROS进程。六类服务的`Services count`都应为1；`ros2 daemon stop`只停发现缓存，
不会停止MoveIt。清理后重新打开/同步圆柱页并重新规划。

本门检查当前ROS发现图，不是跨进程的加密身份或对所有外部场景修改的持续认证。
相同节点组重启、场景服务图尚未收敛时应重新同步。MoveIt退出也不等于Native已停车或机械臂
已失能，真实后端运动应先使用面板受控停止。HTML用户指导同步见SDK仓库
`applications/dummy_ref_control/docs/six-axis-stability-debugging-review.html`的09A节。

离线Qt后端测试中的Apply成功回执必须配套唯一发现图替身，不能依赖环境恰好启动MoveIt。
补齐三项图查询后，原模拟运动/STOP/后端切换断言通过；这只验证软件界面与发现门协作，
不启动Native或MoveIt、不进行机械臂运动。该项已执行失败→修复后通过（1项），独立于前述179项回归。
