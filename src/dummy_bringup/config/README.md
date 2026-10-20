# 手动面板配置

2026-10-20起共享描述已更新为完整STEP彩色模型及逐零件碰撞包络。
减速器采用完整模型的支架安装面定位；当前版本见
[完整STEP模型](../../../docs/dummy/full-step-model.md)，REST双向规划修复见
[REST规划说明](../../../docs/dummy/rest-planning.md)，转换经验见
[STEP转换技能](../../../.agents/skills/step-to-ros-model/SKILL.md)。

J6 的 MINIF08 减速器已补入共享描述模型，原输出法兰向外延伸28mm，
控制减速比保持50:1；显示、FK/IK和碰撞检查使用同一模型。
更新安装包后需重启面板及其MoveIt/RViz节点才能生效，详见
`docs/dummy/j6-minif08-model.md`。

`manual_joint_panel.json` 是源码模板，随ROS包安装。模拟器直接使用模板，不需要SDK、Native或硬件身份。

真机默认使用 `$XDG_DATA_HOME/dummy_ros/manual_panel/current/panel-config.json`；未设置XDG_DATA_HOME时根目录是 `~/.local/share`。`ros2 run dummy_bringup deploy_manual_panel.py` 明确接收Native、固件、同次构建的 `generated/deployment-identity.json`，复制到独立release并核验文件，再原子切换current。部署步骤不烧录硬件，不自动挑最新build；重新编译不会静默改变已部署版本。启动核验副本的文件哈希，设备身份继续由Native连接协议核对。

首次安装或更新：工作区根目录执行 `bash scripts/build_manual_panel.sh`，只依赖系统Jazzy并完整安装描述、MoveIt配置与面板三个包。新终端：

```zsh
source /opt/ros/jazzy/setup.zsh
source install_panel_guide/local_setup.zsh
export ROS_DOMAIN_ID=83
ros2 launch dummy_bringup manual_joint_panel.launch.py backend:=simulator
```

真机替换为 `backend:=real`；无需指定config。不使用MoveIt时可加 `moveit:=false`，不会查询其包或导入MoveIt构建模块。旧的显式 `config:=...` 维护入口仍支持。

固件或Native发布更新后，明确执行一次部署：

```zsh
ros2 run dummy_bringup deploy_manual_panel.py \
  --native /选定构建/dummy_joint_cli \
  --firmware /选定主控构建/dpsdk_firmware.bin \
  --identity /同次主控构建/generated/deployment-identity.json
```

`render_manual_panel_config.py` 是旧的仅渲染配置工具；它保留调用者提供的原文件路径。日常部署使用新的 `deploy_manual_panel.py`，避免运行时依赖可清理的构建目录。启动不会使能或运动。

末端规划页的“逆解偏好”提供最短关节行程、保持腕部构型、腕部正向、腕部反向和靠近Home。悬停选项可查看特点与场景；腕部偏好不可用时会记录回退说明。默认按当前实测角度选择最近的合法等效圈数，避免J6多绕圈；靠近Home会优先Home圈数，可能增加行程。此处采用KDL的少量种子候选，未替换为原固件的八解解析求解器。

“生成随机合理位姿”在当前关节姿态附近采样，先验证目标范围/碰撞，再通过FK填入末端位置和姿态并检查整条规划路径；不会自动运动。1秒是性能目标，超过后继续异步计算并保留目标；快速搜索失败会对同一目标自动增加一次搜索时间。服务长时间无响应才结束本次计算，输入保留，可点“重新规划”重试。碰撞采样间隔仍为0.5°，弯折和回头路径仍逐段保留。双限位标定→Rest完成后，已标定且授权的Idle状态也可直接执行检查完成的规划，无需先从关节页移动一次。

自动测试：先使能/恢复控制，选择逆解偏好、变化幅度（小15°／中40°／大80°）、速度和加速度，再设置测试时长（0.01至1440分钟）。随机位姿生成与自动测试共用幅度和逆解偏好；测试开始时记录面板参数并锁定，防止执行中被改动。幅度约束采样目标，逆解偏好可能选择其他构型而增加实际关节行程。每个随机目标经FK、按所选偏好的IK、完整路径规划与0.5°碰撞检查，执行当前段时提前检查下一段；底层仍逐目标到位确认。

随机候选碰撞或无逆解属于正常筛选，会持续寻找下一目标，不因8次失败而退出；状态中显示候选尝试次数。单个候选的服务响应保留12秒截止，总测试截止不会随重试延长。

点击“立即中断测试”会取消全部后续规划，使用设备受控停车并保持使能，不自动回Home；3秒内未确认停车则停止释放。底部停止按钮直接停止释放，同样可以退出。点击“结束并回Home”则完成当前段后回Home。到期完成当前段并确认实际回到Home，显示绿色“测试通过”；提前结束／手动中断显示橙色；规划、碰撞、反馈或身份异常显示红色并说明原因；反馈、身份或控制权无效时直接停止释放，不依赖保持命令。回程不计入测试时长，正常回Home后保持使能，异常不强行回程。

已由设备确认Hold且实际仍在已确认保持点0.003rad范围内时，面板和Native复用设备保持资格，不重复以原始测速抖动阻塞起步；有限值、新鲜度、身份、位置匹配及设备侧静止/停车保护不变。设备尚未开始运动时，同身份的 `MOVE_NOT_READY` 可因反馈微震短暂出现；自动测试按100ms间隔、最多2秒等待原有起步条件，之后仍不满足则中断。此重试不提高速度阈值，不重试策略拒绝、身份变化或已开始运动后的拒绝。测试参数、计数与退出原因记录在 `moveit-events.jsonl`。

日志区域使用中文解释事件、常见设备故障和MoveIt错误，保留原始编号用于排查；相同解释保存到本次报告目录的`panel.log`，规划事件及耗时另存`moveit-events.jsonl`。原始Native/传输证据保持原格式。

自动测试的包络在开始时固定，以防多次随机步进把J6逐渐累积到多圈。小／中／大档的前三轴半径分别为45°／80°／160°，腕部为90°／120°／240°，与实际工作范围内收2°相交；已经换腕到包络外时只允许向内收回，偏好解超出包络时采用同一末端姿态的已知包络内解。腕部偏好相反且当前转角大于单步幅度时，允许分步接近该构型。

地面与基座底面齐平；每次IK、规划和路径校验的RobotState都包含固定在base_link上的地面碰撞体，只豁免基座安装接触。当前模型已将根几何平移，基座底面为z=0；规划保护体顶面z=0.055m，留55mm高度覆盖线缆外形、回差及模型误差。它不依赖服务启动后另一个场景节点及时发布。实际位置与目标分别通过joint_states/target_joint_states在RViz显示，测试中仍可立即中断。

长测面板的Native将每次运动的完整证据页持续写入原始文件，内存仅保留近期128次的摘要。只有最旧摘要对应的页面已由正常writer写完，才允许滚动淘汰；writer丢页、写入失败、身份、页序或链错误仍会保护退出。发生滚动后不能使用近期缓存宣称整个会话资格完整，完整测试须核验磁盘原件。正常记录SHA在writer写入时增量计算，关闭仍使用原100ms预算，不因长测文件增大而重新读全文件。

## 配置目录怎样阅读

| 文件 | 作用与生效边界 |
| --- | --- |
| [manual_joint_panel.json](manual_joint_panel.json) | 面板源码模板。严格JSON保留原字节，字段说明见下表；真机使用部署副本。 |
| [manual_joint_panel.rviz](manual_joint_panel.rviz) | 实际反馈、目标虚影、标记与GUI布局；不会执行目标。 |
| [dummy_ref_real_6dof.urdf.xacro](dummy_ref_real_6dof.urdf.xacro) | 共享几何加受控ros2_control硬件参数；空身份使展开失败，后续插件仍需校验内容。 |
| [backends.yaml](backends.yaml) | 后端合同/测试用的轴、时钟、身份占位与自动授权默认表；不是通用launch分派器。 |
| [scene.yaml](scene.yaml) | Gazebo SDF与MoveIt场景发布器的共同box障碍输入，位置m、姿态rad。 |
| [sensor_consumer.yaml](sensor_consumer.yaml) | 默认关闭的RGB-D只读参数种子；当前相机示例launch未将此文件作为参数传入，不能据此声称TTL已生效。 |

实际模型订阅 `/measured_robot_description`，目标模型订阅
`/target_preview/colored_robot_description` 并使用独立TF前缀。
RViz的 `Collision Enabled: false` 只是不显示碰撞几何，不关闭MoveIt碰撞检测。
旧wheel/motor的Links属性及 `QMainWindow State` 是GUI缓存，不创建机械模型或设备状态。

## JSON字段、单位与默认值

`manual_joint_panel.json` 内没有注释，以下表格逐字段解释；这些模板值不会自动证明设备身份或签发许可。

| 字段 | 含义 |
| --- | --- |
| `native_binary` | Native可执行文件路径；源码为空，部署时写为所选release副本。 |
| `native_arguments` | 原argv字符串数组，不是shell命令。包含home-rest/execute角色、显式串行端点、device/client、profile201、mask63、scope与四类摘要。模板的固件/合同/模型/profile摘要为空，部署器依据同次构建身份填写。`--execute`是传给Native的能力入口参数，加载JSON本身不会执行运动。 |
| `decoder_directory` | 证据解码脚本目录；源码为空，部署副本固定到release资源。 |
| `state_interface_id` | 当前State协议接口号4139，供原始反馈筛选；不是CAN节点号。 |
| `joints` | 按Joint1..6排序的六个关节记录，不得按任意字典顺序重排。 |
| `joints[].name` | 对应URDF/协议使用的关节名。 |
| `joints[].direction` | 电机位置到模型位置的符号；当前为[-1,-1,-1,+1,-1,-1]。 |
| `joints[].gear_ratio` | 电机转数/关节转数，当前六轴均50，J6也有减速器。 |
| `joints[].nominal_limits_deg` | 名义机械范围，用度表示；J6为±720°，不折回一圈。 |
| `joints[].limits_deg` | 面板工作范围，按名义范围每端内收0.55°；不用于修改模型合同。 |
| `working_margin_deg` | 上述工作域内收量0.55°。 |
| `home_deg` | Home六轴度数[0,0,90,0,0,0]，不是全零关节姿态。 |
| `rest_deg` | 工作REST度数[0,-60,163.97260273972603,0,0,0]，保留边界停车余量。 |
| `description_xacro` | 同版几何描述路径；源码为空，部署器解析并固定所选资源。 |
| `motion_speed_max_deg_s` / `motion_speed_default_deg_s` | 面板最大/初始关节速度63/60°每秒；不能和末端mm/s混用。 |
| `motion_acceleration_max_rad_s2` / `motion_acceleration_default_rad_s2` | 最大/初始关节加速度均4rad/s²，不是电机rev/s²寄存器值。 |
| `inward_recovery_enabled` | 选择向内恢复支持；实际恢复仍受参考、身份、方向及设备状态约束。 |
| `wire_schema_major` / `wire_schema_minor` | 预期协议2.5，与Native/固件/解码器须相符；不是ROS发行版号。 |
| `calibration_mode` | 当前manual_limits人工双限位参考流程，不能从字段存在推断已完成标定。 |
| `calibration_pose_deg` | 人工参考姿态[0,-73,180,0,0,0]度，区别于工作REST。 |
| `manual_limit_reference_enabled` | 选择人工限位参考能力；启动不会自动写入参考。 |
| `moveit_jog_speed_default_mm_s` | 末端位置微调初速100mm/s，用于目标编辑。 |
| `moveit_preview_speed_default` | 轨迹预览动画倍率20×，无量纲，不改变设备速度上限。 |
| `deployment_required` | 源码模板为true；真机必须完成部署绑定，渲染后的配置移除此模板标记。 |
| `moveit_segment_timeout_s` | 单段Native宿主等待预算660s，当前渲染器/面板只接受(0,660]的有限数；它不会延长设备permit或30s会话租约。 |

其中 `native_arguments` 的 `--device/--client` 分别为128bit身份（32个十六进制字符），
`--firmware/--contract/--model/--profile-sha256` 是各32字节SHA身份；profile编号201和profile摘要不是同一字段。
部署还会添加 `deployment_manifest` 指向该release的清单，启动时校验文件副本；
源模板硬编码的USB by-id名称并不证明同名设备当前在线。

真实六轴Xacro的 `stop_confirmation_timeout_us` 默认字符串0与当前transport非零解析规则
存在静态接口差异：模板显式输出0并不能触发“省略参数时”的默认回退。
不要把launch允许0的输入校验当成下游已接受该配置；现有入口还提供显式300000µs预算。
