# 手动面板的三种目标与独立ROS链

面板保留Python/Qt界面，下拉选项和连接状态统一为“真机-USB/Native”（`real`，既有USB Native进程）、“真机-USB/ROS”（`ros`，独立C++ ROS控制链）和“仿真-本地”（`simulator`，软件模型）。内部backend键仍为real/ros/simulator，显示名称不改变通信、许可或释放语义。选择ROS不会运行Native标定/运动/释放，也不使用Native stdout或Python raw decoder作实时反馈。三种目标的设备身份、参考、许可和旧规划不会互相继承。

本轮已调整为SDK先行：先从Native真实行为提取公共参考/状态预算策略并安装SDK，再让ROS C++消费SDK公开常量，最后进行软件回归和实机验收。Status预算沿用Native真实1秒，默认与上限由SDK/C++约束；不继续先修ROS局部500ms规则。中央协议与REF不因此增设副本，Native CLI行为不改。公共化完成、软件测试和实机验收是分别证明的事项，不能相互代替。

## GUI39长轮实测：两轮正式完成

`gui39-long-auto/final-events.json`记录普通请求300秒、实耗303.840503秒完整完成171个随机目标加Home；连续请求30秒完整完成50个随机目标加Home，整轮仅1条FJT，含预规划耗时47.235482秒。两轮结果均为`completed`，不是early或测试中断。实际回Home后Stop取得`ROS_RELEASE_CONFIRMED`，ref3保留、session0。GUI38观察器主动中断及旧93目标首槽拒绝保留为历史；本轮首槽预算修复已有独立真实长轮证明。随后正常UI重连保留ref3，显式MoveIt“规划至工作Rest”与“规划至Home”均实际到位（J2/J3分别约−60.03550°/163.96917°与0.07072°/89.99733°）；再次Stop确认释放，正常Reconnect及启动No后交还ReadOnly/session0/permit0。连续轨迹1563点、末时间31.28秒，整轮1条FJT。首次重连因已退出旧节点DDS信息尚未消退而保护性拒绝新owner，等待后同UI重连成功；不是首次无干扰重连通过。两份运动raw终审与footer封存结论另核，不预写全部raw通过。原件另见`endpoint-events.json`。

## 本轮释放修复与历史长轮边界

接受后、Begin前被拒绝的目标可能没有terminal timeline，控制链因此明确报告`EVIDENCE_INCOMPLETE;RELEASED_IDLE`：任务证据不完整不能改写为自动完成，但已释放的设备状态应独立收敛。Backend仅在显式释放意图下识别这一精确复合结果，保留原警告；20秒超时仍记失败且不重发D10，继续只读查询，允许迟到的权威释放确认恢复收尾。确认释放时还会清除旧pending，只发布一次released事件；真实旧只读回执在途场景已先RED后GREEN。本轮6相关suite共186项通过；长轮与最终释放实机结论仍待补齐。

用户首次内收后，维护CLI仅清故障与SelectMode，同boot/ref3保留，无Grant、Capture或运动。GUI38随后真实完成38个随机目标，accepted/success全部匹配、无limitfail；但普通300秒未完整结束：Root观察器对半行JSONL执行json.loads抛异常，测试脚本except主动STOP，非产品自发失败。观察器已改为仅读取完整换行，并核UTF-8片段。GUI37报告目录预建FileExists仅属启动脚本边界，不是运动失败。

Stop后旧ROS UI未立即撤销未来目标，本轮将autoabort/retire移到共有前置，再置ROS busy、撤授权并请求权威释放；自动结果记interrupted而非error，设备仍仅由ROS控制链D10释放。新增2例先RED后GREEN，三相关suite81项通过，与此前186项分别记录，不相加冒充完整suite。GUI38失能后J2/J3滑出软范围，等待用户调整到约J2=0°、J3=90°；该时刻GUI39长轮和30秒连续尚未执行；后续已由上述GUI39完整轮补齐，无需新标定。

## 长轮新发现与当前验证边界

GUI35短轮通过范围保持不变。交接后普通5分钟大幅度测试在第93个目标暴露起点连接问题：J2微跟随误差0.00165145rad仍在既有0.003rad位置窗内，但原20ms首槽计算加速度4.1286rad/s²超过4，控制链在Begin前正确拒绝；不是内部路径曲线超限。

本轮保留全部路径位置、碰撞几何及空导数，只为既有起点窗口预留首槽动态预算：同时满足从Hold零速连接首点及下一重复点停车，按20ms向上取整。60°/s、4rad/s²时首槽为40ms，之后仍20ms且内部路径速度不变；首两点须重复，完整轨迹含首槽仍不超过630秒，实时准入检查不变。最终6相关suite186项通过；C++真实68点夹具验证旧20ms首槽拒绝、新40ms接纳，六轴±0.003rad共12项通过。GUI36当时因J2软限位未执行；GUI38真实38个随机目标无limitfail但被测试脚本主动STOP，GUI39长轮及30秒连续在用户再次内收后完整完成，详见上述实测摘要；无新固件或重新标定，不能以GUI35旧session短轮8/8或GUI34 Native证据代替。

## 当前关节、自动与连续入口

ROS关节页直接根据新鲜实测起点、关节范围、速度和加速度生成20ms完整FJT，交给ROS C++权威链执行，不调用MoveIt碰撞检查。末端页的IK、避障规划和场景检查仍走MoveIt。两者都不能由Python绕过C++身份、参考、许可、动态限值或截止检查。

ROS自动测试使用普通多次FJT，不再调用用于固定标定验收的`auto_calibrate_and_test`基线服务。ROS连续模式预先规划有界多段blend及返回Home，整合为一条完整FJT，使中间段真实连续过渡；不是逐段停稳，也不使用Native滚动append。ROS UI时长上限9.5分钟，整条轨迹上限630秒；Native原有自动/连续功能保持独立。

切换Native/ROS时应销毁旧实际反馈发布链并重建对应`joint_states`来源；实际模型只发布新鲜实测关节角，不以目标、缓存旧角度或零值冒充实测。本轮不改SDK或固件。GUI35已完成本轮ROS实测：关节直达REST/HOME；普通自动0.10分钟、medium、60°/s和4rad/s²完成4随机段加Home，共5条FJT；连续同参数完成11随机段加Home，仅1条完整FJT。自动流程正式完成并实际回Home，最后双Idle/session0且界面确认RELEASED_IDLE。原件为`gui35-auto-final/final-events.json`及`ros-session-idg4xjm3`；GUI33早期失败仍保留，不由旧GUI32证据替代本轮验收。

本轮Native反馈已补实测：GUI34普通生产面板REST/HOME的`joint_states`与raw实测匹配，18个实际link的TF无缺失，`joint_states`仅一个发布源，最后取得Native `RELEASE_CONFIRMED`。原件为`gui34-native-rviz/native-phase-events.json`；这是Native显示与执行链证据，与GUI35的ROS自动/连续验收分别成立。正常退出同时回收该轮launcher及子进程，启动器已增加面板退出时的链路收尾。

## 当前使用入口与最终验证边界

ROS按钮“双限位标定 / 恢复控制”只准备手动控制，不启动自动验收。ref0须按真实双限位确认建立参考；同boot已有参考则核实后复用。正常确认耗时不计入实际子链启动或首次场景同步预算。末端页MoveIt规划仍须六服务唯一同场景及有效实测起点，完整路径成功后再执行；已消耗的路径需要重新规划。停止以ROS控制链确认RELEASED_IDLE及新鲜设备状态为准，不能仅看硬件Idle或按钮状态。

最近修复均保留原安全门：只读历史准入仅接受已明确清故障的Safety2/0x1002严格形状，不消除历史Fault或授予许可；启动30秒与初次场景20秒从实际开始使用时计时，不把人工确认等待算作子链故障，也不重开真实超时；无关观察节点退出不再锁死场景，六服务唯一同组owner与未知异常拒绝仍完整。

释放还区分真实硬件与Host确认：GUI30五轨迹到位及D10原件成功，但当时Host RELEASE_UNCONFIRMED保留为失败；已补manual保留参考的Stopping7/Disabling8→Idle2有限等待，不放宽原新鲜度/身份/截止。GUI31实际TF18links/6states齐全、C++/UI释放通过但无新轨迹。GUI26的启动假超时、GUI28/29驱动边界与GUI30失败详情保留在Bug台账及原报告，不作为成功验收覆盖。最终GUI32已在普通生产窗口完成两条路径、立即Stop、C++/UI释放确认和正常重连，具体证据见下列当前验收摘要。

### 当前验收摘要

GUI30正常生产Qt的45.008秒人工确认等待、连续五轨迹实际到位均通过，但该轮软件释放失败保持原结论。最终GUI32补齐两条完整公开FJT与独立到位（jointRest 0.048719°、末端页namedHome 0.077467°），Active TF18links/6states齐全，观察节点退出不再误锁场景；立即Stop后硬件与C++/UI均确认释放，真实7→2跨流512ms正常等待，随后正常No重连并独立确认fresh双Idle/ref2/session0/permit0。

交还前已有有效ref2，无需重新采零；先在正常确认框恢复手动控制，关节页可直接执行六轴目标，末端页须先规划再执行，已消费路径需重新规划。保持默认60°/s、4rad/s²，不以此宣称驱动独立失能读回或正式资源资格。报告gui32-verified-handoff/final-evidence.json和handoff-completion.json保留关联证据；驱动AT-SPI短超时已单列，不能把驱动的FAILED当成实际重连失败，也不掩盖GUI30旧Host释放失败。

## 历史GUI24交还：当时正常确认、Hold及失能有效参考

GUI22交还后用户报告“使能无反应”，不能只凭此前无运动Stop/重连成功认定后续确认交互也通过。现场XQueryKeymap仅显示Escape（keycode9）held=True；无ROS、无hook的纯标准Qt QMessageBox实际收到KeyPress16777216并clicked No，约20–75ms自动关闭。AA_DontUseNativeDialogs不改变此行为。执行xdotool keyup Escape后，同样标准确认框保持5.05秒，直到诊断Timer主动reject。该证据定位到当时的输入状态及Qt行为，不推断Esc按下状态的来源，也不是ROS管理服务或设备拒绝的证据。

遇到确认框一闪即逝，可先释放键盘Esc；X11/Xwayland环境可手动运行`xdotool keyup Escape`。该命令只释放输入键，不发送ROS/USB指令、不确认双限位、不使能或授权运动；随后重新点击按钮，按当前真实物理状态正常回答确认框。

GUI23仅使用sys.setprofile只读跟踪，不改动作、不自动答复；真实恢复按钮打开的正常确认框保持约60秒，AT-SPI实际点击Yes后达到MANUAL_CONTROL_ACCEPTED、Hold及READY，session2316631754780395781，随后真实Stop→RELEASED_IDLE。该过程作为诊断证据，不冒充无hook人工交还验收。GUI24最终使用正常生产窗口，无private PYTHONPATH/sitecustomize/TestApp/profiling。实际启动确认通过AT-SPI点击No保留已有参考；实际恢复按钮打开的正常QMessageBox保持后，再通过AT-SPI实际点击Yes，未绕过或替换正常确认逻辑。16:48:50 MANUAL_CONTROL_ACCEPTED，16:48:51 READY；独立类型化双源证明真实Hold4、State hold_ready=true、hold_valid_mask/position_valid_mask均63，session4357804659931202042，Status permitted=true且permit_remaining_us=873663631，boot/ref1不变。证据在gui24-human-ready/normal-enable-proof.json。

实际Stop于16:49:27 QUEUED/PENDING，16:49:28已released/RELEASED_IDLE；单次Reconnect及正常启动No后建立新ReadOnly。最终独立双源证明双Idle2/session0/ref1/permit0/position_valid63，见final-independent-idle.json；desktop-input-check.json证明当时held_keycodes为空、Escape=false。窗口已交还用户，后续执行者不再进行设备IO或重启。用户此前失能时曾手动移动关节，本轮Enable仅在实测姿态保持，不能将当前有效参考或Hold解释为当前名义HOME。本轮无FJT或MOVE；GUI21四项运动及TF全部通过保持，GUI22后来交互失败保留。

GUI23的cpp-enable-release-proof.json另有Enable COMMITTED与双Hold、D10全63 SENT及无丢证终页/最终Idle的只读诊断关联；不含16–19或FJT，不冒充GUI24无hook交还证明。产品源码、b603后端/d632 UI、SDK/FW与协议pin未因此改变。

## GUI22当时无运动复测：只读查询有限超时

GUI21四项默认动态运动、正式FJT终态/实际到位及原D10证据保持通过，但后续长时ReadOnly连接暴露新的薄客户端阻塞：15:47:54连接后，服务端15:47:55出现bootstrap_status响应发送timeout、client will not receive response的WARN；15:53点击Stop未出现RELEASE_QUEUED，原20秒后报release deadline expired。同时独立订阅仍有fresh双Idle，服务端正常响应独立调用。WARN与一次只读Future不返回的路径吻合，但不足以定位或证明具体DDS丢包原因。

代码缺口是RosBackend._request仅靠Future回调清pending，而RosTransport的Trigger请求无截止；一个永不完成的只读status查询可长期占用客户端槽，阻止后续正常释放请求。已实现的最小修复只为bootstrap_status、manual_control_status、baseline_status、manual_reference_status及last_goal_rejection这些只读Trigger查询设置1秒宿主单调超时；先退休本次Future/token再cancel，返回success=false的READONLY_REQUEST_TIMEOUT，由薄Backend清pending并报告progress，随后按原轮询继续。迟到或取消同步回调不得确认后续释放，关闭后不回写状态；超时不等于设备失败或已失能，也不据此补造回应。该1秒机制不退休或自动重发带副作用的prepare/自动任务/disable_and_release；既有UNAVAILABLE有界重试与QUEUED后只poll原终态的规则不变，原20秒释放绝对截止不重开，C++/SDK/FW/FJT不改。

Root随后通过独立公共disable_and_release取得RELEASED_IDLE，并独立确认fresh双Idle/session0/permit0/ref1，才按已证明归属关闭旧控制组及自己的外层窗口；不能把外部收尾记为当时UI自动Stop成功。单元复现先RED后GREEN，43 pass/1 skip（保留原隔离集成跳过），覆盖丢失只读回复后仍能释放、迟到旧回复不得假确认、mutator不超时重发、关闭退休查询以及反复只读超时不延长原释放截止。最终Backend源码及安装SHA256均为b603be79c10c07bee60e25941f625696b853801ee200a331efb9c07226b810d3；Main、C++、SDK、固件及FJT未改。

GUI22正常生产窗口没有私有sitecustomize/TestApp/PYTHONPATH，也没有自动确认hook；仅通过正常启动问题Escape回答双限位No，后续正常确认均保留。真实单次Stop约3.1秒取得RELEASED_IDLE，单次Reconnect约3.8秒恢复ReadOnly；gui22-human-ready/normal-stop-reconnect-proof.json保留现场结果。随后独立类型化订阅确认fresh双Idle/session0/permit0/ref1，证据见final-independent-idle.txt。本轮没有Enable、MOVE或FJT，不能冒充再次四项运动或丢包故障注入实机复放；GUI21四项目标及实际TF全部通过的结论保持，旧UI后来Stop失败也保留。最终交还的是正常人工使用窗口，不代替用户继续操作，后续不再由执行者主动操作设备。

## 当前验证边界：GUI21默认动态四项通过

GUI21在用户明确授权的一次Native维护恢复之后，从实际HOME起始，完整通过ROS默认60°/s、4rad/s²四项目标：关节页REST→关节页HOME→末端页面命名REST→末端页面命名HOME。四条公开FJT均accepted并正式terminal_success，随后均由fresh实际反馈独立确认到位；每条76个路径点、78条命令、1.54秒。没有降低动态上限，不借Native完成这些ROS目标，也未跳过真实模型碰撞检查。末端页本轮使用命名Home/Rest，不把此前XYZ/RPY→IK等效腕解强制等同于nominal关节解。

Native维护另存native-maintenance-home-21：同boot/ref1保留，3次MOVE_COMPLETED实际恢复到HOME，随后RELEASE_CONFIRMED并退出0，raw TransportComplete且无丢帧。此路线只恢复起点，不计ROS标定、ROS四项目标或ROS释放通过；维护与ROS owner串行，无共享USB FD。新boot最初reference0→1仍由此前GUI8真实ROS16及17–19建立，不在HOME重采零。

| ROS目标 | 公开Goal UUID | 原始run | 最大到位误差（度） |
|---|---|---:|---:|
| 关节页 REST | `1b194d2aa98747bb8f2802a551e0bd0e` | 31 | 0.045765644 |
| 关节页 HOME | `742a6211fd584b0c957f942898863138` | 32 | 0.076345962 |
| 末端页命名 REST | `cb17b0daf69d4867b1c892b67e952d33` | 33 | 0.048858982 |
| 末端页命名 HOME | `6ba57d510d4940428b97cc866176ae45` | 34 | 0.077609390 |

四目标完整轨迹SHA及accepted/before_begin/terminal_success同UUID关联见ros-session-35ljocwo/goal-timeline.jsonl（文件内容实际为CSV，按字段读取，末行COMPLETE不是第五个Goal）。motion epoch依次71→73、73→75、75→77、77→79，四次完整业务终态分别属于run31–34。gui21-high-from-home保留本轮面板/MoveIt记录，原始证据由中央SDK布局解析，chain_errors=[]。最终FINAL_RELEASE_CONFIRMED后重连ReadOnly/ref1/session0/core2通过，不把GUI20无运动重连当作本轮enabled释放证据。

GUI21最终D10根请求13191556776482737655，经对应Query取得operation429、transaction_phase3（COMMITTED），release run35的category3 Disable页expected/submitted/sent均63、failed0，终页release_committed=true且noEvidenceLost，整链chain_errors=[]。四个业务run31–34均motion_complete、无Failure、无丢证。冻结raw SHA256为d0cd9729cdbf8c30f04403b95a60f6413ca12726d164aee8abfd5b5cccce3d7e。首Idle State宿主时间268369968828µs，首Idle Status为268370475753µs，跨流间隔506925µs；之前最后Status core7，不是8。因此本次证明正常使能后释放及约507ms跨流等待真实成功，不能宣称精准复放GUI17的8→2竞态。该精确分支仍由510799µs软件RED/GREEN与严格身份/Fault反例覆盖。

GUI21四项目标后当时另独立执行一次安全HOME的Prepare，仅核Active实际TF：18个actual links全部可解析，missing_tf=[]，六轴实际joint_states齐全，没有新增FJT或MOVE。随后真实UI Stop完成释放，独立新鲜订阅再次确认同boot/ref1、双Idle/session0/permit0；面板保持已失能。此额外Hold/TF检查属独立session，不改写四Goal的UUID/run证据。gui21-high-from-home/final-evidence.json、active-tf.json及final-independent-idle.txt分别保留业务/释放、显示链和独立失能观测。CAN SENT只证明提交/发送，不等于驱动器独立失能readback；不扩为正式资源、长时负载或外部位移资格。

## 从只读连接到有条件运动

普通启动默认`auto_calibrate_and_test:=false`、`operator_confirmed_double_limits:=false`。连接只建立通信与读态；双限位确认必须来自本次明确的人机交互，绑定当前部署、设备与boot，不能从按钮名称、先前日志或上次任务推断。

`manual_joint_panel.launch.py`在三个初始目标下持有同一套实际RSP和可选MoveIt；目标虚影使用独立RSP、`/target_joint_states`和`target_preview/`坐标。GUI通过`ros_launch_command`管理自己的ROS控制子launch，显式`reuse_display_planning:=true`。独立运行`manual_ros_control.launch.py`默认自行创建显示/规划。ROS实测`/joint_states`由joint_state_broadcaster发布，GUI不再发布第二份实测状态。

gate进程内部嵌入ControllerManager，因此launch不得给gate节点添加全局`name=`重映射：该重映射会同时改掉同进程CM的名字，使`controller_manager`键下的inactive YAML不再匹配。当前入口依靠程序内的固定节点名，动态图测试核gate没有全局node-name remap；源码改动曾经红/绿验证这一差异，但这不是物理运动结果。

ROS部署配置由明确的`sdk_prefix`（也可通过`DPSDK_HOST_SDK_PREFIX`传入）与固定release生成；当前符号链接会被解析成固定目录并记录manifest原字节SHA。逐文件SHA与编译钉住的SDK模型只证明产物身份，不能签发控制许可。当前受限范围是`manual-home-empty-supported`，不是`f405_6dof_bench`资格，也不宣称发布RAM/栈/截止测量通过。

C++链按以下顺序推进：

1. 硬件初始inactive，`on_configure`启动唯一transport worker；这时可建立管理通信，不要求已经有绝对位置。
2. 先分两条路径：无参考时，`SelectMode`与`GrantSession`后由专用操作16 `ConfirmManualHome`建立绝对参考，普通操作3 `CaptureReference`不代替它；已有同boot绝对参考时，必须先核当前身份、新鲜双流Idle、有效六轴绝对位置、无session/permit、真实release_committed且无证据丢失，才允许保留原reference。复用路径不发送16、不重新采绝对零点，仍重新走17→18→19。仅看reference非零或曾经连接过都不够。
3. 操作17 `BeginQualification`沿REF受控路径进入REST，再以独立、新鲜、静止的反馈窗口推进18 `ValidateRest`和19 `CompleteQualification`。QUEUED/受理成功都不能推进为完成；错boot/session/reference、重复样本或过期反馈会阻断。
4. 参考与REST条件完成后才建立运动许可，再激活硬件/广播器/受保护控制器并请求Hold。基线与面板的业务轨迹都提交到公开`/arm_controller/follow_joint_trajectory`，继续经过同一C++ gate。

现场只读观察曾发现同boot保留已资格的`reference_epoch=1`，而实际姿态仍在双限位附近。这说明“已有参考”和“已在REST”是两项不同事实，不能因接近双限位就重新采零，也不能因reference有效就跳过17→18→19。本观察不代表本轮ROS完整物理基线已通过。

面板捕获姿态J2=-73°、J3=180°位于内缩0.55°的普通规划工作域外。不能放宽普通FJT限位来“起步”；这正是必须先走专用REF参考/REST路径的原因。

## 没有参考时，原始反馈仍然有意义

`position_valid_mask=0`且`raw_position_valid_mask=63`表示六轴原始电机圈数可用，不表示六个关节角都是0。transport可向管理流程交付经身份/新鲜度检查的raw State，同时失效普通关节缓存。`panel_feedback`仅投影已接纳的中央消息，GUI展示原始量与“尚无绝对位置”，不得把缺失关节角补零喂给规划器。

定时发布次数也不是新样本数。C++协调器按源序号、接收时刻和窗口跨度核独立样本；跨boot或迟到回调不能接到旧窗口后面。

## 独立固定验收基线和释放如何读结果

固定验收服务与ROS面板普通自动测试分开；`auto_calibrate_and_test`的Trigger响应只表示受理。C++ baseline节点通过同一个公开FJT检查保持、六轴小幅往返、多轴、规划REST往返、显式圆柱碰撞场景和取消；实际任务与raw证据、goal timeline关联。没有合法路径时明确失败，不删除障碍求通过。默认缺少圆柱配置不能得到完整碰撞基线通过结论。

流程结束必须主动请求并确认失能释放；`RELEASE_QUEUED`仍是进展。释放成功不能覆盖之前的失败。已完成绝对参考/REST的同boot设备可能在失能后保留机械参考；应核两条流的Idle、相同reference及无session/permit，而不是强制reference清零。跨boot、未完成资格或reference错配不能复用。关闭窗口、切换目标或终止进程也不是失能证明。

软件组件/PTY对端/ROS假服务测试只证明其具体刺激覆盖的行为；物理运动、停车及最终释放必须来自真实设备证据。正式发布资源资格另行保留边界。本页不宣称本轮物理验收已经完成。

## 自动基线结束后恢复人工FJT

自动基线最后仍必须主动释放，不为了留下可操作界面而省略失能。收到带本次boot/报告信息的`BASELINE_PASSED`后，薄Backend关闭命令准入并记已释放。若还需人工运动，操作者明确点击重连：只有C++已确认释放才停止自有runtime、销毁旧Backend，并创建新的同ROS只读链；未释放时首次点击只请求释放，等待确认后再次点击，不自动恢复运动或复活旧gate。

新链读到同boot保留参考、双流新鲜Idle且无session/permit后，“恢复控制”只向`/device_action_gate/prepare_manual_control`发薄请求。C++复用参考/REST→激活硬件与控制器→EnableHold，在保持确认后报告`manual_control_status=MANUAL_CONTROL_READY`，不执行自动基线的小幅单轴/多轴/绕障轨迹，也不声称BASELINE_PASSED。Python不编排各管理步骤、许可或续租。业务运动仍由MoveIt规划后提交同一公开FJT。

人工Ready持续受C++身份、源反馈、owner/reference和固定总期限监测。外部释放经双流与原件确认后才报告MANUAL_CONTROL_RELEASED；失鲜、异常、关闭或期限超时仍主动清理，并保留失败。旧流程终态不能通过再次点恢复复活；需要新的只读链重新核实实际设备。

## 每次启动的证据原件与实际安装层

raw采集与goal timeline按`O_EXCL`创建，不覆盖已有原件。GUI每次创建ROS子进程前，`PanelRuntime.start`都会生成独立`ros-session-*`目录，向子argv补齐并替换三个路径：`baseline.log`为C++逐行event文本日志，`transport.raw`为原始采集，`goal-timeline.jsonl`为目标时间线。只预建目录，文件由C++创建。即使旧配置传空路径或重连同一窗口，也不会复用前次文件；原件不因失败而删除。实际spawn参数保存在runtime的`launch_argv`，报告应记录这份参数，而非改写前的模板argv。附着外部进程的空命令不创建第二个owner。

本轮现场使用模型/MoveIt的`install_panel_guide`作为underlay，控制与消息的`install_protocol`作为overlay，source顺序是：

```bash
source /opt/ros/jazzy/setup.bash
source install_panel_guide/local_setup.bash
source install_protocol/local_setup.bash
ros2 pkg prefix dummy_moveit_config
ros2 pkg prefix dummy_motion_client
```

前者应指向选定模型/规划安装区，后者应指向本次控制overlay；不要将旧shell偶然残留的路径当作已验证产物。新的`scripts/build_manual_panel.sh`默认先把中央接口装到`install_protocol`，再将模型、MoveIt、两个C++控制包及bringup统一装到`install_panel_guide`。按这套新配方构建时，应按脚本打印的接口→统一面板安装区顺序source；它与现场增量overlay布局是两种明确的产物装配，不可混写成同一安装结果。构建脚本本身不更新SDK或操作硬件。

## 查看规划服务与关闭自己创建的MoveIt

在与面板相同ROS环境和Domain的终端只读查看：

```bash
ros2 node list --no-daemon
ros2 service list --no-daemon | rg 'plan_kinematic_path|apply_planning_scene|get_planning_scene'
ros2 topic info /joint_states --verbose
ps -eo pid,ppid,args | rg 'manual_joint_panel|manual_ros_control|move_group|device_action_gate'
```

应核唯一规划节点/公共场景服务和唯一实测发布者；不能只数服务名就排除同名重复节点。需要关闭时，先在面板发起停止释放并等C++确认，再关闭本窗口拥有的launch。只需关闭规划器时，在上面进程列表核准自己创建的`move_group`PID后执行：

```bash
kill -INT <本窗口创建并已核准的move_group_PID>
```

不使用`pkill -f`。关闭MoveIt不会停止设备、清除参考或证明失能；重新规划前还须核同一场景和最新实测状态。父launch退出与ROS子launch的释放/退出应分别对账。

## 现场失败记录与恢复验证待办

本轮现场已有一轮明确失败：自动REST实际到位后，ROS基线把250ms门槛误套到约260ms间隔的Status，释放又与17确认竞态未及时发D10；设备最终以`0x1067`（RestValidating超时）进入Fault9，观测到session0/ref1、release_committed与cleanup3。到位不等于参考资格流程已经提交完成，也不能把设备自主故障失能写成Host释放成功。

最终文档必须保留该轮失败的原始证据、两项ROS根因、针对性改正和重新验收结果，不能以重试通过改记“一次成功”。Status与State应按各自真实发布周期核新鲜度；修Status预算不等于放宽运动反馈。清已处理故障必须显式进行，不能随连接自动ClearFault。已完成同boot参考可在清错后保留，但清错会撤控制身份并使位置资格重新建窗；必须重新核真实Idle、原参考与独立新鲜反馈，再沿原ref→17/18/19，不能在REST重新发16采零。未知物理/总线故障、未确认的Disable或缺失反馈不能靠ClearFault掩盖。

当前ROS公开服务尚无ClearFault入口；既有SDK的管理请求和根Query足够承载它，恢复接缝应继续复用唯一C++管理owner。上述待办只描述已经发现的问题和必要验证，不宣称修复或后续物理验收已经完成；最终报告与原件链接由验收记录补齐。

已新增ROS自己的C++窄恢复工具`manual_ros_recovery_node`及validator/测试，复用既有SDK管理请求，默认只读。它没有Grant/Permit/Enable/FJT/D10路径；只有已定位并修正的0x1067根因、固定boot与身份、原件全SENT和实际失能事实核准后，才允许显式清错。ClearOK不等于DisableSENT，不能替代原释放证据。

首次现场只读verify因State.reason=0、Status.reason=4199来自不同字段域而被validator挡住；当时没有mutation。此拒绝、字段域修正及后续实际验证必须分别记录，目前不能写成清错通过或恢复成功。工具必须在旧串口owner退出后独占运行，不调用Native恢复。最终源码SHA、全文bundle与HTML等SDK公共策略、ROS接入及后续实测源码冻结后统一生成。

GUI不复制SDK控制预算：RosTransport异步读取`/manual_ros_baseline/get_parameters`的`status_max_age_ms`，按返回的实际生效值显示Status预算和按钮新鲜度。首次读取成功前保持只读；读取失败不填Python默认值，不影响主动释放。State/CAN的新鲜度仍独立，不能借Status预算放宽。该标准参数读取不编排标定、许可、续租或完成状态机。


## SDK兼容范围与当前物理边界

公共`dummy_ref/manual_reference_policy.hpp`经`DPSDKHostSDK::dummy_manual_reference`提供纯C++17策略，SDK Config/sdk_export/manifest负责接口导出、头安装与实际载荷SHA。Native和ROS共享RestSettleWindow、危险侧动态REST、五次插值时长、预算及RPC键；ROS coordinator已实际调用这些接口，transport只转发Native既有mutation XORop/Query无salt规则。共享头没有串口、ROS、线程、许可或CAN所有权，既不发命令也不证明SENT。Native CLI行为不变，两个目标仍各有唯一独立通信owner，不能并行占设备，更不共享FD。

SDK公共策略parity和独立consumer绿色属于软件兼容性证据。run16受限实机基线已BASELINE_PASSED：18条真实公开FJT受理，其中17条SUCCEEDED及17次独立ARRIVAL（1次REST对齐、12次单轴、2次多轴、2次REST），另1条完成CANCEL_TERMINAL/CANCEL_STOP_CONFIRMED。固定请求圆柱A/B/A与FCL同ID contacts通过；释放先UNAVAILABLE后在原20秒期限内重试确认。 冻结raw为4895495字节，SHA256 b57e063ad1a61eaab5eef9e738afbe2e9bcf03e410f1eaadb8665d61244b8f5e，chain_errors=[]。末D10 run98/kind1 expected_mask=sent_mask=63、failed_mask=0，terminal final2、releaseCommitted=true、无evidenceLost；State/Status同boot双Idle2、session0、ref1。SENT是发送终结证据，不是驱动器独立失能readback。

历史run5失败（当时formal release=false）及之后逐次失败原件保留，不能被run16通过追溯改写。此次结论限于固定部署的受限功能基线；薄面板接口随后由panel06真实通过；Qt只读显示检查单列，不声称完整GUI点击运动或正式资源资格均通过。


## 可选模型REST对齐与完整轨迹边界

资格17→18→19沿Native/REF真实规则只退让危险J2/J3，可能把已有安全13°余量的J3冻结在166.33°，并不强求模型REST163.97°。仅自动基线在Hold后可先增加一次公开MoveIt/FJT模型REST对齐；J2/J3差值不得超过SDK13°，其他轴不得超过0.001rad。人工prepare不执行这次额外归位。

对齐要求规划首点t=0且六轴位置/速度/加速度齐全。每个五次Hermite段转Bezier凸包，整条曲线受origin↔REST±0.0001rad走廊及速度/加速度界约束，不能只看离散路点。统一时间缩放同时按1/scale、1/scale²缩放导数，保留全部原路点和同一几何曲线；不删点掩盖问题。公开Action成功后还需新鲜到位与静止反馈，才把实际到位姿态冻结为原1°测试的origin。因此标准自动基线为17个公开FJT目标，需此对齐时为18个；这不是重做操作16或放宽普通1°测试。

SDK共享move budget由完整末点时长、3秒目标容差与2秒传输余量组成，并同时受当前permit、当前180秒单项剩余时间和840秒总任务剩余时间约束。执行截止固定为发送时刻加预算，ACK与Status不续期；服务/目标ACK保留独立15秒界。失败仍主动释放并保留首次原因。Bezier纯测试证明数学包络和已知反例，不证明真实设备SENT、正式释放或这次归位已经实机通过。


## 共享REF故障释放缺陷与验证边界

后续只读/恢复记录还暴露了共享底层缺陷：D10 `DisableAndRelease`即使携带正确当前tuple仍被PolicyDenied(10)拒绝。先前将kind3误读为Safety不正确，中央定义为Safety=2、Release=3；实际Release run59在故障撤销实时owner后缺少原owner安全释放回退。这不是ROS独有问题，Native同样受Core分支影响；“Native曾验证过其他路径”不能证明本故障不受影响。根线程对当前部署46b361映像的88个Core机器码符号比对一致，用于确认实际固件包含这一缺口。

最小修正仅允许有原failure、同boot/session/client的活动Safety或Release原owner完成释放，当前管理/参考/运动tuple仍精确核对。有已有failure的Release须落FaultLatchedAndReleased4，RunLedger要求failure_recorded及自己完整Disable SENT/释放事实，不能伪装成无故障Released8；中央协议不改，历史失败也不抹除。

F405构建、Core软件回归、后续标定/运动及正常D10释放是不同证据。后续run16已实测受限基线及正常正式释放；新固件下同类故障注入与原owner恢复尚未重新实机演练，不能据正常释放通过宣称该故障恢复路径物理通过，也不构成正式资源资格。


## 显式部署串口端点

公开模板中的`DEMO00000001`是脱敏默认值，不是现场设备。部署可选`--endpoint 'serial//dev/serial/by-id/<实际设备整名>#baudrate=115200'`，或Python `deploy(..., endpoint=...)`；不传时维持原模板，原4参数API不变。替换发生在render之后、配置和manifest摘要固定之前，仍经过现有load_panel_config原件校验。空值/空白/选项格式及重复或缺值模板旗标拒绝；工具不自动发现串口、不证明URI在线可达，URI实际通信由SDK transport负责。

协议包复制排除`__pycache__`及`*.pyc/*.pyo`，即使SDK输入已有缓存也不会把它们带入不可变release，SDK原件不改。曾有新部署继承Demo端点导致worker context_init_failed、无State且没有运动；这属于配置封装缺口，不能把NoMotion写成参考/REST通过。测试在临时目录实际部署fixture、带缓存输入和真实格式端点，验证完整manifest摘要，未打开设备。


## run16固定夹具与碰撞请求对照

圆柱目标锚定部署模型`rest_`，避免实测原点的微小正负偏移反转目标方向。GetScene读回按有效`object.pose * primitive_pose`比较：全局位置可能保存在object.pose而primitive为局部单位位姿。真实GetMotionPlan冻结同一start/target请求，严格A/B/A验证clear成功、同ID圆柱FCL contacts且wire错误99999拒绝、移除后原请求恢复成功；`CYLINDER_ABA_CONFIRMED`才记录闭环完成。内部MoveIt错误枚举不能替代线上99999。


## 薄面板positions-only首命令槽

薄面板接口01另发现positions-only首槽问题，不推翻run16 Node全导数基线：Node完整位置/速度/加速度轨迹t=0可作为初始状态；GUI完整positions-only路径必须保留全部点并把所有时间统一加一个控制周期，第一真实命令槽为20ms，不能用duration=0的首点与实测prime计算有限差分。该路径实际峰值速度0.01536rad/s、加速度0.09549rad/s²原本合规，不应放宽限值或填假导数。修复还需Begin前同等实测准入预检，非法轨迹先拒绝，避免Begin后Cancel撞busy的副作用；最终薄接口实机重测已由panel06完整通过。此次异常由固件自动Disable全63、terminal FaultLatchedReleased4收尾，同boot ref1保留；不是Host D10成功。


## 历史只读准入、高水位与外部释放

历史已清故障只读baseline为manual专用窄分支：0x1067、0x1064及当前0x1024；必须clean_idle、coldStart或同历史boot/run/ref/链、0 pages与0 terminalWritten，当前first failure清零、双Idle/session0/无permit、正式releaseCommitted且无丢证。原Fault4/source不改写成成功，仅用于只读历史准入，不能授予运动；保留参考后17/18/19资格完整重做。该白名单不是产品窄恢复工具Clear允许表扩展。

ROS命令编号高水位沿Native规则分配，当前编号、真实accepted与last-assigned取后继最大值；旧44使新命令分配45，旧44不能冒充45的ACK。仅同boot/epoch且新鲜的真实观测可抬floor，错身份、失鲜及溢出均拒绝；伪TTY正负例通过。panel03实际停在EnablingHold/core3，Node阶段名Hold不代表actual core4；设备State仍连续，停更的是qualified缓存。后续panel04已实际达到core4，但不能称清零后又实机重现旧44故障。

外部D10已完成时，Node直接poll原bootstrap_status权威终态，不再重复提交D10；原fresh双流、同ref、原页链与20秒绝对收尾截止保留，正负定向测试通过。已证正式释放与后续重复D10导致的软件UNCONFIRMED循环必须分开，不能用重复服务失败推翻实际释放事实。

## panel04验证器边界与最终覆盖范围

panel04私有验证器SceneHash误报经独立MoveItOnly只读实验确认：GetPlanningScene即使只请求attached objects仍带live robot joint positions，joint-state更新会改变整条消息hash，而world/attachments/ACM/transforms完全不变。外部/tmp harness仅剔除robot_state内joint/multi_dof动态值，保留attachments、is_diff及其他全部scene字段，实际初态仍单独受fresh姿态guard约束。这是验证器修正，不是产品障碍功能改动；软件原件whole hash不同、投影相同已assert。

薄接口真实运动/自动释放由panel06通过；官方Qt只读显示检查单列，headless通信/运动实测与Qt显示不能互相代替，初始双限位问题仍默认回答No。


## 最终薄面板接口与有界释放

panel-interface-06真实薄面板接口完整通过：25点positions-only完整轨迹、0.48秒、J1+0.20°，公开FJT成功并有独立fresh arrival，随后生产Backend自动release→RELEASED_IDLE→OWNED_CHAIN_STOPPED/INTEGRATION_PASSED。同boot/ref1保留，无重新采参考。

06冻结raw SHA256 eb65fe8b2f8fd6841eee891b49b4c6189c31a00f15c530da368f00efefbd4fd9，chain_errors=[]；run104 motionComplete final6/releasefalse，随后run105 final2/releaseCommitted/noLost、Disable SENT63 failed0。Complete本身不等于释放；SENT不等于驱动器独立失能readback。

panel05原始完整轨迹也成功，但Python Backend首次UNAVAILABLE未重试；Root随后通过原owner服务主动D10成功。05冻结SHA256 4ccdd58e9b6010712aff1ae313407c005ee20146d844a00569c4393b253ad6b6，run102 final6/run103 final2、Disable SENT63。不能将事后主动释放记成当时Backend自动释放成功。

生产Python Backend已增加固定20秒绝对释放期限与200ms重试：UNAVAILABLE才重试，QUEUED后仅poll原终态不重发D10，重复点击不延长截止，超时不关闭owner。7/7定向测试及全文件35 pass/1 skip；panel06随后实机验证自动完整收尾。


## 已知退出限制

安装版MoveIt在SIGINT退出时多次出现-11（panel01–06亦见），这是保留的第三方退出限制；控制节点正常结束、真实D10释放成功，不据此声称vendor问题已修复。


## 官方Qt拓扑修复与最终只读验证

官方Qt实际窗口已可见且初始双限位问题自动回答No，但父显示RSP与子控制RSP共用/robot_description产生竞争，控制Gate启动失败；该次失败保留；后续topic隔离与统一文案修正完成，最终只读连接已实测通过。panel06 headless完整接口实机PASS仍成立，与Qt拓扑失败分开；不因此重新采参考或冒充双限位确认。


最终官方Qt launch已实测连接成功：窗口visible=true、backend=ros、runtime=ReadOnly、manual_ready=false；初始双限位问题明确回答No，没有冒充已到双限位或再使能。独立ROS订阅确认同boot新鲜双Idle2、session0、permit0、ref1，真实控制链已接通。该项证明Qt只读显示/连接，不替代panel06 headless完整运动及自动释放证据，也不声称Qt全部点击运动已实测。

控制描述隔离为/manual_control/robot_description；控制RSP始终创建并独名manual_control_state_publisher，Gate/CM使用同一remap，与父显示RSP分开。7/7 launch定向测试通过。三项下拉及连接状态标签统一为“真机-USB/Native”“真机-USB/ROS”“仿真-本地”。

公共SDK独立目录整理是后续独立事项；本轮保留现有task11 SDK安装位置，不迁移目录，不扩大已冻结的公共策略/协议/固件范围。


## 后续真实Qt人工执行失败

随后用户真实Qt人工操作已到READY，但点击执行时MoveIt重复服务数量2触发ValueError，UI slot未捕获导致abort；当前人工执行失败，不能声称Qt全部人工运动测试通过。Root接手主动失能与重复来源排查，UI异常保护修复中。此前Qt只读连接成功及panel06 headless完整实机PASS分别保留，不覆盖本次失败；最终HTML须待UI源码和实际结果重新冻结后重建。


## Qt orphan根因与最终生命周期保护

用户Qt实际prepare已到actualHold，点击start_move时重复MoveIt服务触发ValueError；重复来源已证为此前headless06残留orphan MoveIt与当前GUI parent并存，不是修复后的parent/child描述topic冲突。旧PanelRuntime只terminate launch父进程，不能保证children退出。Root先通过原owner Gate主动D10并独立核fresh双Idle/session0/permit0/ref1，再清理原parent与已归属orphan；未改固件、未重采参考。

最终Runtime默认start_new_session并记录owned PGID，仅在released已确认后向自身进程组优雅SIGINT，共享6秒截止等待parent与活children退出，不影响独立external；超时保留Error与owner，不强杀。8/8真实进程组生命周期测试通过。UI同步FK/IK/plan及异步preset FK图异常均显示错误并回滚，不退出、不发运动；clicked(bool)包装回归及18个旧fixture补backend=real后，页面75/75通过。该软件回归不提升为全部Qt人工运动已实测。


## 历史GUI4–7：用户重启与当时不可测试边界

后续GUI4操作暴露旧GUI3崩溃遗留的child launch/Gate未随parent退出，与新GUI4 child/Gate并存，导致CM冲突且prepare可能送到旧链。Root按已证明归属准确SIGINT旧child、新child与新GUI parent，未使用killall；新PGID保护不能追溯接管旧版本遗留实例，首次仍须明确清理。当前不能称Qt可直接测试。用户随后主动重启机械臂并确认双限位，先前boot/ref1不再可复用；后续仅唯一控制链按新boot与本次明确确认重新标定，实际结果待提供。旧run16/panel06及Qt只读证据仅属于旧boot历史，不代表重启后的当前状态。

此前生成HTML仅是旧冻结点快照，最终发布暂停，待新boot单链事实与源码再次冻结。


## 历史GUI7快照：当时新boot单链只读

最终owner防误绑定保护已安装，Backend软件37 pass/1 skip、Qt针对性5 pass。GUI5/6拒绝启动是正确保护：旧run06还残留baseline orphan（并非Gate/MoveIt）；Root按owned-chain日志证明归属后准确SIGINT清理。GUI7随后真正唯一控制链连接成功，实际visible=true、backend=ros、runtime=ReadOnly、error与graph为空。用户主动重启后的新boot为0x1858026637c57a9d、reference=0，本次双限位明确确认true已传入，但连接本身不使能；等待用户本次使能/恢复零点与人工标定，不自动抢操作，不声称新boot标定或运动已实测。旧run16基线与panel06完整接口实机PASS仍成立，但仅属旧boot历史。

上述当前状态取代先前按时间保留的“当前不可测试/待冻结”表述；不覆盖历史失败。最终文档按此事实冻结，无需等待或代替用户后续人工操作。


## GUI8新boot首次标定与零目标释放边界

GUI8已在新boot1754154692906416797从reference0、真实双限位确认出发，经生产C++ capture16、完整17–19、REST与Hold约17秒成功建立reference1；不是仍在等待用户首次标定。随后生产MoveIt start_joint_plan的J1+0.2°只读规划成功（4路点、6碰撞样本），尚无FJT执行。后续同boot/ref1可复用参考，不需重新摆双限位、重烧或重新采零。

reference0首次建立参考应使用“双限位标定→Rest”；“使能/恢复零点”只用于已有同boot参考的恢复。此前Root曾指导错误按钮，已明确纠正；不得把已有参考恢复与首次采参考混为一谈。

新发现只prepare/只读规划、0 goal后的释放，Gate错误向GoalTimeline写kRelease导致stickyIncomplete。原始raw202组chain_errors=[]、原D10 COMMITTED、六轴Disable SENT63/failed0及新鲜双Idle真实成立；不能把该软件账本错误写成设备未失能，也不能据原始释放事实忽略sticky错误。这是GUI8当时的失败快照；后续最小RED/GREEN修复及GUI11 prepare→plan-only→D10→重连已完成，不代表当前仍在等待源码/HTML冻结。


## 历史GUI11真实释放与重连交接

最终GUI11实机闭环通过：同boot1754154692906416797/ref1、operator_confirmed=false复用已有参考且不新capture；PREPARE_SUBMITTED→REAL_HOLD，生产MoveIt规划J1+0.2°成功（4点、6碰撞样本），本次无FJT。随后AUTHORITY_RELEASED，重连达到USER_HANDOFF_DISABLED_REFERENCE_READY，实际ReadOnly/ref1、resumeEnabled=true、graph为空，已交用户测试。此轮证明prepare/plan-only/真实释放/重连，不替代旧panel06完整FJT执行及到位证据。

零目标释放错误写GoalTimeline kRelease的最小修复及定向RED/GREEN已完成，GUI8原失败保留；GUI11同boot/ref1 prepare→只读规划→D10→重连真实通过。首次ref0必须“双限位标定→Rest”，已有同boot/ref恢复才用“使能/恢复零点”；此次无需重新摆限位、烧录或采零。

该最终状态更新先前按时间保留的待测表述，不改写GUI8与其他失败原件。


## 用户HOME运动0x1008与固定逻辑周期修复草稿

用户GUI HOME实际运动失败首因已闭环为0x1008导数拒绝，不是随后session lost/Cancel/STOP_FAILED。匹配固件ELF的objcopy SHA与实际eae映像一致；运行态只读快照明确J3/stage3、producer1731/前1730、固定dt20ms，速度由-0.03908634622120655变为-0.12039255250244629 rad/s，计算加速度4.065310314061987 rad/s²超过cap4。原终页Failure保留，不因恢复改成motionSucceeded。

真实已安装JTC4.42.1配CM4.48、原轨迹前4samples及实际period19.582279ms/20.384057ms的fixture，精确重放三个double位置与超限加速度并先RED。CM内部period为wall-clock，JTC原直接使用实际period推进采样phase，与REF按seq差固定20ms计算导数不一致；小幅低速0.2°通过不能证明贴近4rad/s²的大跨度路径有相同裕量。

最小修正将基类轨迹推进改为已配置update_period_，真实time和外层deadline保持，miss/逆跳拒绝sink，不放宽速度/加速度或固件规则。真实JTC GREEN/install及后续joint/末端Home/Rest四case实机结果待Root提供；当前不生成最终HTML。

ROS历史0x1008仅在显式ClearFault后当前first_failure_reason0、core2/session0/permit0、cleanup3/正式释放/noLost及同boot/ref/原链/水位严格满足时接纳只读baseline；Fault4/source4104保留，不自动清错、不伪成功、不绕过17–19资格。旧实现红测、修后17/17针对性回归与control构建/install通过。


## 历史GUI14–16修复与低速验证边界

GUI14低速Home实测到位，但不能称业务通过：Complete消费permit后C++自动恢复处于core3时，Qt沿用Native通用in-flight/not-authorized门误Cancel，外部harness又把executing=false误认为正式终态并二次STOP，后续产生0x1024历史run18。修复为ROS accepted in-flight等待同UUID正式C++ Action结果，保留fresh boot/ref与原deadline；正常permit0/core3→7→4继续等，身份变化或超时仍取消，Native原逻辑不改。completed后清旧path不覆盖成功。真实Qt两轮77项RED/GREEN通过；harness也改为正式结果+fresh actualHold，不按executing=false或瞬时到位判通过，错误不自行重复STOP。

2°/s、0.1rad/s²仅是此前低速基础回归条件，不是交付限制；用户已拒绝该降限，临时ROS cap已撤回。最终须以默认60°/s、4rad/s²分别实测四leg；部署更低上限仍取更低，不修改硬件/固件caps或容差。低速通过不能代替默认动态实测，Native/本地仿真原语义不改。

GUI15暴露普通安全区已有同boot参考恢复不应无条件再17：Native此时直接SetPermit→EnableHold，ROS原无条件BeginQualification被REF拒。最终coordinator复用SDK policy guard分流：首次无参考16流程不变；已有参考仅危险贴边才走17/18/19，普通安全区按原参考直接SetPermit→EnableHold。不改SDK或FW、不重新采零；定向RED/GREEN和安装已完成。

GUI16普通Home/ref1恢复约1.5秒达actualHold，未发17；低速setupRest完整2318命令/46.34秒及随后四leg均已有正式Action SUCCEEDED。末端Rest等效腕角导致外部nominal关节断言失败，不能记为默认高速验收；最终D10及fresh双Idle已独立确认。


## 实际RViz动态TF与控制器激活握手修复

GUI16实际RobotModel红色错误已只读定位为缺实测动态TF：joint_state_broadcaster处于inactive而arm_controller active，/joint_states虽存在publisher但无样本，六轴link动态TF全缺；实际URDF灰色，非目标预览红或碰撞材质。启动时JSB仍unconfigured，Node BEST_EFFORT双激活仅凭ok推进，掩盖了JSB失败。旧RSP已由Root按日志归属准确关闭。

Node现先ListControllers等待两者配置完成，在同一次STRICT事务激活尚inactive集合，随后复核JSB与arm均active，才EnableHold及moving_ready/MANUAL_CONTROL_READY；已active者不重复激活。轮询不重开原step/total截止。真实Node旧binary晚配置fixture先RED，late配置arm inactive/active、一直未配置、switch ok但JSB未active及旧收尾回归8/8 GREEN；已仅增量构建安装Node，不改变TF/topic设计。

GUI16低速2°/s、0.1rad/s²的setupRest、joint HOME/REST、末端XYZ/RPY→IK HOME/REST共5次实际FJT均正式SUCCEEDED；末端Rest选择等效腕角约±118°，外部按nominal关节角比较失败，不是Action失败。末端等效解应按真实FK的Cartesian pose比较，不能要求同一关节解；该轮仍不能充当默认60°/s、4rad/s²验收。Root已D10并核新鲜双Idle/session0/permit0/ref1；下一轮默认动态以命名预设具体六角做四leg并独立验证。

## 历史GUI17释放跨流修复与GUI21复验边界

GUI17实测已验证六轴非零joint_states及实测RobotModel动态TF齐全，JSB配置/STRICT激活修复有效。该轮默认动态setupRest在发任何FJT前被真实起点自碰撞拒绝，不能记为高速运动通过。随后原D10已COMMITTED，独立raw双Idle2/session0/permit0/ref1、完整释放终页均成立；Host却误报RELEASE_UNCONFIRMED。根因是D10后Disabling Status仍携原owner/ref1，Idle State先到且session0/ref1；Status合法约500ms周期，511378µs后才到Idle Status。旧有限过渡helper仅接受bench 7→1/ref0，未识别manual 8→2/ref1，释放路径又以250ms检查Status，导致下一Status前579µs先按连续性失效，Bootstrap进入Fault，后续IDENTITY_CHANGED掩盖最早原因。

最小修复只在manual、明确expectedReference非零、原D10绑定及stage2启用保留参考的8→2有限等待：boot/原owner/reference相同、motion与management均恰+1，State session0且同ref，Status使用SDK独立1秒预算，State仍250ms。原5秒意图截止、最终fresh双Idle验收、普通RUN/Fault/身份/hash门均不放宽，bench默认7→1/ref0不变；这不是把诊断cache变成控制授权。真实510799µs时序先RED，身份/epoch/过期/Fault负例及相关释放回归7/7 GREEN，Gate已增量安装；当前物理释放证据与修复后下一次整链复验分开记录，此GUI17当时未有高速四leg通过证据；后续GUI21的四项真实通过另行记录，不反推该失败为通过。

## 正常Stop后的DDS退订等待（现场无运动复测通过）

正常owned Stop后，进程已退出与DDS图端点全部消退不是同一个时刻。旧预检在0.5秒发现期一到就锁存仍可见的旧端点，因而误拒随后重连。修复仅调整RosTransport.assert_no_existing_owner的有界等待：空图仍保留原0.5秒发现期；若该时刻仍有旧endpoint，最多等到本次调用开始即固定的2秒截止，只有实际服务/发布者counts全部为0才解除冲突并返回。若截止仍有任何真实owner端点则严格拒绝；点击不延长截止，也不接管、杀进程、发管理请求或自动使能。循环只等DDS独立更新，不spin ROS回调或重入Qt业务。

纯Python隔离测试先RED后GREEN，40 pass/1 skip，覆盖空图发现期、700ms后退订、持续真实owner拒绝及无回调重入。此变更仅Backend及其测试，不改Main、Native、固件或协议；GUI20真实无运动Stop/单次重连已通过，现场证据与40项软件验证分别保留。

GUI20已用新Python进程完成真实无运动Stop/重连复测：方法签名为(self, timeout=2.0)，确认加载本次Backend；UI Stop收到RELEASED_IDLE后只点击一次Reconnect，ReadOnly→Starting→ReadOnly，error与graph_error均为空，约4.21秒完成（包含新进程启动）。旧自有进程组及其记录成员均退出，新组只读单链建立；同boot/ref1、session0/core2，界面速度60°/s与加速度4rad/s²未改，实际RobotModel隐藏且Python joint-state publisher不存在。本次没有使能、FJT或运动，故只证明无运动收尾与重连，不覆盖默认高速四项，也不是enabled的8→2保留参考释放修复实机复放。
