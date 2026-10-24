# 冻结生成codec的中文阅读指南

这里的 `.dox` 把说明挂到11个保留原头的真实符号上，原件不插注释、不重新生成。修改这些原字节会影响payload/fixture身份，所以外部说明与源码身份分开保存。[source-map.json](source-map.json)列出每个原件的完整SHA与文档映射；它只是文档索引，不是新的产品manifest、可信pin或授权输入。

先选下表的一套Doxyfile，在ROS仓库根运行 `doxygen docs/dummy/generated-codecs/Doxyfile.<family>`。四套INPUT互相隔离；不要把所有v2目录同时传给一次Doxygen。同名 `dp::protocol::dummy_robot_v2` 在不同minor/fixture里不代表同一物理文件；同时输入会让字段/方法说明错误地合并。配置只提取XML，不运行编译器、产品生成器或测试。

| family | 原件范围 | 符号说明 | 实际提取的函数 /变量 /类结构 /宏 |
| --- | --- | --- | --- |
| [v2-current](Doxyfile.v2-current) | 当前正式v2.5三个codec头 | [v25-symbols.dox](v25-symbols.dox) | 414 / 242 / 19 / 4 |
| [application-fixture](Doxyfile.application-fixture) | application_wire_source三个头 | 复用字节相同的v25说明 | 414 / 242 / 19 / 4 |
| [trusted-sil-fixture](Doxyfile.trusted-sil-fixture) | trusted_sil_bundle三个头 | 复用字节相同的v25说明 | 414 / 242 / 19 / 4 |
| [fault146](Doxyfile.fault146) | 两个历史故障数据头 | [fault146-symbols.dox](fault146-symbols.dox) | 0 / 11 / 0 / 0 |

输出位于 `/tmp/dummy-doxygen-audit/generated-codecs18m/<family>/xml`。统计来自Doxygen 1.9.8：函数/变量必须在原头的location上有brief，不能只在sidecar生成一段导航文字。每个配置还与EXTRACT_ALL解析的原符号集合对照；宏、类型别名、枚举类型的说明也在相应符号中。数组元素不是独立函数，故障数据头按数组/时间常量说明。

旧V1/C03及旧2.4完整合同已移除，逐符号页面仅在Git历史 `c76b1eb06924010daa735288066f9c43a26ed6d1` 中保留。2.4 codec/layout迁至 [内部兼容资源](../../../compatibility/wire2_4/README.md)，只供旧故障回放与兼容处理，不作为当前合同入口。下文2.4尺寸只用于内部兼容资源对照。

## 从值对象读到wire字节

`Dummy…` 类拥有固定标量和数组，默认全部零。默认对象不会自动填schema；必须显式设置major/minor和业务字段。标量getter返回拷贝；数组getter返回内部const数组引用，借用到对象销毁为止，后续setter/对象赋值会改变所见数据。setter对数组做完整memcpy，返回后不保留输入数组；setter不检查限位、NaN、mask、owner或许可。同一对象并发读写需要外部同步。

`…Codec::encode`要求非空输出且size恰为kEncodedSize。它逐字段取值，以memcpy取整数/浮点位模式，再用put_le8/16/32/64按小端写字节。大小正确不意味着缓冲实际有那么多可写内存，caller仍负责有效地址和生存期；函数不保存缓冲、不发送网络或CAN。除v2 Evidence外，这些encoder没有对schema/业务枚举作与decoder对称的校验，因此可能成功编码出另一版本或非有限数值。

`…Codec::decode`先查非空指针和精确长度，在局部零初始化对象中逐字段读入。bool字节须0/1，末尾按消息检查精确schema及部分枚举。全部通过才执行 `*message=decoded`，拒绝不会提交局部结果，也不让输出对象借用wire内存。成功对象拥有自己的数组拷贝。

codec返回kOk只表示这些格式门通过：没有认证来源、核当前owner/epoch/permit、检查位置有限性/工作限位、计算轨迹碰撞、核State新鲜度或等待CAN ACK。各消息的实际枚举检查写在对应Codec类型与encode/decode说明里；不能把某类的约束推广到另一类。例如v2 Status decoder没有统一加core_state范围判断，而State decoder会检查它。

`kEncodedSize`是wire长度；`kAlignment`、`kMaxObjectBytes`和`kMaxObjectAlignment`属于目标C++ ABI的值对象槽，可能含padding。不要用sizeof对象替代wire长度，也不要把对象结构整体memcpy到链路。`kScratchSize=0`不代表没有解码栈对象；`kMaxAge=0`不提供新鲜度监督。

## 版本与字段角色

| payload | 内部兼容2.4字节 | 当前2.5字节 |
| --- | ---: | ---: |
| Status | 651 | 651 |
| Command | 133 | 133 |
| State | 301 | 350 |
| AttemptEvidence | 372 | 372 |
| Manage Request | 233 | 233 |
| Manage Response | 132 | 132 |

2.4→2.5保持Evidence372与Status651，State在offset301追加6×8字节hold_position_rad和offset349的hold_valid_mask，301→350。所有这里的decoder都要求精确minor；2.4数组不能直接交2.5 decoder当作兼容成功。旧故障consumer若没有显式适配，也不能因fixture存在就宣称新decoder已接受它。

数组轴序J1..J6，掩码bit0=J1；motor rev与joint rad不同。position_min/max是设备声明边界，BENCH规划相对界和机械绝对界不是同一坐标。字段 `acceleration_max_rad_s` 的实际单位为rad/s²，历史名称少一个2。

Host收到State的时间、Source produced_at_us、Source CAN queued/taken/submitted/sent/abort时间分别来自自己的角色。即使都写µs，也不能直接跨域相减求单向延迟。freshness_ticket是opaque票号，不是时间；producer_seq、state_seq、request_seq、request_id、operation_id、run_id和motion_epoch分别标记不同因果层，不能互换。

Queued receipt、Accepted及Committed是不同阶段。Committed结果也须匹配原请求、operation/RUN、owner/epoch及正确终态；编解码不会完成这些绑定。operator_confirmed只记录意图，不是认证或审批。SENT只说明CAN发送证据，不是机械已到位；Release committed/Idle也不自动证明实体物理断电。

## Evidence页与冻结故障数据

v2 Evidence组页category1/2/3与终页category4有不同形状：终页组ID/producer/组mask和五个时间数组须零、final1、有效终态；特定Fault final4/5/9才可带非零合法六轴failure mask。组分支当前是sent/failed的数值<=expected检查，不是完整按位子集/全轴提交验证；组final也没有统一强制1。这些缺口不能由“decode kOk”补成更强保证。

页链工作在codec外：首seed由run(8)+boot(8)+client(16)+四摘要(128)=160字节，再补32零到192；当前链由previous(32)+canonical前缀(307)+evidence_lost(1)=340字节求SHA。previous_chain位于307..338，chain位于339..370，lost在371。codec仅搬运链字节，没有重算SHA、连续页序/组计数/全轴SENT或父RUN关系验证；setter能写任意摘要，自洽SHA也不是设备签名。

两故障头包含146×372=54312字节旧2.4 Evidence与301字节State/651字节Status。页seq0..142为普通组（motion1后转2），143为Safety、144为Disable（motion3、来源0x1003、轴63），145终页使用birth motion1、final4、组总数145。page_when是原Host收页时间，fault_state_when/fault_status_when是后续各自收包时间。页本身不带boot/session；重放consumer另给boot3498049537/session71，并用client/四摘要建立链seed。

这些常量可以保持一次历史故障的精确刺激，支持collector/清理关联的重放与负例。但它们不是新现场采集，不是完整ROS会话/管理回执文件，也不证明真实电机Stop、当前固件或新机械资格。firmware d8标记与旧合同摘要都应按历史fixture解释。数组元素只需原字节与字段解释，不应虚构每个scalar的函数API。

## dp_comm特化与容量

MessageCodec特化只把raw codec结果转bool，并给类型interface IDs；服务描述提供Request/Response别名。当前V2消息4137/4138/4139、服务4140、请求/响应4141/4142、Evidence4143。它们不是CAN节点号或已启动的ROS服务。

kStaticRequirements按SDK实际字段顺序声明：nodes1、publishers3、subscriptions1、timers0、event_queue8、max_message_bytes651、max_topic_bytes96、scratch0、对象alignment，以及services/clients/pending/completed各1。宏头只有对应DP_COMM_RUNTIME_MAX_*宏已定义时才在容量不足时报编译错误，不自动配storage。当前v2 metadata另写message_bytes672，是独立元数据预算；不要把它当本头实际kEncodedSize，或擅改原manifest“统一”两者。

## 原件到外部文档的完整映射

SHA按原文件全部字节计算。保留的v2.5 codec及适配/资源头的三处副本均按SHA确认相同后复用说明；不同manifest聚合（正式2ae31387…、application1d5d4cd6…、SIL0afb58dd…）仍各自保留，不能因codec相同就替代bundle信任身份。

| 原件 | 原SHA-256 | family / 文件映射sidecar |
| --- | --- | --- |
| [contracts/dummy_robot_v2/codec/dp_dummy_robot_v2.hpp](../../../contracts/dummy_robot_v2/codec/dp_dummy_robot_v2.hpp) | `649ebc7fc37ec7b85d9eaa116c8174438fac236e592da68350a9d3ae599a9143` | v2-current / [v2-current-files.dox](v2-current-files.dox) |
| [contracts/dummy_robot_v2/codec/dp_dummy_robot_v2_dp_comm.hpp](../../../contracts/dummy_robot_v2/codec/dp_dummy_robot_v2_dp_comm.hpp) | `78d2fc3f6cc9cd3eef1e32f989434cb5fc284231593307f63e4499f48c219c40` | v2-current / [v2-current-files.dox](v2-current-files.dox) |
| [contracts/dummy_robot_v2/codec/dp_dummy_robot_v2_static_requirements.h](../../../contracts/dummy_robot_v2/codec/dp_dummy_robot_v2_static_requirements.h) | `c24ee5036d82b3925de01b79d229af9dc4a892d2c68a62bb77186a5f0f391352` | v2-current / [v2-current-files.dox](v2-current-files.dox) |
| [src/dummy_motion_client/test/fixtures/application_wire_source/codec/dp_dummy_robot_v2.hpp](../../../src/dummy_motion_client/test/fixtures/application_wire_source/codec/dp_dummy_robot_v2.hpp) | `649ebc7fc37ec7b85d9eaa116c8174438fac236e592da68350a9d3ae599a9143` | application-fixture / [application-fixture-files.dox](application-fixture-files.dox) |
| [src/dummy_motion_client/test/fixtures/application_wire_source/codec/dp_dummy_robot_v2_dp_comm.hpp](../../../src/dummy_motion_client/test/fixtures/application_wire_source/codec/dp_dummy_robot_v2_dp_comm.hpp) | `78d2fc3f6cc9cd3eef1e32f989434cb5fc284231593307f63e4499f48c219c40` | application-fixture / [application-fixture-files.dox](application-fixture-files.dox) |
| [src/dummy_motion_client/test/fixtures/application_wire_source/codec/dp_dummy_robot_v2_static_requirements.h](../../../src/dummy_motion_client/test/fixtures/application_wire_source/codec/dp_dummy_robot_v2_static_requirements.h) | `c24ee5036d82b3925de01b79d229af9dc4a892d2c68a62bb77186a5f0f391352` | application-fixture / [application-fixture-files.dox](application-fixture-files.dox) |
| [src/dummy_motion_client/test/fixtures/trusted_sil_bundle/codec/dp_dummy_robot_v2.hpp](../../../src/dummy_motion_client/test/fixtures/trusted_sil_bundle/codec/dp_dummy_robot_v2.hpp) | `649ebc7fc37ec7b85d9eaa116c8174438fac236e592da68350a9d3ae599a9143` | trusted-sil-fixture / [trusted-sil-fixture-files.dox](trusted-sil-fixture-files.dox) |
| [src/dummy_motion_client/test/fixtures/trusted_sil_bundle/codec/dp_dummy_robot_v2_dp_comm.hpp](../../../src/dummy_motion_client/test/fixtures/trusted_sil_bundle/codec/dp_dummy_robot_v2_dp_comm.hpp) | `78d2fc3f6cc9cd3eef1e32f989434cb5fc284231593307f63e4499f48c219c40` | trusted-sil-fixture / [trusted-sil-fixture-files.dox](trusted-sil-fixture-files.dox) |
| [src/dummy_motion_client/test/fixtures/trusted_sil_bundle/codec/dp_dummy_robot_v2_static_requirements.h](../../../src/dummy_motion_client/test/fixtures/trusted_sil_bundle/codec/dp_dummy_robot_v2_static_requirements.h) | `c24ee5036d82b3925de01b79d229af9dc4a892d2c68a62bb77186a5f0f391352` | trusted-sil-fixture / [trusted-sil-fixture-files.dox](trusted-sil-fixture-files.dox) |
| [src/dummy_motion_client/test/fixtures/ros05_fault146/original-once-decoded-146-payloads-fixture.hpp](../../../src/dummy_motion_client/test/fixtures/ros05_fault146/original-once-decoded-146-payloads-fixture.hpp) | `e8cdcb29921ab38e09b980828972436c7135459ebcaaf3fc442d0021a2241b12` | fault146 / [fault146-files.dox](fault146-files.dox) |
| [src/dummy_motion_client/test/fixtures/ros05_fault146/original-once-decoded-current-fault-observations.hpp](../../../src/dummy_motion_client/test/fixtures/ros05_fault146/original-once-decoded-current-fault-observations.hpp) | `3e7c0ff5a3c32cb489039097068446a661e16bc432a15e95c16ee99a045b4a1f` | fault146 / [fault146-files.dox](fault146-files.dox) |

本目录不含C03 prototype族；也不更改生成器、manifest、payload、SDK合同或测试fixture。需要更新这些身份时，应走各自正式发布/生成流程，不能通过改阅读说明改变运行保证。
