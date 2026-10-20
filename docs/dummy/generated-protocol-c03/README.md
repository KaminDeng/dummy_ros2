# C03冻结生成协议的中文阅读指南

本目录为25个原头提供外部Doxygen说明。原件15554行保持全部原始字节，未重生成、未插注释。[source-map.json](source-map.json)逐件记录完整SHA、原行数、family和符号sidecar；它是文档索引，不是产品manifest、可信pin、许可或新协议发布。基线固定于 `65196124a6846846f2fe86001e51b94db5cd2633`；后续HEAD变化不改变这组原件身份。

只有dummy v1 raw的3个头与上一批generated-codecs原件SHA相同，因此本地复用相应逐符号说明；模型头不在此范围，也没有带入那两个模型函数。其它头逐一按真实实现解释，名字相似不代表ABI、wire格式、检查或权限相同。

## 选择隔离的文档入口

在ROS仓库根运行 `doxygen docs/dummy/generated-protocol-c03/Doxyfile.<family>`。配置只静态解析并输出XML，不调用编译器、生成器、测试、网络或设备。需要Doxygen 1.9.8的行为基准；输出在 `/tmp/dummy-doxygen-audit/generated-protocol-c03/<family>-documented/xml`。

```sh
mkdir -p /tmp/dummy-doxygen-audit/generated-protocol-c03
doxygen docs/dummy/generated-protocol-c03/Doxyfile.v1
```

上例先创建Doxygen输出父目录；如需其它family，再替换Doxyfile后缀。

| family | 原件数 | 类型 | 函数 | 变量/字段 | 宏 | 别名 | 有名枚举 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| [dummy-v1](Doxyfile.dummy-v1) | 3 | 16 | 378 | 215 | 4 | 2 | 1 |
| [zenbedded](Doxyfile.zenbedded) | 3 | 9 | 38 | 43 | 22 | 0 | 2 |
| [portable-tlv](Doxyfile.portable-tlv) | 3 | 125 | 782 | 598 | 0 | 127 | 1 |
| [catalog](Doxyfile.catalog) | 7 | 2 | 15 | 214 | 1 | 1 | 0 |
| [lite-wire](Doxyfile.lite-wire) | 2 | 12 | 25 | 60 | 48 | 0 | 7 |
| [compat](Doxyfile.compat) | 7 | 44 | 222 | 253 | 91 | 3 | 4 |
| [compat-optional](Doxyfile.compat-optional) | 上述7件另一宏视图 | 44 | 222 | 257 | 89 | 3 | 4 |

前6行恰覆盖25原件，计1460函数、208类型、1383变量/字段；最后一行重读相同字节，仅将 `DP_CRASH_ENABLE_TASK_SNAPSHOT` / `DP_CRASH_ENABLE_USER_CAPTURE` 设1以检查4个可选成员。它不会改默认宏或重新生成布局。两项宏由PREDEFINED提供，因此这一视图的源码宏计数少2。两个匿名offline opcode/status枚举不能用有名类型定位，值域在compat文件sidecar的专页逐项说明，不冒造类型名。

C与C++头分语言解析；C++ catalog `.h` 用h=C++，C兼容/Lite `.h` 用h=C。packing宏仅在Doxygen解析中展开为空，以识别真实typedef struct名字；原编译器packing及所有原字节仍保留。六种格式/命名空间不能混为一次INPUT，尤其 `dp::comm::MessageCodec` 特化、服务名和同ID的raw/TLV对象。

逐函数/字段说明位于各 `*-symbols.dox`，`*-files.dox`将职责与SHA挂到原文件。所有函数、变量、类型别名和有名枚举都用实际XML确认brief与原符号一致；每套另跑EXTRACT_ALL=YES，完整符号集合与EXTRACT_ALL=NO及无sidecar原始清单一致。不是通过EXTRACT_ALL强行显示无文档函数冒充覆盖。

有两处Doxygen位置细节需要正确理解：主模板 `dp::iface::tlv::Codec` 在原头只有前置声明，没有通用函数体，XML的file会指向外部说明，bodyfile/bodystart仍准确指回portable.hpp第13行；校验明确检查该原行。raw头第163和612行重复定义同值480的 `DP_PROTOCOL_CRASH_DUMP_STACK_SNAPSHOT_OFFSET`，XML共享同一宏ID；外部说明挂在第163行的真实宏，第二个重复条目无独立brief，按一个宏身份计数。它不是两个不同偏移，也没有修改原宏消除重复。

## 四种消息表示不要互换

| 表示 | 内存/长度 | 检查与提交方式 |
| --- | --- | --- |
| `dp::protocol::dummy_robot_v1` raw | Status297/Command125/State356/Manage Req233/Resp618字节 | 精确schema1.0及各消息部分枚举/bool检查；局部decoded成功后提交，详见对应codec。 |
| `dp::protocol::zenbedded`演示raw | State16/Command8/Watchdog14字节 | 精确长度，浮点只保位；Watchdog直接写结果，后遇非法bool可部分改写。 |
| `dp::iface` portable + `tlv::Codec<T>` | 对象有固定容量和逻辑长度，wire每字段带4字节头 | scalar setter不查业务；TLV查形状/required并用独立scratch最后提交，不继承Dummy raw的精确schema门。 |
| canonical C消息+手写Protobuf/旧packed | canonical数组加独立len；packed布局另有固定offset/可选段 | C声明不自动初始化；Protobuf decoder常先清out再逐字段写，拒绝也可改变结果。 |

同一interface ID不决定编码：演示JointCommand raw是8字节，portable同4098 ID的TLV上限12字节；状态4097 raw16/TLV24；CapabilityBitmap raw41/TLV69。必须同时知道profile/layout，不能只看ID或类型后缀。文件传输canonical含transfer_id，旧兼容聚合RPC没有该字段；旧raw Open尾随名字可到120/Chunk尾随数据可到1024，而portable有界数组分别64/64，不能把其中一个预算套到另一种格式。

历史zenbedded topic固定组成 `dpsdk/test_robot/<device>/zenbedded_control/<endpoint>`，device需32个小写十六进制字符且非全零（128bit文本）。校验时会访问32字符及终止位，caller保证至少33字节可读；endpoint本身未限制斜杠/内容。通用 `dp_protocol_format_topic_*` 只做snprintf和截断判断，不核这个身份格式，也不注册任何服务。演示位置IDL只写float64，没有物理单位；不能当当前六轴joint rad。Dummy数组则按其合同解释J1..J6，关节rad、速度rad/s、原电机rev及各有效mask分别使用。

## 借用、容量与失败结果

Span/StringView仅借用原指针；size是元素数，StringView为字节数，不负责释放或延长寿命。FixedArray拥有N项并要求set输入恰N；BoundedVector最多N项；BoundedString最多N字节。set成功复制有效前缀，不清旧尾、不补NUL、不检查UTF-8或命令内容。getter的引用/视图随所属对象销毁失效，setter/赋值会改变所见内容；需要稳定快照时主动复制，同对象并发访问需外部同步。memcpy来源/目标不能随意重叠。

TLV头为小端tag16+length16。encoder先清used=0、reserve所有字段的4+payload长度，容量和本类型最大编码尺寸通过后写；会编码所有已知字段，包括optional默认值。decoder拒绝tag0、截断、已知字段重复、长度不符/容量越界和bool非法，未知tag可跳过。必填集合逐类型列在sidecar；如CrashAnnounce uploaded可缺省，CrashDump多个快照段可缺省，某些响应甚至允许空TLV返回默认对象。它不要求input总长小于本类型kMaxEncodedSize，仅限制<=65535，因此带未知扩展可更长。

TLV output与scratch须为不同有效对象，失败保持output但scratch会改变；部分重叠对象/输入内存仍须caller避免。adapter自动在栈上建完整scratch，CrashDump的sizeof(Value)较大；“不分配堆”不等于无栈成本。kScratchSize、kAlignment指对象ABI，不能拿sizeof对象代替wire字节。

旧tiny7字节getter用memcpy避非对齐访问，仍是宿主端序；raw_fixed scalar helper则显式小端。tiny try_get分别按4/6/7字节END判断完整出现，电压另查1800..3600mV。decode_view返回OK只说明至少一个字段出现，即使某字段非法仍须核present/valid；可接受部分短包或大于7字节的输入，不等精确完整合法消息。

C canonical独立len必须由caller保证不超过数组真实容量；一些encoder直接信任len，output_capacity并不能保护错误的输入读取范围。手写Protobuf writer可能先写key/length后发现容量不足，reader失败可能已消耗前缀。字符串decode留一字节NUL，但不验证内嵌NUL/UTF-8；bytes允许满容量且不补NUL。失败缓冲/对象保留语义必须看各函数，不能统一假定回滚。

## Protobuf分发与桥转换的实际范围

`dp_protocol_encode/decode`仅在EMBEDDED_NANOPB/FULL_PROTOBUF两个profile查6行手写分发表：offline命令Request/Response/Ack，以及旧文件FileTransferRpc/FileRequest/FileResponse投影。未列schema和Tiny profile返回UNSUPPORTED，不自动转到其它已存在的TLV或raw codec。void message/out必须与所选schema真实类型匹配；不存在RTTI保护。名字 `try_native_encode` 也只是转发此表，不能据此推断支持所有native格式。

offline函数名带nanopb，但这一头是手写wire0/wire2编码，不调用Nanopb运行库。命令decoder缺少必填位门，重复字段以后值覆盖，bool任何非零真，窄整数转换未统一拒绝高位溢出；unknown仅支持wire0/2，固定32/64位不在skip范围。文件oneof可出现多个变体并累计字段，最后opcode覆盖，空外层才拒绝。它们不执行command、文件IO、CRC重算、路径安全或请求关联。

bridge DeviceStatus tiny→Protobuf要求7字节，反向要求uptime/flags、voltage可缺省0，不继承1800..3600约束；电压/flags窄化超限报LOSSY。CrashAnnounce反向字段1..8必填、uploaded9可缺省false，字符串满32字节未保证NUL。tiny↔nanopb两函数为UNSUPPORTED stub，不能因其它手写nanopb函数存在就声称这两方向已实现。

旧文件raw→平面Protobuf先写opcode，再用长度识别请求/响应；Open/Chunk的逻辑OR尝试共享offset，失败不自动回滚前缀。反向解析完成后会先memset整个output_capacity，再判具体组合/容量，失败也可能清缓冲。Open请求优先、Chunk响应优先；CRC只搬声明，不核文件真实数据。部分容量错误归INVALID_INPUT或LOSSY而非统一OUTPUT_TOO_SMALL，按真实返回值处理。

## CRC、安全、时钟及资源声明

这里的CRC32字段只是旧采集/文件传输声明，所有列出codec、getter和bridge均不重算CRC；catalog指纹、SHA字段和能力bitmap也不是HMAC、签名或独立实机认证。LinkHello/Capability页decode不核catalog等于编译常量、不核页号/valid_bytes关系；KeyRotate消息不写密钥槽，LinkReset消息不reset硬件。Queued/Accepted/有效结果声明和真实设备动作不同，构造或成功编解码不能签发permit或证明physical disable。

Source uptime_ms/timestamp_ms/produced_at_us与Host收到时刻分域；freshness_ticket是opaque票号，不是时间差。20ms QoS deadline、历史100Hz/10ms控制周期、命令100ms/状态50ms年龄及SafeStop都是生成声明，codec不测时钟、不创建线程或实施策略。

Embedded4096/Full65536/Tiny128为profile最大payload声明；桥表、Lite route的encoded_size为预算，未必等实际TLV最大值，例如能力query Request表44而本TLV上限20、Response表72而上限48。路由数组不自动分配缓冲、注册服务、协商QoS或实施access授权。has_capability允许查询1..128保留位，已置位不等于18项catalog中存在，也不等功能实测。

历史 `dp_comm_static_requirements.h` 预算runtime15360/exchange256/diagnostic14/aux270、executor栈20480/control栈4096字节；节点1/发布2/订阅1/事件1/服务0。与dummy raw请求响应各1、最大消息618的需求不同。sizeof fold不核总和溢出，checked sum遇溢出返回SIZE_MAX，但合法总和恰SIZE_MAX也同返回；它们都不分配资源。旧raw crash默认stack snapshot2048，task/user可选段会改变dump总长与offset；裸user_data访问器即使可选字段关闭仍在，caller先核所选布局。

## 原件逐件SHA映射

下表SHA覆盖完整原始字节，逐符号说明按family分开；代码和任何manifest/生成器均未改。

| 原件 | 原SHA-256 | family |
| --- | --- | --- |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_comm_static_requirements.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_comm_static_requirements.h) | `43cc933d85f7588a585d1abdb5b7350c6615236809d6441086eaa98c6bc065a0` | [zenbedded](zenbedded-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_dummy_robot_v1.hpp](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_dummy_robot_v1.hpp) | `e9c04f74bd73edcf5721401c3f2c35217ed9eed528e808fea0870fa56dcc8951` | [dummy-v1](dummy-v1-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_dummy_robot_v1_dp_comm.hpp](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_dummy_robot_v1_dp_comm.hpp) | `b7959cff2853c32a4405aeff13b0ee1b4fdf1e9e5eb05f1b797a8598bb4e6743` | [dummy-v1](dummy-v1-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_dummy_robot_v1_static_requirements.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_dummy_robot_v1_static_requirements.h) | `b72871ece7fe611e8111f99deefac7d1f9f08f994470b06dad166b008c89fce0` | [dummy-v1](dummy-v1-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_lite_control_v1.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_lite_control_v1.h) | `da6f27ff8751afd3adba0c2377d253a86a66876ddd765088f9a8d28f061c6cd8` | [lite-wire](lite-wire-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_bridge_converters.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_bridge_converters.h) | `7f39178a381a4a76edfcfcdb8838621d74c33b336fa85d05c2f2e52696ee79ac` | [compat](compat-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_bridge_routes.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_bridge_routes.h) | `18c29c19a391c8782ba885c71e4cef2be777fd7b4d26ab517895567091068d17` | [compat](compat-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_capabilities.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_capabilities.h) | `41126f3bddc42584f7457a0b9dac2570dba8cbf6368d36c4188cb0adf8e28d7c` | [catalog](catalog-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_codec_types.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_codec_types.h) | `dc2a5f113005ebf5b629e3825751ee1169ffaea686b0e29701eb054bf36ff2f2` | [compat](compat-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_codecs.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_codecs.h) | `8e3ae5961abf28e58d600de09def3e1bf0527937f2e68866e177e642a5fe7f42` | [compat](compat-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_dp_comm.hpp](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_dp_comm.hpp) | `ef89826a306333d4adb28c806bf710a7a171da63d0d8b558129b3e508c86d5c7` | [portable-tlv](portable-tlv-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_legacy_payloads.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_legacy_payloads.h) | `955bad408079fab1b89933d3a5b4a40b89a122c0cd6ff7ebab23f0099ed70470` | [catalog](catalog-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_lite_routes.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_lite_routes.h) | `c060887f9cc79ea8447bce4197dc18e3dfc5d9afa701e0b26a7253dcc41012d0` | [catalog](catalog-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_lite_topics.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_lite_topics.h) | `995b57ad5dc3deb96a6438527feb6a986e98568f193c67b2aa6ca395230258d9` | [catalog](catalog-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_messages.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_messages.h) | `721d8aa36a530c6573b0d599e22461e29865eb16295512ffb068b0c5bf83bbb5` | [compat](compat-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_offline_log_codecs.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_offline_log_codecs.h) | `ab187e67263196b93875161ffe4cf9227f4225a203b3f7a299fab03579c0a789` | [compat](compat-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_portable.hpp](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_portable.hpp) | `24c28894454cd4065ddd874f7cc6122fb877bbf9aed40466469c9b880026dfb1` | [portable-tlv](portable-tlv-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_profiles.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_profiles.h) | `2bff7b185e729d999791b243882f4d77c346370ca5a327f5a587b3a50105e325` | [catalog](catalog-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_raw_fixed_payloads.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_raw_fixed_payloads.h) | `476fb2c706307755f5397949cd143be3cd1deb0ed3edac06960c336d94169de2` | [compat](compat-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_schemas.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_schemas.h) | `590b271c1b81f6e4edae898e3cc4dd0cadaa416dc4a5639dd9935e17aae2a5d0` | [catalog](catalog-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_tiny_device_status.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_tiny_device_status.h) | `08d5f6f458b2a5cff32d89d9cb2b6e5314301116c1fdf1f8916b4812a855b8e6` | [lite-wire](lite-wire-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_tlv.hpp](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_tlv.hpp) | `b28bf0864c3aaf5fe4a9d712737a84c54db0da66a8b44a86f4bcb2cb6d35a506` | [portable-tlv](portable-tlv-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_topics.h](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_protocol_topics.h) | `7fe79bdf7c86b6ca2f6302d0a8f3062ceaf21bd712418fc7f686ad1edf6b0542` | [catalog](catalog-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_zenbedded_control_v1.hpp](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_zenbedded_control_v1.hpp) | `0203d4d1c245ea1e9dcd4e074340c9cbabe54f88e50c6233ef5c522b4daea4e1` | [zenbedded](zenbedded-files.dox) |
| [contracts/dummy_robot_v1/c03_prototype/generated/include/dp_zenbedded_control_v1_dp_comm.hpp](../../../contracts/dummy_robot_v1/c03_prototype/generated/include/dp_zenbedded_control_v1_dp_comm.hpp) | `96b4f395b85dab3d9e1b775185c8221f8efdc3dda935fe6cf4ae39051b0ec3a0` | [zenbedded](zenbedded-files.dox) |

本范围是C03历史生成代码的静态说明。字节身份不变、XML覆盖和说明一致不等于这些所有路径已运行或实机合格；生成、编译、测试、fixture/probe、ROS、SDK及设备动作都应由各自明确流程单独验证。
