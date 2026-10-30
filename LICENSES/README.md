# 许可与第三方归属

根目录 [LICENSE](../LICENSE) 适用于本项目有权以 Apache-2.0 发布的原创贡献。
第三方源码、模型和依赖保留各自的许可；目录位置、转换格式或新增注释不会使其自动改用根许可。
[NOTICE](../NOTICE) 提供归属摘要，本目录保存已核实的原文与模型来源证据。

| 文本 | 对应范围 |
| --- | --- |
| [PickNik-2020-BSD-3-Clause.txt](PickNik-2020-BSD-3-Clause.txt) | `servo_cpp_interface_demo.cpp` 中逐字保留的 PickNik Inc. 2020 BSD 三条款声明 |
| [PickNik-2021-BSD-3-Clause.txt](PickNik-2021-BSD-3-Clause.txt) | `servo_keyboard_input.cpp` 中逐字保留的 PickNik LLC 2021 BSD 三条款声明 |
| [Apache-2.0.txt](Apache-2.0.txt) | OSRF 三份 Python 测试模板的原有 Apache-2.0 许可，版权年份及文件对应见 NOTICE |
| [GPL-3.0.txt](GPL-3.0.txt) | switchpi/Dummy 上游仓库随附的 GPL v3 原文；本仓库中已匹配的旧版七份 STL 及证据见下文 |
| [MODEL-PROVENANCE.md](MODEL-PROVENANCE.md) | 随附完整 STEP 派生网格、MINIF08 的 Apache-2.0 范围及旧版七份 GPL STL 的来源 |

PickNik 两份文本直接复制各源文件中的完整许可注释块，包含原有注释标记。
源码中的版权、许可、作者及适配说明均须继续保留。
MoveIt 教程的对应上游源码可见
[C++ 接口示例](https://github.com/moveit/moveit2_tutorials/blob/humble/doc/examples/realtime_servo/src/servo_cpp_interface_demo.cpp)
和[键盘示例](https://github.com/moveit/moveit2_tutorials/blob/humble/doc/examples/realtime_servo/src/servo_keyboard_input.cpp)；
这里保留本仓库文件中已有的声明，不将上游后续分支内容视为本仓库的完整字节来源证明。
Apache-2.0 的权威文本见 [Apache Software Foundation](https://www.apache.org/licenses/LICENSE-2.0)。

本目录不是 ROS、MoveIt、Gazebo、Qt、DPSDK 等独立依赖的完整许可清单；
安装或再分发这些依赖时，仍须保留其各自发行包和源码中的许可与声明。
模型来源记录中的不确定项不会因本目录提供某一许可证全文而自动获得该许可。

完整装配的23份派生网格与 MINIF08 近似网格随仓库提供，采用 Apache-2.0；
旧版七份 STL 继续采用 GPL-3.0。模型目录中的 `LICENSE` 保存对应范围和完整许可文本，
与网格一同安装。正常克隆和构建无需另行准备模型文件；原始 STEP 仅在重新转换时需要。

## 离线源码阅读指南

`docs/dummy/source-reading-guide.html` 的压缩资料包内含部分 DPSDK 及其依赖的源码快照。
源码保留自身许可头；不能把这些片段统一解释为本仓库原创 Apache-2.0 代码。

| 材料范围 | 许可与原文 |
| --- | --- |
| DPSDK Core 的五份 OSAL 接口/契约文件 | MIT；[原许可](DPSDK-Core-MIT.txt) |
| 四份带 MIT 标识的通信源码/接口快照 | MIT；[许可正文](MIT.txt)，逐文件范围见根 NOTICE |
| FreeRTOS ARM_CM4F `portmacro.h` | MIT；保留 Amazon 版权及完整头，[组件许可](FreeRTOS-Kernel-MIT.txt) |
| zenoh-pico 四份源码及三份补丁上下文 | 保留双许可原头，本次选择 Apache-2.0；[原 NOTICE](Zenoh-Pico-NOTICE.md)、[Apache 原文](Apache-2.0.txt) |

页内附有上述补充文本，脱离仓库阅读时仍可查阅。旧内部工作记录不封装进公开阅读指南；
维护工作区原件及清理前 Git 备份另行保留，不属于公开运行资源。
