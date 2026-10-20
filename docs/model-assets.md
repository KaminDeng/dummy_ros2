# 模型资源与许可

克隆已包含完整运行模型，无需另行下载或准备网格。23份完整 STEP 派生资源与 MINIF08 模型
由维护者按 Apache-2.0 分发；七份旧版 STL 保留原 GPL-3.0 许可，不能用根 Apache-2.0 替换。
描述包使用这些已随仓库提供的资源进行显示、MoveIt 碰撞检查及几何回归。

源 STEP 仅在从 CAD 重新生成派生资源时需要；运行现有模型不要求准备源 CAD。
转换原理与输入身份见[转换说明](dummy/model-conversion/README.md)。

## 仓库中的运行资源

资源位于 `src/dummy-ros2_description/meshes/`。下表保持现有转换/碰撞回归所绑定的摘要；
SHA-256用于核对文件字节，不代表机械安装、碰撞安全或设备运动资格。
自行重新生成或修改资源后，须重新验证关节链、碰撞覆盖、间隙和 REST 双向规划，不能只改摘要跳过检查。

| 文件 | 当前本地转换样本SHA-256 |
| --- | --- |
| `MINIF08_approx.stl` | `779e8be3ed23ae571dd36d66719d1838f93a46148d18827195627b194564e93a` |
| `full_step_base-collision-local-clearance-5mm.stl` | `1e83390aec6dfc9c25a10c0aea3f1337991e86d6012b9e97dbf0c3f394e23d1e` |
| `full_step_base-collision-local.stl` | `2db87db875b0aaec18e091327c80c4a1d79e34877fa42d8174359c19731007d1` |
| `full_step_base-colors.mtl` | `ace2544eb8e4c342576d590d79eb3730fa0a27095711e45895b487631f03089a` |
| `full_step_base-colors.obj` | `e0ed7e89b7cefec39970e8d8cc8fec8e3cee71227c9403b2cca0b9a26e4a12ed` |
| `full_step_link1-bracket-source.stl` | `e579bfc4d06ed646e894a037980e74a4786279b27a72499a29a12f5f51ac5c4c` |
| `full_step_link1-collision-local-clearance-5mm.stl` | `c36497c4ee3b988a8d116dc1fd6766363c70036b10cce016750eee0b3bb38fc3` |
| `full_step_link1-collision-local.stl` | `038d033a9fee033981e84320968ae6fc41faa264409eb8c7f497303f18e938b9` |
| `full_step_link1-colors.mtl` | `fa7666b06147a929a038ff3ed5bdbb72fbda05bcf88415bf98f2a06144f73549` |
| `full_step_link1-colors.obj` | `cbd38c8a51a36dc5a1c373f126d6c9fbfb0c8ad83e6e2607629fd64f2fb9f163` |
| `full_step_link1-fasteners-source.stl` | `89977225fc4350993223b6b961ed922a36e07f89edf2b150e56ec4de45a78d6b` |
| `full_step_link2-collision-local-clearance-5mm.stl` | `75e1a48d1ae9c2d57809ef28256ba0c02ff35bbc57d5faa9ae237f84275578a7` |
| `full_step_link2-collision-local.stl` | `9efe3423d3a6174650dd43a3878d8a0bc75e342a5fcad17a143032a638b5d9a4` |
| `full_step_link2-colors.mtl` | `da671782470ab5230de0c4d13648b560ef6d1a5bac7048c576b1d7f9c8505140` |
| `full_step_link2-colors.obj` | `76ad0dd7c5c070dd2efc8655810cbd4ec9beb6dd2e955714ab2e3bc68f91d45c` |
| `full_step_link3-collision-local.stl` | `864d57f863b7cbb93d7f10a6efac82126b6ca150ba3daa7214b79632e0d46572` |
| `full_step_link3-colors.mtl` | `8b46f0c8b505704619b49fbe90d57d74acf9b387bce874ee608c425687efab29` |
| `full_step_link3-colors.obj` | `2d9012a16dd5994469843227e89eb0dc57b2e1169a6c85fef07c7567730a251c` |
| `full_step_link4-collision-local.stl` | `a88e00d019bfd713b9c4f50a2d3daf4c50e7c54305336f67a863c25a19d16cb1` |
| `full_step_link4-colors.mtl` | `5e319e28f4464b477074ec114c82aed07653e96b0c308429a27b31bbd35a1443` |
| `full_step_link4-colors.obj` | `189a2fe8dccf2986d3d468bd7b1f62ad9725aaaf994545094e5347edca5beab9` |
| `full_step_link5-collision-local.stl` | `96460d96d0871da40702eae038a408630c2348e41eabae7644465bf3f460a2b0` |
| `full_step_link5-colors.mtl` | `a0790db2586208e176afa8a547205851d8500a925d4a3109023c8e36ade6a70b` |
| `full_step_link5-colors.obj` | `4a7d5ff13d2267623b129a012db9e5d1addf781a3f82ec9a9fe29433835bbb5a` |

## 使用与验证

1. 使用克隆中已提供的网格；仅在重新生成时准备源 STEP，并核对其输入身份。
2. 按[转换说明](dummy/model-conversion/README.md)核对实例、局部坐标和米/毫米单位。
3. 构建所需 ROS 包，重新 source 对应安装区；已运行的 MoveIt 需要重新加载更新后的几何。
4. 修改几何后，按[REST 回归](dummy/rest-planning.md)和[间隙说明](dummy/collision-clearance.md)验证完整碰撞模型。

描述包现有安装规则会把模型资源复制到安装区。制作安装包或共享安装目录时，
应同时保留相应许可及来源说明；静态模型通过不等同于物理运动或 Stop/Disable 已确认。

23份 STEP 派生资源及 MINIF08 模型按 Apache-2.0 分发；七份旧 STL 仍按原 GPL-3.0 分发。
详细来源、例外和许可文本见[NOTICE](../NOTICE)、[许可清单](../LICENSES/README.md)
与[模型来源](../LICENSES/MODEL-PROVENANCE.md)。
