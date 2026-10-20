# 模型来源与许可范围

仓库随附当前 Xacro 所需的完整显示、碰撞和减速器网格，克隆后无需另行补齐模型资源。
完整装配派生网格与 MINIF08 近似网格按维护者选择采用 Apache-2.0；
已识别的旧版七份 STL 保留 GPL-3.0。各自许可不因放在同一目录而相互替换。
文件内完整许可文本见 `src/dummy-ros2_description/meshes/LICENSE`，资源清单见
[模型资源](../docs/model-assets.md)。

## 旧版七份 STL

目录：`src/dummy-ros2_description/meshes/`。
下列文件与 [switchpi/Dummy](https://gitee.com/switchpi/dummy) 的
`ros2/dummy_ws/src/dummy-ros2_description/meshes/` 对应文件逐字节一致。
核对版本为 `4c15112e0154353e5405302fa320606bdb680730`（`add dummy ros2 workspace files`）。
该版本根目录已有 GPL v3 `LICENSE`，本仓库在 [GPL-3.0.txt](GPL-3.0.txt) 原样保存；
该上游许可继续适用于这七份文件；其余随附模型的许可范围见下文。

| 文件 | SHA-256 |
| --- | --- |
| `base_link.stl` | `55ece6abb6ecc37f56868df5327991d2676e20f178862f7a9ed3fb6a9669b275` |
| `link1_1_1.stl` | `30cd28b038c726f873aa75903fbca368d95e24bfe1779bf303b3fb36f0a6983f` |
| `link2_1_1.stl` | `a47ea8c78d80b79874e0cea2a69ad0bd285af239a0d5b7a1d16b9b60f81e419d` |
| `link3_1_1.stl` | `ecec6ece159b9c41409f20792937773266b1c9d240576bf4a31c3cee32b5b3d6` |
| `link4_1_1.stl` | `35c318fffd8f09e5585024dc2df2ffcdb0b06e7181254eabb14173596681e012` |
| `link5_1_1.stl` | `a5a7257d58e3850a0e115f972496e1c8068550725da01f79bea56e6c8cc44bf1` |
| `link6_1_1.stl` | `5554538d053939e45d1ad6f8b321ce72d0d181fe55b269ba99d93368acdbaa6f` |

上游 README 将该项目描述为 Dummy 的复现，3D 模型目录另有致谢；
这些项目级介绍不能替代对特定新版 STEP 的来源和授权核实。
这里记录的是可核对的文件来源及随附许可，不额外断言上游对全部设计的权利链已经审计完成。

## 完整装配 STEP 的派生资产

`src/dummy-ros2_description/meshes/full_step_*.obj`、`full_step_*.mtl` 和
`full_step_*.stl` 共23份随附模型文件采用 [Apache-2.0](Apache-2.0.txt)。
它们包含彩色显示模型、分区碰撞包络、间隙保护网格和转换中间网格。

转换输入为提供的 `Dummy-B-v2.step`，SHA-256：
`7099178daac19d822df14ca0495921032a858feabcb8ffb8678c8b06c68b5682`。
现有转换记录未注明公开下载地址或原作者，STEP 头的 author、organization、authorisation 字段为空；
这里保留可核对的文件身份和来源记录，不补写未经记录的作者或来源。
本仓库的 Apache-2.0 发布声明覆盖上述随附派生模型，不将未随仓库提供的原始 CAD 文件纳入声明范围。

`full_step_manifest.json`、`full_step_visual_optimization.json`、
`rest-collision-refinement.json` 和 `clearance-guards.json` 保留转换、摘要及碰撞配置关系。
模型文件的具体 SHA-256 见资源清单；此次许可与收录调整不修改网格字节、关节关系或碰撞几何。

## MINIF08 近似网格

随附文件：`src/dummy-ros2_description/meshes/MINIF08_approx.stl`；采用
[Apache-2.0](Apache-2.0.txt)，与项目原创代码使用同一许可。
SHA-256：`779e8be3ed23ae571dd36d66719d1838f93a46148d18827195627b194564e93a`。
现有[减速器模型说明](../docs/dummy/j6-minif08-model.md)记为提供的近似网格，
未记载公开下载地址或原作者；本记录保留这一来源边界，不额外推断制造商授权或产品背书。

## 安装与使用

描述包将上述模型与目录内许可文件一同安装。当前 Xacro 直接使用完整装配 OBJ/STL、
碰撞保护网格及 MINIF08 网格；这些资源均随代码提供，旧版七份 STL 继续保留用于来源追溯。
正常构建无需重新转换 CAD；重新生成或修改几何时仍须核验坐标、尺度、碰撞覆盖及 REST 双向规划。
