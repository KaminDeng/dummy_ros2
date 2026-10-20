# 模型资源逐文件阅读说明

本页解释 `src/dummy-ros2_description/meshes/` 中35项资源的角色，全部随克隆提供。
23份完整STEP派生资源与MINIF08模型由维护者按Apache-2.0分发；七份旧版STL仍保留原GPL-3.0，
另有四份JSON记录来源和生成阶段证据。运行完整模型无需另行补齐网格，源STEP仅在重新生成时需要。
文件身份与许可边界见[模型资源说明](../model-assets.md)。

网格是几何/材质数据，四份JSON是严格数据或生成阶段证据；阅读解释集中于本页。
历史清单中的摘要与当前文件身份应分别核对，不向二进制插入说明文字。

先从 [共享Xacro](../../src/dummy-ros2_description/urdf/dummy-ros2.xacro) 看文件用在
`visual`、`collision` 还是固定保护连杆，再看本页的资源角色。
[setup.py](../../src/dummy-ros2_description/setup.py) 会安装 `meshes/*`，
但“随包安装”不代表当前Xacro一定引用该文件；历史简化网格也仍保留在包里。

当前这些网格以毫米数值保存，Xacro中的 `scale="0.001 0.001 0.001"` 转成ROS的米。
不要把STL本身当作带单位格式，也不要对已转成米的关节平移再次缩放。
碰撞网格名中的 `local` 表示已转换到该link局部坐标，不能再重复套用CAD全局变换。

## 文件与实际用途

| 文件 | 阅读说明 |
| --- | --- |
| [MINIF08_approx.stl](../../src/dummy-ros2_description/meshes/MINIF08_approx.stl) | J6减速器外壳近似网格，当前Xacro用于显示；其碰撞另由Xacro中的基本体定义，不能把显示近似直接当已验证的完整实物表面。 |
| [base_link.stl](../../src/dummy-ros2_description/meshes/base_link.stl) | 历史简化底座网格。当前共享Xacro改用full_step_base；保留原件供旧模型/来源对照。 |
| [clearance-guards.json](../../src/dummy-ros2_description/meshes/clearance-guards.json) | 近端到腕部保护的适用link集合、源/输出摘要及序列化后外扩检查，字段见下表。 |
| [full_step_base-collision-local-clearance-5mm.stl](../../src/dummy-ros2_description/meshes/full_step_base-collision-local-clearance-5mm.stl) | 固定底座base_link近端保护体的外扩网格；无视觉，配合SRDF的有限接触对范围使用，不能当作所有相邻部件的全局padding。 |
| [full_step_base-collision-local.stl](../../src/dummy-ros2_description/meshes/full_step_base-collision-local.stl) | 固定底座base_link在本link局部坐标中的碰撞网格；按零件保守包络组合，和视觉减面资源分开。 |
| [full_step_base-colors.mtl](../../src/dummy-ros2_description/meshes/full_step_base-colors.mtl) | 固定底座base_link的cad_color材质定义；由同名OBJ的usemtl选择，改变材质不会改变关节运动学。 |
| [full_step_base-colors.obj](../../src/dummy-ros2_description/meshes/full_step_base-colors.obj) | 固定底座base_link的彩色视觉三角网格；OBJ的mtllib引用同名MTL。仅用于视觉，不替代碰撞网格。 |
| [full_step_link1-bracket-source.stl](../../src/dummy-ros2_description/meshes/full_step_link1-bracket-source.stl) | REST修复的原凹支架三角表面输入；按局部Z=24mm切分后分别构造保守凸包，不直接作为最终整件凸包。 |
| [full_step_link1-collision-local-clearance-5mm.stl](../../src/dummy-ros2_description/meshes/full_step_link1-collision-local-clearance-5mm.stl) | 连杆link1_1_1近端保护体的外扩网格；无视觉，配合SRDF的有限接触对范围使用，不能当作所有相邻部件的全局padding。 |
| [full_step_link1-collision-local.stl](../../src/dummy-ros2_description/meshes/full_step_link1-collision-local.stl) | 连杆link1_1_1在本link局部坐标中的碰撞网格；按零件保守包络组合，和视觉减面资源分开。J1凹支架采用REST修复后的分区包络。 |
| [full_step_link1-colors.mtl](../../src/dummy-ros2_description/meshes/full_step_link1-colors.mtl) | 连杆link1_1_1的cad_color材质定义；由同名OBJ的usemtl选择，改变材质不会改变关节运动学。 |
| [full_step_link1-colors.obj](../../src/dummy-ros2_description/meshes/full_step_link1-colors.obj) | 连杆link1_1_1的彩色视觉三角网格；OBJ的mtllib引用同名MTL。仅用于视觉，不替代碰撞网格。 |
| [full_step_link1-fasteners-source.stl](../../src/dummy-ros2_description/meshes/full_step_link1-fasteners-source.stl) | REST修复的12个紧固件源三角面输入；与支架分开处理，保留其各自碰撞覆盖。 |
| [full_step_link2-collision-local-clearance-5mm.stl](../../src/dummy-ros2_description/meshes/full_step_link2-collision-local-clearance-5mm.stl) | 连杆link2_1_1近端保护体的外扩网格；无视觉，配合SRDF的有限接触对范围使用，不能当作所有相邻部件的全局padding。 |
| [full_step_link2-collision-local.stl](../../src/dummy-ros2_description/meshes/full_step_link2-collision-local.stl) | 连杆link2_1_1在本link局部坐标中的碰撞网格；按零件保守包络组合，和视觉减面资源分开。 |
| [full_step_link2-colors.mtl](../../src/dummy-ros2_description/meshes/full_step_link2-colors.mtl) | 连杆link2_1_1的cad_color材质定义；由同名OBJ的usemtl选择，改变材质不会改变关节运动学。 |
| [full_step_link2-colors.obj](../../src/dummy-ros2_description/meshes/full_step_link2-colors.obj) | 连杆link2_1_1的彩色视觉三角网格；OBJ的mtllib引用同名MTL。仅用于视觉，不替代碰撞网格。 |
| [full_step_link3-collision-local.stl](../../src/dummy-ros2_description/meshes/full_step_link3-collision-local.stl) | 连杆link3_1_1在本link局部坐标中的碰撞网格；按零件保守包络组合，和视觉减面资源分开。 |
| [full_step_link3-colors.mtl](../../src/dummy-ros2_description/meshes/full_step_link3-colors.mtl) | 连杆link3_1_1的cad_color材质定义；由同名OBJ的usemtl选择，改变材质不会改变关节运动学。 |
| [full_step_link3-colors.obj](../../src/dummy-ros2_description/meshes/full_step_link3-colors.obj) | 连杆link3_1_1的彩色视觉三角网格；OBJ的mtllib引用同名MTL。仅用于视觉，不替代碰撞网格。 |
| [full_step_link4-collision-local.stl](../../src/dummy-ros2_description/meshes/full_step_link4-collision-local.stl) | 连杆link4_1_1在本link局部坐标中的碰撞网格；按零件保守包络组合，和视觉减面资源分开。 |
| [full_step_link4-colors.mtl](../../src/dummy-ros2_description/meshes/full_step_link4-colors.mtl) | 连杆link4_1_1的cad_color材质定义；由同名OBJ的usemtl选择，改变材质不会改变关节运动学。 |
| [full_step_link4-colors.obj](../../src/dummy-ros2_description/meshes/full_step_link4-colors.obj) | 连杆link4_1_1的彩色视觉三角网格；OBJ的mtllib引用同名MTL。仅用于视觉，不替代碰撞网格。 |
| [full_step_link5-collision-local.stl](../../src/dummy-ros2_description/meshes/full_step_link5-collision-local.stl) | 连杆link5_1_1在本link局部坐标中的碰撞网格；按零件保守包络组合，和视觉减面资源分开。 |
| [full_step_link5-colors.mtl](../../src/dummy-ros2_description/meshes/full_step_link5-colors.mtl) | 连杆link5_1_1的cad_color材质定义；由同名OBJ的usemtl选择，改变材质不会改变关节运动学。 |
| [full_step_link5-colors.obj](../../src/dummy-ros2_description/meshes/full_step_link5-colors.obj) | 连杆link5_1_1的彩色视觉三角网格；OBJ的mtllib引用同名MTL。仅用于视觉，不替代碰撞网格。 |
| [full_step_manifest.json](../../src/dummy-ros2_description/meshes/full_step_manifest.json) | 完整装配转换来源、CAD到link变换和历史加载/人工查看记录；不是机械运动合格证。 |
| [full_step_visual_optimization.json](../../src/dummy-ros2_description/meshes/full_step_visual_optimization.json) | 按材质分组的视觉减面与采样误差报告；保留优化当时的碰撞/运动学/ACM摘要快照。 |
| [link1_1_1.stl](../../src/dummy-ros2_description/meshes/link1_1_1.stl) | 历史简化Joint1对应连杆网格；当前共享Xacro使用full_step_link1资源。保留原件用于来源/旧模型对照，不与完整CAD资源叠加装配。 |
| [link2_1_1.stl](../../src/dummy-ros2_description/meshes/link2_1_1.stl) | 历史简化Joint2对应连杆网格；当前共享Xacro使用full_step_link2资源。保留原件用于来源/旧模型对照，不与完整CAD资源叠加装配。 |
| [link3_1_1.stl](../../src/dummy-ros2_description/meshes/link3_1_1.stl) | 历史简化Joint3对应连杆网格；当前共享Xacro使用full_step_link3资源。保留原件用于来源/旧模型对照，不与完整CAD资源叠加装配。 |
| [link4_1_1.stl](../../src/dummy-ros2_description/meshes/link4_1_1.stl) | 历史简化Joint4对应连杆网格；当前共享Xacro使用full_step_link4资源。保留原件用于来源/旧模型对照，不与完整CAD资源叠加装配。 |
| [link5_1_1.stl](../../src/dummy-ros2_description/meshes/link5_1_1.stl) | 历史简化Joint5对应连杆网格；当前共享Xacro使用full_step_link5资源。保留原件用于来源/旧模型对照，不与完整CAD资源叠加装配。 |
| [link6_1_1.stl](../../src/dummy-ros2_description/meshes/link6_1_1.stl) | 输出法兰网格，当前共享Xacro同时用于visual和collision；不是J6减速器外壳。 |
| [rest-collision-refinement.json](../../src/dummy-ros2_description/meshes/rest-collision-refinement.json) | J1凹支架分区重建输入、padding、最小外扩和输出/guard身份。 |

## OBJ、MTL和STL怎么看

OBJ中的 `v x y z` 是顶点，`f` 是面索引；`mtllib` 引入材质文件，`usemtl` 为后续面选材质。
MTL中 `newmtl` 定义名称，`Ka/Kd/Ks` 分别是环境/漫反射/镜面颜色，`Ns` 是高光指数，
`d` 是不透明度，`illum` 是着色模型编号。它们负责显示，不参与关节限位或许可判断。
不要手工将OBJ的坐标与另一份STL互换；两者可能有不同三角剖分，但应保持同一装配坐标。

这里的STL保留生成器输出。二进制STL含80字节头、32位三角形数及每面法线/顶点等记录，
不能按文本逐行添加Doxygen。其包络正确性须由生成脚本及几何回归检查，不靠文件名推断。

## 严格JSON字段解释

`full_step_manifest.json` 记录完整装配转换和安装阶段；其 `stage`、人工确认与报告路径
属于当时的操作记录，不是实时查询。当前控制资格仍由设备/部署合同单独建立。

| 字段 | 意义和单位 |
| --- | --- |
| `source`、`source_sha256` | 原STEP的来源路径和原件摘要；下载目录路径不是运行期依赖，包内网格已经随包安装 |
| `joint2_height_correction_m`、`joint4_origin_m`、`j6_reducer_mount_m` | 当前转换记录中的J2高度校正、J4原点、减速器安装平移，单位米 |
| `wrist_source_alignment_mm` | 源腕部装配对齐量，单位毫米，不能与上述米字段直接相加 |
| `groups` | base和link1～link5的分组；每组以`link`关联URDF连杆 |
| `groups.*.triangles`、`collision_triangles` | 转换记录中的视觉/碰撞三角形数；后续视觉减面应再查专门报告 |
| `motor_collision_links` | 从该组拆出的电机碰撞子连杆，避免相邻整件豁免掩盖非相邻接触 |
| `local_bounds_mm` | 转换后本link坐标中的轴对齐包围范围，单位毫米；不是机器人全局工作空间 |
| `cad_to_link` | 源CAD到link的4×4齐次变换记录；旋转无量纲，平移按转换器米制变换记录解释，不能重复施加到已local化网格 |
| `axis_line_errors_m` | 当时转换中J4/J5/J6轴线的数值对齐残差，单位米；数值小不等同实物标定精度 |
| `user_visual_review`、`unresolved` | 人工查看状态与当时尚未闭合的范围；人工显示确认不授予物理运动资格 |
| `runtime_verification` | 历史运行期模型核验报告名，原件位置见完整STEP文档，不是本目录中的实时状态文件 |
| `collision_refinement_manifest` | 后续REST分区包络的详细清单文件名 |

`full_step_visual_optimization.json` 只描述视觉减面。其最大误差来自原/新网格全部顶点及
三角形中心的双向采样；不应称为连续曲面的Hausdorff严格上界。
2026-10-21 已恢复原始视觉网格，`active_visual_variant` 为 `original`；
当前 OBJ 应与 `original_sha256` 核对，优化阶段的面数、误差和摘要保留为历史记录。

| 字段 | 意义和判读 |
| --- | --- |
| `stage`、`source_unit`、`algorithm` | 视觉优化阶段、毫米单位及保拓扑/边界的分材质算法 |
| `active_visual_variant`、`restored_on` | 当前启用原始或优化视觉网格；原始网格恢复日期 |
| `original_triangles`、`triangles` | 优化前后视觉总面数，不是当前碰撞复杂度 |
| `validation`、`max_sample_error_mm` | 当次采样方法与最大采样误差，单位毫米；0.15mm是该优化流程的拒绝阈值 |
| `files[].file`、`original_sha256`、`sha256` | 每个OBJ优化前后的原始字节身份；按当前启用版本选择对应摘要检查 |
| `files[].groups[].material`、`original_faces`、`faces` | 每种材质的优化前后面数，解释为何不同材质组减面比例不同 |
| `max_bidirectional_sample_error_mm` | 对应材质组的最大双向采样误差，不能从全局值反推每处表面精度 |
| `collision_kinematics_and_acm_sha256` | 优化当时碰撞、Xacro和SRDF未改动的历史摘要快照。后来根坐标/REST修复会使部分当前摘要不同，应结合后续清单判断，不回写旧阶段原件 |

`clearance-guards.json` 为近端/远端指定接触范围提供至少5mm的规划保护：

| 字段 | 意义和判读 |
| --- | --- |
| `minimum_clearance_mm` | 目标最小间隙5mm；并非所有相邻装配接触都统一外扩 |
| `generation_offset_mm` | 生成时外扩5.01mm，额外0.01mm用于STL浮点写出余量 |
| `proximal`、`distal` | 保护生效的近端和远端link集合；还需SRDF接触矩阵配套，不能单看网格 |
| `meshes[].source/output`、`source_sha256/output_sha256` | 每个原碰撞网格及保护输出的文件名/原始字节身份 |
| `parts`、`triangles` | 拆分的凸包连通件数量和输出面数；不等同源STEP的全部装配零件数量 |
| `minimum_expansion_after_stl_roundtrip_mm` | 按STL float32序列化精度回转后计算的最小支撑面外扩，单位毫米；针对相应生成算法的覆盖检查 |

`rest-collision-refinement.json` 说明为何不能把J1凹支架整件凸包化：

| 字段 | 意义和判读 |
| --- | --- |
| `source_step_sha256`、`source_part` | 原STEP身份及`J1/Dummy-0001`源支架身份 |
| `units`、`cut_axis`、`cut_mm` | 在源局部毫米坐标沿Z=24mm裁分；不是把机器人关节高度改成24mm |
| `padding_mm` | 分区外包络使用0.301mm外扩，最终序列化覆盖要求至少0.3mm |
| `minimum_source_margin_mm` | 两个分区最终包络对各自裁剪源三角面的最小余量，不代表机器人所有碰撞对都有该距离 |
| `source_bracket_sha256/source_fasteners_sha256` | 两份源输入的原始身份；切分时须几何裁剪并纳入新生成的跨面交点，不能只按整三角形重心归类 |
| `output_sha256`、`guard_sha256` | 最终link1碰撞网格及对应5mm保护网格的身份 |
| `guard_minimum_expansion_mm`、`triangles` | 最终保护外扩和最终碰撞网格面数；不修改REST角度、关节轴或视觉 |

## 重建与验证入口

完整装配过程见 [full-step-model.md](full-step-model.md)；
视觉减面见 [rviz-performance.md](rviz-performance.md)；
根坐标见 [ground-origin.md](ground-origin.md)；
近端保护见 [collision-clearance.md](collision-clearance.md)；
J1包络重建和只读REST回归见 [rest-planning.md](rest-planning.md)。

几何正确性、真实MoveIt状态/路径有效性和实际机械执行是三个不同结论。
更改模型后还应核对安装资源与运行进程实际加载的模型；旧进程可能缓存旧碰撞几何，
不能仅以磁盘文件已更新推断正在运行的MoveIt已经更新。
