# Dummy Model Alignment

## 中文导读：本页是早期 C02 对照记录

合同摘要、关节方向/减速比、标准DH和四组固定/128组随机FK对照都对应本页锁定的旧输入。
rad是关节角，deg是旧初始化角，m是DH长度；J6有限多圈值不归一化成[-π,π]。
这里的“current CAD”“REST不可执行”和七网格盒体描述属于早期模型阶段，不能覆盖后续完整STEP与
[REST包络修复](rest-planning.md)；当前工作REST与旧`[0,-73°,180°,0,0,0]`数学样本也不是同一姿态。
当前模型、全50:1映射及后端选择以 [主 README](../../README.md) 和匹配的model/profile为准。
旧数学FK通过不证明碰撞、质量惯量、负载、机械标定或真实安装正确；下文原数值保留供溯源。

数学回归保留的 [固定FK病例](../../src/dummy_moveit_config/test/fixtures/fk_cases.json) 与 [随机FK病例](../../src/dummy_moveit_config/test/fixtures/fk_random_cases.json) 已迁入MoveIt测试夹具；旧合同身份仅在Git历史 `c76b1eb06924010daa735288066f9c43a26ed6d1` 中查阅。

## Historical Contract Identity

The historical C02 record consumed the read-only bundle at `contracts/dummy_robot_v1`. Its release
is `dummy-model-v1`, stage `model`, with payload digest
`f7f5495c21e03840d38b56da4a5da09115c20594350c71958d62c95c0ba21634`.
The legacy FK source hash is
`a0efaf04f70f0d6f683b5154e29dfb1ddbe1adbd3fbb5b5e2bd9abd9389c1ca3`.

## Joint Identity

| Joint | Slot / CAN | Direction | Ratio | ROS range (rad) | Legacy init (deg) |
| --- | --- | ---: | ---: | --- | ---: |
| Joint1 | 0 / 1 | -1 | 50 | [-2.9670597283903604, 2.9670597283903604] | 0 |
| Joint2 | 1 / 2 | +1 | 30 | [-1.2740903539558606, 1.5707963267948966] | -73 |
| Joint3 | 2 / 3 | -1 | 30 | [0.6108652381980153, 3.141592653589793] | 180 |
| Joint4 | 3 / 4 | +1 | 24 | [-3.141592653589793, 3.141592653589793] | 0 |
| Joint5 | 4 / 5 | -1 | 30 | [-2.0943951023931953, 2.0943951023931953] | 0 |
| Joint6 | 5 / 6 | -1 | 50 | [-12.566370614359172, 12.566370614359172] | 0 |

Joint6 is finite multi-turn and is never normalized to [-pi, pi]. The Gazebo
initial pose is `[0, 0, pi/2, 0, 0, 0]`; it is a simulation-only valid pose, not
a captured bench reference. The SRDF `home` uses that same pose. MoveIt's
`/check_state_validity` service reports both `home` and `reset` valid. The legacy
rest fixture `[0, -73deg, 180deg, 0, 0, 0]` is retained only as a mathematical FK
sample: the current CAD collision model reports contacts between
`base_link`/`link4_1_1` and `link2_1_1`/`link4_1_1`, so it is not an executable
named pose.

## Mathematical Reference Chain

`dummy_reference_chain.urdf` expresses each standard-DH row as a revolute Z-axis
joint followed by a fixed translation and X rotation. This preserves the public
joint names while keeping the constant transform explicit.

| Axis | Offset (rad) | d (m) | a (m) | alpha (rad) |
| --- | ---: | ---: | ---: | ---: |
| 1 | 0 | 0.109 | 0.035 | -pi/2 |
| 2 | -pi/2 | 0 | 0.146 | 0 |
| 3 | +pi/2 | 0 | -0.052 | +pi/2 |
| 4 | 0 | 0.115 | 0 | -pi/2 |
| 5 | 0 | 0 | 0 | +pi/2 |
| 6 | 0 | 0.072 | 0 | 0 |

Axis 3 deliberately uses `d=0, a=-0.052`. This is derived from the actual legacy
FK vector expression and is not a literal copy of its temporary DH table.
PyKDL evaluates the checked-in reference-chain file against four fixed C02 cases
and 128 independently generated legacy-solver samples. Acceptance limits are
0.05 mm position and 0.001 maximum rotation-matrix element error.

## Visual And Collision Assets

The existing `dummy-ros2.xacro` retains the seven uploaded STL files and their
CAD-derived visual origins. Those origins are not used as the FK oracle. Gazebo
Harmonic's DART backend rejected the STL collision shapes, so simulation collision
uses axis-aligned boxes computed from every STL vertex after applying the visual
origin. These boxes are conservative simulation-only approximations, not measured
link geometry.
The reference chain intentionally contains no visual, collision, mass, or inertia
claims. The CAD xacro continues to provide simulation appearance and provisional
collision geometry while the reference chain proves mathematical kinematics.

The current mass/inertia values and mesh offsets have not been physically
measured in this task. Passing FK and Gazebo checks does not establish real
collision clearance, payload capability, motor dynamics, joint direction, or
calibration accuracy. Those properties remain false or unverified in the C02
contract and must not unlock hardware motion.
