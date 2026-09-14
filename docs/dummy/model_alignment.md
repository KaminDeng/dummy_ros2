# Dummy Model Alignment

## Contract Identity

ROS consumes the read-only C02 bundle at `contracts/dummy_robot_v1`. Its release
is `dummy-model-v1`, stage `model`, with payload digest
`f7f5495c21e03840d38b56da4a5da09115c20594350c71958d62c95c0ba21634`.
The legacy FK source hash is
`a0efaf04f70f0d6f683b5154e29dfb1ddbe1adbd3fbb5b5e2bd9abd9389c1ca3`.

## Joint Identity

| Joint | Slot / CAN | Direction | Ratio | ROS range (rad) | Legacy init (deg) |
| --- | --- | ---: | ---: | --- | ---: |
| Joint1 | 0 / 1 | -1 | 30 | [-2.9670597283903604, 2.9670597283903604] | 0 |
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
