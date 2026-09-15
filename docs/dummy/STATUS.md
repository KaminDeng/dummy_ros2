# Dummy ROS Status

| Task | Status | Evidence | Remaining condition |
| --- | --- | --- | --- |
| R00 | PASS | Case-specific runner AUD01-AUD03 and actual native Gazebo identity probe pass (4/4). | Evidence: `build_dummy/reports/R00-repair.json`. |
| R01 | PASS_WITH_LIMITATION | Model bundle, actual FK reference chain, finite multiturn and mesh AABB mapping probes pass (7/7). | Physical mesh/frame calibration and collision safety remain unverified. |
| R02 | PASS_WITH_LIMITATION | Native Gazebo identity, trajectory feedback, six-axis stability, clock pause/resume and loaded JTC probes pass (6/6). | MoveIt Ctrl-C teardown still exits -11; simulation parameters are not physical measurements. |
| R03 | PASS | Case-specific codec/RPC/caller-owner snapshot probes pass (7/7); C04/C05 consumed the ROS-owned local extension, with no upstream source patch claimed. | Device-mode cross-process bypass closure remains R06. |
| R04 | IN_PROGRESS | Existing nonblocking/generation/state probes pass (10/10); production transport is under final review. | Final transport/RT regression and report pending. |
| R05 | PASS_WITH_LIMITATION | Actual native trajectory feedback, scene equality, collision rejection, Cartesian failure and cancellation probes pass (7/7). | Physical collision safety and MoveIt teardown remain unverified. |
| R06 | BLOCKED | C05-compliant lifecycle components pass locally; previous aggregate 23/23 claim was withdrawn. | DPSDK D07 operation values drift from wire-v1 and production dp_comm/JTC bridge remains incomplete; see `handoffs/R06-to-DPSDK-operation-enum-blocker.json`. |
| R07 | BLOCKED | Fail-closed backend launches and upstream component evidence exist; no case is promoted to cross-process PASS. | Requires completed R06 and a committed long-lived shared-core SIL peer for six-axis and J1. |
| R08 | BLOCKED | Previous release claim was withdrawn. | Requires truthful R00-R07 reports and a regenerated source/model/profile/version lock. |
| Hardware/J1 | NOT_RUN | `--hardware none` enforced; no probe, upload, reset, or motion performed. | Requires separate human-gated procedure. |

The detailed R00 report is `docs/dummy/task-reports/R00.json`; build evidence is
under `build_dummy/reports/` and is intentionally not presented as Gazebo runtime
evidence.
