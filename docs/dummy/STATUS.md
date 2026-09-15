# Dummy ROS Status

| Task | Status | Evidence | Remaining condition |
| --- | --- | --- | --- |
| R00 | PASS | Case-specific runner AUD01-AUD03 and actual native Gazebo identity probe pass (4/4). | Evidence: `build_dummy/reports/R00-repair.json`. |
| R01 | PASS_WITH_LIMITATION | Model bundle, actual FK reference chain, finite multiturn and mesh AABB mapping probes pass (7/7). | Physical mesh/frame calibration and collision safety remain unverified. |
| R02 | PASS_WITH_LIMITATION | Native Gazebo identity, trajectory feedback, six-axis stability, clock pause/resume and loaded JTC probes pass (6/6). | MoveIt Ctrl-C teardown still exits -11; simulation parameters are not physical measurements. |
| R03 | PASS | Case-specific codec/RPC/caller-owner snapshot probes pass (7/7); C04/C05 consumed the ROS-owned local extension, with no upstream source patch claimed. | Device-mode cross-process bypass closure remains R06. |
| R04 | PASS | Case suite 10/10 and production transport CTest 15/15 pass; real D06 negative path, synthetic component positive paths, identity invalidation, source locks and zero-allocation read/write are separated. | A committed authorized positive device peer is required by R06/R07, not claimed here. |
| R05 | PASS_WITH_LIMITATION | Actual native trajectory feedback, scene equality, collision rejection, Cartesian failure and cancellation probes pass (7/7). | Physical collision safety and MoveIt teardown remain unverified. |
| R06 | PASS | 23/23; real D08 peer verifies explicit bootstrap, two successes, cancel, abort, operation IDs, epochs, tickets and HOLD barriers. | Evidence: `build_dummy/reports/R06.json`. |
| R07 | PASS | 8/8; independent J1 and six-axis D08 processes plus guarded lifecycle/stale probes pass. | Evidence: `build_dummy/reports/R07.json`; six-axis is not inferred from J1. |
| R08 | PASS | REL01 release lock and evidence separation pass. | Evidence: `build_dummy/reports/R08.json`. |
| Hardware/J1 | NOT_RUN | `--hardware none` enforced; no probe, upload, reset, or motion performed. | Requires separate human-gated procedure. |

The detailed R00 report is `docs/dummy/task-reports/R00.json`; build evidence is
under `build_dummy/reports/` and is intentionally not presented as Gazebo runtime
evidence.
