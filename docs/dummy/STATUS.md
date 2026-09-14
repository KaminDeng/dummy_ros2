# Dummy ROS Status

| Task | Status | Evidence | Remaining condition |
| --- | --- | --- | --- |
| R00 | PASS | R00 acceptance and original static tests pass; isolated build succeeds; ROS01 was closed by R02 headless Gazebo runtime without DPSDK/physical IO. | None for R00; runtime details remain in R02. |
| R01 | PASS | C02 hashes verified; joint limits/named poses aligned; PyKDL matches 4 fixed + 128 random legacy FK samples; MoveIt validates final `home` and `reset`. | CAD/inertia/physical collision accuracy remains explicitly unverified. |
| R02 | PASS_WITH_LIMITATION | Headless Gazebo 8.15 spawns the model and one controller manager; six-axis state and nonzero FJT feedback are real; pause/resume follows `/clock`; primitive collisions load. | `move_group` reproducibly exits -11 on Ctrl-C; collision boxes and position servo are simulation-only approximations. |
| R03 | PASS | C05 `dummy-wire-v1` pins R03 commit `2f9ebf1`; codec, real zenoh-c RPC, RequestHandle, caller/owner, credential gate, pluginlib, and endpoint-negative probes pass. | None for R03. |
| R04 | IN_PROGRESS | C04/C05 are PASS; closed wire bundle hash verifies and ROS build consumes its codec directory. | Implement nonblocking snapshot/write generation and lifecycle tests. |
| R05 | PASS_WITH_LIMITATION | Same YAML generates Gazebo/MoveIt scene; collision target rejected; Cartesian fraction 0 not executed; nearby pose IK succeeds; public FJT gate proves BUSY/cancel/third-goal and limit rejection. | Internal JTC bypass closure belongs to R06; `move_group` Ctrl-C teardown still exits -11. |
| R06-R08 | NOT_STARTED | No device action lifecycle, SIL, or release tests executed. | Ordered dependencies remain. |
| Hardware/J1 | NOT_RUN | `--hardware none` enforced; no probe, upload, reset, or motion performed. | Requires separate human-gated procedure. |

The detailed R00 report is `docs/dummy/task-reports/R00.json`; build evidence is
under `build_dummy/reports/` and is intentionally not presented as Gazebo runtime
evidence.
