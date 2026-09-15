# Dummy ROS Status

| Task | Status | Evidence | Remaining condition |
| --- | --- | --- | --- |
| R00 | PASS | R00 acceptance and original static tests pass; isolated build succeeds; ROS01 was closed by R02 headless Gazebo runtime without DPSDK/physical IO. | None for R00; runtime details remain in R02. |
| R01 | PASS | C02 hashes verified; joint limits/named poses aligned; PyKDL matches 4 fixed + 128 random legacy FK samples; MoveIt validates final `home` and `reset`. | CAD/inertia/physical collision accuracy remains explicitly unverified. |
| R02 | PASS_WITH_LIMITATION | Headless Gazebo 8.15 spawns the model and one controller manager; six-axis state and nonzero FJT feedback are real; pause/resume follows `/clock`; primitive collisions load. | `move_group` reproducibly exits -11 on Ctrl-C; collision boxes and position servo are simulation-only approximations. |
| R03 | PASS | C05 `dummy-wire-v1` pins R03 commit `2f9ebf1`; codec, real zenoh-c RPC, RequestHandle, caller/owner, credential gate, pluginlib, and endpoint-negative probes pass. | None for R03. |
| R04 | PASS | C05 wire codec, nonblocking system plugin, actual JTC update sink, dp_comm command/state worker, identity/age/mask, 1000-update backpressure, inactive management and async stop latch pass. | Live management/action lifecycle is R06; interprocess SIL is R07. |
| R05 | PASS_WITH_LIMITATION | Same YAML generates Gazebo/MoveIt scene; collision target rejected; Cartesian fraction 0 not executed; nearby pose IK succeeds; public FJT gate proves BUSY/cancel/third-goal and limit rejection. | Internal JTC bypass closure belongs to R06; `move_group` Ctrl-C teardown still exits -11. |
| R06 | PASS | Strict runner 23/23: shared D04/D05 RobotCore motion barriers, ROS producer/ACL gates, and live Gazebo public action lifecycle. | Physical device remains NOT_RUN. |
| R07 | PASS | Strict runner 8/8: real localhost Zenoh dual peers, D07 RobotCore+VirtualCAN faults, three backend contracts, J1-only interfaces and read-only real defaults. | Physical device remains NOT_RUN. |
| R08 | PASS | REL01, runbook, backend matrix and versions lock pass; 45/45 CTest and strict R05-R08 reports are green with evidence layers separated. | Physical device remains NOT_RUN; known MoveIt Ctrl-C limitation retained. |
| Hardware/J1 | NOT_RUN | `--hardware none` enforced; no probe, upload, reset, or motion performed. | Requires separate human-gated procedure. |

The detailed R00 report is `docs/dummy/task-reports/R00.json`; build evidence is
under `build_dummy/reports/` and is intentionally not presented as Gazebo runtime
evidence.
