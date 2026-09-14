# Dummy ROS Status

| Task | Status | Evidence | Remaining condition |
| --- | --- | --- | --- |
| R00 | IN_PROGRESS | 5 R00 acceptance tests and 3 original static tests pass; clean 3-package build succeeds. | ROS01 is NOT_RUN (runner exit 3); it needs the R02 headless Gazebo execution path. |
| R01 | PASS | C02 hashes verified; joint limits/named poses aligned; PyKDL matches 4 fixed + 128 random legacy FK samples; MoveIt validates final `home` and `reset`. | CAD/inertia/physical collision accuracy remains explicitly unverified. |
| R02 | PASS_WITH_LIMITATION | Headless Gazebo 8.15 spawns the model and one controller manager; six-axis state and nonzero FJT feedback are real; pause/resume follows `/clock`; primitive collisions load. | `move_group` reproducibly exits -11 on Ctrl-C; collision boxes and position servo are simulation-only approximations. |
| R03 | PASS | Pinned codec, real zenoh-c RPC, explicit/implicit RequestHandle, caller/owner, credential gate, pluginlib load, and controller-manager endpoint-negative probes pass. | DPSDK current C03 regeneration is broken; R03's immutable prototype remains auditable but is not wire-v1. |
| R04 | BLOCKED | DPSDK C04/C05 wire interoperability and release are NOT_STARTED; current C03 suite has 1 failure + 1 error because its generated tree is deleted. | DPSDK owner must restore C03 generation, consume R03 handoff, pass C04, and publish C05 wire-v1. |
| R05 | PASS_WITH_LIMITATION | Same YAML generates Gazebo/MoveIt scene; collision target rejected; Cartesian fraction 0 not executed; nearby pose IK succeeds; public FJT gate proves BUSY/cancel/third-goal and limit rejection. | Internal JTC bypass closure belongs to R06; `move_group` Ctrl-C teardown still exits -11. |
| R06-R08 | NOT_STARTED | No device action lifecycle, SIL, or release tests executed. | Ordered dependencies remain. |
| Hardware/J1 | NOT_RUN | `--hardware none` enforced; no probe, upload, reset, or motion performed. | Requires separate human-gated procedure. |

The detailed R00 report is `docs/dummy/task-reports/R00.json`; build evidence is
under `build_dummy/reports/` and is intentionally not presented as Gazebo runtime
evidence.
