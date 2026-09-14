# Dummy ROS Status

| Task | Status | Evidence | Remaining condition |
| --- | --- | --- | --- |
| R00 | IN_PROGRESS | 5 R00 acceptance tests and 3 original static tests pass; clean 3-package build succeeds. | ROS01 is NOT_RUN (runner exit 3); it needs the R02 headless Gazebo execution path. |
| R01 | PASS | C02 hashes verified; joint limits/named poses aligned; PyKDL matches 4 fixed + 128 random legacy FK samples; MoveIt validates final `home` and `reset`. | CAD/inertia/physical collision accuracy remains explicitly unverified. |
| R02 | PASS_WITH_LIMITATION | Headless Gazebo 8.15 spawns the model and one controller manager; six-axis state and nonzero FJT feedback are real; pause/resume follows `/clock`; primitive collisions load. | `move_group` reproducibly exits -11 on Ctrl-C; collision boxes and position servo are simulation-only approximations. |
| R03 | IN_PROGRESS | ROS-owned C++ codec and goal-registration-gate probes pass against DPSDK C03 generated headers. | Register/build package, publish report, and hand probe evidence to C04; running JTC hook remains R06 scope. |
| R04-R08 | NOT_STARTED | No device plugin, action lifecycle, SIL, or release tests executed. | Ordered dependencies remain. |
| Hardware/J1 | NOT_RUN | `--hardware none` enforced; no probe, upload, reset, or motion performed. | Requires separate human-gated procedure. |

The detailed R00 report is `docs/dummy/task-reports/R00.json`; build evidence is
under `build_dummy/reports/` and is intentionally not presented as Gazebo runtime
evidence.
