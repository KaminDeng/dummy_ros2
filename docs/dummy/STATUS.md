# Dummy ROS Status

| Task | Status | Evidence | Remaining condition |
| --- | --- | --- | --- |
| R00 | IN_PROGRESS | 5 R00 acceptance tests and 3 original static tests pass; clean 3-package build succeeds. | ROS01 is NOT_RUN (runner exit 3); it needs the R02 headless Gazebo execution path. |
| R01 | IN_PROGRESS | C02 `dummy-model-v1` is mirrored under `contracts/dummy_robot_v1`; all payload and sorted-manifest hashes pass ROS-side verification. | Complete FK/URDF, limits, initial-state, mesh-offset, and collision checks. |
| R02 | NOT_STARTED | No Gazebo runtime tests executed. | R01 must pass first. |
| R03 | READY | DPSDK C03 is PASS; no ROS wire probe has run yet. | Execute after the current R01/R02 sequence. |
| R04-R08 | NOT_STARTED | No device plugin, action lifecycle, SIL, or release tests executed. | Ordered dependencies remain. |
| Hardware/J1 | NOT_RUN | `--hardware none` enforced; no probe, upload, reset, or motion performed. | Requires separate human-gated procedure. |

The detailed R00 report is `docs/dummy/task-reports/R00.json`; build evidence is
under `build_dummy/reports/` and is intentionally not presented as Gazebo runtime
evidence.
