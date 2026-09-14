# Dummy ROS Status

| Task | Status | Evidence | Remaining condition |
| --- | --- | --- | --- |
| R00 | IN_PROGRESS | 5 R00 acceptance tests and 3 original static tests pass; clean 3-package build succeeds. | ROS01 is NOT_RUN (runner exit 3); it needs the R02 headless Gazebo execution path. |
| R01 | BLOCKED | No model checks executed. | Await C02 `model-v1` bundle and manifest hash. |
| R02-R08 | NOT_STARTED | No runtime, wire, SIL, or hardware tests executed. | Ordered dependencies are unmet. |
| Hardware/J1 | NOT_RUN | `--hardware none` enforced; no probe, upload, reset, or motion performed. | Requires separate human-gated procedure. |

The detailed R00 report is `docs/dummy/task-reports/R00.json`; build evidence is
under `build_dummy/reports/` and is intentionally not presented as Gazebo runtime
evidence.
