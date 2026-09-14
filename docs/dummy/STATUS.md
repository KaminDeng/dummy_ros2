# Dummy ROS Status

| Task | Status | Evidence | Remaining condition |
| --- | --- | --- | --- |
| R00 | IN_PROGRESS | 5 R00 acceptance tests and 3 original static tests pass; clean 3-package build succeeds. | ROS01 is NOT_RUN (runner exit 3); it needs the R02 headless Gazebo execution path. |
| R01 | PASS | C02 hashes verified; joint limits/named poses aligned; PyKDL matches 4 fixed + 128 random legacy FK samples; MoveIt validates final `home` and `reset`. | CAD/inertia/physical collision accuracy remains explicitly unverified. |
| R02 | READY | No Gazebo runtime tests executed. | Implement native Gazebo launch/runtime checks; track MoveIt Ctrl-C teardown SIGSEGV. |
| R03 | READY | DPSDK C03 is PASS; no ROS wire probe has run yet. | Execute after the current R01/R02 sequence. |
| R04-R08 | NOT_STARTED | No device plugin, action lifecycle, SIL, or release tests executed. | Ordered dependencies remain. |
| Hardware/J1 | NOT_RUN | `--hardware none` enforced; no probe, upload, reset, or motion performed. | Requires separate human-gated procedure. |

The detailed R00 report is `docs/dummy/task-reports/R00.json`; build evidence is
under `build_dummy/reports/` and is intentionally not presented as Gazebo runtime
evidence.
