# Dummy ROS Status

| Task | Status | Evidence | Remaining condition |
| --- | --- | --- | --- |
| R00-R05 | IN_PROGRESS | Existing model, wire, nonblocking, Gazebo and MoveIt evidence retained; case-specific runner and required launch artifacts are being revalidated. | Regenerate truthful per-case reports before restoring individual PASS states. |
| R06 | IN_PROGRESS | Component lifecycle repair is underway; previous aggregate 23/23 claim was withdrawn. | Device FJT management/prime/Complete/cancel chain and per-case evidence must pass. |
| R07 | BLOCKED | Previous static/backend aggregation claim was withdrawn. | Requires completed R06 and a runnable shared-core SIL peer for six-axis and J1. |
| R08 | BLOCKED | Previous release claim was withdrawn. | Requires truthful R00-R07 reports and a regenerated source/model/profile/version lock. |
| Hardware/J1 | NOT_RUN | `--hardware none` enforced; no probe, upload, reset, or motion performed. | Requires separate human-gated procedure. |

The detailed R00 report is `docs/dummy/task-reports/R00.json`; build evidence is
under `build_dummy/reports/` and is intentionally not presented as Gazebo runtime
evidence.
