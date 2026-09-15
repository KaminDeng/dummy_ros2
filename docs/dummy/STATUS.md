# Dummy ROS Status

| Task | Status | Evidence | Remaining condition |
| --- | --- | --- | --- |
| R00-R05 | IN_PROGRESS | Existing model, wire, nonblocking, Gazebo and MoveIt evidence retained; case-specific runner and required launch artifacts are being revalidated. | Regenerate truthful per-case reports before restoring individual PASS states. |
| R06 | BLOCKED | C05-compliant lifecycle components pass locally; previous aggregate 23/23 claim was withdrawn. | DPSDK D07 operation values drift from wire-v1 and production dp_comm/JTC bridge remains incomplete; see `handoffs/R06-to-DPSDK-operation-enum-blocker.json`. |
| R07 | BLOCKED | Fail-closed backend launches and upstream component evidence exist; no case is promoted to cross-process PASS. | Requires completed R06 and a committed long-lived shared-core SIL peer for six-axis and J1. |
| R08 | BLOCKED | Previous release claim was withdrawn. | Requires truthful R00-R07 reports and a regenerated source/model/profile/version lock. |
| Hardware/J1 | NOT_RUN | `--hardware none` enforced; no probe, upload, reset, or motion performed. | Requires separate human-gated procedure. |

The detailed R00 report is `docs/dummy/task-reports/R00.json`; build evidence is
under `build_dummy/reports/` and is intentionally not presented as Gazebo runtime
evidence.
