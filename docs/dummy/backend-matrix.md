# Dummy ROS Backend Matrix

| Backend | Axes | Clock | Transport | Startup safety | Evidence |
| --- | --- | --- | --- | --- | --- |
| `mock_6dof` | Joint1-Joint6 | Gazebo `/clock` | Gazebo | No DPSDK session or hardware | R02/R05 runtime |
| `sil_j1` | Joint1 | steady | localhost Zenoh + VirtualCAN | No physical IO | R07 D06/D07 |
| `real_j1` | Joint1 | steady | explicit Zenoh endpoint | read-only; no Grant/Capture/Enable | NOT_RUN |

The real backend does not expose Joint2-Joint6 as measured state. Missing six-axis
feedback is an error in the six-axis SIL profile, not a five-axis fallback. None of
the launch files performs automatic authorization, reference capture, enable, or
motion.
