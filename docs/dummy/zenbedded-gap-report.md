# R03 Zenbedded And Controller Hook Report

## Locked Inputs

The upstream repository is locked by DPSDK `tools/ros2/zenbedded.repos` to
`ros-controls/zephyr-zenoh-integration` commit
`96a7a912a250780bc15c086deb65210296d6fb11`. That upstream source checkout is not
present locally, so R03 does not publish or claim an upstream patch. Any future
patch must record this exact parent and a patch SHA-256 under
`patches/zenbedded_dummy_v1/`.

DPSDK C03 commit `6ce5a55` provides the generated raw-fixed implementation used
by the ROS-owned probe. The probe includes `dp_dummy_robot_v1_dp_comm.hpp`
directly and calls the generated encode/decode functions; it does not reimplement
the byte layout.

## Proven Surface

`wire_probe_tests.cpp` compiles against the actual C03 output and verifies encoded
sizes 125/356/297/233/618, dp_comm service IDs 4135/4136, three distinct uint64
values above 2^53, a six-position array including Joint6 at 3*pi, a QUEUED but
not accepted management response, and rejection without destination mutation for
a one-byte-short command.

`GoalRegistrationGate` is a C++17 value object for the controller hook. Empty,
registered, mismatched UUID, mismatched trajectory digest, BUSY, and clear paths
are executed by `goal_gate_tests.cpp`. Registration creates a new gate value;
transport or publisher activity cannot mutate the credential.

The ROS-owned RPC orchestrator runs the real DPSDK zenoh-c server/client through
`rmw_zenohd`, preserving request IDs above 2^53 and observing QUEUED with
`accepted=false`. It also executes the exact dp_comm RequestHandle completion,
cancel, and remote-correlation quarantine gtests. The wire probe calls the actual
`dp::comm::service_invoke` path with caller B and verifies the decoded response
retains caller B, owner A, and the same session snapshot.

## Remaining Integration Boundary

`GuardedJointTrajectoryController` derives from the installed Jazzy JTC rather
than copying its algorithm. After base configure it destroys the inherited action
server and trajectory subscriber. A same-process owner must register a UUID and
trajectory digest before submitting a trajectory through the inherited validation,
ordering, and realtime-buffer methods. The plugin compiles and exports through
pluginlib; controller-manager runtime loading and endpoint-negative evidence remain
required before MOV14 is PASS.

ROS CDR is a separate representation. DPSDK C03 owns and has passed its ROSIDL
semantic probe; this R03 probe compares and validates raw-fixed device encoding
only. C04 remains responsible for the cross-owner compatibility decision and C05
for publishing `wire-v1`.
