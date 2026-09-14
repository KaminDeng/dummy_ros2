# Dummy wire-v1 validation

C04 validates the pinned R03 handoff at ROS commit `2f9ebf1` against the DPSDK
C03 generated profile. The shared raw-fixed include trees are byte-identical.
ROS CDR is validated as a separate semantic round trip and is not compared to
the packed device frame.

Validated sizes in command/state/status/request/response order are
`125/356/297/233/618`. Large non-adjacent `uint64_t` values, static capacity
rejection, legacy generator coexistence, real interprocess Zenoh service,
QUEUED-not-accepted receipt semantics, and caller/owner separation pass.

RPC01 and RPC12 are service semantic fixtures: they prove bounded callback
enqueue/receipt behavior and construction from one latched snapshot. They do
not prove that the D04 core executed Capture or that D06 transported a live
device snapshot.

This report validates wire and service fixtures only. It does not claim that
the D04 state machine executed a management operation or that hardware motion
was authorized.
