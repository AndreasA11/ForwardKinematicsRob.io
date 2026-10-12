# Task: Runtime Topic Publishing and Lifecycle Guarantees

Ensure the runtime continuously exposes complete, current topic messages with the required timing and lifecycle behavior.

This task covers publication scheduling and black-box topic behavior for `/tf` and `/xform_world`. It does not replace the kinematics or world-state math implemented in other tasks.

## Goal

Meet the externally observable runtime contract under idle operation, motion, late subscriptions, and robot reloads.

## Requirements

### `/tf`

* Publish at least 5 times per second whenever a robot is loaded.
* Do not publish `/tf` while no robot is loaded.
* Every publication contains exactly one transform per joint, including fixed joints.
* Continue publishing even if the robot and joint state have not changed.
* A late subscriber receives a complete, current message within one publishing period.

### `/xform_world`

* Once a `/tf` message has been received, publish at least 5 times per second.
* Continue publishing while the input remains unchanged.
* A late subscriber receives a complete, current message within one publishing period.
* Do not preserve links from an older robot after a new robot's transform tree arrives.

### Lifecycle and status ordering

* Accepted descriptions become the active robot atomically.
* Description status must not report success before the new robot is fully in effect.
* Messages published after successful status reporting must reflect that robot.
* Rejected descriptions must not disrupt publication for the previously loaded robot.

Use the existing middleware's supported subscription mechanisms. Periodic complete publications are sufficient to satisfy the late-subscriber requirement; do not introduce a new transport protocol.

## Acceptance criteria

1. `/tf` publishes at least 5 Hz with a loaded robot.
2. `/tf` remains silent before any robot is loaded.
3. `/xform_world` publishes at least 5 Hz after receiving `/tf`.
4. Both streams remain live with no state changes.
5. Late subscribers receive complete current messages within one publishing period.
6. No publication contains a mixture of old and new robot trees.
7. A rejected description leaves the previous robot's stream functioning.
8. Publishing is not dependent on repeated client requests.
9. Repeated `make run` and shutdown cycles do not leave the gateway or publishing threads unusable.
10. Timing and completeness are tested through an external TCP/JSON client.
