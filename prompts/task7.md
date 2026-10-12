# Task: Joint-State Updates and `/tf` Forward Kinematics

Implement the moving-joint behavior of `robot_state_publisher`, using the loaded robot model and reusable FK engine.

This task handles `/joint_states` input and construction of `/tf` messages. Do not implement global-pose composition or the optional joint servo.

## Goal

Subscribe to `/joint_states` and publish a complete set of parent-to-child transforms for every joint in the currently loaded robot.

Use the message contract in `spec/FK_API.md`.

## Input: `/joint_states`

Accept the `sensor_msgs/JointState` message shape:

`{"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""}, "name": [...], "position": [...], "velocity": [...], "effort": [...]}`

Each message replaces the current joint state.

* For each index `i`, set joint `name[i]` to `position[i]`.
* Any movable joint omitted from the latest message returns to position `0`.
* Ignore names that are unknown or identify fixed joints.
* Ignore `velocity` and `effort`; they may be absent or empty.
* Use positions exactly as supplied. Do not clamp to limits or wrap continuous-joint positions.
* Copy the latest `header.stamp` verbatim into every transform in the next `/tf` message.
* Before the first joint-state message, use `{"sec": 0, "nanosec": 0}`.

The contract leaves mismatched name/position lengths and malformed message fields unspecified; these must not crash or hang the runtime.

## Output: `/tf`

Publish a `tf/tfMessage`-shaped object:

`{"transforms": [<TransformStamped>, ...]}`

Each entry must contain:

* `header.frame_id`: joint parent link.
* `child_frame_id`: joint child link.
* `header.stamp`: the common timestamp for the message.
* `transform.translation`: translation as numeric `x`, `y`, and `z`.
* `transform.rotation`: unit quaternion as numeric `x`, `y`, `z`, and `w`.

For each joint, compute:

`T_joint(q) = T_origin · T_motion(q)`

Include exactly one entry per joint, including fixed joints. The root link has no joint entry. Do not omit fixed joints merely because they do not move.

Reuse the model and transform code from earlier tasks rather than duplicating FK logic inside the subscriber.

## Acceptance criteria

1. A robot at zero configuration produces the expected `/tf` transforms.
2. Revolute and continuous joints rotate about normalized axes.
3. Prismatic joints translate using their written, unnormalized axes.
4. Origin transforms are composed before joint motion.
5. Fixed joints are included.
6. Each new joint-state message replaces the previous state.
7. Omitted movable joints return to zero.
8. Unknown and fixed-joint names are ignored without rejecting valid entries in the same message.
9. Joint positions are not clamped or wrapped.
10. Every transform in one `/tf` message carries the same input timestamp.
11. Quaternions meet the required normalization and accuracy tolerances.
12. The implementation does not depend on an external kinematics library.
