# Task: `robot_world_state_publisher` and `/xform_world`

Implement the world-state publisher that composes the latest `/tf` tree with the robot root's pose in the global frame.

This task depends on the reusable transform math from Task 04 and the runtime `/tf` contract from Tasks 07–08.

## Goal

Subscribe to `/tf` and `/global_pose`, then publish the global pose of every link on `/xform_world`.

## Input: `/global_pose`

Accept the `geometry_msgs/Pose` shape:

`{"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}`

Interpret it as:

`T_global_root = Trans(position) · Rot(orientation)`

Maintain the latest pose internally.

* Before any `/global_pose` message, use the identity pose.
* After receiving a pose, retain it until another arrives.
* Do not apply a viewer-specific axis correction. `global_frame` is z-up.

## Processing `/tf`

Treat each `/tf` message as a complete tree of parent-to-child edges.

For each new message:

1. Replace the previous stored transform tree completely.
2. Identify its root as the frame that occurs as a parent but never as a child.
3. Traverse from the root, composing transforms to obtain every `T_root_link`.
4. Compose each root-relative pose with the current global pose.

Use only the latest `/tf` message. Do not combine edges across messages.

## Output: `/xform_world`

Publish:

`{"transforms": [<MatrixTransform>, ...]}`

Each entry must contain:

* `header.frame_id`: `"global_frame"`.
* `child_frame_id`: the name of the link.
* `header.stamp`: the stamp from the `/tf` message used for this computation.
* `matrix`: exactly 16 JSON numbers representing `T_global_link` in column-major order.

The matrix contract is:

`matrix[4*c + r] = M[r][c]`

Therefore the translation occupies `matrix[12]`, `matrix[13]`, and `matrix[14]`.

Include exactly one entry for every link, including the root link. With no `/global_pose`, the root's matrix is the identity.

## Acceptance criteria

1. Correctly discovers the root from `/tf`.
2. Composes transforms in parent-first order.
3. Produces correct poses for chain and branching trees.
4. Includes the root and all child links.
5. Uses the most recent `/tf` message only.
6. Replaces the old tree after robot reloads.
7. Composes the most recent global pose with every link pose.
8. Uses identity global pose until the first valid pose arrives.
9. Produces exactly 16 column-major numeric values per matrix.
10. Carries through the stamp from the source `/tf` message.
11. Meets the required translation, rotation, orthonormality, and determinant tolerances.
12. Does not add a rendering correction or depend on a transform library.
