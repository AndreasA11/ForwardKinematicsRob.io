# Task: Reusable 3D Transform and Quaternion Mathematics

Implement the standalone mathematical primitives needed for URDF forward kinematics and world-state composition.

Do not implement URDF parsing, robot tree traversal, middleware nodes, or publication behavior.

## Goal

Provide a tested transform library using 4×4 homogeneous transformation matrices and unit quaternions.

Do not use a kinematics, transform, or URDF-model library. Implement the required mathematics directly.

## Required operations

1. Vector arithmetic, dot products, norms, and normalization where appropriate.
2. Unit quaternion construction and normalization.
3. Quaternion multiplication and conversion to a rotation matrix, if needed.
4. Rotation construction from fixed-axis roll-pitch-yaw.
5. Rotation construction from axis-angle.
6. Homogeneous transformation construction from translation and rotation.
7. Homogeneous transform multiplication.
8. Translation and rotation extraction where needed.
9. Conversion of rotation matrices to unit quaternions using a numerically robust method near 180-degree rotations.

## Conventions

URDF roll-pitch-yaw uses:

`R = Rz(yaw) · Ry(pitch) · Rx(roll)`

Joint origin transforms use:

`T_origin = Trans(xyz) · Rot(rpy)`

The transform convention for joint motion is:

`T_joint(q) = T_origin · T_motion(q)`

A child pose is composed as:

`T_root_child = T_root_parent · T_parent_child`

Do not reverse these multiplication orders.

Use unit quaternions for 3D rotations. Published quaternions must have norm within `1e-4` of 1.

## Acceptance criteria

* Translation-only transforms are correct.
* Rotations about each individual axis are correct.
* Combined roll-pitch-yaw rotations follow the specified multiplication order.
* Axis-angle rotations work for general axes.
* Quaternion conversions remain stable near 180-degree rotations.
* Homogeneous transform composition produces the expected translation and orientation.
* Rotation matrices are orthonormal with determinant `+1` within the specified tolerance.
* Numerical accuracy meets the Project 3 requirement of `1e-4` m and `1e-4` rad.
* No external kinematics or transform library is used.
* The library is reusable by both FK and world-state publishing code.
