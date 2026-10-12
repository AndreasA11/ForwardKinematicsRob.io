# Task: Kinematic Tree and Reusable Forward Kinematics Engine

Implement the reusable forward kinematics engine that converts a validated robot model and a set of joint positions into transforms for every joint and link.

This task depends on Task 03 (URDF model) and Task 04 (transform mathematics). Do not implement runtime services, parameter polling, or topic publication.

## Goal

Build the kinematic tree from the validated URDF model and calculate the robot's forward kinematics (FK).

## Kinematic tree

* Determine the root as the unique link that is not the child of any joint.
* Do not assume the root is the first link in the URDF.
* Preserve every joint, including fixed joints.
* Support chains and branching trees.
* Support joints appearing before links in the original XML.
* Retain parent-to-child relationships and the joint transform information needed for FK.

## Joint motion

For a joint with position `q`, calculate `T_joint(q) = T_origin · T_motion(q)`.

* `revolute`: rotate by `q` radians about the normalized joint axis.
* `continuous`: same rotation convention as revolute, without wrapping or clamping `q`.
* `prismatic`: translate by `q · axis`, using the axis exactly as written, without normalization.
* `fixed`: identity motion transform.

For zero configuration, every joint's transform must equal its origin transform.

## Forward kinematics

Traverse from the root and calculate each child link's pose using:

`T_root_child = T_root_parent · T_joint(q)`

Produce:

1. One parent-to-child transform for every joint.
2. One root-relative pose for every link, including the root.
3. A complete result that can be serialized by later runtime nodes.

A root-relative root pose is the identity.

Do not apply joint limits. Do not implement mimic-joint behavior: mimic joints are controlled by their own names, like any other joint.

## Acceptance criteria

* Correctly identifies a root that is not the first link.
* Supports arbitrary valid tree branching.
* Includes every fixed joint and every link.
* Correctly composes origin translation and rotation before joint motion.
* Supports revolute, continuous, prismatic, and fixed joints.
* Normalizes axes only for revolute and continuous rotation.
* Uses the written axis directly for prismatic motion.
* Produces correct zero-configuration transforms.
* Matches hand-calculated results for translation-only, rotation-only, two-link, and branching examples.
* Meets the required translation and rotation tolerances.
* Can be tested without launching the runtime or gateway.
