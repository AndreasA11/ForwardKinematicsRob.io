# Task: Recommended `joint_state_publisher` Servo Node

Implement the demo-only joint servo that moves the robot toward named joint-space setpoints.

This task is not part of the graded runtime. Do not modify the core `/tf` computation or launch this node from `make run`.

## Goal

Read the robot description, accept `/joint_trajectory`, maintain joint targets, and publish `/joint_states` approximately 10 times per second.

## Inputs

* Read `robot_description` from `/param_server/get_param`.
* Subscribe to `/joint_trajectory` using the `trajectory_msgs/JointTrajectory` shape.
* Use the last point of each trajectory message as the setpoint for the listed joint names.

Unknown joint names do not move anything currently, but retain their setpoints so that they can take effect if a later robot description contains those joints.

Keep targets by joint name across robot reloads. If a joint exists in both the old and new robot, preserve its position, clamped to the new joint's limits where applicable.

## Servo behavior

Publish all movable joints in `/joint_states`, not only the joints explicitly named in the latest trajectory.

Move each joint toward its target using a proportional servo with gain 50 per second, clamped to joint limits. Snapping directly to the target is also acceptable for this recommended node.

For a reset operation, provide:

`/joint_state_publisher/reset`

Reset revolute and prismatic joints to the middle of their limits and continuous joints to zero.

The joint servo may clamp to limits; the graded forward kinematics must not.

## Acceptance criteria

1. The node reads the robot description through the parameter server.
2. It publishes all movable joints at approximately 10 Hz.
3. It consumes the last trajectory point as the setpoint.
4. Joint positions move toward targets.
5. Unknown targets can be retained for a subsequently loaded robot.
6. Shared joint positions are preserved appropriately across reloads.
7. The reset service is callable and resets joints as specified.
8. The node is launched by `make demo` only, not `make run`.
