# Task: Robot Description Loading and `/robot_state_publisher/description_status`

Implement the description-management portion of `robot_state_publisher`.

This task integrates the parameter server, URDF parser, and kinematic model. Do not implement moving-joint transforms or the world-state publisher here.

## Goal

Read `robot_description` through `/param_server/get_param`, process new parameter versions, and atomically load valid robots.

The runtime starts with no robot loaded. The robot arrives through `/param_server/set_param`; do not require a file path, command-line argument, or environment variable.

## Reading descriptions

* Read `robot_description` using the parameter server service, not by directly accessing its internal storage.
* Polling is acceptable.
* Process new versions as they appear. Intermediate versions may be skipped if a newer version is available, but the latest version must be processed.
* After a client sets `robot_description` and receives version `V`, report a description status for some version greater than or equal to `V` within 2 seconds.

## Loading and atomic replacement

1. Retrieve the complete parameter value and version.
2. Reject values that are not JSON strings.
3. Parse and validate the description using the reusable parser.
4. Construct the complete new robot model and kinematic tree.
5. Only after the entire robot has passed validation, atomically replace the currently loaded robot.
6. Publish the corresponding description status only after the accepted robot is fully in effect.

If a description is rejected, preserve the previously loaded robot, its joint state, and its published transforms. Continue running and process future versions normally.

## Description status

Write the following parameter using `/param_server/set_param`:

`/robot_state_publisher/description_status`

Its value must have this shape:

`{"version": 7, "accepted": true, "error": "", "loaded_version": 7, "root_link": "base_link"}`

The fields mean:

* `version`: the `robot_description` version just processed.
* `accepted`: whether this description was loaded.
* `error`: an empty string on success and a non-empty reason on rejection.
* `loaded_version`: the version of the robot currently in effect, or `0` if none.
* `root_link`: the currently loaded robot's root link, or an empty string if none is loaded.

Versions are relative to the running parameter server. Do not assume a starting version.

After a rejection, `loaded_version` and `root_link` must continue to describe the previous robot. After acceptance, every subsequent `/tf` publication must reflect the newly loaded robot.

## Acceptance criteria

1. The node retrieves descriptions through the parameter server service.
2. Every observed latest description version is processed.
3. A status for version `>= V` is produced within 2 seconds of setting version `V`.
4. Valid descriptions are completely loaded before success is reported.
5. Invalid descriptions produce a useful non-empty error.
6. Rejected loads leave the previous robot and its transforms intact.
7. Accepted reloads replace the previous robot rather than merging its links or joints.
8. The node starts with no robot loaded.
9. Description failures do not crash or hang the runtime.
10. No robot-specific file or startup argument is required.
