# Task: URDF Parser, Robot Model, and Description Validation

Implement the reusable URDF parsing and validation layer for Project 3.

This task converts URDF XML text into an internal robot model. Do not implement the parameter server, runtime description polling, or forward kinematics.

## Goal

Parse a real, flat, xacro-expanded URDF and construct a structured representation of its links and joints.

The implementation must follow `spec/ROBOT_DESCRIPTION.md`. The numbered validity list in that specification is authoritative.

## Required model information

For each direct child `<link>` of `<robot>`, record its name and first `<visual>` if present.

For each direct child `<joint>` of `<robot>`, record:

* Name and type.
* Parent and child link names.
* Origin translation `xyz` and rotation `rpy`.
* Axis `xyz`.
* Limit information.

Support joint types:

* `revolute`
* `continuous`
* `prismatic`
* `fixed`

A missing origin, `xyz`, or `rpy` defaults to zero. A missing axis defaults to `1 0 0`. The robot name is optional and defaults to `"robot"`.

A revolute or prismatic joint must have a `<limit>` element. Limits are informational for forward kinematics and must not be used to clamp joint positions.

## XML handling

* Only direct children of `<robot>` count as links and joints.
* Ignore nested links or joints inside unrelated elements such as `<gazebo>` or `<transmission>`.
* Accept XML declarations, comments, namespace declarations such as `xmlns:xacro`, entity references, CDATA, single or double quotes, self-closing tags, CRLF line endings, and joints appearing before links.
* Decode XML entities before comparing names.
* Ignore collision and inertial content, materials, transmissions, mimic metadata, safety-controller metadata, unknown attributes, and other unspecified elements.
* If a first `<visual>` exists, validate its geometry according to the specification. Supported geometry types are box, cylinder, sphere, and mesh.
* Parse exactly three decimal numbers for each `xyz`, `rpy`, and axis vector, accepting ordinary numeric spellings such as `.25`, `3.`, and `1.5e-1`.
* Reject malformed XML and a root element other than `<robot>`.
* Reject a description with no direct-child links.

## Joint and link validity

Reject descriptions that violate any specified validity rule, including:

* A link missing its `name`.
* A joint missing its `name` or `type`.
* A joint missing its direct-child `<parent>` or `<child>` with a `link` attribute.
* A joint referencing a link not in the robot.
* A joint with an unsupported type.
* A robot with anything other than exactly one root.
* A revolute or prismatic joint without `<limit>`.
* Invalid numeric vector attributes.
* Invalid first visual geometry.

The specification identifies some cases as unspecified, including duplicate names, cycles, and zero-length axes for movable joints. Such cases must not crash or hang the runtime.

## Parsing and validation behavior

* Return either a complete validated model or a clear error.
* Do not modify any currently loaded robot.
* Do not conflate XML parsing with parameter-server storage.
* Keep the parser independent of middleware and runtime publishing.

## Acceptance criteria

* Valid descriptions produce a complete internal model.
* Invalid descriptions produce a non-empty error.
* Nested fake links and joints are ignored.
* Legal defaults and number spellings are handled correctly.
* Joint-before-link ordering is supported.
* XML entities are decoded before name comparisons.
* The first visual geometry is validated.
* Every specified validity rule is tested.
* The parser does not depend on a URDF, kinematics, or transform library.
* The supplied XML parser may be reused, but XML syntax errors must be detected reliably.
* Parsing is independent of the runtime's parameter server and publication nodes.
