# Task: Project 3 Runtime Integration, Makefile, and End-to-End Verification

Integrate the graded Project 3 nodes into a complete submission that satisfies the external TCP/JSON contract.

This task owns runtime assembly, build and launch behavior, and regression verification. Prefer fixing integration defects in their responsible tasks rather than rewriting working subsystems.

## Goal

Produce a self-contained project that builds offline and launches a working parameter server, robot state publisher, and world-state publisher through the existing Project 1 gateway and middleware.

## Makefile contract

Provide these top-level targets:

* `make build`
* `make run`
* `make clean`

Also provide `make demo` for the recommended demo nodes.

### `make build`

* Build noninteractively, repeatably, and offline.
* Use only the permitted course toolchain, libraries, and vendored source.
* Complete within the 180-second setup limit.
* Do not depend on network downloads at build time.

### `make run`

* Launch the existing gateway, middleware, `param_server`, `robot_state_publisher`, and `robot_world_state_publisher`.
* Run in the foreground.
* Listen on `127.0.0.1:9095`.
* Require no input files, environment variables, or command-line arguments.
* Start with no robot loaded.
* Reach a genuinely functional ready state within 10 seconds.

Readiness means an external client can set a small valid URDF through `/param_server/set_param`, observe an accepted description status, and receive the robot's transforms on `/tf` and `/xform_world`.

**Do not launch** `joint_state_publisher`, `finite_state_machine`, or any component that publishes `/joint_states`, `/joint_trajectory`, or `/global_pose` from `make run`. Those inputs come from external clients during grading.

### `make demo`

Launch everything from `make run` plus the recommended joint servo and FSM nodes.

### `make clean`

Remove generated build artifacts without deleting source files, vendored dependencies, specifications, or saved source-controlled examples.

## Integration requirements

* Reuse the existing Project 1 TCP/JSON protocol without inventing a new protocol.
* Ensure all communication hops handle lines up to at least 1 MiB; dynamic line reading is required.
* Keep service response envelopes, message shapes, parameter names, topic names, and version semantics consistent with `spec/FK_API.md`.
* Ensure a successful description status is not reported before the newly accepted robot is active.
* Keep all generated numeric JSON fields numeric and finite. Do not emit `NaN` or `Infinity` as JSON numbers.
* Ensure all services and topic publishers are ready within the specified startup window.
* Do not hard-code expected parameter version numbers.

## Black-box tests

Use an external TCP/JSON client connecting to `127.0.0.1:9095`. Do not treat successful unit tests or internal function calls as a substitute for testing the actual gateway and runtime.

Test at least:

1. Parameter set/get behavior, independent versions, and missing-name rejection.
2. Description acceptance and every specified rejection condition.
3. Failed description loads preserving the previous active robot.
4. Zero-configuration FK with translations, rotations, chains, fixed joints, and branching trees.
5. Root detection when the first link is not the root.
6. Joint-state replacement, ignored names, non-unit axes, and exact timestamps.
7. Global-pose composition into every link's world transform.
8. Publication frequency, complete messages, and late subscribers.
9. Reloads that replace old links and joints.
10. Large URDFs approaching 1 MiB through both set and get parameter requests.
11. Repeated runtime startups, shutdowns, and rejected requests.
12. `make run` not starting the demo input publishers.

Run against the supplied specifications and, when available, unmodified real-robot descriptions including the Fetch URDF and a PR2 URDF.

## Acceptance criteria

* All required Make targets work.
* The project builds offline and meets the build time limit.
* The runtime becomes functional within 10 seconds.
* The complete external client workflow works over port `9095`.
* No demo-only input publishers start under `make run`.
* All core grading requirements have automated regression tests.
* A rejected request does not prevent subsequent valid requests.
* No startup path requires a preloaded robot description.
* The submission can be packaged in the required project-root layout.
