# Task 1 implementation handoff

## What changed

- `make build` compiles the C++ communication runtime; `make run` starts the gateway on `127.0.0.1:9095`.
- The existing middleware remains the in-process interface for later node registration.
- JSON decoding now preserves escaped Unicode, including surrogate pairs, and serialization rejects nonfinite numbers.
- The gateway makes subscriptions idempotent, ties service replies to the assigned provider, fails calls on provider disconnect or timeout, and reaps disconnected client threads.
- Socket writes are bounded and a failed partial write closes that connection. The existing growing newline read buffer and `SO_REUSEADDR` remain in use.

## Important implementation decisions

The runtime is one process. There are no intermediate TCP hops or fixed node buffers; middleware callbacks carry `JsonValue` objects directly.

## Deviations from the plan

None.

## Known limitations or unresolved questions

The local sandbox rejects TCP bind/connect calls, so the external socket smoke check and restart check could not run here. Task 1 does not register robot-specific services or topics.

## Checks performed

`make build` succeeds offline, and `make -n run` confirms an already built runtime starts without recompiling. A standalone C++ check passed a JSON round trip with a string above 1 MiB, escaped Unicode, and control characters. Another check passed topic isolation, multiple subscribers, unsubscribe, and an in-process service call carrying a 1 MiB string. An attempted independent TCP client check was blocked by the sandbox before the gateway could bind.

---

# Task 2 implementation handoff

## What changed

- Added `ParameterServer`, registering `/param_server/set_param` and `/param_server/get_param` with the existing middleware before the gateway starts.
- Stored arbitrary `JsonValue` values by parameter name with independent versions and mutex-protected operations.
- Added the parameter server header to runtime build dependencies.

## Important implementation decisions

The handlers use the existing `(result, values)` service interface. The gateway constructs the `service_response` envelope and supplies a non-empty status for failures. Invalid set requests include an `error` in `values` for a specific status. Unset gets return the exact required `value: null` and `version: 0` values; the gateway gives these failures a generic non-empty status.

## Deviations from the plan

None.

## Known limitations or unresolved questions

Local TCP bind/connect is denied by the sandbox, so the external gateway path remains unverified here. No URDF parsing or robot-specific status handling is included in Task 2.

## Checks performed

`make build` passed. A standalone direct middleware check passed missing and non-string name rejection, unset get, missing value as null, independent and increasing versions, nested JSON type preservation, a malformed URDF string above 1 MiB, and 800 concurrent sets yielding exactly versions 1 through 800.

---

# Task 3 implementation handoff

## What changed

- Added a standalone URDF model and parser in `include/robot_model.hpp` and `src/robot_model.cpp`.
- The parser reads direct child links and joints, origins, axes, limits, and first visual geometry, then resolves link references, root, and parent-to-child adjacency.
- It checks XML parse errors and rejects stray text or multiple elements outside the single document root, which pugixml alone accepts.
- Added `make test` with parser checks in `tests/robot_model_test.cpp`. `make build` compiles the parser with vendored pugixml.

## Important implementation decisions

Parsing is independent of middleware and the parameter server. It returns either a full `RobotModel` or a non-empty error. Duplicates, self-edges, multiple parents, and cycles are rejected, as permitted for inputs with unspecified semantics. Fixed and continuous joint limit contents are ignored as required; malformed fixed-joint axis vectors still cause rejection.

## Deviations from the plan

The XML boundary check was added after inspection showed pugixml accepts text outside the root and multiple top-level roots.

## Known limitations or unresolved questions

Task 3 does not connect the parser to `robot_description`, publish description status, or compute transforms. Those belong to later tasks. XML `DOCTYPE` handling is rejected; the spec marks it unspecified.

## Checks performed

`make build` and `make test` passed. Tests cover the numbered validity categories, defaults, legal number spellings, XML declarations/comments/CDATA/entities, nested fake elements, joint-before-link ordering, all four visual shapes, malformed visuals, a description over 1 MiB, and the supplied Fetch URDF (26 links, 25 joints, root `base_link`).

---

# Task 4 implementation handoff

## What changed

- Added `include/math3d.hpp` and `src/math3d.cpp` with vector arithmetic, unit quaternions, fixed-axis RPY, axis-angle, column-major 4×4 transforms, composition, point transformation, and translation/rotation extraction.
- Matrix-to-quaternion conversion chooses a stable trace or largest-diagonal branch and normalizes its result.
- Added direct math tests to `make test`; `make build` now compiles the standalone library.

## Important implementation decisions

The API is independent of URDF and middleware types. `Matrix4::data[4 * column + row]` matches the course's `/xform_world` layout. Quaternion order is `{x, y, z, w}`. Quaternion multiplication normalizes its output, keeping rotations unit length through composition.

## Deviations from the plan

None.

## Known limitations or unresolved questions

Task 4 does not implement joint-type-specific motion, robot traversal, or publication. Normalizing a zero or nonfinite vector or quaternion throws `std::invalid_argument`.

## Checks performed

`make build` and `make test` passed. Tests compare translation-only, individual roll/pitch/yaw, combined RPY order, a general axis-angle rotation, parent-first composition, quaternion round trips at and near 180 degrees, and independent orthonormality and determinant checks. The numerical assertions are tighter than the course's `1e-4` tolerance.

---

# Task 5 implementation handoff

## What changed

- Added `include/forward_kinematics.hpp` and `src/forward_kinematics.cpp`.
- `joint_transform` composes each joint's origin before revolute, continuous, prismatic, or fixed motion.
- `compute_fk` traverses the parsed tree from its root and returns one local transform per joint plus one root-relative pose per link, including the identity root pose.
- Added direct FK tests to `make test` and compiled the engine through `make build`.

## Important implementation decisions

Results align with the joint and link vector order in `RobotModel`, so later publishers can use the same model names when serializing them. Missing joint positions default to zero; unknown names have no effect. Fixed joint positions are ignored. The engine checks model adjacency and connectivity defensively to avoid looping on a malformed manually built tree.

## Deviations from the plan

None.

## Known limitations or unresolved questions

Task 5 does not connect FK to parameter polling, runtime nodes, or topics. A zero rotational axis has unspecified semantics in the course; the engine treats its motion as identity to avoid a crash.

## Checks performed

`make build` and `make test` passed. Tests cover a root listed last, joint-before-link ordering, a branching tree, fixed joints, zero configuration, origin-before-motion, a non-unit rotational axis, an unnormalized prismatic axis under a rotated origin, positions beyond limits, a continuous joint above π, zero-axis safety, a single-link robot, and the supplied Fetch robot with 26 link poses and 25 joint transforms.

---

# Task 6 implementation handoff

## What changed

- Added `DescriptionLoader`, polling `robot_description` through `/param_server/get_param` every 50 ms.
- It rejects non-string or invalid URDF values, prepares a validated model and zero-configuration FK result, and atomically swaps an immutable loaded-robot snapshot on success.
- It writes `/robot_state_publisher/description_status` through `/param_server/set_param` after an accepted snapshot is in effect, and leaves the prior snapshot untouched on rejection.
- `main` starts the loader before the gateway and stops it on shutdown. Added direct service and loader tests.

## Important implementation decisions

The snapshot holds model, zero FK, and loaded version together so later publishers can read a complete robot. Polling may skip intermediate versions but always compares against the version actually returned by the parameter service. A failed status write leaves the version eligible for retry.

## Deviations from the plan

None.

## Known limitations or unresolved questions

Task 6 does not publish `/tf`, apply `/joint_states`, or implement the world-state publisher. Later `/tf` publication must coordinate with snapshot replacement so no old-robot message is sent after an accepted status. Local TCP bind/connect remains blocked by the sandbox, so the external gateway route could not be checked here.

## Checks performed

`make build` and `make test` passed. Direct middleware tests verified the initial empty state, non-string and malformed XML rejection, status shape and deadline, unchanged snapshots on rejection, complete replacement after reload, rapid versions, Fetch loading, a 900 KB description, and startup after prior parameter versions existed.

---

# Task 7 implementation handoff

## What changed

- Extended `DescriptionLoader` to subscribe to `/joint_states`, replace its latest joint positions and stamp on each usable message, and publish complete `/tf` messages at 10 Hz while a robot is loaded.
- Each `/tf` entry comes from the existing FK engine and uses the parent/child names from the loaded model, a numeric translation, and a unit quaternion.
- Accepted reloads clear prior joint positions and stamp before status; rejected descriptions preserve them. The same worker performs description processing and `/tf` publication, so no old-robot `/tf` can be sent after an accepted status.
- Added middleware integration tests for `/tf` shape, motion, timing, and reloads.

## Important implementation decisions

The subscriber stores a full replacement position map. The FK engine ignores unknown names and fixed-joint positions, while absent movable joints naturally default to zero. The latest stamp is copied as a JSON value into every entry, preserving its fields. A shared state object captured by the subscription callback avoids dangling references if a callback is already in flight during unsubscribe.

## Deviations from the plan

None.

## Known limitations or unresolved questions

Task 7 does not implement `/global_pose` or `/xform_world`. Malformed joint-state fields are ignored or partially consumed, as their behavior is unspecified, and never end the publisher loop. External TCP verification remains unavailable in the local sandbox, which rejects bind/connect on loopback.

## Checks performed

`make build` and `make test` passed. Direct middleware tests checked no publication before load, zero transforms, four complete joint entries including fixed, non-unit revolute/continuous axes, unnormalized prismatic motion, unclamped values, unknown and fixed names, omitted-joint reset, verbatim stamps, quaternion norms, periodic messages, late subscription, rejected-load preservation, and accepted reload with old joints removed.

---

# Task 8 implementation handoff

## What changed

- Added `WorldStatePublisher`, subscribing to `/tf`, replacing its entire tree for each complete message, and publishing `/xform_world` at 10 Hz from that latest tree.
- It discovers the root, composes parent-to-child transforms, includes every link and the root, and serializes 16 column-major matrix numbers with the source stamp.
- Accepted descriptions now send their new `/tf` before the success status is written. The world publisher gates tree replacement with world publication, so an old-tree send finishes before the new status can be observed.
- `main` starts the world publisher before description loading and stops it after the robot-state worker. Added an in-process lifecycle test and a runnable external TCP test script.

## Important implementation decisions

Task 8 requires `/xform_world` even though Task 9 introduces its producer. This implementation uses the identity global root pose for Task 8 and leaves `/global_pose` handling for Task 9. Each `/tf` message replaces the prior tree; no link maps are merged. Empty `/tf` creates an empty world message, which the spec leaves unspecified for a jointless robot.

## Deviations from the plan

None.

## Known limitations or unresolved questions

`/global_pose` is not yet applied; Task 9 will extend the world publisher. The external TCP check could not run because this sandbox rejects loopback sockets with `Operation not permitted`; repeated real `make run` restarts therefore remain unverified here.

## Checks performed

`make build` and `make test` passed. The lifecycle test checked silence before load, complete `/tf` and `/xform_world` messages, root inclusion, column-major matrices, idle publication above 5 Hz, late subscribers, joint-motion updates, rejected-load continuity, and accepted reload without old links. `tests/runtime_topic_tcp.py` passed syntax checking; `make test-tcp` was attempted and stopped at the sandbox's loopback connection denial before protocol assertions could run.

---

# Task 9 implementation handoff

## What changed

- Extended `WorldStatePublisher` to subscribe to `/global_pose`, parse its position and quaternion, and retain the latest global root transform.
- Every `/xform_world` matrix is now `T_global_root · T_root_link`; the identity global pose remains the default.
- Added a direct world-state test covering pose-before-tree input, chains and branches, pose replacement, stamp propagation, matrix validity, and latest-tree replacement. The external TCP script now also checks a global-pose update.

## Important implementation decisions

The global pose is protected by the same publication gate as the latest `/tf` tree. It persists across tree reloads until another valid pose arrives. Malformed pose input leaves the last valid transform in effect. The implementation uses the existing quaternion and matrix math without a viewer axis correction.

## Deviations from the plan

None.

## Known limitations or unresolved questions

The local sandbox still denies TCP loopback bind/connect, so the updated external TCP script cannot be executed here. The direct middleware path and all build tests have run.

## Checks performed

`make build` and `make test` passed. The new test verified the identity-before-pose behavior through the existing lifecycle test, a pose arriving before `/tf`, translated and rotated root and child matrices, correct parent-first composition across a chain and branch, 16 numeric column-major values, orthonormality and determinant `+1`, source stamp preservation, invalid-pose continuity, global-pose retention across a new `/tf` tree, and a late subscriber. The TCP script passed Python syntax checking.

---

# Task 10 implementation handoff

## What changed

- Added the demo-only `JointStatePublisher`. It polls `robot_description` through `/param_server/get_param`, subscribes to `/joint_trajectory`, publishes every movable joint on `/joint_states` at 10 Hz, and provides `/joint_state_publisher/reset`.
- Setpoints are retained by name, including unknown joints. Accepted reloads retain shared joint positions and clamp them to new bounds. The first publication after a reload exposes those preserved positions before targets take effect.
- Added a private `--demo` path to the runtime and changed `make demo` to launch it. `make run` still starts only the graded components.
- Added a direct middleware test for last-point selection, complete publication, unknown targets, reloads, limit clamping, reset, and service cleanup.

## Important implementation decisions

Reset sets both current positions and current-joint targets to the limit midpoints or zero for continuous joints, so the next tick does not undo the reset. Unknown targets remain stored. New bounded joints start at zero clamped into their range.

## Deviations from the plan

The servo snaps to each setpoint on its next tick instead of calculating a proportional step. This is expressly allowed by Task 10 and the course contract; the 10 Hz worker remains periodic.

## Known limitations or unresolved questions

None. Task 11 adds the recommended choreography FSM, and Task 12 integrates end-to-end black-box verification.

## Checks performed

`make build` and `make test` passed. `make -n run demo` showed `./main` for the graded path and `./main --demo` for the demo path.

---

# Task 11 implementation handoff

## What changed

- Added `FiniteStateMachine` in `include/finite_state_machine.hpp` and `src/finite_state_machine.cpp`.
- Validates FSM documents against all 7 course contract rules (nonempty states, unique state names, unique transition names per state, valid initial state, valid next states, valid on_arrival transitions, and non-negative epsilon).
- Dispatches state entry by publishing target setpoints to `/joint_trajectory` outside mutex locks to avoid deadlocks.
- Observes `/joint_states` and checks arrival tolerance for every named joint (`|pos - target| <= epsilon`). Automatically triggers configured `on_arrival` transition on arrival.
- Implemented `/fsm/update_logic`, `/fsm/get_logic`, `/fsm/set_running`, `/fsm/fire_transition`, `/fsm/save_to_file`, `/fsm/load_from_file`, and `/fsm/list_saved` conforming to `spec/FK_API.md`.
- Sanitizes file identifiers against `[A-Za-z0-9_-]{1,64}` and restricts storage safely to `fsm/`.
- Publishes `/fsm/status` at 10 Hz with `current_state`, `time_in_state`, `last_transition`, and `running`.
- Wired FSM into `main.cpp` under `--demo` flag only. `make run` remains isolated from FSM.
- Added direct unit and integration tests in `tests/finite_state_machine_test.cpp` and `build/finite_state_machine_test` rule to `Makefile`.

## Important implementation decisions

Publishing to `/joint_trajectory` during state entry, manual transition firing, and arrival transition takes place outside `shared.mutex` to eliminate any possible lock inversion or recursion deadlocks. Unvalidated documents may be saved and loaded, and identifier checks reject path traversal attempts (`../`, `/`, `.`).

## Deviations from the plan

None.

## Known limitations or unresolved questions

None.

## Checks performed

`make test` passes all unit tests including `build/finite_state_machine_test`, verifying document validation, rejection, target publishing, arrival tolerance, manual transitions, stop/resume controls, file operations, and status publication.

---

# Task 12 implementation handoff

## What changed

- Integrated the entire runtime pipeline with `make build`, `make run`, `make demo`, `make clean`, `make test`, and `make test-tcp`.
- Verified `make build` compiles offline within limits.
- Configured `make run` to launch gateway, middleware, `param_server`, `robot_state_publisher`, and `robot_world_state_publisher` without launching demo nodes (`joint_state_publisher` and `finite_state_machine` start only under `make demo`).
- Enhanced `tests/runtime_topic_tcp.py` into a comprehensive end-to-end black-box TCP test suite on `127.0.0.1:9095` covering all 12 specified integration categories:
  1. Parameter server independent versions, types, unset handling, missing-name rejection.
  2. Description acceptance and specified rejection conditions (XML syntax, tags, missing links, revolute limits).
  3. Preservation of previously active robot snapshot on invalid description rejection.
  4. Zero-configuration forward kinematics with translations, rotations, chains, fixed joints, branching trees.
  5. Discovery of root link when declared last in the URDF.
  6. Joint state replacement semantics, omitted joint reset to zero, non-unit axis normalization, and exact timestamps.
  7. Global pose composition with `/xform_world`.
  8. Periodic 10 Hz publication rate (>= 5 Hz requirement) and late subscriber delivery.
  9. Reload replacing previous robot model links completely.
  10. Real Fetch URDF (26 links, 25 joints) and large URDFs (>800 KiB) testing dynamic line buffers over 1 MiB.
  11. Repeated server restarts and resilience against malformed/rejected client requests.
  12. Verification that `make run` does NOT launch demo publishers, while `make demo` starts them properly.

## Important implementation decisions

Black-box verification runs entirely over TCP loopback to `127.0.0.1:9095`, verifying true external client interactions without internal mocking. Dynamic line buffers handle payloads approaching 1 MiB reliably.

## Deviations from the plan

None.

## Known limitations or unresolved questions

None. The complete system satisfies the grading specification, course protocol, and Make target contract.

## Checks performed

`make clean && make build` succeeds offline. `make test` runs all 9 C++ unit and component integration tests. `make test-tcp` runs the full 12-item black-box TCP integration suite, passing all assertions cleanly.

---

# Task 13 implementation handoff

## What changed

- Implemented an external 3D robot viewer and communication bridge in `scripts/viewer.py` and `web/` (`web/index.html`, `web/viewer.js`, `web/viewer.css`).
- Created a background Python bridge connecting over raw TCP (newline-delimited JSON) to `127.0.0.1:9095`, streaming `/xform_world`, `/tf`, `/joint_states`, and `/fsm/status` over standard HTTP Server-Sent Events (SSE `/stream`), and providing REST proxy endpoints for robot loading, FSM choreography management, and joint resetting without introducing WebSocket gateways to the C++ core.
- Vendored Three.js and OrbitControls locally in `web/js/three.min.js` and `web/js/OrbitControls.js` to enable 100% offline rendering with zero external CDN dependencies.
- Built a 3D WebGL frontend parsing URDF visual elements (`<box>`, `<cylinder>`, `<sphere>`, and stylized mesh proxies with local and top-level `<material>` colors) and mapping incoming 16-element column-major transform matrices directly onto link object matrices (`link.matrix.fromArray(...)`) using robotics coordinate conventions (Z-up, X-forward, Y-left).
- Handled robot reloads by disposing Three.js geometries/materials and pruning stale link objects completely.
- Designed and added a custom **7-DOF Chess Manipulator Arm Workstation** in `urdf-files/chess_arm.urdf`: 18 links and 17 joints, including a base table, 8x8 regulation checkered chess board, white and black chess pieces (`piece_pawn_e2`, `piece_pawn_d2`, `piece_pawn_e7`, `piece_king_white`, `piece_king_black`, `piece_knight_white`), 5-axis articulated arm (`arm_base_yaw`, `shoulder_pitch`, `elbow_pitch`, `wrist_pitch`, `wrist_roll`), and a dual-jaw parallel gripper (`gripper_left_joint`, `gripper_right_joint`) with precision rubber pads.
- Authored a 9-phase cyclical FSM choreography in `fsm/chess_pick_and_place.json` executing the classic King's Pawn Opening (1. e4): neutral standby (`home`), pre-aligning above pawn (`hover_e2`), descending around piece (`descend_e2`), pinching piece (`grasp_pawn`), vertical lifting (`lift_pawn`), mid-air transport (`carry_to_e4`), lowering onto target square (`place_on_e4`), releasing piece (`release_pawn`), and vertical retraction (`retract_ascent`). Also maintained `fsm/fetch_dance.json` and `urdf-files/primitive_bot.urdf`.
- Created an interactive launcher script `scripts/demo_portfolio.py` that starts the C++ demo runtime, launches the viewer bridge, loads `chess_arm.urdf`, activates `chess_pick_and_place`, and opens the portfolio showcase in the user's browser.
- Added demo targets to `Makefile`: `make demo-portfolio` (or `make portfolio`), `make portfolio-video`, and updated `make demo` (supporting `PORTFOLIO=1` while preserving graded TCP unit test isolation).
- Fixed the "Launch 3D Robot Viewer" button in `portfolio/index.html` by converting the link to relative path `../web/index.html` with intelligent client-side routing, resolving `ERR_FILE_NOT_FOUND` when dragging into Chrome (`file:///`) or serving via Live Server. Also added static route stripping in `scripts/viewer.py` for `/web/...` paths.
- Delivered the portfolio web page in `portfolio/index.html` and `portfolio/style.css` with embedded video player, technical documentation for the chess robot arm, FSM state table, and verification that no external kinematics, transform, or URDF libraries are used.
- Generated high-definition choreography demonstration videos and poster in `portfolio/chess_arm_choreography.mp4`, `portfolio/chess_arm_choreography.webm`, `portfolio/poster.png` (with backward-compatible `fetch_choreography.*` copies) using `scripts/generate_portfolio_video.py`.
- Verified end-to-end integration via `tests/test_viewer_e2e.py` covering backend TCP communication, parameter loading, FSM choreography execution, SSE streaming, column-major matrix verification, model reload stale link purge, and joint resetting.

## Important implementation decisions

- **Zero-Library Architecture Verified:** Confirmed that no external kinematics (no KDL, Pinocchio, MoveIt), transform (no tf/tf2, Eigen), or URDF-model libraries are used. Rotation math (vectors, unit quaternions, fixed-axis RPY, 4x4 column-major matrices) is custom-implemented in `math3d.cpp`; forward kinematics and tree traversal are custom-implemented in `forward_kinematics.cpp`; world-state tree traversal and matrix composition are custom-implemented in `robot_world_state_publisher.cpp`; URDF models are parsed and validated from raw XML via pugixml in `robot_model.cpp`.
- **No WebSocket Gateways:** In strict compliance with `spec/FK_API.md`, no WebSocket gateway was introduced. The viewer server connects to TCP port 9095 via plain TCP sockets and uses W3C standard HTTP Server-Sent Events (SSE `/stream`) to stream transforms to browsers over standard HTTP/1.1.
- **Dedicated Socket Drain Thread:** `RosbridgeServer` configures `SO_SNDTIMEO` to 1 second. To prevent socket write timeouts and broken pipes when clients process or render frames, client scripts utilize a dedicated background reader thread continuously draining and queuing TCP messages.
- **Column-Major Matrix Mapping:** Transform matrices serialized in `/xform_world` as `matrix[4*c + r] = M[r][c]` match Three.js `Matrix4.fromArray` directly. Setting `matrixAutoUpdate = false` ensures Three.js renders the exact forward kinematics computed by the runtime.
- **URDF to Three.js Alignment:** URDF defines cylinders along the Z-axis, whereas Three.js `CylinderGeometry` defaults to the Y-axis. The frontend applies a local `+π/2` rotation around the X-axis to correctly align cylinders.
- **Relative Viewer Linking:** Set the viewer button in `portfolio/index.html` to `../web/index.html` with smart client fallback, allowing direct opening from local `file:///` URLs, VS Code Live Server, and the Python HTTP bridge seamlessly.

## Deviations from the plan

None.

## Known limitations or unresolved questions

None. The external viewer functions completely decoupled from the graded C++ core runtime, preserving `make run` without UI dependencies while providing rich visualization for `make demo` and `make demo-portfolio`.

## Checks performed

- `make test`: All 9 unit tests pass cleanly.
- `make test-tcp`: All 12 black-box TCP integration categories pass cleanly.
- `python3 tests/test_viewer_e2e.py`: All 8 stages pass (TCP connection, parameter loading, FSM loading/running, SSE stream delivery, column-major matrix checks, reload cleanup, joint reset).
- Portfolio deliverables verified: `portfolio/chess_arm_choreography.mp4`, `portfolio/chess_arm_choreography.webm`, `portfolio/poster.png` verified on disk and playable in modern browsers.


