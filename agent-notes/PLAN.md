# Task 1: Communication foundation

## Goal

Run the existing C++ Project 1 middleware and TCP/JSON gateway as the Project 3 communication layer, without adding robot services or topics.

## Relevant requirements

- Preserve the rosbridge subset in `spec/ROSBRIDGE_PROTOCOL.md` on `127.0.0.1:9095`.
- Carry request and response lines of at least 1 MiB through the gateway and middleware.
- Support concurrent clients, topic routing, correlated service calls, cleanup, and repeated starts.
- Build offline with the included C++ code.

## Repository observations

- The reusable layer is header only in `include/middleware/`.
- `src/main.cpp` was a placeholder, and `Makefile` did not compile the middleware.
- The gateway already accumulated reads until newline and enabled `SO_REUSEADDR`.
- Internal node communication uses direct `JsonValue` calls, so it has no separate line buffer.

## Proposed architecture

Instantiate one in-process `Middleware` and `RosbridgeServer` in the runtime. Keep transport and application logic separate. Extend the existing gateway only where its current behavior violates Task 1 or the protocol.

## Interfaces and data flow

External TCP client → newline JSON decoder → gateway operations → in-process middleware callbacks or service handlers → gateway serializer → external TCP client. Later nodes can register directly with `Middleware`.

## Implementation steps

1. Wire the existing headers into `make build` and the runtime entry point.
2. Preserve growing line buffers and full writes; correct JSON escape handling.
3. Correct service provider ownership, bounded pending calls, and connection cleanup.
4. Check compilation and exercise large JSON round trips and external TCP behavior where permitted.

## Edge cases and failure modes

Malformed JSON, escaped non-ASCII characters, repeated subscriptions, disconnected service providers, unresponsive providers, slow subscribers, partial socket writes, and repeated client connections.

## Open questions and assumptions

Task 1 intentionally leaves robot-specific services and topics for later tasks. The sandbox may prevent a local TCP smoke check, so socket behavior may require verification in a less restricted environment.

## Verification strategy

Build offline. Check JSON round trips above 1 MiB. Use independent TCP clients to check publish/subscribe, service calls, malformed input, disconnects, and restart once local networking is available.

---

# Task 2: Parameter server

## Goal

Register `/param_server/set_param` and `/param_server/get_param` on the existing middleware, with independent per-name versions and exact JSON value storage.

## Relevant requirements

- A successful set stores any JSON value, including malformed URDF text, and returns the new version.
- Missing `value` stores `null`; missing or non-string `name` fails without modifying storage.
- An unset get fails with `value: null`, `version: 0`, and a non-empty response status.
- Concurrent requests must not race.
- This task does not interpret robot descriptions or publish robot topics.

## Repository observations

`Middleware` registers handlers returning `(result, values)`. The gateway wraps these in the protocol envelope and supplies a non-empty status on failure. The server and gateway share one process, so a mutex-protected map is sufficient.

## Proposed architecture

Add a small `ParameterServer` class that owns a map from names to `{JsonValue, version}` and registers the two handlers. Construct it before starting the gateway. Use one mutex for atomic reads and version increments.

## Interfaces and data flow

External `call_service` → gateway → middleware handler → parameter map → middleware result → gateway `service_response`.

## Implementation steps

1. Add the parameter server class and handlers.
2. Register it in `main` and add the header to build dependencies.
3. Check invalid names, missing values, independent versions, large strings, JSON type preservation, and concurrent calls.

## Edge cases and failure modes

Non-object arguments, absent names, numeric/null names, absent values, never-set names, repeated sets, arbitrary parameter names, and concurrent writers.

## Open questions and assumptions

The specification does not prescribe `values` for an invalid set request. Return a diagnostic `error` field so the existing generic gateway can put a useful message in `status`. For an unset get, preserve the specified `values` shape; the gateway's generic non-empty failure status satisfies the contract.

## Verification strategy

Compile the runtime and run a direct middleware integration check, including concurrent sets. Attempt an external TCP client check if the sandbox permits local networking.

---

# Task 3: URDF model and validation

## Goal

Parse a flat URDF string into a complete, validated robot model without loading it into the runtime or computing FK.

## Relevant requirements

Follow all nine numbered validity rules in `spec/ROBOT_DESCRIPTION.md`. Read only direct child links and joints. Preserve the first visual, origin, axis, and limit values needed by later tasks. Accept real XML syntax and the specified defaults and decimal spellings.

## Repository observations

The vendored pugixml 1.14 parser is available. The current runtime has no robot model or parser. The provided example demonstrates direct child iteration and parse result checking.

## Proposed architecture

Add `include/robot_model.hpp` for simple value types and `src/robot_model.cpp` for parsing and validation. Return a result holding either a model or non-empty error. Use pugixml only for XML syntax and entity decoding. Use a dedicated decimal parser for specified numeric spellings.

## Interfaces and data flow

URDF string → pugixml document → direct child link/joint records → cross-reference and root validation → `RobotModel`. No middleware or parameter server dependency.

## Implementation steps

1. Define link, joint, origin, visual, and robot model value types.
2. Parse XML and read required fields with defaults.
3. Validate numeric attributes, visuals, link references, and unique root; construct adjacency.
4. Build with the vendored parser and check the supplied Fetch URDF plus hand-picked valid and invalid cases.

## Edge cases and failure modes

Malformed XML, escaped names, joints before links, unknown nested elements, empty descriptions, invalid number counts, malformed first visuals, missing limits, duplicate names, cycles, and zero-length movable axes. Inputs the spec marks unspecified may be rejected, but the parser must not crash or hang.

## Open questions and assumptions

The spec does not require a public model layout, so use straightforward structures for later FK tasks. Duplicate names, multiple parents, and cycles have unspecified semantics; reject them with an error to keep subsequent traversal well-defined.

## Verification strategy

Compile offline. Check all numbered validity categories, legal defaults and number forms, real-world XML forms, direct-child filtering, and Fetch. Keep parser tests in a standalone harness so this task does not add runtime robot behavior.

---

# Task 4: 3D transform mathematics

## Goal

Provide standalone, reusable vector, unit quaternion, and homogeneous transform operations for later FK and world-state code.

## Relevant requirements

Use fixed-axis `Rz(yaw) · Ry(pitch) · Rx(roll)`, axis-angle rotation, column-major 4×4 matrices, parent-first transform composition, and robust matrix-to-quaternion conversion near 180 degrees. Meet `1e-4` translation and rotation tolerances without a transform library.

## Repository observations

The URDF model uses `std::array<double, 3>` for vectors. No transform code exists. `Makefile` already offers a `test` target for parser tests.

## Proposed architecture

Add `include/math3d.hpp` and `src/math3d.cpp` with a `math3d` namespace. Use `std::array<double, 3>` vectors, `{x,y,z,w}` quaternions, and flat column-major 4×4 matrices. Keep the library independent of the robot model and middleware.

## Interfaces and data flow

Vector and angle inputs → normalized quaternions → rotation matrices and homogeneous transforms → composition and extraction. Later tasks can convert a parsed joint's origin and motion into these primitives.

## Implementation steps

1. Implement vector arithmetic, norm, and safe normalization.
2. Implement quaternion normalization, multiplication, fixed-axis RPY, and axis-angle.
3. Implement quaternion/matrix conversion, transform construction, multiplication, and extraction.
4. Add direct numerical tests for axis rotations, combined RPY order, general axis-angle, composition, orthonormality, and near-π conversion.

## Edge cases and failure modes

Zero-length vectors and quaternions cannot be normalized and should fail clearly. Re-normalize quaternions at representation boundaries to avoid drift. Near-180-degree conversion must choose a stable branch instead of dividing by a tiny trace term.

## Open questions and assumptions

The public math API is not specified by the course. Use simple value types with column-major indexing documented in the header. Joint-type-specific motion and tree traversal remain for later tasks.

## Verification strategy

Run `make build` and `make test`. Compare transforms against hand-computed cases and independently check matrix orthonormality and determinant.

---

# Task 5: Reusable forward kinematics

## Goal

Compute one parent-to-child transform per joint and one root-relative pose per link from a validated `RobotModel` and a map of joint positions.

## Relevant requirements

- Compose `T_origin · T_motion(q)` for every joint, including fixed joints.
- Normalize rotational axes; use prismatic axes exactly as written.
- Traverse parent first from the discovered root, across chains and branches.
- Treat unspecified positions as zero and do not clamp to limits or apply mimic behavior.
- Keep the engine independent of runtime services and topics.

## Repository observations

The parser already resolves the unique root, parent/child indices, and adjacency, including joint-before-link ordering. `math3d` provides column-major matrices, RPY origins, axis-angle quaternions, translation, and composition.

## Proposed architecture

Add `include/forward_kinematics.hpp` and `src/forward_kinematics.cpp`. Return indexed vectors of joint local transforms and root-relative link poses, aligned to `RobotModel::joints` and `RobotModel::links`. Use an iterative tree traversal and validate indices defensively to avoid hangs on malformed manually built models.

## Interfaces and data flow

`RobotModel` plus name-to-position map → local joint transforms → parent-first link traversal → complete `FKResult`. Later nodes can use model names and result indices to serialize `/tf` and `/xform_world`.

## Implementation steps

1. Implement joint origin and joint motion composition for each type.
2. Traverse the model tree from its root and fill all transform slots.
3. Add hand-checked tests for zero configuration, root ordering, chains, branches, origin-before-motion, non-unit axes, fixed joints, and a real robot.

## Edge cases and failure modes

Empty position map, unknown names, fixed joint names in the map, zero-length movable axes (unspecified), and invalid manually constructed trees. A zero axis can safely produce identity rotation to avoid a runtime crash on unspecified input.

## Open questions and assumptions

The task does not define an FK C++ API. Use indexed arrays for predictable correspondence with the parsed model. The parser already rejects cycles and multiple parents; the FK engine will still detect invalid adjacency to prevent nontermination.

## Verification strategy

Build offline and run direct engine tests with hand-computed translations and rotations. Include the Fetch URDF as a larger tree check. No TCP or runtime node is needed for this task.

---

# Task 6: Robot description loading

## Goal

Poll `robot_description` through `/param_server/get_param`, validate each observed new version, atomically replace the loaded robot on success, and write `/robot_state_publisher/description_status` through `/param_server/set_param`.

## Relevant requirements

- Start with no robot loaded and no file or argument dependency.
- Report a status for version `>= V` within 2 seconds of a set returning V; newer versions may supersede intermediate versions.
- Reject non-string or invalid URDF values with a useful error and preserve the previous robot, joint state, and transforms.
- Write accepted status only after the new robot is fully in effect.
- Keep robot-specific TF publication and moving-joint handling for later tasks.

## Repository observations

The parameter server provides in-process middleware services; the parser returns a validated model; FK computes a complete zero-configuration result. `main` currently starts the parameter server and gateway only.

## Proposed architecture

Add a description loader class with one polling thread and an immutable, shared loaded-robot snapshot containing the model, zero-configuration FK, and loaded version. Prepare a candidate snapshot completely before swapping it under a mutex. Publish status through the parameter service after the swap. A rejected value leaves the snapshot unchanged.

## Interfaces and data flow

`set_param(robot_description)` → parameter service → polling `get_param` call → URDF parse and zero FK → atomic snapshot swap → `set_param(description_status)`. Later TF code can read the snapshot through a thread-safe accessor.

## Implementation steps

1. Add the loader, thread lifecycle, service calls, validation, atomic replacement, and status handling.
2. Construct and start it in the runtime before accepting external clients; stop it on shutdown.
3. Test valid load, invalid non-string/XML, preserved prior snapshot, reload replacement, rapid versions, and Fetch without a TCP socket.

## Edge cases and failure modes

Unset parameter, malformed parameter value, parser/FK exception, rapid sets, repeated invalid versions, and shutdown while polling. Exceptions must become rejection statuses without terminating the runtime.

## Open questions and assumptions

Task 6 does not define a class API. Use a snapshot accessor for later publishers. The status parameter is expected to be writable because `ParameterServer` is created before the loader; if a service call fails, retry the version on the next poll.

## Verification strategy

Run `make build` and `make test`. Use direct middleware calls to test service-only description loading and enforce a 2-second status deadline. The local sandbox has rejected TCP binding in prior tasks, so external gateway verification may remain unavailable here.

---

# Task 7: Joint states and `/tf`

## Goal

Subscribe to `/joint_states`, apply each complete replacement message to the loaded robot, and publish a complete `/tf` message at least 5 Hz while a robot is loaded.

## Relevant requirements

- Include every joint, fixed joints included, exactly once per message.
- Use `T_origin · T_motion(q)` from the existing FK engine; no clamping or wrapping.
- Missing movable joints return to zero; unknown and fixed names do not affect valid entries.
- Copy the latest `header.stamp` verbatim to every transform, defaulting to `{sec:0,nanosec:0}`.
- Do not publish before a robot loads. Serve late subscribers within one period.
- No old-robot `/tf` may be published after an accepted description status.

## Repository observations

`DescriptionLoader` currently has the only model swap and status writer. `Middleware` supports direct subscriptions and publications. A single worker already polls descriptions, so it can also publish `/tf` in a known order relative to the status write.

## Proposed architecture

Extend the loader's shared state with latest joint positions and stamp. Register a `/joint_states` callback that validates the useful fields and atomically replaces those values. Have the loader's worker poll descriptions and publish `/tf` every 100 ms. It prepares a complete FK result and serializes its joint transforms using `math3d` translation and quaternion extraction. A successful reload clears the old joint state before status; a rejection leaves it unchanged.

## Interfaces and data flow

External `/joint_states` publication → middleware callback → latest joint state. Each worker tick: parameter poll/reload → current model and joint-state snapshot → `compute_fk` → complete `/tf` message → middleware subscribers/gateway.

## Implementation steps

1. Add joint-state subscription and state replacement with safe handling of malformed fields.
2. Add complete `/tf` serialization and periodic publication using the FK engine.
3. Coordinate reload, status, and publication in one worker thread.
4. Test zero configuration, motion, stamps, replacement, fixed/unknown names, periodic rate, late subscribers, rejection, and accepted reload.

## Edge cases and failure modes

No robot loaded, one-link robot with empty transforms, malformed message arrays, unknown names, duplicate names, non-finite positions, and concurrent joint messages during a reload. The spec leaves malformed input unspecified, but it must not crash or hang.

## Open questions and assumptions

The spec leaves joint-state carry-over across accepted reloads unspecified; reset positions and stamp to zero on acceptance. Rejection preserves both. Keep `/global_pose` and world-state publishing for later tasks.

## Verification strategy

Use direct middleware integration tests, including timed periodic messages and a description reload. Build and run the full test target. External TCP checks may remain unavailable under the local sandbox's network restrictions.

---

# Task 8: Topic lifecycle and continuous publication

## Goal

Ensure `/tf` and `/xform_world` remain complete and live at 5 Hz or more through idle periods, late subscriptions, rejections, and accepted reloads.

## Relevant requirements

- `/tf` is silent before a robot loads and continues publishing complete sets afterward.
- `/xform_world` starts after `/tf`, includes the root and all links, and replaces each prior tree.
- New status must not be followed by an old-robot topic publication.
- External TCP/JSON behavior must be checked when local networking is available.

## Repository observations

Task 7 already publishes `/tf` at 10 Hz. Task 9 defines the full world-state publisher but is not yet implemented, while Task 8 requires `/xform_world` now. A small world publisher must therefore be introduced here, composing `/tf` with the identity global root pose; Task 9 will extend it for `/global_pose`.

## Proposed architecture

Add a world publisher that subscribes to `/tf`, treats every message as a complete tree, computes root-relative link matrices with the existing math library, and publishes `/xform_world` at 10 Hz. A publication gate serializes old world sends with tree replacement. Have the description loader send the first new `/tf` before its accepted status, so world state is updated before that status is observable.

## Interfaces and data flow

Description loader `/tf` → world publisher tree replacement → periodic `/xform_world` for all links. The gateway exposes both unchanged. Task 9 will add latest `/global_pose` composition before serialization.

## Implementation steps

1. Add `/tf` tree ingestion, root discovery, traversal, and full world message serialization.
2. Add the 10 Hz world publication loop and lifecycle-safe subscription cleanup.
3. Adjust accepted status ordering to publish the new `/tf` first.
4. Test idle streams, late subscribers, reloads, rejection, and stamp/matrix shape directly; add a runnable external TCP check for an environment that permits loopback sockets.

## Edge cases and failure modes

No `/tf` yet, empty `/tf`, malformed trees, a slow subscriber, and concurrent world publishing during a reload. The course leaves empty-tree world content unspecified; publishing an empty array is safe.

## Open questions and assumptions

`/global_pose` is assigned to Task 9, so Task 8 uses the identity global root pose. The course permits one process and direct middleware subscriptions. Local sandbox rules have previously denied TCP bind/connect, so external validation may be limited to a prepared black-box script.

## Verification strategy

Run `make build` and direct integration tests. Attempt the external TCP script only if loopback networking is permitted; otherwise record the exact limitation without treating unrun checks as passed.

---

# Task 9: Global world-state composition

## Goal

Extend the existing `/tf`-driven world publisher to compose the latest `/global_pose` with each root-relative link matrix, publishing every link in `global_frame`.

## Relevant requirements

- The default global root pose is identity; each valid `/global_pose` replaces it.
- Use `T_global_link = T_global_root · T_root_link` with translation and unit quaternion orientation.
- Keep using only the latest complete `/tf` tree, including its root and stamp.
- Emit exactly 16 column-major numeric matrix values per link at the existing periodic rate.
- Do not add a viewer axis correction.

## Repository observations

Task 8 already discovers the `/tf` root, traverses chains and branches, replaces old trees, and publishes root-relative matrices at 10 Hz. The shared state mutex gates tree replacement and publication, so it can also guard the latest global pose.

## Proposed architecture

Add an independent `/global_pose` subscription to `WorldStatePublisher`. Parse a valid Pose into `math3d::Matrix4`, then atomically replace one shared global transform. During each world publication, multiply that transform by every stored root-relative pose. Preserve the latest pose across `/tf` reloads.

## Interfaces and data flow

`/global_pose` → translation and quaternion → shared global matrix. `/tf` → latest root-relative tree. Periodic publisher combines both snapshots into `/xform_world`.

## Implementation steps

1. Add pose subscription, parsing, replacement, and cleanup.
2. Compose the global matrix with each stored link matrix at publication.
3. Test default identity, translated and rotated global poses, pose replacement, chain and branch, stamp propagation, and latest-tree replacement using direct middleware publications.

## Edge cases and failure modes

Malformed pose input must not terminate publishing. The spec guarantees valid, unit orientations during grading; normalizing the parsed quaternion provides stable matrices. A pose before `/tf` should be retained and applied once a tree arrives.

## Open questions and assumptions

The supplied spec says `/global_pose` is a Pose without a header. It does not reset on robot reload, so keep the latest pose until another pose arrives.

## Verification strategy

Run `make build` and `make test`, including the new direct world-state test. The external TCP harness from Task 8 is available, but this sandbox has denied loopback sockets in prior runs.

---

# Task 10: Demo joint servo

## Goal

Add a demo-only joint state publisher that reads `robot_description` through the parameter service, consumes `/joint_trajectory` setpoints, and publishes all movable joints at about 10 Hz.

## Relevant requirements

- Use the last trajectory point and retain targets by joint name, including unknown names.
- Preserve shared joint positions across robot reloads, clamped to new limits where relevant.
- Move toward targets with a proportional servo or snap, and clamp revolute/prismatic joints.
- Provide `/joint_state_publisher/reset`, setting bounded joints to limit midpoints and continuous joints to zero.
- Launch under `make demo` only; graded `make run` must not publish `/joint_states`.

## Repository observations

The runtime is one process with reusable parameter services and middleware. The URDF parser supplies joint types and limits. `make demo` is still a placeholder; `make run` launches the graded nodes from `src/main.cpp`.

## Proposed architecture

Add a `JointStatePublisher` component with a 10 Hz worker, a `/joint_trajectory` subscription, and a reset service. It polls `robot_description` through `/param_server/get_param`, keeps setpoints and current positions in a mutex-protected shared state, and publishes a complete JointState for every loaded robot. The main program accepts a private `--demo` flag used only by `make demo`.

## Interfaces and data flow

`robot_description` service → parsed model/version → movable joint positions. `/joint_trajectory` → retained targets by name. Each worker tick applies the servo and publishes `/joint_states`; the graded robot-state publisher receives those messages through the existing middleware.

## Implementation steps

1. Add description polling, target and position state, trajectory callback, reset service, and periodic JointState publication.
2. Add the demo-only entry path and Makefile target without changing `make run`.
3. Test last-point setpoints, all movable joints, unknown targets, reload preservation and clamping, reset behavior, and 10 Hz publication through direct middleware calls.

## Edge cases and failure modes

No robot loaded, malformed trajectory fields, invalid descriptions, fixed joints, missing limit bounds (default zero), reversed bounds, and concurrent trajectory or reset calls. Invalid messages should not crash or hang the demo node.

## Open questions and assumptions

The recommended node has no graded numeric servo tolerance; use gain 50 per second with a step factor capped at 1, as the reference describes. On a valid reload, publish preserved positions once before advancing toward targets, making preservation observable. No status service is required for this demo node.

## Verification strategy

Run `make build` and direct middleware tests. Check `make -n run` and `make -n demo` command lines to confirm the servo is demo-only. External TCP tests remain unavailable under this sandbox's loopback restriction.

---

# Task 11: Choreography finite state machine

## Goal

Add the demo-only finite state machine (FSM) that drives robot choreographies by publishing joint-space targets on `/joint_trajectory`, observing `/joint_states` arrival within `epsilon`, and executing automatic and manual transitions.

## Relevant requirements

- Validate logic table with all 7 validity rules (`initial_state` exists, nonempty states, unique state and transition names, valid `next_state` references, valid `on_arrival` reference, `epsilon >= 0`).
- Reject invalid tables without replacing the active table.
- Publish current state's target to `/joint_trajectory` on entering state.
- Check arrival against `/joint_states` considering all named joints within `epsilon`.
- Automatically fire `on_arrival` transition when arrival condition is met.
- Support manual transitions via `/fsm/fire_transition` when running; reject when stopped.
- Implement `/fsm/update_logic`, `/fsm/get_logic`, `/fsm/set_running`, `/fsm/save_to_file`, `/fsm/load_from_file`, `/fsm/list_saved`.
- Enforce identifier safety: `[A-Za-z0-9_-]{1,64}` confined to `fsm/`.
- Publish `/fsm/status` at approximately 10 Hz.
- Launch under `make demo` only, not `make run`.

## Proposed architecture

Add `FiniteStateMachine` class owning a worker thread and shared mutex-protected state. To prevent deadlock, compute transitions and trajectory structures under mutex but invoke `middleware_.publish("/joint_trajectory", ...)` outside the lock. Connect `/joint_states` subscriber to arrival observer. Wire into `main.cpp` behind the private `--demo` flag.

## Interfaces and data flow

Choreography table → `/joint_trajectory` → `JointStatePublisher` (demo servo) → `/joint_states` → `FiniteStateMachine` observe → arrival check → automatic transition → `/joint_trajectory`. External clients query `/fsm/*` services and observe `/fsm/status`.

## Verification strategy

Direct unit testing in `tests/finite_state_machine_test.cpp` and integration under `make demo`.

---

# Task 12: Runtime integration and end-to-end verification

## Goal

Integrate all Project 3 components, enforce Make target contracts (`make build`, `make run`, `make demo`, `make clean`), and verify complete system functionality over external TCP loopback (`127.0.0.1:9095`).

## Relevant requirements

- `make build` builds offline within the 180-second limit.
- `make run` launches only the graded components (`param_server`, `robot_state_publisher`, `robot_world_state_publisher`), reaching readiness in <10 seconds.
- `make demo` launches graded components plus `joint_state_publisher` and `finite_state_machine`.
- Dynamic newline reading supports lines up to and exceeding 1 MiB without fixed-size truncation.
- Black-box TCP suite tests all 12 specified functional categories over `127.0.0.1:9095`.

## Verification strategy

1. Unit and direct integration suite (`make test`) running all 9 standalone test targets.
2. Comprehensive end-to-end black-box TCP test suite (`make test-tcp` / `tests/runtime_topic_tcp.py`) exercising all 12 integration items over real TCP connections on port 9095 against `make run` and `make demo`.
