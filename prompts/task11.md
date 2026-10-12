# Task: Recommended `finite_state_machine` Choreography Node

Implement the demo-only finite-state machine (FSM) that runs a choreography of named joint-space setpoints.

This task depends on the joint trajectory and joint state message contracts. Do not implement a new robot control protocol.

## Goal

Represent a choreography as a table of named states, each containing a joint-space target, an arrival tolerance, and named transitions.

Publish the current state's target to `/joint_trajectory`, observe `/joint_states`, and transition when the configured arrival condition is satisfied.

## FSM document

Use the following document shape:

`{"initial_state":"home","states":[{"name":"home","target":{"joint_names":["shoulder","elbow"],"positions":[0.0,0.0]},"epsilon":0.02,"on_arrival":"to_wave","transitions":[{"name":"to_wave","next_state":"wave"}]}]}`

A valid table must have:

* At least one state.
* Unique state names.
* Unique transition names within each state.
* An existing state named by `initial_state`.
* Valid `next_state` values for every transition.
* An `on_arrival`, if supplied, that names a transition in that same state.
* `epsilon >= 0`.

An invalid replacement must leave the existing valid table running.

## Runtime behavior

* On entering a state, publish its target on `/joint_trajectory`.
* Observe `/joint_states`.
* Consider a state reached when every named joint is within its `epsilon` of the state's target.
* Follow the configured `on_arrival` transition on arrival when one exists.
* Allow manual transitions when the FSM is running.
* Publish `/fsm/status` approximately 10 times per second, with `current_state`, `time_in_state`, `last_transition`, and `running`.

## Services

Implement the following services using the standard response envelope:

* `/fsm/update_logic`: validate and replace the current table, enter the initial state, and start running.
* `/fsm/get_logic`: return the current table.
* `/fsm/set_running`: accept `{"data": true}` or `{"data": false}`; if `data` is omitted, report the current running state.
* `/fsm/fire_transition`: accept `{"transition": "<name>"}` and manually take a transition; reject while stopped.
* `/fsm/save_to_file`: save an FSM document under `fsm/<identifier>.json`.
* `/fsm/load_from_file`: retrieve a saved document without deploying it.
* `/fsm/list_saved`: return saved identifiers in sorted order.

Use the document and service contracts in `spec/FK_API.md` as the authoritative API.

## File safety

Validate identifiers against:

`[A-Za-z0-9_-]{1,64}`

Do not allow saving or loading paths outside the `fsm/` directory.

Saving and loading do not themselves deploy a choreography; use `/fsm/update_logic` to validate and run a loaded document.

## Acceptance criteria

1. Valid logic tables load and begin at `initial_state`.
2. Invalid logic tables are rejected without replacing the active table.
3. Entering a state publishes its target.
4. Arrival checks consider every named joint and the state's `epsilon`.
5. Automatic and manual transitions behave as specified.
6. The running state can be started and stopped.
7. FSM status is published at approximately 10 Hz.
8. Save, load, and list operations follow the service contracts.
9. Invalid identifiers cannot escape the `fsm/` directory.
10. The node is launched by `make demo` only, not `make run`.
