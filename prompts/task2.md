# Task: Parameter Server and `/param_server` Services

Implement the reusable parameter server required by Project 3.

This task is responsible only for parameter storage, per-name version tracking, and the two parameter services. Do not parse URDF, validate robot descriptions, or compute transforms.

## Goal

Provide these services through the existing middleware:

* `/param_server/set_param`
* `/param_server/get_param`

Both services use the standard response envelope:

`{"values": {...}, "result": <bool>, "status": <string>}`

## `/param_server/set_param`

Accept:

`{"name": "<name>", "value": <any JSON>}`

Store the value under the supplied name and return:

`{"version": <n>}`

Each parameter name has an independent version counter.

* An unset parameter begins at version `0`.
* The first successful set increments its version to `1`.
* Every subsequent successful set increments it by `1`.
* The parameter server stores values without interpreting them. In particular, it must accept and store malformed URDF text.

Reject requests whose `name` is missing or is not a string. Return `result: false` and a non-empty status message.

If `value` is absent, store JSON `null`.

## `/param_server/get_param`

Accept:

`{"name": "<name>"}`

For an existing parameter, return its stored value and current version:

`{"value": <stored value>, "version": <n>}`

For a parameter that has never been set, return `result: false` and:

`{"value": null, "version": 0}`

## Requirements

* Preserve JSON value types.
* Maintain independent versions for different names.
* Do not hard-code any expected version numbers.
* Make parameter reads and writes safe for the runtime's threading or process model.
* Make the services available through the existing Project 1 middleware.
* Follow the exact service contract in `spec/FK_API.md` if it is present in the repository.

## Acceptance criteria

1. Both services are callable through the TCP/JSON gateway.
2. Setting a new parameter returns version `1`.
3. Setting the same parameter repeatedly increments only that parameter's version.
4. Setting another parameter does not change the first parameter's version.
5. Missing and non-string names are rejected cleanly.
6. Unset parameters return the required failure response.
7. Arbitrary JSON values are stored and retrieved without semantic modification.
8. Missing `value` is stored as `null`.
9. Malformed URDF text can be stored successfully.
10. Failed requests do not crash the server or corrupt unrelated parameters.
