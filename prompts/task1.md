# Task: Reuse Project 1 Middleware and TCP/JSON Gateway

Integrate the existing Project 1 publish/subscribe middleware and rosbridge-style TCP/JSON gateway into the Project 3 runtime.

This task establishes the communication foundation. Do not implement the parameter server, URDF parser, forward kinematics, or robot-specific nodes yet.

## Goal

Reuse the existing Project 1 communication protocol unchanged. External clients must communicate with the runtime through `127.0.0.1:9095`.

The protocol uses newline-delimited JSON over a plain TCP socket. It is not HTTP or WebSocket.

## Requirements

1. Reuse the existing middleware, gateway, message routing, service calls, and subscription behavior.
2. Preserve the existing protocol's request and response envelopes.
3. Ensure every hop can handle messages of at least 1 MiB, including:

   * External client to gateway.
   * Gateway to middleware.
   * Middleware to nodes.
   * Node responses and publications back to clients.
4. Read incoming messages until a newline is encountered, growing buffers as needed. Do not rely on a fixed 64 KiB buffer.
5. Ensure JSON serialization correctly escapes arbitrary strings.
6. Ensure the server can be restarted repeatedly without failing to bind port `9095`. Use appropriate socket reuse settings.
7. Preserve compatibility with the Project 1 protocol and the existing node communication interfaces.
8. Do not introduce a new transport, HTTP server, WebSocket server, or ROS/ROS 2 dependency.

## Acceptance criteria

* An external TCP/JSON client can connect to `127.0.0.1:9095`.
* Existing Project 1 protocol behavior remains compatible.
* A single JSON request containing a URDF string approaching 1 MiB can be received intact.
* A response containing a large string can be transmitted intact.
* Multiple requests can be processed on the same connection according to the existing protocol.
* The gateway can be stopped and restarted repeatedly.
* Malformed client input does not corrupt the runtime.
* `make build` can compile the communication layer offline.

## Scope boundary

Do not implement robot-specific services or topics. Expose the existing middleware interfaces so subsequent tasks can register the required services, nodes, and publishers.
