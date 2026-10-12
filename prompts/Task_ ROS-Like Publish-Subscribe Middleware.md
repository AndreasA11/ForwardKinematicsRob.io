# Task: ROS-Like Publish/Subscribe Middleware

Implement the reusable ROS-like communication layer needed for Project 1.

This task is focused on the **internal publish/subscribe system and external rosbridge-style gateway**. Do not implement map interpretation or A* search yet.

The heap primitive from the previous task may remain in the repository, but this task should not depend on heap behavior.

## Goal

Build a small communication system supporting:

- topics;
- publishers;
- subscribers;
- services;
- service clients;
- service providers;
- independent external clients through the required TCP/JSON interface.

Do not use ROS, ROS 2, or ROS client/runtime libraries.

The internal architecture is your choice. Use the simplest reasonable design that satisfies the required behavior.

## Internal communication model

Provide reusable abstractions or equivalent behavior for:

### Topics

A publisher can publish a message to a named topic.

Subscribers to that topic receive the published message.

Required behavior includes:

- multiple topic names;
- multiple subscribers;
- isolation between unrelated topics;
- repeated publications;
- subscribers connecting and disconnecting without corrupting future communication.

Do not hard-code Project 1 topics such as `/map` or `/path` into the middleware itself.

### Services

A service provider can register a named service.

A client can send a request to that service and receive the corresponding response.

Required behavior includes:

- identifying services by name;
- routing requests to the correct provider;
- returning responses to the correct requester;
- supporting repeated service calls;
- preserving request/response correlation when multiple calls are active.

Do not hard-code `/heapify`, `/heap_sort`, or `/plan_path` behavior into the middleware.

The middleware should transport service requests and responses without owning the application logic.

## External TCP/JSON gateway

Expose the communication system to independent external clients on:

```text
127.0.0.1:9095
```

Use the TCP/JSON protocol defined in `ROSBRIDGE_PROTOCOL.md`.

The gateway should translate external protocol operations into the internal topic and service semantics.

External clients must be able to perform the supported operations required by the Project 1 protocol, including:

- subscribing to topics;
- publishing topic messages;
- calling services;
- receiving service responses.

The gateway represents the internal publish/subscribe system externally. It may be implemented as part of the same process or component as the internal middleware.

## Multiple clients

Support multiple independent external client connections.

One client's state must not incorrectly affect another client's state.

In particular:

- subscriptions belong to the appropriate client;
- topic messages reach the appropriate subscribers;
- service responses return to the client that issued the request;
- disconnecting one client must not break other clients.

## Connection cleanup

When a client disconnects:

- remove stale subscriptions associated with that connection;
- remove other connection-specific state that is no longer valid;
- do not continue attempting to send messages to the disconnected client;
- do not damage subscriptions or requests belonging to other clients.

The system should remain usable after clients connect and disconnect repeatedly.

## Scope and architecture

The following internal designs are all potentially acceptable:

- one process or multiple processes;
- queues;
- threads;
- direct dispatch;
- TCP;
- Unix sockets;
- shared memory;
- another reasonable internal design.

There is no required internal wire protocol.

Choose the simplest architecture that provides the required semantics and can later support the Project 1 nodes.

Do not add unnecessary distributed-systems machinery merely to imitate ROS internally.

## Reusability

The middleware should be generic enough that later tasks can add application components such as:

- heap service providers;
- a map subscriber;
- an A* planner;
- `/path` publication;

without rewriting the basic topic or service infrastructure.

Application-specific logic should remain separate from generic communication behavior.

## Runtime requirements

The eventual Project 1 runtime must expose the gateway at `127.0.0.1:9095`.

For this task, integrate the middleware into the existing project structure in a way that can later be launched through the project's normal runtime.

Keep startup, shutdown, and failure behavior simple and deterministic.

## Acceptance criteria

The completed middleware should make it possible to demonstrate that:

1. An external client can connect to `127.0.0.1:9095`.
2. A client can subscribe to a topic.
3. Publishing to a topic delivers the message to appropriate subscribers.
4. Publishing to one topic does not leak messages to unrelated topics.
5. Multiple subscribers can receive appropriate publications.
6. A service can be registered/provided and called through the middleware.
7. A service request reaches the correct provider.
8. A service response returns to the correct requesting client.
9. Request/response IDs remain correctly correlated.
10. Multiple external clients can operate without corrupting each other's state.
11. Disconnecting a client cleans up its connection-specific state.
12. Repeated connections, publications, subscriptions, and service calls continue to work.
13. Generic middleware code does not contain application-specific implementations of heap, map, or A* behavior.
14. No ROS or ROS runtime/client libraries are used.