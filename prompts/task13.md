# Task: External Robot Viewer and Portfolio Demonstration

Build a small external viewer for the Project 3 runtime and use it to demonstrate a finite-state-machine choreography.

Do not modify the graded runtime protocol or introduce a UI dependency into the core nodes.

## Goal

Display the robot moving by consuming the runtime's existing externally visible data, especially `/xform_world`.

A browser-based viewer using three.js is one suitable implementation.

## Viewer behavior

* Connect to the existing TCP/JSON gateway using the project's supported protocol. Do not introduce a WebSocket gateway.
* Load or otherwise obtain a robot description for the geometry metadata needed for rendering.
* Display links using their visual geometry where practical.
* Update each link's pose from `/xform_world`.
* Interpret the supplied matrices in the specified column-major order.
* Render the robot with the coordinate convention specified by the runtime. Apply any required rendering-axis adjustment only in the viewer, not in `/xform_world`.
* Handle new robot descriptions or a refreshed robot model without retaining stale link objects.
* Make it possible to start a choreography and visually see the robot move.

Support box, cylinder, and sphere geometry at a minimum. Support mesh display when practical; mesh file resolution is not part of the graded runtime contract.

## Demonstration workflow

1. Start the runtime with `make demo`.
2. Load a valid robot through the parameter server.
3. Configure an FSM choreography through the recommended services.
4. Run the choreography.
5. Observe joint motion in the viewer.
6. Record a video of the robot performing the choreography.

Use the supplied Fetch model or a robot of your own.

## Portfolio deliverable

Add a page to the course portfolio website containing the video.

Briefly describe the robot, the choreography, and how the viewer renders the robot from its transforms.

If displaying the supplied Fetch robot, include this credit:

`Fetch robot description © Fetch Robotics, Inc., CC BY-NC-SA 4.0`

Retain the applicable license when redistributing the description.

## Acceptance criteria

* The viewer connects through the existing external protocol.
* The viewer uses `/xform_world` to position the links.
* Transform matrices are interpreted correctly.
* The displayed robot moves in response to the choreography.
* The viewer is separate from the graded runtime.
* The portfolio page embeds a video of the choreography and includes the required description and attribution where applicable.
