## 1. Goal

A planning codebase must be able to:

1. Fetch the robot's current **map**.
2. Fetch the robot's current **pose**.
3. Compute a path locally.
4. Send the **path** back to the robot to follow.

The codebase talks only to the MBot Bridge server over a websocket. It never uses
LCM or ROS directly.

## 2. Connection

| Item | Value |
|---|---|
| URL | `ws://<mbot-host>:5005` |
| Encoding | JSON text frames (one message per frame) |
| Connect timeout | 5 s recommended |
| Prerequisite | MBot Bridge server running on the robot; port 5005 reachable |

The reference client opens a **new connection per call** and closes it after.
A client may instead keep one connection open and reuse it for several
requests; the server answers each `request` on the same connection.

## 3. Message envelope

Every message is a JSON object with a `type` string. Use the string names
(the reference client sends strings, e.g. `"publish"`):

| `type` | Direction | Purpose |
|---|---|---|
| `request` | client -> server | Ask for the latest message on a channel |
| `response` | server -> client | Reply to a `request` |
| `publish` | client -> server | Publish a message to a channel. No reply is sent |
| `error` | server -> client | Reply to a bad or unservable request |

Other fields: `channel` (LCM channel name), `dtype` (LCM type name, `mbot_lcm_msgs`
package prefix omitted), `data` (payload), `as_bytes` (requests only).

## 4. Operations

### 4.1 Read the pose

Request (set `as_bytes` to `false` to get JSON):

```json
{"type": "request", "channel": "SLAM_POSE", "dtype": "pose2D_t", "as_bytes": false}
```

Response:

```json
{"type": "response", "channel": "SLAM_POSE", "dtype": "pose2D_t",
 "data": {"utime": 0, "x": 0.0, "y": 0.0, "theta": 0.0}}
```

- Units: `x`, `y` in meters; `theta` in radians.
- The pose is in the SLAM map frame. The controller follows the SLAM pose, so this is the
  pose that matches the map and the path (see section 5).

### 4.2 Read the map

Request:

```json
{"type": "request", "channel": "SLAM_MAP", "dtype": "occupancy_grid_t", "as_bytes": false}
```

Response `data`:

```json
{"utime": 0, "origin_x": -5.0, "origin_y": -5.0, "meters_per_cell": 0.05,
 "width": 200, "height": 200, "num_cells": 40000,
 "cells": "<base64 string>"}
```

- `cells` is a base64 string of `num_cells` bytes (`width * height`), one byte per cell.
- Decode: base64-decode `cells` to `width * height` bytes and treat each byte as a signed
  8-bit integer (int8). Lay them out row-major, row = y index, column = x index. the layout and signedness.
- Cell value meaning: The usual MBot convention is `0` unknown, negative
  free, positive occupied. Confirm against a real map before relying on it.
- Cell -> world: `x = origin_x + col * meters_per_cell`, `y = origin_y + row * meters_per_cell`
  (cell corner or center:).
- Do not request the map with `as_bytes: true` unless the client decodes raw LCM itself.

### 4.3 Send a path

Publish (no response):

```json
{"type": "publish", "channel": "CONTROLLER_PATH", "dtype": "path2D_t",
 "data": {"path_length": 3,
          "path": [{"x": 0.5, "y": 0.0, "theta": 0.0},
                   {"x": 0.5, "y": 0.5, "theta": 0.0},
                   {"x": 0.0, "y": 0.5, "theta": 0.0}]}}
```

- `path_length` must equal `len(path)`.
- Units match 4.1: meters and radians, in the SLAM map frame. Waypoints are ordered from first to last.
- The path is followed by the robot's onboard controller. The client does not stream commands.

### 4.4 Stop the robot (optional)

To halt, publish zero velocity:

```json
{"type": "publish", "channel": "MBOT_VEL_CMD", "dtype": "twist2D_t",
 "data": {"vx": 0, "vy": 0, "wz": 0}}
```

**UNVERIFIED:** whether this cancels a path already in progress.

## 5. Coordinate frames

The robot's controller follows the **SLAM pose** (confirmed by the project owner), so
everything is in the SLAM map frame:

- Pose: read from `SLAM_POSE`.
- Map: `SLAM_MAP`, positioned by `origin_x` / `origin_y`.
- Path: waypoints published to `CONTROLLER_PATH` are in the same map frame.

SLAM must be running for any of this to work (see section 9).

Axes: x forward, y left, theta counter-clockwise (assumed, standard convention).

## 6. Errors

The server replies with `{"type": "error", "data": "<message>"}`. Known cases:

| Message | Meaning |
|---|---|
| `No data on channel: <ch>` | Nothing received on that channel yet (e.g. SLAM not running) |
| `Data on channel <ch> is old.` | Data is staler than the server's discard threshold |
| Bad request / invalid type | Malformed message |

Client behavior:

- Treat `error` as a failed read; do not use stale data.
- Connection failure or timeout: report it and abort. Do not publish a path.
- A `publish` has no acknowledgment, so a successful send does not prove the robot accepted it.
  Confirm by watching the pose change.

## 7. Typical flow

1. Connect to `ws://<host>:5005`.
2. `get_map()` and `get_pose()` (pose from `SLAM_POSE`).
3. Plan a path from the pose to the goal on the map.
4. Validate: every waypoint is inside the map and in free space.
5. `send_path(path)`.
6. Poll `get_pose()` until the goal is reached or a timeout expires; call `stop()` on abort.