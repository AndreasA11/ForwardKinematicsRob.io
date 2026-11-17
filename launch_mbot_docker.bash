#!/usr/bin/env bash
# ==============================================================================
# launch_mbot_docker.bash
#
# Builds upon the Forward Kinematics (FK) runtime and 3D visualizer to interface
# with an MBot Omni robot via the MBot Bridge WebSocket protocol.
#
# Usage:
#   ./launch_mbot_docker.bash [MBOT_HOST] [MBOT_PORT] [OPTIONS]
#
# Examples:
#   ./launch_mbot_docker.bash 192.168.1.100 5005
#   ./launch_mbot_docker.bash mbot.local 5005
#   ./launch_mbot_docker.bash --docker 192.168.1.100 5005
#   ./launch_mbot_docker.bash --native 127.0.0.1 5005
# ==============================================================================

set -euo pipefail

# Script directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Defaults
DEFAULT_MBOT_HOST="127.0.0.1"
DEFAULT_MBOT_PORT=5005
DEFAULT_VIEWER_PORT=8092
DEFAULT_ROSBRIDGE_PORT=9095

MBOT_HOST=""
MBOT_PORT=""
VIEWER_PORT="$DEFAULT_VIEWER_PORT"
ROSBRIDGE_PORT="$DEFAULT_ROSBRIDGE_PORT"
USE_DOCKER=false
DOCKER_IMAGE="${DOCKER_IMAGE:-eecs367_mbot:latest}"
NO_BROWSER=false

print_usage() {
    cat << EOF
Usage: $(basename "$0") [MBOT_HOST] [MBOT_PORT] [OPTIONS]

Arguments:
  MBOT_HOST             IP address or hostname of the MBot Bridge server (default: 127.0.0.1)
  MBOT_PORT             WebSocket network port of the MBot Bridge (default: 5005)

Options:
  --docker              Run within a Docker container environment with port forwarding
  --native              Run directly on the host machine (default)
  --viewer-port PORT    Port for the Three.js web visualizer (default: 8092)
  --rosbridge-port PORT Port for the C++ TCP Rosbridge gateway (default: 9095)
  --no-browser          Do not automatically open the web visualizer in a browser
  -h, --help            Show this help message and exit
EOF
}

# Parse command line options
POSITIONAL_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --docker)
            USE_DOCKER=true
            shift
            ;;
        --native)
            USE_DOCKER=false
            shift
            ;;
        --viewer-port)
            VIEWER_PORT="$2"
            shift 2
            ;;
        --rosbridge-port)
            ROSBRIDGE_PORT="$2"
            shift 2
            ;;
        --no-browser)
            NO_BROWSER=true
            shift
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            POSITIONAL_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ ${#POSITIONAL_ARGS[@]} -ge 1 ]]; then
    MBOT_HOST="${POSITIONAL_ARGS[0]}"
else
    MBOT_HOST="$DEFAULT_MBOT_HOST"
fi

if [[ ${#POSITIONAL_ARGS[@]} -ge 2 ]]; then
    MBOT_PORT="${POSITIONAL_ARGS[1]}"
else
    MBOT_PORT="$DEFAULT_MBOT_PORT"
fi

echo "======================================================================"
echo " EECS 367: MBot Omni Robot Navigation & MBot Bridge Launcher"
echo "======================================================================"
echo " MBot Bridge Host:    $MBOT_HOST"
echo " MBot Bridge Port:    $MBOT_PORT (WebSocket)"
echo " C++ Gateway:         127.0.0.1:$ROSBRIDGE_PORT (TCP Rosbridge)"
echo " Web Visualizer:      http://127.0.0.1:$VIEWER_PORT"
echo " Execution Mode:      $([ "$USE_DOCKER" = true ] && echo "Docker Container" || echo "Native Host")"
echo "======================================================================"

# Handle Docker execution mode if explicitly requested
if [[ "$USE_DOCKER" == true ]]; then
    if ! command -v docker >/dev/null 2>&1; then
        echo "Error: 'docker' command not found. Falling back to native execution."
        USE_DOCKER=false
    else
        echo "Launching via Docker container: $DOCKER_IMAGE..."
        docker run -it --rm \
            --network host \
            -p "${ROSBRIDGE_PORT}:${ROSBRIDGE_PORT}" \
            -p "${VIEWER_PORT}:${VIEWER_PORT}" \
            -p "${MBOT_PORT}:${MBOT_PORT}" \
            -v "$SCRIPT_DIR:/workspace" \
            -w /workspace \
            "$DOCKER_IMAGE" \
            bash -c "./launch_mbot_docker.bash --native '$MBOT_HOST' '$MBOT_PORT' --viewer-port '$VIEWER_PORT'"
        exit $?
    fi
fi

# 1. Compile C++ Runtime Executable
echo "[1/4] Checking and building C++ Forward Kinematics runtime..."
if [[ ! -f "main" ]]; then
    make build
else
    # Quick build check
    make -q build 2>/dev/null || make build
fi

# Ensure MBot URDF exists
if [[ ! -f "robots/mbot/mbot.urdf" ]]; then
    echo "Warning: robots/mbot/mbot.urdf not found. Creating..."
    mkdir -p robots/mbot
    cp urdf-files/mbot.urdf robots/mbot/mbot.urdf 2>/dev/null || true
fi

# PIDs for cleanup
MAIN_PID=""
VIEWER_PID=""

cleanup_port() {
    local port="$1"
    local name="$2"
    if command -v lsof >/dev/null 2>&1; then
        local pids
        pids=$(lsof -ti -sTCP:LISTEN :"$port" 2>/dev/null || true)
        if [[ -n "$pids" ]]; then
            echo "[Launcher] Port $port ($name) is in use by PID(s): $pids. Stopping stale process(es)..."
            for p in $pids; do
                kill "$p" 2>/dev/null || true
            done
            sleep 0.4
            local rem
            rem=$(lsof -ti -sTCP:LISTEN :"$port" 2>/dev/null || true)
            if [[ -n "$rem" ]]; then
                for p in $rem; do
                    kill -9 "$p" 2>/dev/null || true
                done
                sleep 0.3
            fi
        fi
    elif command -v fuser >/dev/null 2>&1; then
        fuser -k "${port}/tcp" 2>/dev/null || true
        sleep 0.3
    fi
}

cleanup() {
    echo ""
    echo "[Launcher] Terminating background processes..."
    if [[ -n "$VIEWER_PID" ]] && kill -0 "$VIEWER_PID" 2>/dev/null; then
        kill "$VIEWER_PID" 2>/dev/null || true
    fi
    if [[ -n "$MAIN_PID" ]] && kill -0 "$MAIN_PID" 2>/dev/null; then
        kill "$MAIN_PID" 2>/dev/null || true
    fi
    sleep 0.3
    if [[ -n "$VIEWER_PID" ]] && kill -0 "$VIEWER_PID" 2>/dev/null; then
        kill -9 "$VIEWER_PID" 2>/dev/null || true
    fi
    if [[ -n "$MAIN_PID" ]] && kill -0 "$MAIN_PID" 2>/dev/null; then
        kill -9 "$MAIN_PID" 2>/dev/null || true
    fi
    echo "[Launcher] Shutdown complete."
}
trap cleanup EXIT INT TERM

# Free ports from any orphaned or previous runs
cleanup_port "$ROSBRIDGE_PORT" "C++ Rosbridge"
cleanup_port "$VIEWER_PORT" "Web Visualizer"

# 2. Launch C++ Core Runtime
echo "[2/4] Starting C++ Forward Kinematics & Gateway node (./main --demo)..."
./main --demo &
MAIN_PID=$!

# Wait for TCP gateway to become ready on port 9095
echo "Waiting for C++ Rosbridge gateway on 127.0.0.1:$ROSBRIDGE_PORT..."
READY=false
for i in {1..50}; do
    if ! kill -0 "$MAIN_PID" 2>/dev/null; then
        echo "Error: C++ runtime process (PID $MAIN_PID) terminated unexpectedly."
        exit 1
    fi
    if command -v lsof >/dev/null 2>&1 && lsof -i :"$ROSBRIDGE_PORT" 2>/dev/null | grep -q LISTEN; then
        READY=true
        break
    elif python3 -c "import socket; s = socket.create_connection(('127.0.0.1', $ROSBRIDGE_PORT), timeout=0.2); s.close()" 2>/dev/null; then
        READY=true
        break
    fi
    sleep 0.1
done

if [[ "$READY" != true ]]; then
    echo "Error: C++ runtime failed to start on port $ROSBRIDGE_PORT within timeout."
    exit 1
fi
echo "C++ Rosbridge gateway is online!"

# 3. Launch 3D Visualizer Server & MBot Bridge Client
echo "[3/4] Starting 3D Robot Viewer & MBot Bridge client on port $VIEWER_PORT..."
VIEWER_ARGS=(
    "scripts/viewer.py"
    "--port" "$VIEWER_PORT"
    "--rosbridge-port" "$ROSBRIDGE_PORT"
    "--mbot-host" "$MBOT_HOST"
    "--mbot-port" "$MBOT_PORT"
)

if [[ "$NO_BROWSER" == true ]]; then
    VIEWER_ARGS+=("--no-open")
fi

python3 "${VIEWER_ARGS[@]}" &
VIEWER_PID=$!

# Ensure viewer started successfully
sleep 0.5
if ! kill -0 "$VIEWER_PID" 2>/dev/null; then
    echo "Error: 3D Robot Viewer failed to start on port $VIEWER_PORT."
    exit 1
fi

# 4. Automatically load MBot Omni URDF into the parameter server
echo "[4/4] Setting active robot description to MBot Omni..."
sleep 1.0
python3 - << EOF
import json, socket, sys, time

def load_mbot():
    try:
        with open('robots/mbot/mbot.urdf', 'r') as f:
            urdf = f.read()
    except Exception as e:
        print("Notice: Could not read robots/mbot/mbot.urdf:", e)
        return

    try:
        s = socket.create_connection(('127.0.0.1', $ROSBRIDGE_PORT), timeout=2.0)
        req = {
            "op": "call_service",
            "service": "/param_server/set_param",
            "id": "init-mbot-urdf",
            "args": {"name": "robot_description", "value": urdf}
        }
        s.sendall((json.dumps(req) + "\n").encode('utf-8'))
        s.close()
        print("[Launcher] MBot Omni description loaded into parameter server.")
    except Exception as e:
        print("[Launcher] Parameter load notice:", e)

load_mbot()
EOF

echo "======================================================================"
echo " System is READY!"
echo " -> Web Visualizer:  http://127.0.0.1:$VIEWER_PORT"
echo " -> MBot Bridge:     ws://$MBOT_HOST:$MBOT_PORT"
echo " Press Ctrl+C to stop all nodes."
echo "======================================================================"

wait "$VIEWER_PID"
