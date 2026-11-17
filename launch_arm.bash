#!/usr/bin/env bash
# ==============================================================================
# launch_arm.bash
#
# Launches the C++ Forward Kinematics runtime and 3D Robot Viewer pre-configured
# to interface with a physical or simulated Trossen ReactorX 200 (RX200) robot arm
# over the ROSBridge protocol (WebSocket or TCP).
#
# Usage:
#   ./launch_arm.bash [ARM_HOST] [ARM_PORT] [OPTIONS]
#
# Examples:
#   ./launch_arm.bash                          # Localhost rosbridge on :9090
#   ./launch_arm.bash 192.168.1.105 9090       # Real robot arm IP
#   ./launch_arm.bash rx200.local 9090         # Domain name
#   ./launch_arm.bash --no-auto-connect        # Launch viewer & arm model without immediate connect
#   ./launch_arm.bash --no-browser             # Headless / don't pop open browser
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

DEFAULT_ARM_HOST="127.0.0.1"
DEFAULT_ARM_PORT=9090
DEFAULT_VIEWER_PORT=8092
DEFAULT_ROSBRIDGE_PORT=9095

ARM_HOST=""
ARM_PORT=""
VIEWER_PORT="$DEFAULT_VIEWER_PORT"
ROSBRIDGE_PORT="$DEFAULT_ROSBRIDGE_PORT"
NO_BROWSER=false
AUTO_CONNECT=true

print_usage() {
    cat << EOF
Usage: $(basename "$0") [ARM_HOST] [ARM_PORT] [OPTIONS]

Arguments:
  ARM_HOST              Hostname or IP address of the rosbridge server (default: 127.0.0.1)
  ARM_PORT              Network port of the rosbridge server (default: 9090)

Options:
  --viewer-port PORT    Port for the Three.js web visualizer (default: 8092)
  --rosbridge-port PORT Port for the C++ TCP Rosbridge gateway (default: 9095)
  --no-auto-connect     Do not automatically connect to external arm on startup
  --no-browser          Do not automatically open the web visualizer in a browser
  -h, --help            Show this help message and exit
EOF
}

POSITIONAL_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --viewer-port)
            VIEWER_PORT="$2"
            shift 2
            ;;
        --rosbridge-port)
            ROSBRIDGE_PORT="$2"
            shift 2
            ;;
        --no-auto-connect)
            AUTO_CONNECT=false
            shift
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
    ARM_HOST="${POSITIONAL_ARGS[0]}"
else
    ARM_HOST="$DEFAULT_ARM_HOST"
fi

if [[ ${#POSITIONAL_ARGS[@]} -ge 2 ]]; then
    ARM_PORT="${POSITIONAL_ARGS[1]}"
else
    ARM_PORT="$DEFAULT_ARM_PORT"
fi

echo "======================================================================"
echo " EECS 367: Trossen RX200 Robot Arm ROSBridge Launcher"
echo "======================================================================"
echo " ROSBridge Arm Host:  $ARM_HOST"
echo " ROSBridge Arm Port:  $ARM_PORT (rosbridge protocol)"
echo " C++ Core Gateway:    127.0.0.1:$ROSBRIDGE_PORT (TCP Rosbridge)"
echo " Web Visualizer:      http://127.0.0.1:$VIEWER_PORT"
echo " Active Robot Model:  Trossen RX200 Manipulator (12-DOF URDF)"
echo " Auto-connect:        $([ "$AUTO_CONNECT" = true ] && echo "Enabled" || echo "Disabled (manual via Web UI)")"
echo "======================================================================"

# 1. Compile C++ Runtime Executable
echo "[1/4] Checking and building C++ Forward Kinematics runtime..."
if [[ ! -f "main" ]]; then
    make build
else
    make -q build 2>/dev/null || make build
fi

# Ensure RX200 URDF exists in robots/rx200
mkdir -p robots/rx200
if [[ ! -f "robots/rx200/rx200.urdf" ]]; then
    cp urdf-files/rx200.urdf robots/rx200/rx200.urdf 2>/dev/null || true
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

# 3. Launch 3D Visualizer Server
echo "[3/4] Starting 3D Robot Viewer & Arm ROSBridge interface on port $VIEWER_PORT..."
VIEWER_ARGS=(
    "scripts/viewer.py"
    "--port" "$VIEWER_PORT"
    "--rosbridge-port" "$ROSBRIDGE_PORT"
)

if [[ "$AUTO_CONNECT" == true ]]; then
    VIEWER_ARGS+=(
        "--external-rosbridge-host" "$ARM_HOST"
        "--external-rosbridge-port" "$ARM_PORT"
    )
fi

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

# 4. Automatically load Trossen RX200 URDF into the parameter server
echo "[4/4] Setting active robot description to Trossen RX200 Arm..."
python3 - << EOF
import json, urllib.request, socket, sys, time

def load_arm():
    urdf_path = None
    for cand in ['robots/rx200/rx200_chess.urdf', 'urdf-files/rx200_chess.urdf', 'robots/rx200/rx200.urdf', 'urdf-files/rx200.urdf']:
        try:
            with open(cand, 'r') as f:
                urdf_path = cand
                break
        except Exception:
            pass

    if not urdf_path:
        print("[Launcher] Notice: Could not locate rx200.urdf")
        return

    # Try HTTP load first (synchronizes parameter server, viewer cache, and visualizer)
    loaded = False
    for attempt in range(10):
        try:
            req = urllib.request.Request(
                "http://127.0.0.1:$VIEWER_PORT/api/load_robot",
                data=json.dumps({"path": urdf_path}).encode("utf-8"),
                headers={"Content-Type": "application/json"}
            )
            with urllib.request.urlopen(req, timeout=3.0) as res:
                data = json.loads(res.read().decode("utf-8"))
                if data.get("result"):
                    print("[Launcher] Trossen RX200 Chess description loaded and accepted.")
                    loaded = True
                    break
        except Exception:
            time.sleep(0.3)

    if not loaded:
        # Fallback to direct TCP rosbridge call
        with open(urdf_path, 'r') as f:
            urdf = f.read()
        try:
            s = socket.create_connection(('127.0.0.1', $ROSBRIDGE_PORT), timeout=2.0)
            req = {
                "op": "call_service",
                "service": "/param_server/set_param",
                "id": "init-rx200-urdf",
                "args": {"name": "robot_description", "value": urdf}
            }
            s.sendall((json.dumps(req) + "\n").encode('utf-8'))
            s.close()
            print("[Launcher] Trossen RX200 Chess description loaded via TCP.")
        except Exception:
            pass

    try:
        req2 = urllib.request.Request(
            "http://127.0.0.1:$VIEWER_PORT/api/load_fsm",
            data=json.dumps({"identifier": "rx200_chess_pick_and_place"}).encode("utf-8"),
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req2, timeout=3.0) as res:
            print("[Launcher] Primed rx200_chess_pick_and_place FSM choreography.")
    except Exception:
        pass

load_arm()
EOF

echo "======================================================================"
echo " System is READY!"
echo " -> Web Visualizer:     http://127.0.0.1:$VIEWER_PORT"
if [[ "$AUTO_CONNECT" == true ]]; then
    echo " -> Arm ROSBridge:      Connecting to $ARM_HOST:$ARM_PORT"
else
    echo " -> Arm ROSBridge:      Ready to connect via Web UI (select ROSBridge)"
fi
echo " Press Ctrl+C to stop all nodes."
echo "======================================================================"

wait "$VIEWER_PID"
