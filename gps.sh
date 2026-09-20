#!/bin/bash
# GPS anchoring for Point-LIO (VRX sim).
# Launches navsat_transform_node to publish utm→camera_init TF, preventing
# long-term LIO drift on open water. Run AFTER ./lio.sh is active.

set -e
cd "$(dirname "$0")"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

# NVIDIA GPU offload (same as sim.sh/view.sh)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

LOG_DIR=".usv_logs"
PID_DIR=".usv_pids"
mkdir -p "$LOG_DIR" "$PID_DIR"

ACTION="${1:-start}"

start_gps() {
    echo "Starting GPS anchoring (navsat only)..."

    ros2 launch usv_localization navsat_only.launch.py \
        > "$LOG_DIR/gps.log" 2>&1 &
    echo $! > "$PID_DIR/gps.pid"

    echo "GPS anchoring started (PID $(cat "$PID_DIR/gps.pid"))"
    echo "Log: $LOG_DIR/gps.log"
}

stop_gps() {
    if [ -f "$PID_DIR/gps.pid" ]; then
        PID=$(cat "$PID_DIR/gps.pid")
        echo "Stopping GPS anchoring (PID $PID)..."
        kill "$PID" 2>/dev/null || true
        rm -f "$PID_DIR/gps.pid"
        echo "Stopped."
    else
        echo "GPS anchoring not running."
    fi
}

status_gps() {
    if [ -f "$PID_DIR/gps.pid" ]; then
        PID=$(cat "$PID_DIR/gps.pid")
        if ps -p "$PID" > /dev/null 2>&1; then
            echo "GPS anchoring running (PID $PID)"
        else
            echo "GPS anchoring stale PID file (process dead)"
            rm -f "$PID_DIR/gps.pid"
        fi
    else
        echo "GPS anchoring not running"
    fi
}

case "$ACTION" in
    start)
        start_gps
        ;;
    stop)
        stop_gps
        ;;
    restart)
        stop_gps
        sleep 1
        start_gps
        ;;
    status)
        status_gps
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status}"
        exit 1
        ;;
esac
