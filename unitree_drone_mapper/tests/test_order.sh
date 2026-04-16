#!/usr/bin/env bash
# ============================================================
#  DronePi Test Order — UPRM Capstone
#  Autonomous Texture-Mapping Drone
#
#  Usage:
#    bash test_order.sh             — interactive, prompts between phases
#    bash test_order.sh --phase 5   — jump to a specific phase
#
#  All test scripts live in unitree_drone_mapper/tests/.
#  All flight tests require mavros.service already running.
#
#  Service architecture reminder:
#    pointlio-standby.service  — owns /dev/ttyUSB0; do NOT launch
#                                a second Point-LIO manually while this
#                                service is active
#    slam-bridge.service       — forwards /aft_mapped_to_init to MAVROS
#    drone-watchdog.service    — supervises bag recorder; benched by tests
#    dronepi-main.service      — owns collision_monitor.py; kept alive
#    mavros.service            — always running; tests never touch it
#
#  Test phases:
#    0  System health (bench)
#    1  Service verification (bench)
#    2  LiDAR / SLAM validation (bench)
#    3  Sensor bench validation — Hailo, GPS, camera (bench)
#    4  Post-flight pipeline validation (bench)
#    5  Watchdog arm detection (bench, no props)
#    6  OFFBOARD baseline flight (outdoor, props on)
#    7  SLAM bridge flight test (outdoor, props on)
#    8  Multi-altitude hover + collision zones (outdoor, props on)
#    9  Square pattern camera + LiDAR sync (outdoor, props on)
#    10 Full autonomous survey mission (outdoor, props on)
# ============================================================

BASE="$HOME/unitree_lidar_project/unitree_drone_mapper"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$HOME/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

JUMP_PHASE="${2:-0}"
if [ "$1" = "--phase" ]; then
    JUMP_PHASE="$2"
fi

phase()   { echo -e "\n${CYAN}${BOLD}━━━  PHASE $1: $2  ━━━${NC}"; }
step()    { echo -e "\n${BOLD}[$1]${NC} $2"; }
note()    { echo -e "  ${YELLOW}NOTE:${NC} $1"; }
safety()  { echo -e "  ${RED}${BOLD}⚠️  SAFETY:${NC} $1"; }
run_cmd() {
    echo -e "\n  ${GREEN}Command:${NC}"
    echo -e "  ${BOLD}$1${NC}"
    echo -e "\n  Press Enter to run, 's' to skip, 'q' to quit..."
    read -r REPLY
    case "$REPLY" in
        q|Q) echo "Quitting."; exit 0 ;;
        s|S) echo "  Skipped." ;;
        *)   eval "source $ROS_SETUP 2>/dev/null; source $WS_SETUP 2>/dev/null; $1" ;;
    esac
}

echo -e "${BOLD}"
echo "  ╔══════════════════════════════════════════════════╗"
echo "  ║     DronePi Test Order — UPRM Capstone          ║"
echo "  ║     Autonomous Texture-Mapping Drone             ║"
echo "  ╚══════════════════════════════════════════════════╝"
echo -e "${NC}"
echo "  Press Enter to run each command."
echo "  Type 's' + Enter to skip a step."
echo "  Type 'q' + Enter to quit."
if [ "$JUMP_PHASE" -gt 0 ] 2>/dev/null; then
    echo -e "  ${YELLOW}Jumping to Phase $JUMP_PHASE${NC}"
fi
echo ""

# ── PHASE 0 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 0 ]; then
phase "0" "SYSTEM HEALTH CHECK"
note "Run this at the start of every test session."

step "0.1" "Full system preflight check + log generation"
run_cmd "sudo bash $BASE/../preflight_check.sh"

step "0.2" "View latest preflight log"
run_cmd "ls $HOME/unitree_lidar_project/logs/ && \
    cat \$(ls -t $HOME/unitree_lidar_project/logs/ | head -1 | \
    xargs -I{} echo $HOME/unitree_lidar_project/logs/{})"

step "0.3" "Confirm no stale mission lock (should print: no lock file)"
run_cmd "cat /tmp/dronepi_mission.lock 2>/dev/null || echo 'no lock file (correct)'"
fi

# ── PHASE 1 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 1 ]; then
phase "1" "SERVICE VERIFICATION (Bench, No Props)"

step "1.1" "Check all DronePi service statuses"
run_cmd "sudo systemctl status \
    mavros \
    pointlio-standby \
    slam-bridge \
    drone-watchdog \
    dronepi-main \
    drone-mesh-server \
    foxglove-bridge \
    rpi-health \
    led \
    --no-pager --lines=3"

step "1.2" "MAVROS FCU connection"
run_cmd "ros2 topic echo /mavros/state --once --no-daemon"

step "1.3" "Verify SLAM chain (requires LiDAR connected)"
run_cmd "timeout 5 ros2 topic hz /aft_mapped_to_init --window 5 && \
    timeout 5 ros2 topic hz /mavros/vision_pose/pose --window 5"

step "1.4" "Mesh server flight database API"
run_cmd "curl -s http://10.42.0.1:8080/api/flights | python3 -m json.tool"
fi

# ── PHASE 2 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 2 ]; then
phase "2" "LIDAR AND SLAM VALIDATION (Bench, No Props)"
note "pointlio-standby.service manages Point-LIO automatically when LiDAR is connected."
note "Do NOT launch Point-LIO manually while pointlio-standby.service is active."

step "2.1" "Check LiDAR raw cloud topic"
run_cmd "timeout 5 ros2 topic hz /unilidar/cloud --window 5"

step "2.2" "Check registered cloud topic (Point-LIO output)"
run_cmd "timeout 5 ros2 topic hz /cloud_registered --window 5"

step "2.3" "Check SLAM bridge forwarding vision pose to MAVROS"
run_cmd "timeout 5 ros2 topic hz /mavros/vision_pose/pose --window 5"

step "2.4" "Live SLAM walk test — carry LiDAR for 60s (Ctrl+C to stop)"
run_cmd "python3 $BASE/tests/test_slam_live.py --no-rviz"
fi

# ── PHASE 3 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 3 ]; then
phase "3" "SENSOR BENCH VALIDATION (No Props)"

step "3.1" "Hailo-8L device availability, model load, and inference latency"
note "Must run inside hailo_inference_env."
run_cmd "source $HOME/hailo_inference_env/bin/activate && \
    python3 $BASE/tests/test_hailo_flight.py"

step "3.2" "FlowBridge ROS integration (hailo_inference_env, MAVROS running)"
run_cmd "source $HOME/hailo_inference_env/bin/activate && \
    python3 $BASE/tests/test_hailo_flow.py"

step "3.3" "MAVROS GPS quality gate — offline fixture tests only (no GPS needed)"
run_cmd "python3 $BASE/tests/test_gps_reader_mavros.py --synthetic"

step "3.4" "MAVROS GPS live window — 60s (requires GPS antenna near window)"
run_cmd "python3 $BASE/tests/test_gps_reader_mavros.py --duration 60"

step "3.5" "CameraCapture unit tests — no physical camera required"
run_cmd "python3 $BASE/tests/test_camera_capture.py -v"

step "3.6" "IMX477 live camera preview — requires CSI cable seated"
note "Press Q or Esc to quit the preview window."
run_cmd "python3 $BASE/tests/test_camera_preview.py"

step "3.7" "FrameIngestor unit test — synthetic data, no bag required"
run_cmd "python3 $BASE/tests/test_frame_ingestor.py --synthetic"
fi

# ── PHASE 4 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 4 ]; then
phase "4" "POST-FLIGHT PIPELINE VALIDATION (Bench)"
note "Requires a completed bag at /mnt/ssd/rosbags/."

step "4.1" "Process the most recent bag (auto-detect)"
run_cmd "python3 $BASE/utils/run_postflight.py"

step "4.2" "Process a specific bag (edit path before running)"
run_cmd "python3 $BASE/utils/run_postflight.py \
    --bag /mnt/ssd/rosbags/scan_YYYYMMDD_HHMMSS"

step "4.3" "Verify mesh appeared in browser"
note "Open http://10.42.0.1:8080/meshview.html — auto-loads within 10s."
run_cmd "curl -s http://10.42.0.1:8080/latest.json | python3 -m json.tool"

step "4.4" "View flight history log"
run_cmd "python3 $BASE/utils/flight_logger.py --last 5"
fi

# ── PHASE 5 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 5 ]; then
phase "5" "WATCHDOG ARM DETECTION (Bench, No Props)"

step "5.1" "Monitor watchdog — arm on RC to observe MODE 2 (manual_scan) detection"
note "Arm without switching to OFFBOARD — 10s debounce then manual_scan lock."
run_cmd "sudo journalctl -u drone-watchdog -f --since now"

step "5.2" "Monitor main.py — arm then switch to OFFBOARD for MODE 3 detection"
run_cmd "sudo journalctl -u dronepi-main -f --since now"

step "5.3" "Verify lock file written (run while armed)"
run_cmd "cat /tmp/dronepi_mission.lock 2>/dev/null | python3 -m json.tool \
    || echo 'No lock file'"

step "5.4" "Verify lock clears after disarm"
run_cmd "sleep 3 && \
    cat /tmp/dronepi_mission.lock 2>/dev/null || echo 'Lock cleared (correct)'"
fi

# ── PHASE 6 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 6 ]; then
phase "6" "OFFBOARD BASELINE FLIGHT (Outdoor, Props On)"
safety "RC transmitter in hand. Kill switch ready. Area clear. GPS lock confirmed."

step "6.1" "OFFBOARD hover dry run — no motors"
run_cmd "python3 $BASE/tests/test_offboard_flight.py --dry-run"

step "6.2" "Simple OFFBOARD hover — 1.5m AGL, 10s hold"
safety "PROPS ON. Drone arms and takes off automatically after confirmation."
run_cmd "python3 $BASE/tests/test_offboard_flight.py --alt 1.5 --hold 10"

step "6.3" "GPS global waypoint hover dry run — no motors"
run_cmd "python3 $BASE/tests/test_offboard_gps.py --dry-run"

step "6.4" "GPS global waypoint hover — 5m AGL, 10s hold"
safety "PROPS ON."
run_cmd "python3 $BASE/tests/test_offboard_gps.py --alt 5.0 --hold 10"
fi

# ── PHASE 7 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 7 ]; then
phase "7" "SLAM BRIDGE FLIGHT TEST (Outdoor, Props On)"
safety "RC transmitter in hand. Kill switch ready."
note "Validates EKF2 is fusing SLAM vision pose during flight."
note "test script writes autonomous mission lock so watchdog does not interfere."

step "7.1" "Out-and-back dry run — no motors"
run_cmd "python3 $BASE/tests/test_slam_bridge_flight.py --dry-run --no-bag"

step "7.2" "Out-and-back GPS only, no bag (3m alt, 5m distance)"
safety "PROPS ON."
run_cmd "python3 $BASE/tests/test_slam_bridge_flight.py \
    --distance 5 --alt 3.0 --hold 5 --no-bridge --no-bag"

step "7.3" "Out-and-back with SLAM bridge active (no bag)"
run_cmd "python3 $BASE/tests/test_slam_bridge_flight.py \
    --distance 5 --alt 3.0 --hold 5 --no-bag"

step "7.4" "Out-and-back full recording — bag + post-flight pipeline"
note "Mesh appears in browser within 10s of landing."
run_cmd "python3 $BASE/tests/test_slam_bridge_flight.py \
    --distance 8 --alt 4.0 --hold 10"
fi

# ── PHASE 8 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 8 ]; then
phase "8" "MULTI-ALTITUDE HOVER + COLLISION ZONES (Outdoor, Props On)"
safety "RC transmitter in hand. Kill switch ready. Open outdoor area required."
note "Stops drone-watchdog only. dronepi-main keeps collision_monitor.py active."
note "Reuses pointlio-standby.service if already publishing; launches local"
note "Point-LIO as fallback. Teardown only stops what the script started."
note "PLY and CSV export runs after landing — no mid-flight disk I/O."

step "8.1" "Dry run — service checks, sensor liveness, camera FOV table"
run_cmd "python3 $BASE/tests/test_altitude_validation.py --dry-run"

step "8.2" "Quick sweep — 3m and 6m only, 20s hold, 3 pics per level"
safety "PROPS ON."
run_cmd "python3 $BASE/tests/test_altitude_validation.py \
    --altitudes 3 6 --hold 20 --pics 3"

step "8.3" "Full altitude sweep — 3m through 15m, 60s hold, 5 pics per level"
safety "PROPS ON. Estimated ~25 min flight."
run_cmd "python3 $BASE/tests/test_altitude_validation.py \
    --altitudes 3 6 9 12 15 --hold 60 --pics 5"

step "8.4" "Review altitude validation report"
run_cmd "cat $BASE/tests/hover_camera_output/report.txt"
fi

# ── PHASE 9 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 9 ]; then
phase "9" "SQUARE PATTERN CAMERA + LIDAR SYNC (Outdoor, Props On)"
safety "RC transmitter in hand. Kill switch ready."
note "Stops drone-watchdog only. dronepi-main keeps collision_monitor.py active."
note "Single bag covers the full square flight. Ready for postprocess_mesh.py."

step "9.1" "Dry run — sensor checks, footprint + overlap preview"
run_cmd "python3 $BASE/tests/test_square_camera.py --dry-run"

step "9.2" "Default square — 5m altitude, 4m side, 3 frames per corner"
safety "PROPS ON."
run_cmd "python3 $BASE/tests/test_square_camera.py"

step "9.3" "Custom square — 3m altitude, 3m side, 5 frames per corner"
safety "PROPS ON."
run_cmd "python3 $BASE/tests/test_square_camera.py \
    --alt 3 --side 3 --pics 5 --hold 8"

step "9.4" "Review square camera report"
run_cmd "cat $BASE/tests/square_camera_output/report.txt"

step "9.5" "Run post-flight pipeline on the square bag"
run_cmd "python3 $BASE/utils/run_postflight.py"
fi

# ── PHASE 10 ──────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 10 ]; then
phase "10" "FULL AUTONOMOUS SURVEY MISSION (Outdoor, Props On)"
safety "ONLY proceed after Phases 6–9 all pass."
safety "Second person as safety observer strongly recommended."
safety "Area must be completely clear. RC in hand at all times."
note "Upload a survey mission in QGC before running. main.py detects OFFBOARD"
note "mode on the RC transmitter, runs pre-flight checks, then executes waypoints."

step "10.1" "Final preflight check"
run_cmd "sudo bash $BASE/../preflight_check.sh"

step "10.2" "Verify mission waypoints uploaded in QGC"
run_cmd "ros2 topic echo /mavros/mission/waypoints --once --no-daemon"

step "10.3" "Verify SLAM chain healthy before arming"
run_cmd "timeout 5 ros2 topic hz /mavros/vision_pose/pose --window 5"

step "10.4" "Arm on RC transmitter, then switch to OFFBOARD"
note "main.py prints pre-flight check results and begins mission automatically."
note "Monitor: sudo journalctl -u dronepi-main -f"
run_cmd "sudo journalctl -u dronepi-main -f --since now"

step "10.5" "Verify mesh appeared in browser after landing"
note "Open http://10.42.0.1:8080/meshview.html — auto-loads within 10s."
run_cmd "curl -s http://10.42.0.1:8080/latest.json | python3 -m json.tool"

step "10.6" "Review flight history log"
run_cmd "python3 $BASE/utils/flight_logger.py --last 5"
fi

echo ""
echo -e "${GREEN}${BOLD}━━━  Test sequence complete  ━━━${NC}"
echo ""
