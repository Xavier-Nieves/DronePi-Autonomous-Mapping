#!/usr/bin/env bash
# ============================================================
#  DronePi Test Order — Runnable Script
#  UPRM Capstone — Autonomous Texture-Mapping Drone
#
#  Usage:
#    bash test_order.sh           -- interactive, prompts between phases
#    bash test_order.sh --phase 5 -- jump to a specific phase
#
#  Each phase prints the command, waits for you to press Enter,
#  then runs it. You can Ctrl+C at any point.
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

phase()    { echo -e "\n${CYAN}${BOLD}━━━  PHASE $1: $2  ━━━${NC}"; }
step()     { echo -e "\n${BOLD}[$1]${NC} $2"; }
note()     { echo -e "  ${YELLOW}NOTE:${NC} $1"; }
safety()   { echo -e "  ${RED}${BOLD}⚠️  SAFETY:${NC} $1"; }
run_cmd()  {
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

step "0.2" "View latest log"
run_cmd "ls $HOME/unitree_lidar_project/logs/ && cat \$(ls -t $HOME/unitree_lidar_project/logs/ | head -1 | xargs -I{} echo $HOME/unitree_lidar_project/logs/{})"
fi

# ── PHASE 1 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 1 ]; then
phase "1" "SERVICE VERIFICATION (Bench, No Props)"
step "1.1" "Check all service statuses"
run_cmd "sudo systemctl status mavros drone-watchdog dronepi-main drone-mesh-server foxglove-bridge --no-pager"

step "1.2" "MAVROS FCU connection"
run_cmd "ros2 topic echo /mavros/state --once --no-daemon"

step "1.3" "Mesh server API"
run_cmd "curl -s http://10.42.0.1:8080/api/flights | python3 -m json.tool"
fi

# ── PHASE 2 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 2 ]; then
phase "2" "LIDAR AND SLAM VALIDATION (Bench, No Props)"
note "Start Point-LIO in a separate terminal first."
note "Terminal: ros2 launch .../combined_lidar_mapping.launch.py rviz:=false port:=/dev/ttyUSB0"

step "2.1" "Check LiDAR topics publishing"
run_cmd "timeout 5 ros2 topic hz /unilidar/cloud --window 5"

step "2.2" "Check SLAM topics publishing"
run_cmd "timeout 5 ros2 topic hz /aft_mapped_to_init --window 5"

step "2.3" "Check cloud_registered publishing"
run_cmd "timeout 5 ros2 topic hz /cloud_registered --window 5"

step "2.4" "Live SLAM test (walk with LiDAR for 60s, Ctrl+C to stop)"
run_cmd "python3 $BASE/tests/test_slam_live.py --no-rviz"

step "2.5" "SLAM bridge test (run with Point-LIO active)"
run_cmd "timeout 15 python3 $BASE/flight/_slam_bridge.py"

step "2.6" "Verify vision_pose topic publishing"
run_cmd "timeout 5 ros2 topic hz /mavros/vision_pose/pose --window 5"
fi

# ── PHASE 3 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 3 ]; then
phase "3" "POSTFLIGHT PIPELINE VALIDATION (Bench)"
step "3.1" "Process specific bag"
run_cmd "python3 $BASE/utils/run_postflight.py --bag /mnt/ssd/rosbags/scan_20260315_191900"

step "3.2" "Process latest bag"
run_cmd "python3 $BASE/utils/run_postflight.py"
note "Check browser: http://10.42.0.1:8080/meshview.html"
fi

# ── PHASE 4 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 4 ]; then
phase "4" "WATCHDOG ARM DETECTION (Bench, No Props)"
step "4.1" "Monitor watchdog (arm on transmitter to test)"
note "Arm on RC transmitter while watching this output."
run_cmd "sudo journalctl -u drone-watchdog -f --since now"

step "4.2" "Monitor main.py mode detection"
note "Arm without OFFBOARD (10s debounce for MODE 2 / manual scan)."
run_cmd "sudo journalctl -u dronepi-main -f --since now"

step "4.3" "Check lock file state"
run_cmd "cat /tmp/dronepi_mission.lock 2>/dev/null || echo 'No lock file (correct when idle)'"
fi

# ── PHASE 5 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 5 ]; then
phase "5" "OFFBOARD FLIGHT TESTS (Outdoor, Props On, GPS Required)"
safety "RC transmitter in hand. Kill switch ready. Area clear. GPS lock confirmed."

step "5.1" "GPS hover dry run (no motors)"
run_cmd "python3 $BASE/tests/test_offboard_gps.py --dry-run"

step "5.2" "Simple OFFBOARD hover (3m AGL, 15s hold)"
safety "PROPS ON. Drone will arm and take off automatically."
run_cmd "python3 $BASE/tests/test_offboard_flight.py --alt 3.0 --hold 15"

step "5.3" "GPS waypoint hover (5m AGL, 10s hold)"
run_cmd "python3 $BASE/tests/test_offboard_gps.py --alt 5.0 --hold 10"
fi

# ── PHASE 6 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 6 ]; then
phase "6" "SLAM BRIDGE FLIGHT TEST (Outdoor, Props On)"
safety "RC transmitter in hand. Kill switch ready."

step "6.1" "Out-and-back dry run (no motors)"
run_cmd "python3 $BASE/tests/test_slam_bridge_flight.py --dry-run --no-bridge --no-bag"

step "6.2" "Out-and-back GPS only, no bag (5m, 3m alt)"
safety "PROPS ON. Drone arms automatically."
run_cmd "python3 $BASE/tests/test_slam_bridge_flight.py --distance 5 --alt 3.0 --hold 5 --no-bridge --no-bag"

step "6.3" "Out-and-back with SLAM bridge active, no bag"
run_cmd "python3 $BASE/tests/test_slam_bridge_flight.py --distance 5 --alt 3.0 --hold 5 --no-bag"

step "6.4" "Out-and-back full recording (bag + postflight)"
run_cmd "python3 $BASE/tests/test_slam_bridge_flight.py --distance 8 --alt 4.0 --hold 10"
note "Mesh will appear in browser within 10s of landing."
fi

# ── PHASE 7 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 7 ]; then
phase "7" "FULL MISSION DRY RUN (Outdoor, No Motors)"
note "Upload survey mission in QGC before running these steps."

step "7.1" "Mission executor dry run"
run_cmd "python3 $BASE/flight/mission_executor.py --dry-run"

step "7.2" "Full flight mission dry run"
run_cmd "python3 $BASE/flight_mission.py --dry-run"
fi

# ── PHASE 8 ───────────────────────────────────────────────────────────────────
if [ "${JUMP_PHASE:-0}" -le 8 ]; then
phase "8" "FULL AUTONOMOUS SURVEY MISSION (Outdoor, Props On)"
safety "ONLY proceed after all previous phases pass."
safety "Second person as safety observer recommended."
safety "Area must be completely clear. RC in hand."

step "8.1" "Final preflight check"
run_cmd "sudo bash $BASE/../preflight_check.sh"

step "8.2" "Execute full autonomous mission (stop-and-trigger mode)"
note "Upload survey mission in QGC first."
note "Arm on transmitter when prompted, switch to OFFBOARD."
run_cmd "python3 $BASE/flight_mission.py --trigger-mode stop --fov-deg 70.0"
note "Check browser after landing: http://10.42.0.1:8080/meshview.html"
fi

echo ""
echo -e "${GREEN}${BOLD}━━━  Test sequence complete  ━━━${NC}"
echo ""
