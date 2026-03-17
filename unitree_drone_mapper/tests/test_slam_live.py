#!/usr/bin/env python3
"""Live SLAM test — launches LiDAR driver, Point-LIO, and RViz.

Run from the unitree_drone_mapper/ directory:
    python3 -m tests.test_slam_live
    python3 -m tests.test_slam_live --no-rviz
    python3 -m tests.test_slam_live --port /dev/ttyUSB1

Requires:
    - Unitree L1 LiDAR connected on /dev/ttyUSB0
    - ROS 2 Jazzy installed at /opt/ros/jazzy
    - Workspace built at ~/unitree_lidar_project/RPI5/ros2_ws
    - Display available for RViz (DISPLAY env var set)

Press Ctrl+C to stop all processes cleanly.
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time

# ── paths ─────────────────────────────────────────────────────────────────────
ROS_SETUP       = "/opt/ros/jazzy/setup.bash"
WS_SETUP        = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
LAUNCH_FILE     = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)
LIDAR_PORT      = "/dev/ttyUSB0"

# Topics that confirm each layer is alive
TOPIC_CHECKS = {
    "/unilidar/cloud":   "LiDAR driver",
    "/cloud_registered": "Point-LIO cloud",
    "/Odometry":         "SLAM odometry",
}


# ── helpers ───────────────────────────────────────────────────────────────────

def _bash(cmd: str) -> str:
    """Run a bash command string with both ROS setups sourced."""
    full = f"source {ROS_SETUP} && source {WS_SETUP} && {cmd}"
    result = subprocess.run(
        ["bash", "-c", full],
        capture_output=True, text=True, timeout=20,
    )
    return result.stdout.strip()


def check_prerequisites(port: str) -> bool:
    """Verify hardware and environment before launching."""
    ok = True

    # LiDAR device
    if os.path.exists(port):
        print(f"  [OK] LiDAR port {port} found")
    else:
        print(f"  [FAIL] LiDAR port {port} not found — is the LiDAR plugged in?")
        ok = False

    # ROS install
    if os.path.isfile(ROS_SETUP):
        print(f"  [OK] ROS 2 Jazzy at {ROS_SETUP}")
    else:
        print(f"  [FAIL] ROS 2 not found at {ROS_SETUP}")
        ok = False

    # Workspace
    if os.path.isfile(WS_SETUP):
        print(f"  [OK] Workspace install found")
    else:
        print(f"  [FAIL] Workspace not built — run: colcon build in RPI5/ros2_ws")
        ok = False

    # Launch file
    if os.path.isfile(LAUNCH_FILE):
        print(f"  [OK] Launch file found")
    else:
        print(f"  [FAIL] Launch file missing: {LAUNCH_FILE}")
        ok = False

    # Display (for RViz)
    if os.environ.get("DISPLAY"):
        print(f"  [OK] DISPLAY={os.environ['DISPLAY']}")
    else:
        print("  [WARN] DISPLAY not set — RViz will fail (use --no-rviz to skip)")

    return ok


def monitor_topics(stop_event: threading.Event):
    """Poll topic rates every 5 s and print a status line."""
    time.sleep(12)  # Give nodes time to start
    while not stop_event.is_set():
        print("\n--- Topic status ---")
        for topic, label in TOPIC_CHECKS.items():
            try:
                out = _bash(f"ros2 topic hz {topic} --window 3 2>&1 | head -2")
                if "average rate" in out:
                    rate_line = [l for l in out.splitlines() if "average rate" in l]
                    rate = rate_line[0].strip() if rate_line else "running"
                    print(f"  {label:25s} {rate}")
                elif "no messages" in out.lower() or out == "":
                    print(f"  {label:25s} no data yet")
                else:
                    print(f"  {label:25s} {out.splitlines()[0]}")
            except subprocess.TimeoutExpired:
                print(f"  {label:25s} timeout")
        print("--------------------\n")
        stop_event.wait(10)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Launch Point-LIO SLAM with RViz")
    parser.add_argument("--no-rviz", action="store_true",
                        help="Skip RViz (useful on headless systems)")
    parser.add_argument("--port", default=LIDAR_PORT,
                        help=f"LiDAR serial port (default: {LIDAR_PORT})")
    parser.add_argument("--no-check", action="store_true",
                        help="Skip prerequisite checks")
    args = parser.parse_args()

    print("=" * 55)
    print("  Point-LIO Live SLAM Test")
    print("=" * 55)

    # ── prerequisite checks ───────────────────────────────────────────────
    if not args.no_check:
        print("\nChecking prerequisites...")
        if not check_prerequisites(args.port):
            print("\nFix the errors above and re-run.")
            sys.exit(1)
        print()

    # ── build launch command ──────────────────────────────────────────────
    rviz_flag = "false" if args.no_rviz else "true"
    launch_cmd = (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"ros2 launch {LAUNCH_FILE} "
        f"rviz:={rviz_flag} "
        f"port:={args.port}"
    )

    print("Starting:")
    print(f"  LiDAR driver  -> /unilidar/cloud")
    print(f"  Point-LIO     -> /cloud_registered, /Odometry")
    if not args.no_rviz:
        print(f"  RViz          -> loam_livox config")
    print("\nPress Ctrl+C to stop all processes.\n")

    # ── launch ───────────────────────────────────────────────────────────
    proc = subprocess.Popen(
        ["bash", "-c", launch_cmd],
        preexec_fn=os.setsid,   # Put in own process group for clean kill
    )

    # ── topic monitor thread ──────────────────────────────────────────────
    stop_event = threading.Event()
    monitor_thread = threading.Thread(
        target=monitor_topics, args=(stop_event,), daemon=True
    )
    monitor_thread.start()

    # ── wait for Ctrl+C ───────────────────────────────────────────────────
    try:
        proc.wait()
    except KeyboardInterrupt:
        print("\n\nCtrl+C — shutting down...")
    finally:
        stop_event.set()

        # Kill the entire process group (catches all child nodes)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass  # Already gone

        # Give nodes a moment to save PCD / flush buffers
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            print("Forcing shutdown...")
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)

        print("All processes stopped.")
        print("\nIf Point-LIO saved a PCD, check:")
        print("  ~/unitree_lidar_project/RPI5/ros2_ws/  (default pcd_save path)")


if __name__ == "__main__":
    main()
