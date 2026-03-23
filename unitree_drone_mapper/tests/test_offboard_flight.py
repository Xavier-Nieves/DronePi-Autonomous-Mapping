#!/usr/bin/env python3
"""OFFBOARD hover test — local ENU setpoints via MAVROS.

Uses the FlightController module for clean, reusable flight control.

Sequence:
    1. Wait for FCU connection
    2. Wait for EKF stability
    3. Pre-stream setpoints at home position
    4. Arm and switch to OFFBOARD
    5. Climb to target altitude
    6. Hold for specified duration
    7. Land and disarm

Usage:
    source /opt/ros/jazzy/setup.bash
    source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash

    python3 test_offboard_flight.py --dry-run        # No arming, checks only
    python3 test_offboard_flight.py --alt 1.5 --hold 10
    python3 test_offboard_flight.py --no-mavros      # Use systemd MAVROS

SAFETY:
    - RC transmitter in hand at all times
    - Kill switch within reach
    - Switch to STABILIZED immediately if behavior is unexpected
"""

import argparse
import os
import signal
import subprocess
import sys
import time

# Add flight directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flight_controller import FlightController

# ── Config ────────────────────────────────────────────────────────────────────

TARGET_ALT      = 1.5    # metres above home
HOLD_SECONDS    = 10     # seconds to hold hover
EKF_TIMEOUT     = 30.0
ARM_TIMEOUT     = 10.0
MODE_TIMEOUT    = 10.0
TAKEOFF_TIMEOUT = 25.0
LAND_TIMEOUT    = 30.0
MAVROS_STARTUP  = 8.0

ROS_SETUP  = "/opt/ros/jazzy/setup.bash"
MAVROS_URL = "serial:///dev/ttyACM0:57600"


# ── MAVROS Launcher ───────────────────────────────────────────────────────────

def launch_mavros() -> subprocess.Popen:
    """Launch MAVROS as a background subprocess."""
    cmd = (
        f"source {ROS_SETUP} && "
        f"ros2 launch mavros px4.launch fcu_url:={MAVROS_URL}"
    )
    print("  Launching MAVROS...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    print(f"  MAVROS PID: {proc.pid} — waiting {MAVROS_STARTUP:.0f}s...")
    time.sleep(MAVROS_STARTUP)
    print("  [OK] MAVROS ready")
    return proc


def kill_mavros(proc: subprocess.Popen):
    """Kill MAVROS process group."""
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
        print("  [OK] MAVROS stopped")
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


# ── Flight Sequence ───────────────────────────────────────────────────────────

def run_flight(fc: FlightController, args) -> bool:
    """Execute the hover test flight sequence."""
    
    # 1 — FCU connection
    print("\n[1/8] Waiting for FCU connection...")
    if not fc.wait_for_connection(timeout=15.0):
        print("  [FAIL] MAVROS not connected")
        return False
    print("  [OK] FCU connected")

    # 2 — EKF initialization
    print(f"\n[2/8] Waiting for EKF initialization...")
    if not fc.wait_for_ekf(timeout=EKF_TIMEOUT, require_gps=False):
        print("  [FAIL] EKF did not stabilize")
        return False
    print("  [OK] EKF stable")

    # Get home position
    hx, hy, hz = fc.get_position()
    target_z = hz + args.alt
    
    # 3 — Pre-stream setpoints
    print(f"\n[3/8] Pre-streaming setpoints at home ({hx:.2f}, {hy:.2f}, {hz:.2f})...")
    fc.stream_setpoint(hx, hy, hz, yaw=0.0, duration=3.0)
    print("  [OK] Setpoint stream active")

    if args.dry_run:
        print("\n  --dry-run: stopping before arm")
        print(f"  Would hover at Z={target_z:.2f}m for {args.hold}s")
        return True

    # 4 — Arm
    print("\n[4/8] Arming...")
    print("  WARNING — motors will spin. Area must be clear.")
    if not fc.arm():
        print("  [FAIL] Arm rejected")
        return False
    
    if not fc.wait_for_arm(timeout=ARM_TIMEOUT):
        print("  [FAIL] Did not arm in time")
        return False
    print("  [OK] Armed")

    # 5 — Switch to OFFBOARD
    print("\n[5/8] Switching to OFFBOARD...")
    if not fc.set_mode("OFFBOARD"):
        print("  [FAIL] Mode switch failed")
        fc.disarm()
        return False
    
    # Keep streaming while waiting for mode
    deadline = time.time() + MODE_TIMEOUT
    while time.time() < deadline:
        fc.publish_setpoint(hx, hy, target_z, yaw=0.0)
        if fc.get_mode() == "OFFBOARD":
            break
        time.sleep(0.05)
    
    if fc.get_mode() != "OFFBOARD":
        print(f"  [FAIL] Still in {fc.get_mode()}")
        fc.disarm()
        return False
    print("  [OK] OFFBOARD active")

    # 6 — Climb
    print(f"\n[6/8] Climbing to {args.alt:.1f}m above home (Z={target_z:.2f}m)...")
    reached = fc.fly_to(hx, hy, target_z, yaw=0.0, timeout=TAKEOFF_TIMEOUT)
    if reached:
        print(f"  [OK] Target altitude reached")
    else:
        print(f"  [WARN] Timeout — continuing at current altitude")

    # 7 — Hold
    print(f"\n[7/8] Holding for {args.hold}s...")
    for i in range(args.hold, 0, -1):
        fc.publish_setpoint(hx, hy, target_z, yaw=0.0)
        alt_rel = fc.get_altitude_above_home()
        print(f"\r  {i}s remaining | alt above home: {alt_rel:.2f}m    ",
              end="", flush=True)
        time.sleep(1.0)
    print()
    print("  [OK] Hold complete")

    # 8 — Land
    print("\n[8/8] Landing...")
    if fc.set_mode("AUTO.LAND"):
        print("  AUTO.LAND active...")
        deadline = time.time() + LAND_TIMEOUT
        while time.time() < deadline:
            alt_rel = fc.get_altitude_above_home()
            print(f"\r  Descending: {alt_rel:.2f}m above home    ",
                  end="", flush=True)
            if alt_rel < 0.15:
                break
            time.sleep(0.2)
        print()
    else:
        print("  [WARN] AUTO.LAND failed — descending manually")
        current_z = fc.get_altitude()
        while current_z > hz + 0.1:
            current_z = max(hz, current_z - 0.03)
            fc.publish_setpoint(hx, hy, current_z, yaw=0.0)
            time.sleep(0.1)

    # Wait for disarm
    time.sleep(1.5)
    fc.disarm()
    
    if fc.wait_for_disarm(timeout=5.0):
        print("  [OK] Disarmed. Safe.")
        return True
    else:
        print("  [WARN] Still armed — use RC kill switch!")
        return False


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="OFFBOARD hover test using FlightController")
    parser.add_argument("--alt", type=float, default=TARGET_ALT,
                        help=f"Hover altitude above home (default: {TARGET_ALT}m)")
    parser.add_argument("--hold", type=int, default=HOLD_SECONDS,
                        help=f"Hold duration (default: {HOLD_SECONDS}s)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Check EKF + setpoints only, no arming")
    parser.add_argument("--no-mavros", action="store_true",
                        help="Use existing MAVROS (systemd)")
    args = parser.parse_args()

    print("=" * 55)
    print("  OFFBOARD Hover Test (FlightController)")
    print("=" * 55)
    print(f"\n  Target altitude : {args.alt}m above home")
    print(f"  Hold duration   : {args.hold}s")
    print(f"  Dry run         : {'YES' if args.dry_run else 'NO — LIVE FLIGHT'}")

    if not args.dry_run:
        print("\n  !! LIVE FLIGHT !!")
        print("  Props installed, area clear, RC in hand, kill switch ready.")
        print("  Ctrl+C to abort. Starting in 5s...")
        try:
            for i in range(5, 0, -1):
                print(f"\r  {i}...", end="", flush=True)
                time.sleep(1)
            print()
        except KeyboardInterrupt:
            print("\n  Aborted.")
            sys.exit(0)

    # Launch MAVROS if needed
    mavros_proc = None
    if not args.no_mavros:
        mavros_proc = launch_mavros()
    else:
        print("  Using existing MAVROS instance")

    # Create flight controller
    fc = FlightController(node_name="offboard_hover_test")
    success = False

    try:
        success = run_flight(fc, args)
    except KeyboardInterrupt:
        print("\n\n  Ctrl+C — emergency abort")
        try:
            fc.set_mode("AUTO.LAND")
            time.sleep(2)
            fc.disarm()
        except Exception:
            pass
        print("  Switch to STABILIZED or use kill switch!")
    except Exception as e:
        print(f"\n  [ERROR] {e}")
        try:
            fc.set_mode("AUTO.LAND")
            time.sleep(2)
            fc.disarm()
        except Exception:
            pass
    finally:
        fc.shutdown()
        kill_mavros(mavros_proc)

    print("\n" + "=" * 55)
    print("  PASSED" if success else "  Did not complete — review output")
    print("=" * 55)


if __name__ == "__main__":
    main()
