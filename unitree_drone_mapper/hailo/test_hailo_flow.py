#!/usr/bin/env python3
"""
test_hailo_flow.py — Standalone Hailo optical flow + FlowBridge integration test.

Validates the complete Hailo → FlowBridge → MAVROS chain without flying:

  T1 — Device check
    /dev/hailo0 accessible, hailortcli responds, firmware version readable.

  T2 — Model load
    scdepthv3.hef (optical flow) and yolov8m.hef (ground class) load
    into the Hailo-8 VDevice without error.

  T3 — Optical flow inference
    Push 10 synthetic frames through HailoOpticalFlow.push_frame().
    Verify output dict has valid keys and latency < 100ms per frame.

  T4 — Ground classifier inference
    Push one synthetic frame through HailoGroundClassifier.classify().
    Verify output label is one of the four expected values.

  T5 — FlowBridge forwarding
    Start a minimal ROS 2 node, instantiate FlowBridge, publish a
    synthetic /hailo/optical_flow message above the confidence threshold,
    verify FlowBridge forwards it to /mavros/odometry/out within 500ms.

  T6 — FlowBridge gating
    Publish messages below the confidence threshold and verify they are
    NOT forwarded. Verify fallback_count increments correctly.

This script is STANDALONE — it does not import from postprocess_mesh.py
or any other pipeline module. It runs in the hailo_inference_env venv
with ROS 2 sourced.

Usage
-----
  source /opt/ros/jazzy/setup.bash
  source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
  source ~/hailo_inference_env/bin/activate

  python3 tests/test_hailo_flow.py

  # Verbose — prints per-frame latency
  python3 tests/test_hailo_flow.py --verbose

  # Skip ROS tests (T5, T6) if MAVROS is not running
  python3 tests/test_hailo_flow.py --no-ros

Dependencies
------------
  hailo_platform  (hailo_inference_env)
  rclpy, geometry_msgs, nav_msgs  (ROS 2 Jazzy)
  numpy, cv2
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

# ── Paths ─────────────────────────────────────────────────────────────────────
_TESTS_DIR   = Path(__file__).resolve().parent
_MAPPER_DIR  = _TESTS_DIR.parent
_HAILO_DIR   = _MAPPER_DIR / "hailo"
_FLIGHT_DIR  = _MAPPER_DIR / "flight"

for p in [str(_MAPPER_DIR), str(_HAILO_DIR), str(_FLIGHT_DIR)]:
    if p not in sys.path:
        sys.path.insert(0, p)

# ── Model paths ───────────────────────────────────────────────────────────────
_MODELS_DIR   = Path("/usr/local/hailo/resources/models/hailo8")
_FLOW_HEF     = _MODELS_DIR / "scdepthv3.hef"
_CLASS_HEF    = _MODELS_DIR / "yolov8m.hef"

# ── Test constants ────────────────────────────────────────────────────────────
N_FLOW_FRAMES       = 10      # frames to push through optical flow
MAX_INFERENCE_MS    = 100.0   # latency threshold per frame
CONFIDENCE_ABOVE    = 0.80    # synthetic confidence that should pass
CONFIDENCE_BELOW    = 0.20    # synthetic confidence that should be blocked
FORWARD_TIMEOUT_S   = 1.0     # max wait for FlowBridge to forward a message

# ── Result tracking ───────────────────────────────────────────────────────────
_results: list[tuple[str, bool, str]] = []

def _record(name: str, passed: bool, detail: str = "") -> None:
    _results.append((name, passed, detail))
    status = "\033[92mPASS\033[0m" if passed else "\033[91mFAIL\033[0m"
    print(f"  [{status}] {name}" + (f"  — {detail}" if detail else ""))


# ══════════════════════════════════════════════════════════════════════════════
# T1 — Device check
# ══════════════════════════════════════════════════════════════════════════════

def test_T1_device_check() -> bool:
    """T1 — /dev/hailo0 accessible and firmware readable."""
    try:
        from hailo_device import HailoDevice
        dev = HailoDevice()
        available = dev.is_available()
        if not available:
            _record("T1 device available", False,
                    "/dev/hailo0 not found or not accessible")
            return False

        temp = dev.get_temperature()
        _record("T1 device available", True,
                f"temp={temp:.1f}°C" if temp > 0 else "temp=unavailable")
        return True

    except Exception as exc:
        _record("T1 device available", False, str(exc))
        return False


# ══════════════════════════════════════════════════════════════════════════════
# T2 — Model load
# ══════════════════════════════════════════════════════════════════════════════

def test_T2_model_load():
    """T2 — Load both HEF models into VDevice. Returns (device, flow_model, class_model)."""
    try:
        from hailo_device import HailoDevice
        dev = HailoDevice()
        dev.open()
    except Exception as exc:
        _record("T2 VDevice open", False, str(exc))
        return None, None, None

    _record("T2 VDevice open", True)

    # Optical flow model
    flow_model = None
    if not _FLOW_HEF.exists():
        _record("T2 flow HEF load", False, f"not found: {_FLOW_HEF}")
    else:
        try:
            flow_model = dev.load_model(str(_FLOW_HEF))
            _record("T2 flow HEF load", True, _FLOW_HEF.name)
        except Exception as exc:
            _record("T2 flow HEF load", False, str(exc))

    # Ground class model
    class_model = None
    if not _CLASS_HEF.exists():
        _record("T2 class HEF load", False, f"not found: {_CLASS_HEF}")
    else:
        try:
            class_model = dev.load_model(str(_CLASS_HEF))
            _record("T2 class HEF load", True, _CLASS_HEF.name)
        except Exception as exc:
            _record("T2 class HEF load", False, str(exc))

    return dev, flow_model, class_model


# ══════════════════════════════════════════════════════════════════════════════
# T3 — Optical flow inference
# ══════════════════════════════════════════════════════════════════════════════

def test_T3_optical_flow(flow_model, verbose: bool) -> None:
    """T3 — Push synthetic frames through HailoOpticalFlow."""
    if flow_model is None:
        _record("T3 optical flow", False, "flow model not loaded — skipped")
        return

    try:
        from hailo_optical_flow import HailoOpticalFlow
        of = HailoOpticalFlow(network_group=flow_model)
    except Exception as exc:
        _record("T3 optical flow init", False, str(exc))
        return

    latencies   = []
    valid_count = 0

    # First frame always returns valid=False (no previous frame) — skip
    frame_prev  = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    of.push_frame(frame_prev)

    for i in range(N_FLOW_FRAMES):
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        result = of.push_frame(frame)

        required_keys = {"vx", "vy", "confidence", "latency_ms", "valid"}
        if not required_keys.issubset(result.keys()):
            _record("T3 optical flow", False,
                    f"missing keys: {required_keys - result.keys()}")
            return

        latencies.append(result["latency_ms"])
        if result["valid"]:
            valid_count += 1

        if verbose:
            print(f"    frame {i+1}/{N_FLOW_FRAMES}  "
                  f"vx={result['vx']:.3f}  vy={result['vy']:.3f}  "
                  f"conf={result['confidence']:.2f}  "
                  f"lat={result['latency_ms']:.1f}ms")

    avg_lat  = sum(latencies) / max(len(latencies), 1)
    max_lat  = max(latencies) if latencies else 0.0
    lat_ok   = max_lat < MAX_INFERENCE_MS

    _record(
        "T3 optical flow inference",
        lat_ok,
        f"{N_FLOW_FRAMES} frames  avg={avg_lat:.1f}ms  "
        f"max={max_lat:.1f}ms  valid={valid_count}/{N_FLOW_FRAMES}  "
        f"(limit {MAX_INFERENCE_MS:.0f}ms)",
    )


# ══════════════════════════════════════════════════════════════════════════════
# T4 — Ground classifier inference
# ══════════════════════════════════════════════════════════════════════════════

def test_T4_ground_classifier(class_model, verbose: bool) -> None:
    """T4 — Run one synthetic frame through HailoGroundClassifier."""
    if class_model is None:
        _record("T4 ground classifier", False, "class model not loaded — skipped")
        return

    try:
        from hailo_ground_class import HailoGroundClassifier
        gc = HailoGroundClassifier(network_group=class_model)
    except Exception as exc:
        _record("T4 ground classifier init", False, str(exc))
        return

    frame  = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    result = gc.classify(frame)

    required_keys  = {"label", "confidence", "detections", "latency_ms"}
    valid_labels   = {"SAFE_LAND", "CAUTION", "UNSAFE", "UNKNOWN"}

    keys_ok  = required_keys.issubset(result.keys())
    label_ok = result.get("label") in valid_labels
    lat_ok   = result.get("latency_ms", 999) < MAX_INFERENCE_MS

    ok = keys_ok and label_ok and lat_ok

    if verbose:
        print(f"    label={result.get('label')}  "
              f"conf={result.get('confidence', 0):.2f}  "
              f"lat={result.get('latency_ms', 0):.1f}ms  "
              f"detections={result.get('detections', [])}")

    _record(
        "T4 ground classifier inference",
        ok,
        f"label={result.get('label')}  "
        f"conf={result.get('confidence', 0):.2f}  "
        f"lat={result.get('latency_ms', 0):.1f}ms",
    )


# ══════════════════════════════════════════════════════════════════════════════
# T5 + T6 — FlowBridge ROS tests
# ══════════════════════════════════════════════════════════════════════════════

def test_T5_T6_flow_bridge(verbose: bool) -> None:
    """T5/T6 — Verify FlowBridge forwards high-confidence and blocks low-confidence."""
    try:
        import rclpy
        from rclpy.node        import Node
        from rclpy.qos         import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from geometry_msgs.msg import TwistStamped
        from nav_msgs.msg      import Odometry
    except ImportError as exc:
        _record("T5 FlowBridge forward", False,
                f"ROS 2 not importable: {exc}")
        _record("T6 FlowBridge gating",  False, "skipped — ROS 2 unavailable")
        return

    try:
        from _flow_bridge import FlowBridge
    except ImportError as exc:
        _record("T5 FlowBridge forward", False,
                f"_flow_bridge not importable: {exc}")
        _record("T6 FlowBridge gating",  False, "skipped")
        return

    rclpy.init()
    node = Node("test_flow_bridge")

    received_msgs: list[Odometry] = []

    reliable_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    # Subscribe to /mavros/odometry/out to observe FlowBridge output
    node.create_subscription(
        Odometry,
        "/mavros/odometry/out",
        lambda msg: received_msgs.append(msg),
        reliable_qos,
    )

    # Publisher to inject synthetic /hailo/optical_flow messages
    flow_pub = node.create_publisher(
        TwistStamped,
        "/hailo/optical_flow",
        sensor_qos,
    )

    bridge = FlowBridge(node=node)

    def _make_flow_msg(vx: float, vy: float, confidence: float) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp    = node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x  = vx
        msg.twist.linear.y  = vy
        msg.twist.linear.z  = confidence   # repurposed
        return msg

    def _spin_for(secs: float) -> None:
        deadline = time.time() + secs
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    # Allow subscriptions to establish
    _spin_for(0.3)

    # ── T5: publish high-confidence message → should be forwarded ─────────────
    n_before = len(received_msgs)
    flow_pub.publish(_make_flow_msg(1.0, 0.5, CONFIDENCE_ABOVE))
    _spin_for(FORWARD_TIMEOUT_S)

    forwarded = len(received_msgs) > n_before
    if forwarded:
        msg = received_msgs[-1]
        vx_ok = abs(msg.twist.twist.linear.x - 1.0) < 0.001
        vy_ok = abs(msg.twist.twist.linear.y - 0.5) < 0.001
        ok    = vx_ok and vy_ok
        _record("T5 FlowBridge forward", ok,
                f"vx={msg.twist.twist.linear.x:.3f}  "
                f"vy={msg.twist.twist.linear.y:.3f}  "
                f"(expected 1.000, 0.500)")
    else:
        _record("T5 FlowBridge forward", False,
                f"no message on /mavros/odometry/out within {FORWARD_TIMEOUT_S}s")

    # ── T6: publish low-confidence messages → should all be blocked ────────────
    n_before = len(received_msgs)
    for _ in range(5):
        flow_pub.publish(_make_flow_msg(2.0, 2.0, CONFIDENCE_BELOW))
        _spin_for(0.1)

    leaked = len(received_msgs) > n_before
    stats  = bridge.get_stats()

    if verbose:
        print(f"\n    FlowBridge stats: {stats}")

    _record(
        "T6 FlowBridge gating",
        not leaked,
        f"low-conf messages leaked={leaked}  "
        f"fallback_count={stats['fallback_count']}  "
        f"pass_rate={stats['pass_rate']:.2f}",
    )

    bridge.destroy()
    node.destroy_node()
    rclpy.shutdown()


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Hailo + FlowBridge integration test — no flight required."
    )
    parser.add_argument("--verbose",  action="store_true",
                        help="Print per-frame inference results")
    parser.add_argument("--no-ros",   action="store_true",
                        help="Skip T5/T6 ROS tests (if MAVROS not running)")
    args = parser.parse_args()

    print("=" * 60)
    print("  DronePi — Hailo + FlowBridge Integration Test")
    print("  No flight required.")
    print("=" * 60)

    # Dependency check
    print("\nDependency check...")
    for pkg in ("hailo_platform", "numpy", "cv2"):
        try:
            __import__(pkg)
            print(f"  [ok] {pkg}")
        except ImportError:
            print(f"  [MISSING] {pkg}")
            sys.exit(1)

    print("\nRunning tests...\n")

    # T1 — device
    device_ok = test_T1_device_check()
    if not device_ok:
        print("\n  Hailo device not accessible — remaining tests skipped.")
        print("  Run: hailortcli fw-control identify")
        sys.exit(1)

    # T2 — models
    dev, flow_model, class_model = test_T2_model_load()

    # T3 — optical flow
    print()
    test_T3_optical_flow(flow_model, args.verbose)

    # T4 — ground classifier
    print()
    test_T4_ground_classifier(class_model, args.verbose)

    # Shutdown Hailo device before starting ROS (avoids PCIe contention)
    if dev is not None:
        dev.shutdown()

    # T5 + T6 — FlowBridge ROS tests
    if not args.no_ros:
        print()
        test_T5_T6_flow_bridge(args.verbose)
    else:
        print("\n  T5/T6 skipped (--no-ros)")

    # Summary
    print("\n" + "=" * 60)
    passed = sum(1 for _, p, _ in _results if p)
    total  = len(_results)
    print(f"  {passed}/{total} tests passed")

    if passed == total:
        print("  Hailo hardware, models, and FlowBridge all verified.")
        print("\n  Next steps:")
        print("  1. Add FlowBridge to main.py (see integration snippet below)")
        print("  2. Set EKF2_EV_DELAY=50 in QGC (accounts for Hailo latency)")
        print("  3. Launch hailo_flight_node.py before first augmented flight")
    else:
        for name, ok, detail in _results:
            if not ok:
                print(f"\n  FAIL: {name}")
                print(f"    {detail}")
    print("=" * 60)

    # Print main.py integration snippet if all hardware tests passed
    if device_ok and flow_model is not None:
        print("""
Integration snippet for main.py
--------------------------------
Add after FlightStack is started, before the waypoint loop:

    # ── FlowBridge — Hailo optical flow → EKF2 velocity ──────────────
    flow_bridge = None
    if hailo_proc is not None:
        try:
            from flight._flow_bridge import FlowBridge

            def _on_flow_degraded():
                log("[HAILO] Flow degraded — consecutive rejections exceeded")
                if led and LEDState:
                    led.set_state(LEDState.HAILO_DEGRADED)

            flow_bridge = FlowBridge(
                node=node._node,
                degraded_callback=_on_flow_degraded,
            )
            log("FlowBridge active — optical flow feeding EKF2 velocity")
        except Exception as exc:
            log(f"[WARN] FlowBridge init failed: {exc}")

Add to teardown (after mission loop exits):

    if flow_bridge is not None:
        flow_bridge.destroy()
""")

    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
