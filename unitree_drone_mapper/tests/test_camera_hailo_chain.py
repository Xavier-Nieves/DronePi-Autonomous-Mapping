#!/usr/bin/env python3
"""
tests/test_camera_hailo_chain.py — Full camera → Hailo inference chain test.

Architecture
------------
    dronepi conda env (this script):
        CameraCapture (Picamera2, numpy 2.x) → /arducam/image_raw

    hailo_inference_env (subprocess):
        hailo_flight_node.py (hailo_platform, numpy 1.26.4)
            subscribes to /arducam/image_raw
            publishes /hailo/optical_flow, /hailo/ground_class

Why two environments are required
----------------------------------
hailo_platform requires numpy < 2.0 (confirmed: numpy 2.x causes
"Memory size ... got 0" errors in the C extension buffer protocol).
picamera2 and piexif require numpy 2.x. They cannot coexist in one
Python process. hailo_inference_env isolates the numpy 1.x requirement.

DDS readiness
-------------
The subprocess needs time to load HEFs and register its ROS subscriber.
The test waits for subscriber count on /arducam/image_raw to reach 2
(the test script's own subscriber + hailo_flight_node's subscriber)
before starting the collection timer. This eliminates the timing race.

Tests
-----
    T1  Dependencies importable
    T2  /arducam/image_raw publishes at >= 10 Hz
    T3  Hailo subscriber appears on /arducam/image_raw
    T4  /hailo/optical_flow receives messages
    T5  /hailo/ground_class receives messages
    T6  trigger() saves JPEG from live buffer

Run:
    cd tests/
    python test_camera_hailo_chain.py
    python test_camera_hailo_chain.py --skip-hailo   # camera only
    python test_camera_hailo_chain.py --duration 30
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

_TESTS_DIR  = Path(__file__).resolve().parent
_MAPPER_DIR = _TESTS_DIR.parent
sys.path.insert(0, str(_MAPPER_DIR))

HAILO_FLIGHT_SCRIPT = _MAPPER_DIR / "hailo" / "hailo_flight_node.py"
HAILO_ENV           = Path.home() / "hailo_inference_env" / "bin" / "python3"
OUTPUT_DIR          = _TESTS_DIR / "chain_test_output"

TOPIC_IMAGE  = "/arducam/image_raw"
TOPIC_FLOW   = "/hailo/optical_flow"
TOPIC_GROUND = "/hailo/ground_class"

MIN_CAMERA_HZ       = 10.0
HAILO_READY_TIMEOUT = 30.0
FLOW_WAIT_S         = 20.0
GROUND_WAIT_S       = 30.0


def _header(t): print(f"\n{'='*55}\n  {t}\n{'='*55}")
def _pass(m):   print(f"  [PASS] {m}")
def _fail(m):   print(f"  [FAIL] {m}")
def _info(m):   print(f"  [INFO] {m}")


def test_imports(skip_hailo: bool) -> bool:
    _header("TEST 1 — Dependency check")
    ok = True
    for name, imp in [
        ("rclpy",         "import rclpy"),
        ("picamera2",     "from picamera2 import Picamera2"),
        ("CameraCapture", "from flight.camera_capture import CameraCapture"),
        ("cv2",           "import cv2"),
        ("numpy",         "import numpy"),
    ]:
        try:
            exec(imp)
            _pass(name)
        except Exception as exc:
            _fail(f"{name}: {exc}")
            ok = False

    if not skip_hailo:
        for label, path in [
            ("hailo_flight_node.py", HAILO_FLIGHT_SCRIPT),
            ("hailo_inference_env",  HAILO_ENV),
        ]:
            if path.exists():
                _pass(label)
            else:
                _fail(f"{label} not found: {path}")
                ok = False

        # Confirm hailo_inference_env has numpy 1.x
        try:
            result = subprocess.run(
                [str(HAILO_ENV), "-c",
                 "import numpy as np; print(np.__version__)"],
                capture_output=True, text=True, timeout=10,
            )
            ver = result.stdout.strip()
            major = int(ver.split(".")[0])
            if major < 2:
                _pass(f"hailo_inference_env numpy={ver} (< 2.0, compatible)")
            else:
                _fail(f"hailo_inference_env numpy={ver} — must be < 2.0")
                _info("Fix: ~/hailo_inference_env/bin/pip install 'numpy==1.26.4'")
                ok = False
        except Exception as exc:
            _info(f"Could not check hailo numpy version: {exc}")

    return ok


def start_camera_and_ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg   import Image
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg      import String
    from flight.camera_capture import CameraCapture

    rclpy.init()
    ros_node = Node("test_chain_node")

    cam = CameraCapture(
        session_id="chain_test",
        output_dir=OUTPUT_DIR,
        ros_node=ros_node,
        enable_ros_publish=True,
        ros_publish_topic=TOPIC_IMAGE,
        ros_publish_fps=15.0,
    )
    if not cam.start():
        return None, None, None, None

    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST, depth=5,
    )
    reliable_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST, depth=5,
    )

    stats = {
        "image_timestamps": [],
        "flow_messages":    [],
        "ground_messages":  [],
        "lock":             threading.Lock(),
    }

    def _image_cb(msg):
        with stats["lock"]:
            stats["image_timestamps"].append(time.monotonic())

    def _flow_cb(msg):
        with stats["lock"]:
            stats["flow_messages"].append({
                "vx": msg.twist.linear.x,
                "vy": msg.twist.linear.y,
                "confidence": msg.twist.linear.z,
                "rx_t": time.monotonic(),
            })

    def _ground_cb(msg):
        with stats["lock"]:
            try:
                d = json.loads(msg.data)
            except Exception:
                d = {"raw": msg.data}
            d["rx_t"] = time.monotonic()
            stats["ground_messages"].append(d)

    ros_node.create_subscription(Image,        TOPIC_IMAGE,  _image_cb,  sensor_qos)
    ros_node.create_subscription(TwistStamped, TOPIC_FLOW,   _flow_cb,   reliable_qos)
    ros_node.create_subscription(String,       TOPIC_GROUND, _ground_cb, reliable_qos)

    threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True, name="chain_ros_spin"
    ).start()

    return rclpy, ros_node, cam, stats


def start_hailo_subprocess():
    if not HAILO_ENV.exists() or not HAILO_FLIGHT_SCRIPT.exists():
        return None

    proc = subprocess.Popen(
        [str(HAILO_ENV), str(HAILO_FLIGHT_SCRIPT)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=os.environ.copy(),
        preexec_fn=os.setsid,
    )

    def _drain():
        for line in proc.stdout:
            print(f"  [Hailo] {line.rstrip()}")
    threading.Thread(target=_drain, daemon=True, name="hailo_stdout").start()
    _info(f"hailo_flight_node.py started (PID {proc.pid})")
    return proc


def stop_hailo_subprocess(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def wait_for_hailo_subscriber(ros_node, timeout_s: float) -> bool:
    """
    Block until subscriber count on /arducam/image_raw reaches 2.

    Count = 1: only the test script's own subscriber (always present).
    Count = 2: hailo_flight_node has also subscribed — safe to collect.

    Uses ros_node.count_subscribers() which queries the DDS graph directly.
    This is the authoritative check — no timing guesswork needed.
    """
    _info(f"Waiting for Hailo to subscribe to {TOPIC_IMAGE}...")
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            count   = ros_node.count_subscribers(TOPIC_IMAGE)
            elapsed = time.time() - (deadline - timeout_s)
            print(f"\r  [{elapsed:.0f}s] subscribers={count} (need >=2)...",
                  end="", flush=True)
            if count >= 2:
                print()
                _pass(f"Hailo subscribed after {elapsed:.1f}s")
                return True
        except Exception:
            pass
        time.sleep(0.5)
    print()
    _info("Hailo subscriber not detected — collection starting anyway")
    return False


def check_camera_rate(stats, duration_s) -> bool:
    _header("TEST 2 — /arducam/image_raw publish rate")
    with stats["lock"]:
        ts = list(stats["image_timestamps"])
    if len(ts) < 2:
        _fail(f"Only {len(ts)} frames")
        return False
    intervals = [ts[i+1]-ts[i] for i in range(len(ts)-1)]
    hz = 1.0 / (sum(intervals)/len(intervals))
    ok = hz >= MIN_CAMERA_HZ
    (  _pass if ok else _fail)(
        f"Rate: {hz:.1f} Hz  ({len(ts)} frames in {duration_s:.0f}s)"
    )
    return ok


def check_hailo_subscriber(ros_node) -> bool:
    _header("TEST 3 — Hailo subscriber registered")
    try:
        count = ros_node.count_subscribers(TOPIC_IMAGE)
        if count >= 2:
            _pass(f"Subscriber count: {count}")
            return True
        _fail(f"Subscriber count: {count} — expected >= 2")
        return False
    except Exception as exc:
        _fail(f"count_subscribers error: {exc}")
        return False


def check_hailo_flow(stats, wait_s) -> bool:
    _header(f"TEST 4 — /hailo/optical_flow  (up to {wait_s:.0f}s)")
    deadline = time.time() + wait_s
    while time.time() < deadline:
        with stats["lock"]:
            n = len(stats["flow_messages"])
        if n > 0:
            break
        elapsed = time.time() - (deadline - wait_s)
        print(f"\r  [{elapsed:.0f}s] flow_msgs={n}", end="", flush=True)
        time.sleep(0.5)
    print()
    with stats["lock"]:
        msgs = list(stats["flow_messages"])
    if not msgs:
        _fail(f"No messages after {wait_s:.0f}s")
        return False
    _pass(f"Received {len(msgs)} flow messages")
    if len(msgs) >= 2:
        span = msgs[-1]["rx_t"] - msgs[0]["rx_t"]
        hz   = (len(msgs)-1) / span if span > 0 else 0
        _pass(f"Flow rate: {hz:.1f} Hz")
    confs = [m["confidence"] for m in msgs]
    _info(f"Confidence min={min(confs):.3f} max={max(confs):.3f} "
          f"valid={sum(1 for c in confs if c>0)}/{len(msgs)}")
    _info("valid=0 expected on bench — needs real nadir motion")
    return True


def check_hailo_ground(stats, wait_s) -> bool:
    _header(f"TEST 5 — /hailo/ground_class  (up to {wait_s:.0f}s)")
    deadline = time.time() + wait_s
    while time.time() < deadline:
        with stats["lock"]:
            n = len(stats["ground_messages"])
        if n > 0:
            break
        elapsed = time.time() - (deadline - wait_s)
        print(f"\r  [{elapsed:.0f}s] ground_msgs={n}", end="", flush=True)
        time.sleep(0.5)
    print()
    with stats["lock"]:
        msgs = list(stats["ground_messages"])
    if not msgs:
        _fail(f"No messages after {wait_s:.0f}s")
        return False
    last = msgs[-1]
    _pass(f"Received {len(msgs)} ground messages")
    _pass(f"Latest: label={last.get('label','?')}  "
          f"conf={last.get('confidence',0):.3f}  "
          f"latency={last.get('latency_ms',0):.1f}ms")
    return True


def check_trigger(cam) -> bool:
    _header("TEST 6 — trigger() saves JPEG from live buffer")
    saved = cam.trigger({
        "waypoint_index": 0,
        "enu": (0.0, 0.0, 5.0),
        "gps": (None, None, None),
        "ros_timestamp": time.time(),
    }, timeout=3.0)
    if saved is None or not saved.exists():
        _fail("trigger() failed")
        return False
    kb = saved.stat().st_size / 1024
    jp = saved.with_suffix(".json")
    _pass(f"JPEG: {saved.name}  ({kb:.0f} KB)")
    _pass(f"Sidecar: {jp.name}" if jp.exists() else "[FAIL] Sidecar missing")
    return jp.exists()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration",   type=float, default=20.0)
    parser.add_argument("--skip-hailo", action="store_true")
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    results = {}

    results["imports"] = test_imports(args.skip_hailo)
    if not results["imports"]:
        sys.exit(1)

    _header("Starting camera and ROS node...")
    rclpy_ref, ros_node, cam, stats = start_camera_and_ros()
    if cam is None:
        print("\n[ABORT] CameraCapture failed")
        sys.exit(1)
    _pass("Picamera2 started")
    _info(f"Publishing {TOPIC_IMAGE} @ 15Hz")

    hailo_proc = None
    if not args.skip_hailo:
        _header("Starting hailo_flight_node (hailo_inference_env)...")
        hailo_proc = start_hailo_subprocess()
        if hailo_proc:
            wait_for_hailo_subscriber(ros_node, HAILO_READY_TIMEOUT)

    try:
        _info(f"Collecting for {args.duration:.0f}s ...")
        t0 = time.time()
        while time.time() - t0 < args.duration:
            with stats["lock"]:
                imgs   = len(stats["image_timestamps"])
                flows  = len(stats["flow_messages"])
                ground = len(stats["ground_messages"])
            print(
                f"\r  t={time.time()-t0:.0f}s  "
                f"image={imgs}  flow={flows}  ground={ground}    ",
                end="", flush=True,
            )
            time.sleep(0.5)
        print()

        results["camera_rate"] = check_camera_rate(stats, args.duration)
        results["trigger"]     = check_trigger(cam)

        if not args.skip_hailo and hailo_proc is not None:
            results["hailo_subscriber"] = check_hailo_subscriber(ros_node)
            results["hailo_flow"]       = check_hailo_flow(stats, FLOW_WAIT_S)
            results["hailo_ground"]     = check_hailo_ground(stats, GROUND_WAIT_S)

    finally:
        stop_hailo_subprocess(hailo_proc)
        cam.stop()
        ros_node.destroy_node()
        if rclpy_ref.ok():
            rclpy_ref.shutdown()

    _header("SUMMARY")
    passed = sum(1 for v in results.values() if v)
    total  = len(results)
    for name, ok in results.items():
        print(f"  {'[PASS]' if ok else '[FAIL]'} {name}")
    print()
    if passed == total:
        print(f"  {passed}/{total} passed")
        print("  Full chain verified — ready for props-on test")
    else:
        print(f"  {passed}/{total} passed")
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
