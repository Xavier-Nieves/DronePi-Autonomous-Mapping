#!/usr/bin/env python3
import asyncio
import json
import math
from typing import Any, Dict, Set

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import State
import websockets


WS_HOST = "0.0.0.0"
WS_PORT = 9001


def quat_to_euler_deg(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
      pitch = math.degrees(math.copysign(math.pi / 2, sinp))
    else:
      pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    return roll, pitch, yaw


class MeshviewTelemetryBridge(Node):
    def __init__(self):
        super().__init__("meshview_telemetry_bridge")

        # Named _ws_clients to avoid collision with rclpy.Node.clients property.
        self._ws_clients: Set[websockets.WebSocketServerProtocol] = set()

        self.latest: Dict[str, Any] = {
            "vehicle": {
                "state": "UNKNOWN",
                "connected": False,
                "armed": False,
                "mode": "—",
                "satellites": None,
                "altitude_m": None,
                "speed_mps": None,
                "roll_deg": None,
                "pitch_deg": None,
                "yaw_deg": None,
                "gps": {"lat": None, "lon": None, "alt": None},
            },
            "rpi": {
                "cpu_pct": None,
                "cpu_temp_c": None,
                "mem_pct": None,
                "disk_pct": None,
                "cpu_freq_mhz": None,
                "throttled": None,
                "uptime_s": None,
            },
        }

        self.create_subscription(State, "/mavros/state", self.on_state, 10)
        self.create_subscription(Float64, "/mavros/global_position/rel_alt", self.on_alt, 10)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self.on_speed, 10)
        self.create_subscription(Imu, "/mavros/imu/data", self.on_imu, 10)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self.on_gps, 10)
        self.create_subscription(String, "/rpi/health", self.on_rpi_health, 10)

        self.create_timer(1.0, self.broadcast_snapshot)
        self.get_logger().info(f"MeshView telemetry bridge ready on ws://{WS_HOST}:{WS_PORT}")

    async def register(self, websocket):
        self._ws_clients.add(websocket)
        await websocket.send(json.dumps({
            "op": "snapshot",
            "data": self.latest,
        }))

    def unregister(self, websocket):
        self._ws_clients.discard(websocket)

    async def _broadcast(self, payload: Dict[str, Any]):
        if not self._ws_clients:
            return
        msg = json.dumps(payload)
        dead = set()
        for ws in self._ws_clients:
            try:
                await ws.send(msg)
            except Exception:
                dead.add(ws)
        for ws in dead:
            self._ws_clients.discard(ws)

    def broadcast_now(self, payload: Dict[str, Any]):
        try:
            loop = asyncio.get_running_loop()
            loop.create_task(self._broadcast(payload))
        except RuntimeError:
            pass

    def broadcast_snapshot(self):
        self.broadcast_now({
            "op": "snapshot",
            "data": self.latest,
        })

    def on_state(self, msg: State):
        self.latest["vehicle"]["connected"] = bool(msg.connected)
        self.latest["vehicle"]["armed"] = bool(msg.armed)
        self.latest["vehicle"]["mode"] = msg.mode or "—"
        self.latest["vehicle"]["state"] = "CONNECTED" if msg.connected else "DISCONNECTED"
        self.broadcast_now({"op": "vehicle", "data": self.latest["vehicle"]})

    def on_alt(self, msg: Float64):
        self.latest["vehicle"]["altitude_m"] = round(float(msg.data), 2)
        self.broadcast_now({"op": "vehicle", "data": self.latest["vehicle"]})

    def on_speed(self, msg: TwistStamped):
        x = float(msg.twist.linear.x)
        y = float(msg.twist.linear.y)
        z = float(msg.twist.linear.z)
        self.latest["vehicle"]["speed_mps"] = round(math.sqrt(x*x + y*y + z*z), 2)
        self.broadcast_now({"op": "vehicle", "data": self.latest["vehicle"]})

    def on_imu(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = quat_to_euler_deg(q.x, q.y, q.z, q.w)
        self.latest["vehicle"]["roll_deg"] = round(roll, 1)
        self.latest["vehicle"]["pitch_deg"] = round(pitch, 1)
        self.latest["vehicle"]["yaw_deg"] = round(yaw, 1)
        self.broadcast_now({"op": "vehicle", "data": self.latest["vehicle"]})

    def on_gps(self, msg: NavSatFix):
        self.latest["vehicle"]["gps"] = {
            "lat": round(float(msg.latitude), 7),
            "lon": round(float(msg.longitude), 7),
            "alt": round(float(msg.altitude), 2),
        }
        self.broadcast_now({"op": "vehicle", "data": self.latest["vehicle"]})

    def on_rpi_health(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        # Field names must match rpi_health_node.py output exactly.
        # cpu_temp      (not cpu_temp_c)
        # mem_percent   → renamed to mem_pct in latest dict
        # disk_percent  → renamed to disk_pct in latest dict
        self.latest["rpi"]["cpu_pct"]      = data.get("cpu_percent")
        self.latest["rpi"]["cpu_temp_c"]   = data.get("cpu_temp")
        self.latest["rpi"]["mem_pct"]      = data.get("mem_percent")
        self.latest["rpi"]["disk_pct"]     = data.get("disk_percent")
        self.latest["rpi"]["cpu_freq_mhz"] = data.get("cpu_freq_mhz")
        self.latest["rpi"]["throttled"]    = data.get("throttled")
        self.latest["rpi"]["uptime_s"]     = data.get("uptime_s")
        self.broadcast_now({"op": "rpi", "data": self.latest["rpi"]})


async def ws_handler(websocket):
    await NODE.register(websocket)
    try:
        async for message in websocket:
            try:
                req = json.loads(message)
            except Exception:
                continue

            if req.get("op") == "ping":
                await websocket.send(json.dumps({"op": "pong"}))
            elif req.get("op") == "get_snapshot":
                await websocket.send(json.dumps({
                    "op": "snapshot",
                    "data": NODE.latest,
                }))
    finally:
        NODE.unregister(websocket)


async def ros_spin():
    while rclpy.ok():
        rclpy.spin_once(NODE, timeout_sec=0.1)
        await asyncio.sleep(0.01)


async def main_async():
    server = await websockets.serve(ws_handler, WS_HOST, WS_PORT)
    NODE.get_logger().info(f"WebSocket server started on ws://{WS_HOST}:{WS_PORT}")
    try:
        await ros_spin()
    finally:
        server.close()
        await server.wait_closed()


def main():
    global NODE
    rclpy.init()
    NODE = MeshviewTelemetryBridge()
    try:
        asyncio.run(main_async())
    finally:
        NODE.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()