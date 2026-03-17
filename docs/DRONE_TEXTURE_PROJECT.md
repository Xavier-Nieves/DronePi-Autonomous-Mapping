# Autonomous Drone Texture Mapping — Project Guide
> ROS 2 Jazzy · Raspberry Pi 5 · Pixhawk/ArduPilot · Unitree LiDAR · ArduCam · Insta360 X3

---

## Table of Contents
1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Directory Structure](#3-directory-structure)
4. [Dependencies & Installation](#4-dependencies--installation)
5. [Hardware Extrinsic Calibration](#5-hardware-extrinsic-calibration)
6. [Camera Intrinsic Calibration](#6-camera-intrinsic-calibration)
7. [Node Reference](#7-node-reference)
8. [Texture Projection Pipeline](#8-texture-projection-pipeline)
9. [Timestamp Synchronization](#9-timestamp-synchronization)
10. [RViz2 Visualization](#10-rviz2-visualization)
11. [Launch Files](#11-launch-files)
12. [Testing Without Hardware](#12-testing-without-hardware)
13. [VS Code Setup](#13-vs-code-setup)
14. [Common Errors & Fixes](#14-common-errors--fixes)

---

## 1. Project Overview

### Goal
Autonomous drone that:
1. Takes off, follows a waypoint path, lands — fully automated via MAVROS
2. Builds a 3D LiDAR point cloud in real time using Point-LIO (SLAM)
3. Captures ArduCam frames timestamped against ROS 2 clock
4. Post-flight: projects ArduCam images onto the LiDAR mesh → colored/textured point cloud
5. Optionally sends to WebODM (OpenDroneMap) for photogrammetry as a backup deliverable

### What "Texture Projection" Means Here
- The LiDAR provides **geometry** (where things are in 3D space)
- The camera provides **color** (what things look like)
- Projection = for each 3D mesh vertex, find which camera frame captured it and sample that pixel's color
- Result = colored mesh viewable in RViz2, Open3D, or Meshlab

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Raspberry Pi 5                         │
│                                                             │
│  ┌─────────────┐   ┌──────────────┐   ┌─────────────────┐  │
│  │ MAVROS Node │   │ Point-LIO    │   │ ArduCam Node    │  │
│  │             │   │ (LiDAR SLAM) │   │                 │  │
│  │ /mavros/... │   │ /slam/odom   │   │ /arducam/image  │  │
│  │ /mavros/cmd │   │ /lidar/cloud │   │ (timestamped)   │  │
│  └──────┬──────┘   └──────┬───────┘   └────────┬────────┘  │
│         │                 │                    │            │
│         └─────────────────┴────────────────────┘           │
│                           │                                 │
│                  ┌────────▼────────┐                        │
│                  │ Texture Node    │                        │
│                  │                 │                        │
│                  │ - pose interp   │                        │
│                  │ - projection    │                        │
│                  │ - occlusion     │                        │
│                  │ /textured_cloud │                        │
│                  └────────┬────────┘                        │
│                           │                                 │
│                  ┌────────▼────────┐                        │
│                  │ RViz2 / Open3D  │                        │
│                  └─────────────────┘                        │
└─────────────────────────────────────────────────────────────┘
         │                    │
    Pixhawk FC           Unitree LiDAR
    (UART TELEM2)        (USB / Ethernet)
```

### Data Flow Summary
```
Pixhawk ──UART──► MAVROS ──► /mavros/state, /mavros/local_position/odom
Unitree LiDAR ──► Point-LIO ──► /slam/odometry + /lidar/pointcloud
ArduCam ──USB──► arducam_node ──► /arducam/image_raw  (ROS-stamped)
                                         │
                     ┌───────────────────┘
                     ▼
             texture_projector ──► /textured_cloud ──► RViz2
```

---

## 3. Directory Structure

```
~/drone_ws/
└── src/
    └── drone_mapping/
        ├── package.xml
        ├── setup.py
        ├── config/
        │   ├── camera_intrinsics.yaml      # ArduCam calibration
        │   ├── camera_extrinsics.yaml      # Physical mount offsets
        │   └── rviz/
        │       └── texture_viz.rviz        # RViz2 config
        ├── launch/
        │   ├── full_mission.launch.py      # Everything at once
        │   ├── slam_only.launch.py         # LiDAR + Point-LIO only
        │   └── texture_postprocess.launch.py
        ├── drone_mapping/
        │   ├── __init__.py
        │   ├── nodes/
        │   │   ├── flight_node.py          # MAVROS offboard control
        │   │   ├── arducam_node.py         # Camera capture + timestamping
        │   │   ├── texture_node.py         # Main texture projection node
        │   │   └── collision_avoidance_node.py
        │   ├── core/
        │   │   ├── texture_projector.py    # Projection math (no ROS deps)
        │   │   ├── pose_interpolator.py    # Timestamp matching
        │   │   ├── mesh_utils.py           # Open3D helpers
        │   │   └── camera_model.py         # Intrinsics + extrinsics
        │   ├── scripts/
        │   │   ├── calibrate_camera.py     # OpenCV checkerboard calibration
        │   │   ├── extract_trajectory.py   # Pull poses from ROS bag
        │   │   ├── postprocess.py          # Offline texture pipeline runner
        │   │   └── upload_to_webodm.py     # WebODM REST API upload
        │   └── utils/
        │       ├── ros_conversions.py      # msg ↔ numpy helpers
        │       └── visualization.py        # RViz2 marker publishers
        └── resource/
            └── drone_mapping
```

---

## 4. Dependencies & Installation

### ROS 2 Packages
```bash
sudo apt install -y \
  ros-jazzy-mavros \
  ros-jazzy-mavros-extras \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-rosbag2 \
  ros-jazzy-rosbag2-py

# MAVROS geographic datasets (required)
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

### Python Libraries
```bash
pip install --break-system-packages \
  open3d \
  opencv-python \
  numpy \
  scipy \
  pyyaml \
  transforms3d \
  requests \
  tqdm

# For rosbag reading in scripts
pip install --break-system-packages rosbags
```

### Point-LIO (build from source)
```bash
cd ~/drone_ws/src
git clone https://github.com/hku-mars/Point-LIO.git
cd ~/drone_ws
colcon build --packages-select point_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 5. Hardware Extrinsic Calibration

### What to Measure
Physically measure from the **drone's IMU center** to the **camera lens center** with a ruler.

```
Drone body frame (ROS convention):
  X = forward (nose direction)
  Y = left
  Z = up
```

### config/camera_extrinsics.yaml
```yaml
# Physical offset of ArduCam lens from drone IMU center
# Units: meters
arducam:
  translation:
    x: 0.08    # 8cm forward of IMU
    y: 0.00    # centered left-right
    z: -0.04   # 4cm below IMU

  # Rotation: how camera frame relates to drone body frame
  # mount: "downward" = camera pointing straight down
  # mount: "forward"  = camera pointing forward
  # mount: "forward_tilted_30" = forward but 30deg down
  mount: "downward"

  # If custom, specify roll/pitch/yaw in degrees
  # Applied as: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  rotation_rpy_deg:
    roll: 0.0
    pitch: -90.0   # -90 = pointing down
    yaw: 0.0
```

### core/camera_model.py
```python
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

class CameraModel:
    def __init__(self, intrinsics_path: str, extrinsics_path: str):
        with open(intrinsics_path) as f:
            intr = yaml.safe_load(f)
        with open(extrinsics_path) as f:
            extr = yaml.safe_load(f)

        # Intrinsics
        self.fx = intr['fx']
        self.fy = intr['fy']
        self.cx = intr['cx']
        self.cy = intr['cy']
        self.width = intr['width']
        self.height = intr['height']
        self.dist_coeffs = np.array(intr['distortion_coefficients'])

        # Extrinsics — camera in drone body frame
        t = extr['arducam']['translation']
        rpy = extr['arducam']['rotation_rpy_deg']
        self.T_cam_body = np.eye(4)
        self.T_cam_body[:3, :3] = Rotation.from_euler(
            'xyz',
            [rpy['roll'], rpy['pitch'], rpy['yaw']],
            degrees=True
        ).as_matrix()
        self.T_cam_body[:3, 3] = [t['x'], t['y'], t['z']]

    def get_camera_world_pose(self, drone_world_pose: np.ndarray) -> np.ndarray:
        """
        drone_world_pose: 4x4 transform of drone body in world frame (from SLAM)
        Returns: 4x4 transform of camera in world frame
        """
        return drone_world_pose @ self.T_cam_body

    def project_point(self, point_cam: np.ndarray):
        """
        point_cam: (3,) point in camera frame
        Returns: (u, v) pixel coords or None if behind camera
        """
        if point_cam[2] <= 0:
            return None
        u = self.fx * point_cam[0] / point_cam[2] + self.cx
        v = self.fy * point_cam[1] / point_cam[2] + self.cy
        if 0 <= u < self.width and 0 <= v < self.height:
            return int(u), int(v)
        return None
```

---

## 6. Camera Intrinsic Calibration

### scripts/calibrate_camera.py
Run this **once** with a printed checkerboard before your first flight.

```python
"""
Camera Intrinsic Calibration Script
Usage: python3 calibrate_camera.py --device 0 --rows 6 --cols 9 --square_size 0.025
  --device: camera device index (0 for first USB camera)
  --rows/cols: inner corners of checkerboard
  --square_size: size of each square in meters (e.g. 0.025 = 2.5cm)
"""
import cv2
import numpy as np
import yaml
import argparse

def calibrate(device, rows, cols, square_size, n_frames=30):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size

    obj_points = []
    img_points = []

    cap = cv2.VideoCapture(device)
    print(f"Collecting {n_frames} frames. Press SPACE to capture, Q to quit early.")
    captured = 0

    while captured < n_frames:
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, (cols, rows), corners, found)
            cv2.putText(display, f"FOUND — press SPACE ({captured}/{n_frames})",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display, "No board detected",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' ') and found:
            corners_refined = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            obj_points.append(objp)
            img_points.append(corners_refined)
            captured += 1
            print(f"  Captured frame {captured}/{n_frames}")
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if captured < 10:
        print("Not enough frames. Need at least 10.")
        return

    print("Running calibration...")
    h, w = gray.shape
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), None, None)

    print(f"RMS reprojection error: {ret:.4f} px  (good if < 1.0)")

    output = {
        'fx': float(K[0, 0]),
        'fy': float(K[1, 1]),
        'cx': float(K[0, 2]),
        'cy': float(K[1, 2]),
        'width': w,
        'height': h,
        'distortion_coefficients': dist.flatten().tolist(),
        'rms_error': float(ret)
    }

    with open('config/camera_intrinsics.yaml', 'w') as f:
        yaml.dump(output, f, default_flow_style=False)

    print("Saved to config/camera_intrinsics.yaml")
    print(f"  fx={output['fx']:.1f}  fy={output['fy']:.1f}")
    print(f"  cx={output['cx']:.1f}  cy={output['cy']:.1f}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--rows', type=int, default=6)
    parser.add_argument('--cols', type=int, default=9)
    parser.add_argument('--square_size', type=float, default=0.025)
    args = parser.parse_args()
    calibrate(args.device, args.rows, args.cols, args.square_size)
```

---

## 7. Node Reference

### nodes/arducam_node.py
```python
"""
ArduCam Capture Node
Publishes: /arducam/image_raw (sensor_msgs/Image)
           /arducam/frame_timestamps (std_msgs/Float64MultiArray) — for bag sync
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ArducamNode(Node):
    def __init__(self):
        super().__init__('arducam_node')
        self.pub = self.create_publisher(Image, '/arducam/image_raw', 10)
        self.bridge = CvBridge()

        # Open camera — change index if needed (ls /dev/video*)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.timer = self.create_timer(1.0/30.0, self.capture_frame)
        self.get_logger().info("ArduCam node started")

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # CRITICAL: use ROS clock for timestamp — this syncs with SLAM poses
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arducam'
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArducamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### nodes/texture_node.py
```python
"""
Texture Projection Node
Subscribes: /slam/odometry (nav_msgs/Odometry)
            /arducam/image_raw (sensor_msgs/Image)
Publishes:  /textured_cloud (sensor_msgs/PointCloud2)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d

from drone_mapping.core.texture_projector import TextureProjector
from drone_mapping.core.pose_interpolator import PoseInterpolator
from drone_mapping.core.camera_model import CameraModel
from drone_mapping.utils.ros_conversions import (
    odometry_to_matrix, mesh_to_pointcloud2
)

class TextureNode(Node):
    def __init__(self):
        super().__init__('texture_node')
        self.bridge = CvBridge()

        # Load camera model from config
        self.camera = CameraModel(
            'config/camera_intrinsics.yaml',
            'config/camera_extrinsics.yaml'
        )

        # Load mesh (built post-flight from Point-LIO output)
        self.declare_parameter('mesh_path', 'output/slam_mesh.ply')
        mesh_path = self.get_parameter('mesh_path').value
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        self.get_logger().info(f"Loaded mesh: {len(mesh.vertices)} vertices")

        self.projector = TextureProjector(mesh, self.camera)
        self.interpolator = PoseInterpolator()
        self.frame_count = 0

        # Subscriptions
        self.create_subscription(Odometry, '/slam/odometry', self.pose_cb, 50)
        self.create_subscription(Image, '/arducam/image_raw', self.image_cb, 10)

        # Publisher
        self.cloud_pub = self.create_publisher(PointCloud2, '/textured_cloud', 10)

        self.get_logger().info("Texture node ready — waiting for data")

    def pose_cb(self, msg: Odometry):
        ts_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        pose = odometry_to_matrix(msg)
        self.interpolator.add_pose(ts_ns, pose)

    def image_cb(self, msg: Image):
        ts_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # Need at least some poses before we can project
        if len(self.interpolator.poses) < 2:
            return

        pose = self.interpolator.get_pose_at(ts_ns)
        if pose is None:
            return

        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.projector.project_frame(image, pose)
        self.frame_count += 1

        # Publish colored cloud to RViz2 every 5 frames
        if self.frame_count % 5 == 0:
            mesh = self.projector.get_current_mesh()
            pc2_msg = mesh_to_pointcloud2(mesh, msg.header.stamp, 'map')
            self.cloud_pub.publish(pc2_msg)
            self.get_logger().info(
                f"Published textured cloud after {self.frame_count} frames"
            )

def main(args=None):
    rclpy.init(args=args)
    node = TextureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## 8. Texture Projection Pipeline

### core/texture_projector.py
```python
import numpy as np
import open3d as o3d
from drone_mapping.core.camera_model import CameraModel

class TextureProjector:
    def __init__(self, mesh: o3d.geometry.TriangleMesh, camera: CameraModel):
        self.mesh = mesh
        self.camera = camera
        self.mesh.compute_vertex_normals()

        n = len(self.mesh.vertices)
        self.color_accum = np.zeros((n, 3), dtype=np.float64)
        self.weight_accum = np.zeros(n, dtype=np.float64)

        # Build raycasting scene for occlusion testing
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        self.scene = o3d.t.geometry.RaycastingScene()
        self.scene.add_triangles(mesh_t)

    def project_frame(self, image_bgr: np.ndarray, drone_world_pose: np.ndarray):
        """
        image_bgr: HxWx3 numpy array from ArduCam
        drone_world_pose: 4x4 drone body pose in world frame (from SLAM)
        """
        # Get camera world pose using extrinsic calibration
        T_cam_world = self.camera.get_camera_world_pose(drone_world_pose)
        cam_pos = T_cam_world[:3, 3]
        R_cam = T_cam_world[:3, :3]

        vertices = np.asarray(self.mesh.vertices)  # (N, 3)

        # Transform all vertices into camera coordinate frame
        verts_centered = vertices - cam_pos
        verts_cam = (R_cam.T @ verts_centered.T).T  # (N, 3)

        # Filter: only vertices in front of camera
        in_front = verts_cam[:, 2] > 0.1

        # Project to pixel coordinates
        Z = np.where(in_front, verts_cam[:, 2], 1.0)  # avoid division by zero
        u = (self.camera.fx * verts_cam[:, 0] / Z + self.camera.cx)
        v = (self.camera.fy * verts_cam[:, 1] / Z + self.camera.cy)

        in_frame = (
            (u >= 0) & (u < self.camera.width) &
            (v >= 0) & (v < self.camera.height)
        )
        valid = in_front & in_frame
        valid_idx = np.where(valid)[0]

        if len(valid_idx) == 0:
            return

        # Occlusion check
        visible_idx = self._check_occlusion(cam_pos, vertices[valid_idx], valid_idx)

        if len(visible_idx) == 0:
            return

        # Sample pixel colors
        u_px = u[visible_idx].astype(int)
        v_px = v[visible_idx].astype(int)
        colors_bgr = image_bgr[v_px, u_px]
        colors_rgb = colors_bgr[:, ::-1] / 255.0  # BGR→RGB, normalize

        # Weight by viewing angle (prefer vertices directly in view)
        verts_cam_valid = verts_cam[visible_idx]
        dists = np.linalg.norm(verts_cam_valid, axis=1, keepdims=True)
        dists = np.maximum(dists, 1e-6)
        forward = verts_cam_valid / dists
        weight = np.clip(forward[:, 2], 0, 1)  # Z component = facing camera

        # Accumulate
        self.color_accum[visible_idx] += colors_rgb * weight[:, np.newaxis]
        self.weight_accum[visible_idx] += weight

    def _check_occlusion(self, cam_pos, target_verts, indices):
        directions = target_verts - cam_pos
        distances = np.linalg.norm(directions, axis=1, keepdims=True)
        distances = np.maximum(distances, 1e-6)
        directions_norm = directions / distances

        rays = np.hstack([
            np.tile(cam_pos, (len(target_verts), 1)).astype(np.float32),
            directions_norm.astype(np.float32)
        ])
        rays_t = o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32)
        hits = self.scene.cast_rays(rays_t)
        hit_dist = hits['t_hit'].numpy()

        tolerance = 0.05  # 5cm
        visible_mask = hit_dist >= (distances.squeeze() - tolerance)
        return indices[visible_mask]

    def get_current_mesh(self) -> o3d.geometry.TriangleMesh:
        """Returns mesh with current best-guess colors (safe to call mid-flight)"""
        safe_w = np.maximum(self.weight_accum, 1e-6)
        colors = self.color_accum / safe_w[:, np.newaxis]
        untextured = self.weight_accum < 0.01
        colors[untextured] = [0.5, 0.5, 0.5]  # gray for untextured faces
        result = o3d.geometry.TriangleMesh(self.mesh)
        result.vertex_colors = o3d.utility.Vector3dVector(np.clip(colors, 0, 1))
        return result

    def finalize(self) -> o3d.geometry.TriangleMesh:
        """Call after all frames processed — returns final textured mesh"""
        return self.get_current_mesh()
```

---

## 9. Timestamp Synchronization

### core/pose_interpolator.py
```python
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from bisect import bisect_left
from typing import Optional

class PoseInterpolator:
    """
    Stores SLAM poses indexed by ROS timestamp (nanoseconds).
    Interpolates between poses for any queried timestamp.
    """
    def __init__(self, max_history: int = 5000):
        self.timestamps = []   # sorted list of int nanoseconds
        self.poses = []        # corresponding 4x4 numpy arrays
        self.max_history = max_history

    def add_pose(self, timestamp_ns: int, pose_4x4: np.ndarray):
        self.timestamps.append(timestamp_ns)
        self.poses.append(pose_4x4)
        # Trim old history to avoid memory growth during long flights
        if len(self.timestamps) > self.max_history:
            self.timestamps.pop(0)
            self.poses.pop(0)

    def get_pose_at(self, query_ts_ns: int) -> Optional[np.ndarray]:
        if len(self.timestamps) < 2:
            return None

        # Check if query is within our time range
        if query_ts_ns < self.timestamps[0] or query_ts_ns > self.timestamps[-1]:
            # Allow small extrapolation (50ms) for camera lag
            margin_ns = 50_000_000  # 50ms
            if query_ts_ns < self.timestamps[0] - margin_ns:
                return None
            if query_ts_ns > self.timestamps[-1] + margin_ns:
                return None

        idx = bisect_left(self.timestamps, query_ts_ns)
        idx = max(1, min(idx, len(self.timestamps) - 1))

        t0 = self.timestamps[idx - 1]
        t1 = self.timestamps[idx]
        p0 = self.poses[idx - 1]
        p1 = self.poses[idx]

        if t1 == t0:
            return p0

        alpha = (query_ts_ns - t0) / (t1 - t0)
        alpha = np.clip(alpha, 0.0, 1.0)

        # Interpolate translation linearly
        trans = (1.0 - alpha) * p0[:3, 3] + alpha * p1[:3, 3]

        # SLERP for rotation (smooth interpolation on SO3)
        rots = Rotation.from_matrix([p0[:3, :3], p1[:3, :3]])
        slerp = Slerp([0.0, 1.0], rots)
        rot = slerp(alpha).as_matrix()

        pose = np.eye(4)
        pose[:3, :3] = rot
        pose[:3, 3] = trans
        return pose
```

### utils/ros_conversions.py
```python
import numpy as np
import struct
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import open3d as o3d

def odometry_to_matrix(msg: Odometry) -> np.ndarray:
    """Convert ROS Odometry message to 4x4 numpy transform matrix"""
    from scipy.spatial.transform import Rotation
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = [p.x, p.y, p.z]
    return T

def mesh_to_pointcloud2(mesh: o3d.geometry.TriangleMesh,
                        stamp, frame_id: str) -> PointCloud2:
    """Convert Open3D colored mesh to ROS PointCloud2 for RViz2"""
    verts = np.asarray(mesh.vertices)
    colors = np.asarray(mesh.vertex_colors)

    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 16
    msg.height = 1
    msg.width = len(verts)
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True

    cloud_data = []
    for i in range(len(verts)):
        x, y, z = verts[i]
        r, g, b = (np.clip(colors[i], 0, 1) * 255).astype(np.uint8)
        rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
        rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
        cloud_data.append(struct.pack('ffff', float(x), float(y), float(z), rgb_float))

    msg.data = b''.join(cloud_data)
    return msg
```

---

## 10. RViz2 Visualization

### In RViz2, add these displays:

| Display Type | Topic | Purpose |
|---|---|---|
| PointCloud2 | `/textured_cloud` | Colored/textured mesh vertices |
| PointCloud2 | `/lidar/pointcloud` | Raw LiDAR scan |
| Odometry | `/slam/odometry` | Drone trajectory |
| TF | — | Coordinate frames |
| Image | `/arducam/image_raw` | Live camera feed |
| Marker | `/coverage_map` | Which faces are textured (optional) |

### RViz2 Settings for Colored Cloud
- **Color Transformer:** `RGB8`
- **Style:** `Points`
- **Size:** `0.02` meters
- **Queue Size:** `5`

### config/rviz/texture_viz.rviz
Save your RViz2 layout after setting it up so it loads automatically:
```bash
# In RViz2: File → Save Config As → config/rviz/texture_viz.rviz
# Then launch with:
ros2 run rviz2 rviz2 -d config/rviz/texture_viz.rviz
```

---

## 11. Launch Files

### launch/full_mission.launch.py
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mesh_path', default_value='output/slam_mesh.ply'),

        # MAVROS — flight control bridge
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[{
                'fcu_url': '/dev/ttyAMA0:921600',  # UART TELEM2
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
            }]
        ),

        # ArduCam capture
        Node(
            package='drone_mapping',
            executable='arducam_node',
            name='arducam_node',
        ),

        # Texture projection (post-flight mode)
        Node(
            package='drone_mapping',
            executable='texture_node',
            name='texture_node',
            parameters=[{
                'mesh_path': LaunchConfiguration('mesh_path')
            }]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'config/rviz/texture_viz.rviz']
        ),
    ])
```

### launch/texture_postprocess.launch.py
```python
"""For running texture projection on a recorded ROS bag after flight"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_mapping',
            executable='texture_node',
            name='texture_node',
            parameters=[{'mesh_path': 'output/slam_mesh.ply'}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'config/rviz/texture_viz.rviz']
        ),
    ])

# Then replay bag in separate terminal:
# ros2 bag play flight_recording.bag --clock
```

---

## 12. Testing Without Hardware

### Replay a ROS bag (most useful)
```bash
# Terminal 1 — start texture node
ros2 launch drone_mapping texture_postprocess.launch.py

# Terminal 2 — replay recorded flight
ros2 bag play ~/bags/flight_2025_04_15.bag --clock

# Terminal 3 — watch topics
ros2 topic echo /textured_cloud --no-arr  # confirm publishing
ros2 topic hz /arducam/image_raw          # confirm frame rate
```

### Simulate with a synthetic scene
```python
# scripts/test_projection.py — no drone needed
import open3d as o3d
import numpy as np

# Create flat ground plane mesh
mesh = o3d.geometry.TriangleMesh.create_box(width=10, height=10, depth=0.1)
mesh.translate([-5, -5, -0.1])

# Simulate drone pose 5m above origin looking down
pose = np.eye(4)
pose[2, 3] = 5.0  # 5m up

# Synthetic "camera image" — checkerboard pattern
image = np.zeros((480, 640, 3), dtype=np.uint8)
for i in range(0, 480, 40):
    for j in range(0, 640, 40):
        if (i//40 + j//40) % 2 == 0:
            image[i:i+40, j:j+40] = [255, 255, 255]

# Run projection and visualize
from drone_mapping.core.camera_model import CameraModel
from drone_mapping.core.texture_projector import TextureProjector

# ... load camera model and run projector
o3d.visualization.draw_geometries([mesh])
```

---

## 13. VS Code Setup

### Recommended Extensions
```json
// .vscode/extensions.json
{
  "recommendations": [
    "ms-python.python",
    "ms-python.pylint",
    "ms-python.black-formatter",
    "ms-vscode-remote.remote-ssh",
    "twxs.cmake",
    "ms-iot.vscode-ros",
    "ms-vscode.cpptools"
  ]
}
```

### Python Path for ROS 2 (critical for autocomplete)
```json
// .vscode/settings.json
{
  "python.defaultInterpreterPath": "/usr/bin/python3",
  "python.analysis.extraPaths": [
    "/opt/ros/jazzy/lib/python3.12/site-packages",
    "/opt/ros/jazzy/local/lib/python3.12/dist-packages",
    "${workspaceFolder}/install/drone_mapping/lib/python3.12/site-packages"
  ],
  "python.linting.enabled": true,
  "python.formatting.provider": "black",
  "files.associations": {
    "*.launch.py": "python"
  },
  "cmake.configureOnOpen": false
}
```

### Build & Run Tasks
```json
// .vscode/tasks.json
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "Build workspace",
      "type": "shell",
      "command": "cd ~/drone_ws && colcon build --symlink-install",
      "group": "build",
      "presentation": { "panel": "dedicated" }
    },
    {
      "label": "Source ROS",
      "type": "shell",
      "command": "source /opt/ros/jazzy/setup.bash && source ~/drone_ws/install/setup.bash",
      "group": "none"
    },
    {
      "label": "Run texture postprocess",
      "type": "shell",
      "command": "source ~/drone_ws/install/setup.bash && ros2 launch drone_mapping texture_postprocess.launch.py",
      "group": "test",
      "dependsOn": "Build workspace"
    },
    {
      "label": "Calibrate camera",
      "type": "shell",
      "command": "cd ~/drone_ws/src/drone_mapping && python3 drone_mapping/scripts/calibrate_camera.py --device 0",
      "group": "test"
    }
  ]
}
```

### Remote SSH Config (develop from laptop)
```json
// In VS Code: Remote SSH → Add New SSH Host
// ssh xavier@<pi-ip-address>
// Then open folder: ~/drone_ws/src/drone_mapping
```

### .vscode/launch.json (debug individual nodes)
```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Debug texture_node",
      "type": "python",
      "request": "launch",
      "program": "${workspaceFolder}/drone_mapping/nodes/texture_node.py",
      "env": {
        "PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages:${env:PYTHONPATH}",
        "ROS_DOMAIN_ID": "0"
      },
      "console": "integratedTerminal"
    },
    {
      "name": "Debug calibrate_camera",
      "type": "python",
      "request": "launch",
      "program": "${workspaceFolder}/drone_mapping/scripts/calibrate_camera.py",
      "args": ["--device", "0", "--rows", "6", "--cols", "9"],
      "console": "integratedTerminal"
    }
  ]
}
```

---

## 14. Common Errors & Fixes

### `No module named 'open3d'`
```bash
pip install open3d --break-system-packages
# If that fails on Pi (ARM):
pip install open3d-python --break-system-packages
# Or build from source (slow but guaranteed):
# https://www.open3d.org/docs/release/compilation.html
```

### `cv_bridge` import error in Python
```bash
sudo apt install ros-jazzy-cv-bridge python3-cv-bridge
```

### Texture node receives images but no poses
```bash
# Check that /slam/odometry is being published
ros2 topic hz /slam/odometry
# Should be ~100Hz from Point-LIO
# If empty, Point-LIO may not be running or LiDAR not connected
```

### Camera timestamps ahead of SLAM timestamps
```bash
# Force ROS to use system clock (not simulation clock)
ros2 param set /arducam_node use_sim_time false
ros2 param set /texture_node use_sim_time false
# When replaying bags, use --clock flag:
ros2 bag play flight.bag --clock
```

### Textured cloud appears gray (no color)
- Check `weight_accum` — if all zeros, no frames are projecting
- Verify camera extrinsic rotation: a wrong mount angle means all vertices fall outside the image
- Add debug print in `project_frame`: how many `valid_idx` are non-zero per frame?

### Point-LIO crashes on startup
```bash
# Verify LiDAR topic name matches Point-LIO config
ros2 topic list | grep lidar
# Edit point_lio/config/avia.yaml → set lid_topic to your actual topic name
```

### MAVROS not connecting to Pixhawk
```bash
# Verify UART device and baud rate
ls /dev/ttyAMA*   # should show ttyAMA0
# Check TELEM2 baud rate matches (921600 recommended)
# In Pixhawk: SER_TEL2_BAUD = 921600
```

---

## Quick Reference — Key Topics

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/slam/odometry` | `nav_msgs/Odometry` | Point-LIO | texture_node |
| `/lidar/pointcloud` | `sensor_msgs/PointCloud2` | Unitree driver | Point-LIO |
| `/arducam/image_raw` | `sensor_msgs/Image` | arducam_node | texture_node |
| `/textured_cloud` | `sensor_msgs/PointCloud2` | texture_node | RViz2 |
| `/mavros/state` | `mavros_msgs/State` | MAVROS | flight_node |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | flight_node | MAVROS |

---

*Last updated for: ROS 2 Jazzy · Ubuntu 24.04 · Raspberry Pi 5 · ArduPilot MAVROS*
