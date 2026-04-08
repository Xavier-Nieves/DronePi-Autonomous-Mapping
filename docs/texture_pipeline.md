# Texture Mapping Pipeline

**Document:** 9 of 9 (supplementary)
**Repo path:** `docs/texture_pipeline.md`
**Last updated:** April 2026
**Project:** DronePi — Autonomous LiDAR Mapping Drone — UPRM Capstone
**Status:** Integrated and ready for real-bag validation. The IMX477 camera is installed, the texture stage is wired into `postprocess_mesh.py`, and the remaining work is first-flight validation and final viewer verification.

---

## Table of Contents

1. [Overview](#1-overview)
2. [What Texture Projection Means](#2-what-texture-projection-means)
3. [Pipeline Architecture](#3-pipeline-architecture)
4. [Data Flow](#4-data-flow)
5. [Core Classes](#5-core-classes)
6. [Timestamp Synchronization](#6-timestamp-synchronization)
7. [Projection Math](#7-projection-math)
8. [Occlusion Culling](#8-occlusion-culling)
9. [Integration with Post-Processing Pipeline](#9-integration-with-post-processing-pipeline)
10. [PX4 Camera Trigger Configuration](#10-px4-camera-trigger-configuration)
11. [RViz2 Visualization](#11-rviz2-visualization)
12. [Testing Without Hardware](#12-testing-without-hardware)
13. [Common Errors and Fixes](#13-common-errors-and-fixes)
14. [Remaining Work](#14-remaining-work)

---

## 1. Overview

The texture mapping pipeline adds color to the LiDAR mesh by projecting
camera images onto mesh vertices. The LiDAR provides geometry — where
everything is in 3D space. The IMX477 camera provides color — what
everything looks like. The projection step combines them into a colored mesh
viewable in the browser.

This pipeline runs post-flight, after the LiDAR mesh has been built by the
existing six-stage post-processing pipeline. It is the final step before a
fully textured deliverable is produced.

**Prerequisites before this pipeline can run:**

- IMX477 camera installed and publishing to the configured image topic
- Camera intrinsic calibration verified — see `docs/calibration.md`
- Extrinsic calibration values confirmed in `config/camera_calibration.yaml`
- `texture_tools/` modules present in the runtime environment
- A rosbag containing both camera frames and SLAM poses for replay

---

## 2. What Texture Projection Means

For each vertex in the LiDAR mesh, the projector finds which camera frame
had a clear view of that point and samples the pixel color at the projected
image coordinate. The result is a per-vertex RGB color stored in the PLY file.

Steps per frame:

1. Find the drone pose at the camera frame's timestamp by interpolating
   SLAM pose history
2. Compute the camera world pose from the drone pose and the extrinsic
   transform
3. Transform each mesh vertex into camera space
4. Project each camera-space point to a pixel coordinate using the
   pinhole projection model and intrinsics
5. Reject points behind the camera, outside the image, or occluded by
   other geometry
6. Sample the image at the projected pixel and accumulate the color into
   a per-vertex weighted buffer
7. After all frames, divide accumulated color by accumulated weight to
   get the final per-vertex color

Multi-frame accumulation gives better coverage than single-frame projection
and smooths noise from individual frames.

---

## 3. Pipeline Architecture

```
IMX477 camera
  | /arducam/image_raw (sensor_msgs/Image, ROS-stamped)
  v
ArducamNode
  | publishes timestamped frames at 30 Hz
  v
TextureNode
  | subscribes to /arducam/image_raw
  | subscribes to /aft_mapped_to_init (pose history)
  |
  | for each frame:
  |   PoseInterpolator.get_pose_at(timestamp)
  |   CameraModel.get_camera_world_pose(drone_pose)
  |   TextureProjector.project_frame(image, camera_pose)
  |
  v
TextureProjector
  | accumulates per-vertex color across all frames
  | applies Z-buffer occlusion culling
  v
finalize() → Open3D TriangleMesh with vertex_colors
  v
Publisher → textured_mesh.ply
  v
meshview.html → browser renders colored mesh
```

The `TextureNode` and `TextureProjector` operate in post-flight replay mode:
the mesh is built first by `postprocess_mesh.py`, then a recorded ROS bag
is replayed and the texture node processes the camera frames against the
finished mesh.

---

## 4. Data Flow

| Topic | Type | Rate | Publisher | Subscriber |
|---|---|---|---|---|
| `/arducam/image_raw` | `sensor_msgs/Image` | 30 Hz | ArducamNode | TextureNode |
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | ~250 Hz | Point-LIO | TextureNode |
| `/textured_cloud` | `sensor_msgs/PointCloud2` | on update | TextureNode | RViz2 |
| `/mavros/cam_imu_sync/cam_imu_stamp` | `mavros_msgs/CamIMUStamp` | per trigger | MAVROS | ArducamNode |

The camera timestamp and SLAM pose timestamp must be synchronized to the
same ROS clock. The ArducamNode stamps each image with
`self.get_clock().now()` at capture time. The PoseInterpolator uses this
timestamp to find the exact drone pose when the shutter fired.

---

## 5. Core Classes

### CameraModel (`core/camera_model.py`)

Holds intrinsics and extrinsics. Provides two methods used by the projector.

```python
class CameraModel:
    def __init__(self, intrinsics_path: str, extrinsics_path: str):
        # Loads fx, fy, cx, cy, distortion_coefficients from intrinsics YAML
        # Loads translation and rotation_rpy from extrinsics YAML
        # Builds T_cam_body — the 4x4 rigid transform from drone body to camera

    def get_camera_world_pose(self, drone_world_pose: np.ndarray) -> np.ndarray:
        # drone_world_pose: 4x4 transform of drone body in world frame (from SLAM)
        # Returns: 4x4 transform of camera in world frame
        return drone_world_pose @ self.T_cam_body

    def project_point(self, point_cam: np.ndarray):
        # point_cam: (3,) point in camera frame
        # Returns: (u, v) pixel coordinates, or None if behind camera or outside image
        if point_cam[2] <= 0:
            return None
        u = self.fx * point_cam[0] / point_cam[2] + self.cx
        v = self.fy * point_cam[1] / point_cam[2] + self.cy
        if 0 <= u < self.width and 0 <= v < self.height:
            return int(u), int(v)
        return None
```

The extrinsic transform `T_cam_body` is built from `config/camera_calibration.yaml`:

```python
self.T_cam_body[:3, :3] = Rotation.from_euler(
    'xyz',
    [rpy['roll'], rpy['pitch'], rpy['yaw']],
    degrees=False          # values in radians
).as_matrix()
self.T_cam_body[:3, 3] = translation  # [-0.070, 0.000, 0.030]
```

### TextureProjector (`core/texture_projector.py`)

Manages the per-vertex color accumulation buffers and runs the projection
loop for each incoming camera frame.

```python
class TextureProjector:
    def __init__(self, mesh: o3d.geometry.TriangleMesh, camera: CameraModel):
        self.vertices = np.asarray(mesh.vertices)   # Nx3
        self.color_accum  = np.zeros((N, 3))        # accumulated RGB per vertex
        self.weight_accum = np.zeros(N)             # accumulated weight per vertex
        self.scene = o3d.t.geometry.RaycastingScene()
        # ... adds mesh to raycasting scene for occlusion

    def project_frame(self, image: np.ndarray, camera_world_pose: np.ndarray):
        # Transform all vertices into camera frame
        # Project to pixel coordinates
        # Apply occlusion culling (see section 8)
        # Accumulate color from image into color_accum / weight_accum

    def get_current_mesh(self) -> o3d.geometry.TriangleMesh:
        # Returns mesh with current per-vertex colors
        # Vertices with weight_accum < 0.01 are colored gray (untextured)

    def finalize(self) -> o3d.geometry.TriangleMesh:
        # Call after all frames processed — returns final textured mesh
        return self.get_current_mesh()
```

### ArducamNode (`nodes/arducam_node.py`)

ROS 2 node that captures IMX477 frames and publishes them with ROS clock
timestamps. The timestamp on each image header is the key that links the
frame to a SLAM pose via the PoseInterpolator.

```python
class ArducamNode(Node):
    def capture_frame(self):
        ret, frame = self.cap.read()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        # CRITICAL: use ROS clock — this syncs with SLAM poses
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arducam'
        self.pub.publish(msg)
```

---

## 6. Timestamp Synchronization

Each camera frame must be matched to the SLAM pose that was active when the
shutter fired. Point-LIO publishes poses at ~250 Hz, so there is always a
pose within 4ms of any camera frame. The PoseInterpolator finds the exact
pose by interpolating between the two bracketing poses.

### PoseInterpolator (`core/pose_interpolator.py`)

```python
class PoseInterpolator:
    def __init__(self, max_history: int = 5000):
        self.timestamps = []   # sorted int nanoseconds
        self.poses = []        # corresponding 4x4 numpy arrays

    def add_pose(self, timestamp_ns: int, pose_4x4: np.ndarray):
        # Appends and trims to max_history to avoid memory growth

    def get_pose_at(self, query_ts_ns: int) -> Optional[np.ndarray]:
        # Binary search for bracketing timestamps
        # Linear interpolation of translation
        # SLERP for rotation — smooth interpolation on SO3
        # Allows up to 50ms extrapolation for camera lag
        alpha = (query_ts_ns - t0) / (t1 - t0)
        trans = (1 - alpha) * p0[:3, 3] + alpha * p1[:3, 3]
        rots  = Rotation.from_matrix([p0[:3, :3], p1[:3, :3]])
        rot   = Slerp([0.0, 1.0], rots)(alpha).as_matrix()
```

SLERP (Spherical Linear Interpolation) is used for rotation because simple
linear interpolation of rotation matrices does not produce valid rotations
and introduces artifacts at large angular differences.

### Clock discipline

All nodes must use the same clock source. When replaying a bag, every node
must have `use_sim_time: false` and the bag must be replayed with `--clock`:

```bash
ros2 bag play flight_recording.bag --clock
```

If the camera node uses wall clock while other nodes use ROS clock, or vice
versa, all timestamp lookups will fail and no vertices will be colored.

---

## 7. Projection Math

For a mesh vertex `P_world` in the world frame:

```
1. Transform to camera frame:
   P_cam = T_world_to_cam * P_world
         = inv(camera_world_pose) * P_world

2. Check visibility:
   if P_cam[2] <= 0: point is behind camera, skip

3. Project to image coordinates (pinhole model):
   u = fx * (P_cam[0] / P_cam[2]) + cx
   v = fy * (P_cam[1] / P_cam[2]) + cy

4. Check image bounds:
   if u < 0 or u >= width or v < 0 or v >= height: skip

5. Optionally apply distortion correction (cv2.undistortPoints):
   Corrects barrel distortion from lens before sampling

6. Sample image at (u, v):
   color = image[int(v), int(u)]  # note: row, col order

7. Accumulate:
   color_accum[vertex_idx]  += color * weight
   weight_accum[vertex_idx] += weight
```

The weight is typically 1.0 per frame per visible vertex, but can be
scaled by distance (closer = higher weight) or by angle of incidence
(more perpendicular = higher weight) for better quality.

Final color per vertex:

```
final_color = color_accum / max(weight_accum, 1e-6)
```

---

## 8. Occlusion Culling

Without occlusion culling, vertices that are geometrically behind a wall
but project inside the camera image would incorrectly receive color from
that frame. The projector uses Open3D's ray casting scene to test visibility.

For each frame, for each visible vertex candidate:

```python
# Cast a ray from the camera origin toward the vertex
ray = normalize(P_world - camera_origin)
ray_origin = camera_origin

# Raycast against the full mesh
hit = raycasting_scene.cast_rays(ray)
hit_distance = hit['t_hit']

# The vertex is visible if the ray hits approximately at the vertex distance
vertex_distance = norm(P_world - camera_origin)
tolerance = 0.05   # 5cm

if hit_distance >= vertex_distance - tolerance:
    # Vertex is visible from this camera pose — accumulate color
```

This implemented in `TextureProjector._get_visible_vertices()`. The Z-buffer
logic from `test_mesh_from_bag.py` (1,811 visible, 24 occluded in the bench
test) has already been validated and is ready to be integrated into this class.

---

## 9. Integration with Post-Processing Pipeline

The texture step is already wired into `postprocess_mesh.py` as Stage 6, with publish moved to Stage 7. It is gated by the same camera/data checks used elsewhere in the stack.

Current orchestrator behavior:

```python
# Stage 6 — Texture projection (optional, non-fatal)
if not no_texture and mesh is not None and camera_enabled:
    textured_mesh = texture_stage.run(
        mesh=combined_mesh,
        bag_path=bag_path,
        pose_topic="/aft_mapped_to_init",
        image_topic="/arducam/image_raw",
        calibration_path="config/camera_calibration.yaml",
    )
else:
    log("[6/7] Texture projection skipped — publishing grey mesh")

# Stage 7 — Publish
publisher.publish(...)
```

The stage is skipped automatically if camera data is missing, the mesh was not built, `--no-texture` is passed, or calibration is unavailable. Backward compatibility is preserved for older non-camera bags.

The output file is `textured_mesh.ply` alongside the existing `mesh_final.ply`. The viewer logic now prioritizes `textured_mesh.ply` when present and falls back cleanly to the untextured mesh.

---

## 10. PX4 Camera Trigger Configuration

Event-driven triggering synchronizes the camera shutter to a hardware GPIO
pulse from PX4, providing precise timestamp correlation between images and
SLAM poses. This is more accurate than time-based capture because it
eliminates software scheduling jitter.

### QGroundControl parameters to set

```
TRIG_MODE      = 4    PWM trigger on GPIO pin
TRIG_INTERFACE = 3    MAVLink trigger messages
TRIG_INTERVAL  = 500  Trigger every 500ms (2 Hz) during survey
```

### Verification

After setting parameters and rebooting:

```bash
# Confirm trigger topic is publishing
ros2 topic hz /mavros/cam_imu_sync/cam_imu_stamp
# Expected: ~2 Hz matching TRIG_INTERVAL

# Confirm camera captures on trigger
ros2 topic hz /arducam/image_raw
# Expected: matches trigger rate
```

### Current fallback

Until PX4 trigger is configured, `test_texture_live.py` uses time-based
capture at a fixed interval. This is functional but less accurate due to
software scheduling jitter of 10–50ms. The timestamp synchronization
tolerance in PoseInterpolator (50ms extrapolation margin) was chosen to
accommodate this fallback.

---

## 11. RViz2 Visualization

The textured point cloud can be visualized live during post-flight processing
in RViz2 or Foxglove Studio.

### RViz2 display setup

| Display type | Topic | Settings |
|---|---|---|
| PointCloud2 | `/textured_cloud` | Color transformer: RGB8, Style: Points, Size: 0.02m |
| PointCloud2 | `/cloud_registered` | Color transformer: AxisColor (Z), for reference |
| Odometry | `/slam/odometry` | Shows drone trajectory |
| Image | `/arducam/image_raw` | Live camera feed for alignment check |

Save the layout after setup:

```bash
# In RViz2: File → Save Config As → config/rviz/texture_viz.rviz
ros2 run rviz2 rviz2 -d config/rviz/texture_viz.rviz
```

### Post-flight replay

```bash
# Terminal 1 — start texture node
ros2 launch drone_mapping texture_postprocess.launch.py \
  mesh_path:=/mnt/ssd/maps/scan_YYYYMMDD/mesh_final.ply

# Terminal 2 — replay recorded bag
ros2 bag play /mnt/ssd/rosbags/scan_YYYYMMDD/ --clock

# Terminal 3 — confirm topics flowing
ros2 topic hz /textured_cloud
ros2 topic hz /arducam/image_raw
```

---

## 12. Testing Without Hardware

### Bag replay (preferred)

Once any flight bag containing camera frames exists, texture projection
can be tested offline by replaying the bag:

```bash
ros2 bag play flight_recording.bag --clock
```

The texture node processes the replayed frames exactly as it would during
a live flight.

### Synthetic scene test

A flat ground plane test can verify the projection math and occlusion logic
without any real flight data:

```python
# scripts/test_projection.py
import open3d as o3d
import numpy as np

# Flat ground plane 10m x 10m
mesh = o3d.geometry.TriangleMesh.create_box(width=10, height=10, depth=0.1)
mesh.translate([-5, -5, -0.1])

# Simulated drone pose 5m above, looking down
pose = np.eye(4)
pose[2, 3] = 5.0

# Synthetic checkerboard image
image = np.zeros((480, 640, 3), dtype=np.uint8)
for i in range(0, 480, 40):
    for j in range(0, 640, 40):
        if (i // 40 + j // 40) % 2 == 0:
            image[i:i+40, j:j+40] = [255, 255, 255]

# Load camera model and run projector
from core.camera_model import CameraModel
from core.texture_projector import TextureProjector
# ... instantiate and call project_frame(image, pose)
```

Expected result: checkerboard pattern visible on the ground plane mesh.
If the mesh appears gray, the extrinsic rotation is likely wrong — verify
`rotation_rpy` in `config/camera_calibration.yaml`.

---

## 13. Common Errors and Fixes

### Textured mesh appears entirely gray

All vertices have zero weight accumulation — no frame is projecting onto
the mesh. Causes and fixes:

- Wrong extrinsic rotation: the camera is pointing in the wrong direction
  in the model. Check `rotation_rpy` in `config/camera_calibration.yaml`.
  For a downward-facing camera: `pitch = -1.5708` (negative pi/2).
- Timestamp mismatch: camera and SLAM poses are on different clocks.
  Confirm all nodes have `use_sim_time: false` and bag is replayed with
  `--clock`.
- No poses in the interpolator when frames arrive: Point-LIO must be
  publishing before camera frames are processed. Check topic Hz for
  `/slam/odometry`.

### Texture edges are shifted from geometry edges

The extrinsic translation is incorrect. Measure the physical offset again
and update `translation` in `config/camera_calibration.yaml`. A consistent
shift in one axis indicates a single axis error.

### Texture appears rotated relative to geometry

The extrinsic rotation is incorrect. A fan-shaped misalignment (edges fan
from a central point) indicates a yaw error. A top-bottom flip indicates a
pitch error. Adjust `rotation_rpy` in small increments and reprocess.

### Camera timestamps ahead of SLAM timestamps

```bash
ros2 param set /arducam_node use_sim_time false
ros2 param set /texture_node use_sim_time false
# When replaying bags:
ros2 bag play flight.bag --clock
```

### Texture node receives images but no poses

```bash
ros2 topic hz /slam/odometry
# If zero or absent: Point-LIO is not running or LiDAR is not connected
ros2 topic hz /aft_mapped_to_init
# This is the actual Point-LIO topic name in this build
```

Note: the topic name in this build is `/aft_mapped_to_init`, not
`/slam/odometry`. Confirm the TextureNode subscription matches.

### cv_bridge import error

```bash
sudo apt install ros-jazzy-cv-bridge python3-cv-bridge
```

### No module named open3d

Open3D must be installed in the active environment:

```bash
conda activate dronepi
python -c "import open3d; print(open3d.__version__)"
# Expected: 0.19.0
```

Do not install via `pip install open3d --break-system-packages` on the Pi —
use the conda-forge version which has a confirmed ARM64 wheel.

---

## 14. Remaining Work

The texture pipeline is no longer blocked by camera installation. The remaining work is execution validation, not architecture.

| Task | Blocked by | Notes |
|---|---|---|
| Real-bag Stage 6 run | Suitable camera+SLAM rosbag | First full end-to-end textured output on a real flight bag |
| Intrinsic calibration verification | Camera online | Confirm current focus setting still matches the stored calibration |
| Timestamp validation | First textured bag | Verify each used image has a matching SLAM pose within the replay tolerance |
| Extrinsic refinement, if needed | First textured bag | Only if color alignment shows consistent offset or rotation error |
| Viewer verification on Pi | First textured output | Confirm `meshview.html` prefers `textured_mesh.ply` and renders vertex colors correctly |
| Full survey flight with texture | End-to-end validation complete | Capstone demo deliverable |
