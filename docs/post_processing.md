# Post-Processing Pipeline

**Document:** 4 of 6
**Repo path:** `docs/post_processing.md`
**Last updated:** March 2026
**Project:** DronePi — Autonomous LiDAR Mapping Drone — UPRM Capstone

---

## Table of Contents

1. [Overview](#1-overview)
2. [Trigger Chain](#2-trigger-chain)
3. [Stage Reference](#3-stage-reference)
4. [CLI Flags](#4-cli-flags)
5. [Output File Reference](#5-output-file-reference)
6. [Performance Benchmarks](#6-performance-benchmarks)
7. [Algorithm Decisions](#7-algorithm-decisions)
8. [Environment Requirements](#8-environment-requirements)
9. [Debugging a Failed Run](#9-debugging-a-failed-run)
10. [Extending the Pipeline](#10-extending-the-pipeline)

---

## 1. Overview

After every flight session the post-processing pipeline automatically converts
the raw rosbag recording into a set of PLY mesh files and JSON manifests that
the browser viewer can load immediately.

The pipeline runs entirely on the Raspberry Pi 5 with no cloud dependency.
It is triggered automatically by `drone_watchdog.py` on disarm and takes
approximately 25–60 seconds depending on scan duration and algorithm choices.

All pipeline stages are independent classes in `utils/mesh_tools/`. A single
orchestrator script `utils/postprocess_mesh.py` calls them in sequence. This
architecture means any stage can be replaced, skipped, or tested in isolation
without touching the others.

---

## 2. Trigger Chain

```
Drone disarms
    |
    v
drone_watchdog.py detects disarm
    |
    | calls PostflightMonitor.trigger()
    v
run_postflight.py
    | waits for bag recorder to close MCAP file cleanly
    | finds latest bag in /mnt/ssd/rosbags/
    | calls postprocess_mesh.py --bag <path> --auto
    v
postprocess_mesh.py
    | stage 1: BagReader
    | stage 2: MLSSmoother
    | stage 3: GroundClassifier
    | stage 4: DTMBuilder
    | stage 5: DSMBuilder
    | stage 6: Publisher
    v
/mnt/ssd/maps/scan_YYYYMMDD_HHMMSS/
    | mesh_final.ply
    | combined_cloud.ply
    | mesh_dtm.ply
    | mesh_dsm.ply
    | metadata.json
    v
latest.json updated
    |
    v
meshview.html auto-loads on next 10s poll
```

All output from the pipeline is piped into `journalctl` via `PostflightMonitor`
and prefixed `[POSTFLIGHT]`. Monitor in real time with:

```bash
sudo journalctl -u drone-watchdog -f
```

---

## 3. Stage Reference

### Stage 1 — BagReader (`mesh_tools/bag_reader.py`)

**Input:** MCAP bag directory path
**Output:** NumPy Nx3 float32 point array

Reads `/cloud_registered` messages from the MCAP file using the `rosbags`
library. Does not require a full ROS 2 installation. Parses `metadata.yaml`
for session duration, frame count, and timestamps.

The output is a flat Nx3 array of (x, y, z) coordinates in the Point-LIO map
frame. All subsequent stages operate on this array or subsets of it.

```python
# Class interface
class BagReader:
    def read(self, bag_path: str) -> np.ndarray:
        # Returns Nx3 float32
```

### Stage 2 — MLSSmoother (`mesh_tools/mls_smoother.py`)

**Input:** Nx3 float32
**Output:** Nx3 float32 (denoised)

Applies Moving Least Squares smoothing to reduce IMU vibration noise in the
point cloud before meshing. Uses Open3D's implementation. Falls back to
Statistical Outlier Removal (SOR) if MLS is unavailable.

Skip with `--no-mls` when speed is more important than mesh smoothness, or
when the raw cloud is already clean.

```python
class MLSSmoother:
    def smooth(self, points: np.ndarray) -> np.ndarray:
```

### Stage 3 — GroundClassifier (`mesh_tools/ground_classifier.py`)

**Input:** Nx3 float32
**Output:** ground Nx3 float32, non-ground Nx3 float32

Separates ground points from object points (buildings, vegetation, structures).
This split allows the terrain mesh (DTM) and surface mesh (DSM) to be built
independently, producing better results than meshing everything together.

**Primary method:** SMRF (Simple Morphological Filter) via PDAL. SMRF is a
professional ground classification algorithm that handles slopes and irregular
terrain well.

**Fallback method:** Z-percentile split. If PDAL is unavailable, points below
a configurable Z threshold are classified as ground. This works reliably for
flat open terrain.

```python
class GroundClassifier:
    def classify(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        # Returns (ground_points, nonground_points)
```

### Stage 4 — DTMBuilder (`mesh_tools/dtm_builder.py`)

**Input:** ground Nx3 float32
**Output:** trimesh terrain mesh

Builds a Digital Terrain Model from ground points using Delaunay 2.5D
triangulation. This is a fast, reliable method that treats the point cloud
as a height field and triangulates it without needing surface normals.

Process:
1. Project ground points onto the XY plane
2. Apply grid binning — median Z per cell at the configured resolution
3. Run Delaunay triangulation on the 2D grid
4. Apply long-edge gap filtering to remove stretched triangles over data gaps

The DTM captures the ground surface and terrain shape. It is the fastest stage
and consistently produces watertight geometry.

```python
class DTMBuilder:
    def build(self, ground_points: np.ndarray) -> trimesh.Trimesh:
```

### Stage 5 — DSMBuilder (`mesh_tools/dsm_builder.py`)

**Input:** non-ground Nx3 float32
**Output:** Open3D TriangleMesh

Builds a Digital Surface Model from non-ground points using the Ball Pivoting
Algorithm (BPA). BPA rolls a virtual sphere across the point cloud and creates
triangles wherever the sphere touches three points simultaneously.

BPA radius is estimated automatically from point density if not specified. It
can also be set manually with `--bpa-radius`.

Note: if SLAM drift has inflated the effective point spacing, BPA will select
a larger radius and produce a coarser mesh. This is expected behavior on longer
scans with drift.

```python
class DSMBuilder:
    def build(self, nonground_points: np.ndarray) -> o3d.geometry.TriangleMesh:
```

### Stage 6 — Publisher (`mesh_tools/publisher.py`)

**Input:** combined mesh, point cloud, session metadata
**Output:** PLY files + JSON manifests on disk

Writes all outputs to `/mnt/ssd/maps/scan_YYYYMMDD_HHMMSS/`. Also updates
`/mnt/ssd/maps/latest.json` to point to the new session, which triggers the
browser viewer to auto-load.

```python
class Publisher:
    def publish(self, cloud, mesh, dtm, dsm, metadata: dict) -> str:
        # Returns output directory path
```

---

## 4. CLI Flags

All flags are passed to `postprocess_mesh.py`.

| Flag | Default | Effect |
|---|---|---|
| `--bag PATH` | required | Path to the rosbag directory |
| `--auto` | off | Find and use the latest bag automatically |
| `--no-mls` | off | Skip MLS smoothing stage |
| `--no-mesh` | off | Skip DTM and DSM stages, output cloud only |
| `--use-poisson` | off | Use Poisson reconstruction instead of BPA for DSM |
| `--poisson-depth N` | 8 | Octree depth for Poisson reconstruction |
| `--grid-res N` | 0.10 | DTM grid cell size in metres |
| `--mls-radius N` | 0.05 | MLS search radius in metres |
| `--bpa-radius N` | auto | Fixed BPA radius in metres |
| `--max-bpa-pts N` | 50000 | Hard cap on BPA input point count |
| `--ground-height N` | 0.3 | Manual ground Z threshold for fallback classifier |
| `--maps-dir PATH` | /mnt/ssd/maps | Output directory |
| `--debug` | off | Save intermediate PLY files to a debug/ subfolder |
| `--fast` | off | Aggressive downsampling for quick preview |

### Common usage patterns

```bash
# Standard post-flight run (called automatically by watchdog)
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260319_230358/ --auto

# Debug run — saves intermediate PLY files at each stage
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260319_230358/ --debug

# Fast preview — aggressive downsampling, skips MLS
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260319_230358/ --fast --no-mls

# Cloud only — skip mesh generation
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260319_230358/ --no-mesh

# Poisson reconstruction instead of BPA
python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260319_230358/ --use-poisson
```

---

## 5. Output File Reference

All outputs are written to `/mnt/ssd/maps/scan_YYYYMMDD_HHMMSS/`.

| File | Description | Viewer |
|---|---|---|
| `combined_cloud.ply` | Full cleaned point cloud | Cloud button in meshview.html |
| `mesh_final.ply` | Combined DTM + DSM | Mesh button in meshview.html |
| `mesh_dtm.ply` | Terrain mesh only | Browse button → select file |
| `mesh_dsm.ply` | Surface mesh only | Browse button → select file |
| `metadata.json` | Processing metadata for this session | Used by /api/flights |
| `latest.json` | Points to most recent session | Polled by viewer every 10s |

### metadata.json structure

```json
{
  "session": "scan_20260319_230358",
  "timestamp": "2026-03-19T23:03:58",
  "duration_s": 273,
  "frame_count": 2663,
  "point_count": 3320692,
  "mesh_vertices": 25168,
  "mesh_faces": 50456,
  "watertight": true,
  "processing_time_s": 25.5,
  "algorithms": {
    "smoother": "MLS",
    "classifier": "SMRF",
    "dtm": "Delaunay2.5D",
    "dsm": "BPA"
  }
}
```

### Debug outputs (--debug flag)

When `--debug` is passed, intermediate files are saved to a `debug/` subfolder:

| File | Stage | Contents |
|---|---|---|
| `debug_1_raw.ply` | After BagReader | Raw unprocessed cloud |
| `debug_2_capped.ply` | After point cap | Cloud after max point limit applied |
| `debug_3_mls.ply` | After MLSSmoother | Denoised cloud |
| `debug_4_ground.ply` | After GroundClassifier | Ground points (blue) |
| `debug_4_nonground.ply` | After GroundClassifier | Non-ground points (red) |
| `debug_5_dtm.ply` | After DTMBuilder | Terrain mesh only |
| `debug_6_dsm.ply` | After DSMBuilder | Surface mesh only |

---

## 6. Performance Benchmarks

Reference scan: `scan_20260319_230358` — handheld indoor room scan,
4 minutes 33 seconds, 3,320,692 raw points, 2,663 frames.
Platform: Raspberry Pi 5, 16 GB RAM, 3.0 GHz.

| Stage | Time | Notes |
|---|---|---|
| BagReader | 6.3 s | MCAP parsing |
| MLSSmoother | TBD | Requires Open3D — not yet benchmarked post-install |
| GroundClassifier (Z-pct fallback) | 5.5 s | SMRF time TBD |
| DTMBuilder | 1.4 s | 75,000 triangles |
| DSMBuilder (BPA) | ~12 s | Slower when SLAM drift inflates radius |
| Publisher | 0.8 s | File writes |
| **Total (without MLS)** | **~25 s** | |

Mesh reconstruction verified watertight: 25,168 vertices, 50,456 faces.

---

## 7. Algorithm Decisions

### Why Delaunay 2.5D for terrain (DTM)

Delaunay 2.5D treats the point cloud as a height field projected onto a 2D
plane. It is fast, always produces a closed mesh, and handles ground points
well because ground is roughly planar. It cannot represent overhanging surfaces,
which is acceptable for terrain.

### Why Ball Pivoting for surfaces (DSM)

BPA adapts to local point density and preserves sharp edges on structures. It
is more accurate than Poisson for architectural geometry. Its main weakness is
sensitivity to point spacing — if SLAM drift spreads points unevenly, the
selected radius becomes too large and the mesh becomes coarse. Poisson
reconstruction is available as an alternative via `--use-poisson`.

### Why not Open3D MLS in original build

The original build used `pymeshlab` for ARM64 compatibility because Open3D
had no ARM64 wheel for Python 3.12 at the time. Open3D 0.19.0 is now available
via conda-forge and is the active smoother. `pymeshlab` remains installed as a
fallback.

### Why SMRF via PDAL

SMRF (Simple Morphological Filter) is a well-validated professional algorithm
used in production LiDAR processing tools. It outperforms simple Z-threshold
classification on sloped or irregular terrain. PDAL 3.5.3 is installed via
conda-forge in the `dronepi` environment.

---

## 8. Environment Requirements

The pipeline must run inside the `dronepi` Conda environment:

```bash
conda activate dronepi
```

Required packages:

| Package | Version | Role |
|---|---|---|
| open3d | 0.19.0 | MLS smoothing, BPA mesh, point cloud I/O |
| pdal | 3.5.3 | SMRF ground classification |
| trimesh | latest | DTM mesh, merge, PLY export |
| rosbags | latest | MCAP bag reading |
| scipy | latest | Spatial operations in DTM builder |
| numpy | latest | All array operations |
| pymeshlab | latest | Fallback mesh processing |

Verify environment before running:

```bash
conda activate dronepi
python -c "import open3d; print('Open3D:', open3d.__version__)"
python -c "import pdal; print('PDAL:', pdal.__version__)"
python -c "import trimesh; print('trimesh OK')"
python -c "from rosbags.rosbag2 import Reader; print('rosbags OK')"
```

---

## 9. Debugging a Failed Run

### Pipeline exits silently

Check the watchdog journal — all output is captured there:

```bash
sudo journalctl -u drone-watchdog -f | grep POSTFLIGHT
```

### Stage fails with import error

The Conda environment was not active. The pipeline was called from a process
without `conda activate dronepi`. This can happen if the systemd service does
not source the Conda environment correctly.

Fix: ensure the service `ExecStart` sources the environment:

```bash
ExecStart=/bin/bash -c "source ~/miniforge3/etc/profile.d/conda.sh && \
  conda activate dronepi && python3 run_postflight.py --auto"
```

### BagReader fails — bag not closed cleanly

Symptom: `rosbags` raises a parse error or returns zero points.

Cause: The bag recorder process was killed before the MCAP file was finalized.
`run_postflight.py` waits for the recorder to close, but a SIGKILL bypasses this.

Fix: Check that `metadata.yaml` exists in the bag directory:

```bash
ls /mnt/ssd/rosbags/scan_YYYYMMDD_HHMMSS/
# metadata.yaml must be present — if missing, bag is corrupt
```

### DSMBuilder produces coarse mesh

Cause: SLAM drift inflated the effective point spacing, causing BPA to select
a large radius.

Fix: Pass a smaller fixed radius:

```bash
python3 postprocess_mesh.py --bag <path> --bpa-radius 0.05
```

Or use Poisson reconstruction which is less sensitive to point spacing:

```bash
python3 postprocess_mesh.py --bag <path> --use-poisson
```

### GroundClassifier falls back to Z-percentile

PDAL is installed but the SMRF filter may fail on very sparse or very dense
clouds. The fallback is logged as a warning. Results are acceptable for flat
open terrain. For complex terrain, investigate the PDAL error in the journal.

---

## 10. Extending the Pipeline

To add a new processing stage, follow the established pattern:

1. Create a new class in `utils/mesh_tools/your_module.py`
2. Implement a single public method with typed inputs and outputs
3. Add an `.is_available()` classmethod if the stage depends on optional hardware
4. Import and call the class from `postprocess_mesh.py` at the appropriate position
5. Add a `--no-your-stage` CLI flag to allow skipping
6. Add output to `Publisher` if the stage produces a new file type

Example skeleton:

```python
# utils/mesh_tools/your_module.py

import numpy as np
from .logging_utils import log

class YourModule:

    @classmethod
    def is_available(cls) -> bool:
        try:
            import your_dependency
            return True
        except ImportError:
            return False

    def process(self, points: np.ndarray) -> np.ndarray:
        if not self.is_available():
            log("[YourModule] dependency not available — skipping")
            return points
        log("[YourModule] processing...")
        # your logic here
        return processed_points
```

### Planned future stages

| Module | Purpose | Blocked by |
|---|---|---|
| Texture projection | Project IMX477 images onto mesh surface | Camera reinstall |
| `hailo/hailo_ground.py` | Neural ground classification via Hailo-8 | Camera + hailo/ submodule |
| `storage_monitor.py` | SSD health and voltage check in preflight | Implementation pending |
| Multi-bag merge | Combine sessions from multiple battery charges | Implementation pending |
