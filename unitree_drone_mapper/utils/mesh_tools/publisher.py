"""
publisher.py — Write output files and update browser manifests.

Handles all file I/O at the end of the pipeline:
  1. Save raw point cloud PLY  (_cloud.ply)
  2. Save merged mesh PLY      (_mesh.ply)
  3. Copy both to /mnt/ssd/maps/ for HTTP serving (skipped if same dir)
  4. Write metadata.json to bag folder
  5. Write latest.json to maps dir  (browser polls this every 10s)
  6. Call flight_logger.log_flight() to append to flight_history.log

Cross-platform note:
    On Windows (or when maps_dir == bag_path), the copy step is skipped
    since files are already saved in the correct location.

metadata.json schema:
    Written per-session to the bag folder. Read by serve.py /api/flights
    to build the flight database sidebar in meshview.html.

latest.json schema:
    Written to maps_dir after every session. meshview.html polls this
    every 10s and auto-loads the new mesh when the timestamp changes.

Dependencies: trimesh, shutil, json
"""

import json
import shutil
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


class Publisher:
    """
    Writes all output files for a completed processing session.

    Parameters
    ----------
    maps_dir : Path or str
        Directory where PLY files are served from.
        Default: /mnt/ssd/maps/

    Example
    -------
        pub = Publisher(maps_dir="/mnt/ssd/maps")
        pub.publish(
            mesh       = merged_trimesh,
            cloud_pts  = raw_pts_array,
            session_id = "scan_20260319_230358",
            bag_path   = Path("/mnt/ssd/rosbags/scan_20260319_230358"),
            bag_meta   = reader.metadata,
            elapsed_s  = 47.2,
        )
    """

    def __init__(self, maps_dir=None):
        self.maps_dir = Path(maps_dir or "/mnt/ssd/maps")

    def publish(self,
                mesh,
                cloud_pts:  np.ndarray,
                session_id: str,
                bag_path:   Path,
                bag_meta:   dict,
                elapsed_s:  float = 0.0,
                dtm_mesh=None,
                dsm_mesh=None):
        """
        Write all output files for this session.

        Parameters
        ----------
        mesh       : trimesh.Trimesh or None (merged DTM+DSM)
        cloud_pts  : Nx3 float array (raw or capped point cloud)
        session_id : str  e.g. "scan_20260319_230358"
        bag_path   : Path to bag directory
        bag_meta   : dict from BagReader.metadata
        elapsed_s  : total processing time in seconds
        dtm_mesh   : trimesh.Trimesh or None (terrain only, for separate export)
        dsm_mesh   : open3d.TriangleMesh or None (objects only, for separate export)
        """
        t0 = time.time()
        self.maps_dir.mkdir(parents=True, exist_ok=True)

        cloud_filename = f"{session_id}_cloud.ply"
        mesh_filename  = None
        mesh_faces     = None

        # Check if maps_dir and bag_path are the same (Windows fallback case)
        same_dir = self._is_same_directory(self.maps_dir, bag_path)

        # ── save cloud PLY ────────────────────────────────────────────────────
        cloud_path = self._save_cloud(cloud_pts, bag_path, cloud_filename)

        # ── save mesh PLY ─────────────────────────────────────────────────────
        if mesh is not None:
            mesh_filename = f"{session_id}_mesh.ply"
            mesh_path     = bag_path / "mesh_final.ply"
            mesh.export(str(mesh_path))
            mesh_faces = len(mesh.faces)
            print(f"  [Publisher] Mesh saved: {mesh_path.name}  "
                  f"({mesh_faces:,} faces)")
        else:
            print(f"  [Publisher] No mesh to save")

        # ── save separate DTM and DSM meshes ──────────────────────────────────
        if dtm_mesh is not None:
            dtm_path = bag_path / "mesh_dtm.ply"
            dtm_mesh.export(str(dtm_path))
            print(f"  [Publisher] DTM saved: {dtm_path.name}  "
                  f"({len(dtm_mesh.faces):,} faces)")

        if dsm_mesh is not None:
            dsm_path = bag_path / "mesh_dsm.ply"
            # DSM is open3d mesh, need to convert or save directly
            try:
                import open3d as o3d
                o3d.io.write_triangle_mesh(str(dsm_path), dsm_mesh)
                dsm_faces = len(np.asarray(dsm_mesh.triangles))
                print(f"  [Publisher] DSM saved: {dsm_path.name}  "
                      f"({dsm_faces:,} faces)")
            except Exception as e:
                print(f"  [Publisher] DSM save failed: {e}")

        # ── copy to maps dir (skip if same directory) ────────────────────────
        if same_dir:
            print(f"  [Publisher] Files already in output directory (no copy needed)")
        else:
            shutil.copy2(cloud_path, self.maps_dir / cloud_filename)
            print(f"  [Publisher] Cloud → {self.maps_dir / cloud_filename}")

            if mesh_filename:
                shutil.copy2(bag_path / "mesh_final.ply",
                             self.maps_dir / mesh_filename)
                print(f"  [Publisher] Mesh  → {self.maps_dir / mesh_filename}")

        # ── write manifests ───────────────────────────────────────────────────
        self._write_metadata(
            bag_path, session_id, bag_meta,
            len(cloud_pts), mesh_faces,
            cloud_filename, mesh_filename,
        )
        self._write_latest(
            session_id, bag_meta,
            cloud_filename, mesh_filename,
            len(cloud_pts),
        )

        # ── log flight ────────────────────────────────────────────────────────
        self._log_flight(
            session_id, bag_meta,
            len(cloud_pts), mesh_faces,
            elapsed_s,
        )

        io_time = time.time() - t0
        print(f"  [Publisher] Done in {io_time:.1f}s")

    # ── helper: check if same directory ────────────────────────────────────────

    def _is_same_directory(self, dir1: Path, dir2: Path) -> bool:
        """
        Check if two paths point to the same directory.
        Resolves symlinks and normalizes paths for cross-platform comparison.
        """
        try:
            return dir1.resolve() == dir2.resolve()
        except (OSError, ValueError):
            # Fallback to string comparison if resolve fails
            return str(dir1).lower() == str(dir2).lower()

    # ── cloud save ────────────────────────────────────────────────────────────

    def _save_cloud(self,
                    pts:       np.ndarray,
                    bag_path:  Path,
                    filename:  str) -> Path:
        """Save raw point cloud as PLY."""
        import trimesh
        out = bag_path / filename
        trimesh.PointCloud(vertices=pts).export(str(out))
        print(f"  [Publisher] Cloud saved: {out.name}  ({len(pts):,} pts)")
        return out

    # ── metadata.json ─────────────────────────────────────────────────────────

    def _write_metadata(self,
                        bag_path:       Path,
                        session_id:     str,
                        bag_meta:       dict,
                        point_count:    int,
                        mesh_faces,
                        cloud_filename: str,
                        mesh_filename):
        """Write per-session metadata.json to bag folder."""
        payload = {
            "id":           session_id,
            "timestamp":    bag_meta.get("timestamp_iso", "unknown"),
            "duration":     bag_meta.get("duration_str",  "unknown"),
            "duration_s":   bag_meta.get("duration_s",    None),
            "point_count":  point_count,
            "face_count":   mesh_faces,
            "cloud_ply":    cloud_filename,
            "mesh_ply":     mesh_filename,
            "ply_file":     mesh_filename or cloud_filename,
            "status":       "complete" if mesh_faces else "cloud_only",
            "cloud_frames": bag_meta.get("cloud_frames", 0),
            "ros_distro":   bag_meta.get("ros_distro",   "jazzy"),
            "processed_at": datetime.now().strftime("%Y-%m-%d %H:%M"),
        }
        out = bag_path / "metadata.json"
        with open(out, "w") as f:
            json.dump(payload, f, indent=2)
        print(f"  [Publisher] metadata.json → {out}")

    # ── latest.json ───────────────────────────────────────────────────────────

    def _write_latest(self,
                      session_id:     str,
                      bag_meta:       dict,
                      cloud_filename: str,
                      mesh_filename,
                      point_count:    int):
        """
        Write latest.json to maps_dir.
        Browser polls this every 10s and auto-loads on timestamp change.
        """
        payload = {
            "session":      session_id,
            "cloud_ply":    cloud_filename,
            "mesh_ply":     mesh_filename,
            "point_count":  point_count,
            "timestamp":    datetime.now(tz=timezone.utc).isoformat(),
            "duration":     bag_meta.get("duration_str", "unknown"),
        }
        out = self.maps_dir / "latest.json"
        with open(out, "w") as f:
            json.dump(payload, f, indent=2)
        print(f"  [Publisher] latest.json → {out}")

    # ── flight logger ─────────────────────────────────────────────────────────

    def _log_flight(self,
                    session_id:  str,
                    bag_meta:    dict,
                    point_count: int,
                    mesh_faces,
                    elapsed_s:   float):
        """Append to flight_history.log via flight_logger module."""
        try:
            import importlib.util
            logger_path = Path(__file__).parent.parent / "flight_logger.py"
            if not logger_path.exists():
                return
            spec = importlib.util.spec_from_file_location(
                "flight_logger", logger_path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            flight_num = mod.log_flight(
                bag_name    = session_id,
                duration_s  = bag_meta.get("duration_s", 0),
                point_count = point_count,
                mesh_ok     = mesh_faces is not None and mesh_faces > 0,
                notes       = f"processing_time={elapsed_s:.0f}s",
            )
            print(f"  [Publisher] Flight {flight_num:03d} logged")
        except Exception as e:
            print(f"  [Publisher] Flight logger unavailable: {e}")
