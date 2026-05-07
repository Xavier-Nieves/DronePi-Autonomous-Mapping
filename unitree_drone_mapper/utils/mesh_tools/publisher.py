"""
publisher.py — Write output files and update browser manifests.

Handles all file I/O at the end of the pipeline:
  1. Save raw point cloud PLY  (_cloud.ply)     → bag_path/
  2. Save merged mesh PLY      (mesh_final.ply) → bag_path/
  3. Write metadata.json                        → bag_path/
  4. Write latest.json                          → maps_dir/
  5. Call flight_logger.log_flight()

PLY files are no longer copied to maps_dir. serve.py already exposes
/rosbags/<session>/ via its _serve_rosbag_file() handler, so the browser
fetches PLYs directly from the rosbags directory. latest.json stores
the full rosbags-relative URL (e.g. rosbags/<session>/mesh_final.ply)
so meshview.html can resolve them against PI_BASE_URL without a copy.

metadata.json schema:
    Written per-session to the bag folder. Read by serve.py /api/flights
    and by the ground station ArtifactFetcher.

    Fields added in this version:
      processing_time_s  — total pipeline wall-clock time (was already
                           passed as elapsed_s but never written)
      stage_timings_s    — per-stage wall-clock dict, keyed by stage name
      bottleneck_stage   — name of the slowest stage
      algorithms         — fixed algorithm labels for this pipeline version

latest.json schema:
    Written to maps_dir after every session. meshview.html polls this
    every 10s and auto-loads the new mesh when the timestamp changes.
    mesh_ply and cloud_ply are rosbags-relative URLs, not bare filenames.

Dependencies: trimesh, json
"""

import json
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
        Directory where manifests (latest.json, ortho_metadata.json) are
        written and served from. Default: /mnt/ssd/maps/
        PLY files are NOT copied here — they are served from bag_path via
        the /rosbags/ route in serve.py.
    """

    def __init__(self, maps_dir=None):
        self.maps_dir = Path(maps_dir or "/mnt/ssd/maps")

    def publish(self,
                mesh,
                cloud_pts:     np.ndarray,
                session_id:    str,
                bag_path:      Path,
                bag_meta:      dict,
                elapsed_s:     float = 0.0,
                dtm_mesh=None,
                dsm_mesh=None,
                stage_timings: dict = None):
        """
        Write all output files for this session.

        Parameters
        ----------
        mesh          : trimesh.Trimesh or None (merged DTM+DSM)
        cloud_pts     : Nx3 float array (raw or capped point cloud)
        session_id    : str  e.g. "scan_20260319_230358"
        bag_path      : Path to bag directory (all PLYs written here)
        bag_meta      : dict from BagReader.metadata
        elapsed_s     : total processing time in seconds
        dtm_mesh      : trimesh.Trimesh or None (terrain only)
        dsm_mesh      : open3d.TriangleMesh or None (objects only)
        stage_timings : dict mapping stage name → wall-clock seconds.
                        Keys: BagReader, MLSSmoother, GroundClassifier,
                        DTMBuilder, DSMBuilder, MeshMerger,
                        TextureProjection, Publisher.
                        Missing keys default to 0.0 in metadata.json.
                        Passed in from postprocess_mesh.py orchestrator.
        """
        t0 = time.time()
        self.maps_dir.mkdir(parents=True, exist_ok=True)

        cloud_filename = f"{session_id}_cloud.ply"
        mesh_filename  = None
        mesh_faces     = None

        # ── save cloud PLY ────────────────────────────────────────────────────
        self._save_cloud(cloud_pts, bag_path, cloud_filename)

        # ── save mesh PLY ─────────────────────────────────────────────────────
        if mesh is not None:
            mesh_filename = "mesh_final.ply"
            mesh_path     = bag_path / mesh_filename
            mesh.export(str(mesh_path))
            mesh_faces    = len(mesh.faces)
            print(f"  [Publisher] Mesh saved: {mesh_filename}  ({mesh_faces:,} faces)")
        else:
            print(f"  [Publisher] No mesh to save")

        # ── save DTM ──────────────────────────────────────────────────────────
        if dtm_mesh is not None:
            dtm_path = bag_path / "mesh_dtm.ply"
            dtm_mesh.export(str(dtm_path))
            print(f"  [Publisher] DTM saved: mesh_dtm.ply  ({len(dtm_mesh.faces):,} faces)")

        # ── save DSM ──────────────────────────────────────────────────────────
        if dsm_mesh is not None:
            dsm_path = bag_path / "mesh_dsm.ply"
            try:
                import open3d as o3d
                o3d.io.write_triangle_mesh(str(dsm_path), dsm_mesh)
                print(f"  [Publisher] DSM saved: mesh_dsm.ply  "
                      f"({len(np.asarray(dsm_mesh.triangles)):,} faces)")
            except Exception as e:
                print(f"  [Publisher] DSM save failed: {e}")

        # ── write manifests ───────────────────────────────────────────────────
        self._write_metadata(
            bag_path      = bag_path,
            session_id    = session_id,
            bag_meta      = bag_meta,
            point_count   = len(cloud_pts),
            mesh_faces    = mesh_faces,
            cloud_filename = cloud_filename,
            mesh_filename  = mesh_filename,
            elapsed_s      = elapsed_s,
            stage_timings  = stage_timings,
        )
        self._write_latest(
            session_id     = session_id,
            bag_meta       = bag_meta,
            cloud_filename = cloud_filename,
            mesh_filename  = mesh_filename,
            point_count    = len(cloud_pts),
        )

        # ── log flight ────────────────────────────────────────────────────────
        self._log_flight(session_id, bag_meta, len(cloud_pts), mesh_faces, elapsed_s)

        io_time = time.time() - t0
        print(f"  [Publisher] Done in {io_time:.1f}s")

    # ── cloud save ────────────────────────────────────────────────────────────

    def _save_cloud(self, pts: np.ndarray, bag_path: Path, filename: str) -> Path:
        """Save raw point cloud as PLY to bag_path."""
        import trimesh
        out = bag_path / filename
        trimesh.PointCloud(vertices=pts).export(str(out))
        print(f"  [Publisher] Cloud saved: {filename}  ({len(pts):,} pts)")
        return out

    # ── metadata.json ─────────────────────────────────────────────────────────

    def _write_metadata(self,
                        bag_path:       Path,
                        session_id:     str,
                        bag_meta:       dict,
                        point_count:    int,
                        mesh_faces,
                        cloud_filename: str,
                        mesh_filename,
                        elapsed_s:      float = 0.0,
                        stage_timings:  dict  = None):
        """
        Write per-session metadata.json to bag folder.

        stage_timings keys (all optional — missing keys written as 0.0):
            BagReader, MLSSmoother, GroundClassifier, DTMBuilder,
            DSMBuilder, MeshMerger, TextureProjection, Publisher
        """
        # Normalise timings — ensure all expected keys present, fill gaps with 0.0
        _stage_keys = [
            "BagReader", "MLSSmoother", "GroundClassifier",
            "DTMBuilder", "DSMBuilder", "MeshMerger",
            "TextureProjection", "Publisher",
        ]
        timings = {k: round((stage_timings or {}).get(k, 0.0), 1)
                   for k in _stage_keys}

        # Bottleneck is the stage with the largest recorded time.
        # Only consider stages that actually ran (time > 0).
        ran = {k: v for k, v in timings.items() if v > 0.0}
        bottleneck = max(ran, key=ran.get) if ran else None

        payload = {
            "id":              session_id,
            "timestamp":       bag_meta.get("timestamp_iso", "unknown"),
            "duration":        bag_meta.get("duration_str",  "unknown"),
            "duration_s":      bag_meta.get("duration_s",    None),
            "point_count":     point_count,
            "face_count":      mesh_faces,
            "cloud_ply":       cloud_filename,
            "mesh_ply":        mesh_filename,
            "ply_file":        mesh_filename or cloud_filename,
            "status":          "complete" if mesh_faces else "cloud_only",
            "cloud_frames":    bag_meta.get("cloud_frames", 0),
            "ros_distro":      bag_meta.get("ros_distro",   "jazzy"),
            "processed_at":    datetime.now().strftime("%Y-%m-%d %H:%M"),
            # ── timing fields (new) ───────────────────────────────────────────
            "processing_time_s": round(elapsed_s, 1),
            "stage_timings_s":   timings,
            "bottleneck_stage":  bottleneck,
            "algorithms": {
                "smoother":   "MLS",
                "classifier": "SMRF",
                "dtm":        "Delaunay2.5D",
                "dsm":        "BPA",
            },
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

        mesh_ply and cloud_ply use rosbags-relative URLs so meshview.html
        can fetch them via serve.py's /rosbags/ route without a copy step.
        e.g. "rosbags/scan_20260319_230358/mesh_final.ply"
        """
        payload = {
            "session_id":  session_id,
            "timestamp":   datetime.now(timezone.utc).isoformat(),
            "point_count": point_count,
            "mesh_ply":    (f"rosbags/{session_id}/{mesh_filename}"
                            if mesh_filename else None),
            "cloud_ply":   f"rosbags/{session_id}/{cloud_filename}",
            "duration":    bag_meta.get("duration_str", "unknown"),
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
        """Call flight_logger.log_flight() — non-fatal if unavailable."""
        try:
            import importlib.util
            logger_path = (
                Path(__file__).resolve().parents[2] / "utils" / "flight_logger.py"
            )
            if not logger_path.exists():
                return
            spec = importlib.util.spec_from_file_location("flight_logger", logger_path)
            mod  = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            mod.log_flight(
                bag_name    = session_id,
                duration_s  = bag_meta.get("duration_s", 0.0) or 0.0,
                point_count = point_count,
                mesh_ok     = mesh_faces is not None,
                mesh_faces  = mesh_faces,
            )
        except Exception as exc:
            print(f"  [Publisher] flight_logger unavailable (non-fatal): {exc}")
