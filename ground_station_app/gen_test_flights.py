"""
gen_test_flights.py — Generate 10 randomised flight sessions for ground station testing.

Creates realistic data with deliberate anomaly injections:
  - Flight 4: CPU thermal spike (throttling)
  - Flight 6: High SLAM drift + erratic odometry
  - Flight 8: GPS fix loss events + EKF2 spikes
  - Flight 10: Battery sag + memory pressure

Run from repo root or ground_station_app/:
    python gen_test_flights.py [--cache-dir PATH]

Writes to %USERPROFILE%/.dronepi-ground/cache/ by default.
"""

import argparse
import json
import os
import random
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

random.seed(42)  # reproducible

# ── Destination ───────────────────────────────────────────────────────────────

def get_cache_dir(override=None):
    if override:
        return Path(override)
    return Path(os.environ.get(
        "DRONEPI_CACHE_DIR",
        Path.home() / ".dronepi-ground" / "cache"
    ))

# ── Flight profiles ───────────────────────────────────────────────────────────
# Each profile drives the generation of all four artifact files.

PROFILES = [
    # (flight_num, label, duration_s, points, drift, loops, anomalies, injections)
    # injections = dict of deliberate fault parameters
    {
        "num": 1, "label": "Nominal short survey",
        "duration_s": 142.0, "points_final": 870000, "drift": 0.09,
        "loops": 2, "end_reason": "AUTO.LAND",
        "cpu_base": 52, "cpu_peak": 64, "throttle_events": 0,
        "mem_peak": 44, "bat_start": 16.7, "bat_end": 15.9, "bat_min": 15.8,
        "vertices": 62100, "faces": 124000,
        "anomalies": [],
        "health_rows": 5,
        "stage_timings": {"BagReader": 6.1, "MLSSmoother": 8.2, "GroundClassifier": 5.4, "DTMBuilder": 1.3, "DSMBuilder": 11.8, "MeshMerger": 0.4, "TextureProjection": 28.1, "Publisher": 0.7},
        "processing_time_s": 62.0,
        "bottleneck": "DSMBuilder",
    },
    {
        "num": 2, "label": "Nominal medium survey",
        "duration_s": 187.4, "points_final": 1200000, "drift": 0.13,
        "loops": 3, "end_reason": "AUTO.LAND",
        "cpu_base": 54, "cpu_peak": 67, "throttle_events": 0,
        "mem_peak": 46, "bat_start": 16.6, "bat_end": 15.7, "bat_min": 15.6,
        "vertices": 84200, "faces": 167800,
        "anomalies": [],
        "health_rows": 7,
        "stage_timings": {"BagReader": 6.3, "MLSSmoother": 8.8, "GroundClassifier": 5.5, "DTMBuilder": 1.4, "DSMBuilder": 12.4, "MeshMerger": 0.5, "TextureProjection": 31.2, "Publisher": 0.8},
        "processing_time_s": 67.0,
        "bottleneck": "DSMBuilder",
    },
    {
        "num": 3, "label": "Good quality — high point density",
        "duration_s": 210.2, "points_final": 1580000, "drift": 0.11,
        "loops": 4, "end_reason": "AUTO.LAND",
        "cpu_base": 55, "cpu_peak": 69, "throttle_events": 0,
        "mem_peak": 47, "bat_start": 16.8, "bat_end": 15.5, "bat_min": 15.4,
        "vertices": 110400, "faces": 220500,
        "anomalies": [],
        "health_rows": 8,
        "stage_timings": {"BagReader": 6.8, "MLSSmoother": 9.1, "GroundClassifier": 5.9, "DTMBuilder": 1.5, "DSMBuilder": 13.1, "MeshMerger": 0.5, "TextureProjection": 35.4, "Publisher": 0.9},
        "processing_time_s": 73.0,
        "bottleneck": "DSMBuilder",
    },
    {
        "num": 4, "label": "ANOMALY: CPU thermal spike — throttling event",
        "duration_s": 195.7, "points_final": 1090000, "drift": 0.17,
        "loops": 3, "end_reason": "AUTO.LAND",
        "cpu_base": 58, "cpu_peak": 83,   # <<< spike over 80°C
        "throttle_events": 4,             # <<< throttling
        "mem_peak": 51, "bat_start": 16.5, "bat_end": 15.6, "bat_min": 15.5,
        "vertices": 78300, "faces": 155900,
        "anomalies": [
            "CPU temperature reached 83.1°C at t=112.4s — thermal throttling active",
            "4 throttle events recorded — processing pipeline slowed",
            "SLAM point rate dropped 18% during throttle window t=110–135s",
        ],
        "health_rows": 8,
        "stage_timings": {"BagReader": 7.8, "MLSSmoother": 14.2, "GroundClassifier": 9.1, "DTMBuilder": 2.1, "DSMBuilder": 18.3, "MeshMerger": 0.8, "TextureProjection": 44.7, "Publisher": 1.1},
        "processing_time_s": 98.0,
        "bottleneck": "TextureProjection",
    },
    {
        "num": 5, "label": "Nominal — recovery after thermal flight",
        "duration_s": 160.8, "points_final": 990000, "drift": 0.12,
        "loops": 2, "end_reason": "AUTO.LAND",
        "cpu_base": 51, "cpu_peak": 65, "throttle_events": 0,
        "mem_peak": 43, "bat_start": 16.7, "bat_end": 15.9, "bat_min": 15.8,
        "vertices": 70200, "faces": 140100,
        "anomalies": [],
        "health_rows": 6,
        "stage_timings": {"BagReader": 6.2, "MLSSmoother": 8.4, "GroundClassifier": 5.6, "DTMBuilder": 1.3, "DSMBuilder": 11.9, "MeshMerger": 0.4, "TextureProjection": 29.3, "Publisher": 0.7},
        "processing_time_s": 64.0,
        "bottleneck": "DSMBuilder",
    },
    {
        "num": 6, "label": "ANOMALY: High SLAM drift + erratic odometry",
        "duration_s": 201.3, "points_final": 740000, "drift": 0.41,   # <<< drift >0.25
        "loops": 1, "end_reason": "AUTO.LAND",
        "cpu_base": 56, "cpu_peak": 70, "throttle_events": 0,
        "mem_peak": 48, "bat_start": 16.6, "bat_end": 15.4, "bat_min": 15.3,
        "vertices": 52000, "faces": 103800,
        "anomalies": [
            "SLAM drift estimate 0.41m — exceeds 0.25m warning threshold",
            "Erratic angular velocity detected at t=88.3s (ω=4.2 rad/s, expected <1.5)",
            "Erratic angular velocity detected at t=143.7s (ω=3.8 rad/s)",
            "Only 1 loop closure — reduced map consistency",
            "Point cloud density 38% below fleet average — possible occlusion",
        ],
        "health_rows": 7,
        "stage_timings": {"BagReader": 6.5, "MLSSmoother": 8.7, "GroundClassifier": 5.8, "DTMBuilder": 1.4, "DSMBuilder": 24.6, "MeshMerger": 0.5, "TextureProjection": 33.8, "Publisher": 0.8},
        "processing_time_s": 82.0,
        "bottleneck": "DSMBuilder",
    },
    {
        "num": 7, "label": "Nominal — stable conditions",
        "duration_s": 223.1, "points_final": 1640000, "drift": 0.08,
        "loops": 5, "end_reason": "AUTO.LAND",
        "cpu_base": 53, "cpu_peak": 66, "throttle_events": 0,
        "mem_peak": 45, "bat_start": 16.9, "bat_end": 15.6, "bat_min": 15.5,
        "vertices": 116700, "faces": 233100,
        "anomalies": [],
        "health_rows": 8,
        "stage_timings": {"BagReader": 7.1, "MLSSmoother": 9.3, "GroundClassifier": 6.1, "DTMBuilder": 1.6, "DSMBuilder": 13.8, "MeshMerger": 0.6, "TextureProjection": 38.2, "Publisher": 0.9},
        "processing_time_s": 78.0,
        "bottleneck": "DSMBuilder",
    },
    {
        "num": 8, "label": "ANOMALY: GPS fix loss + EKF2 innovation spikes",
        "duration_s": 178.5, "points_final": 960000, "drift": 0.22,
        "loops": 2, "end_reason": "AUTO.LAND",
        "cpu_base": 55, "cpu_peak": 71, "throttle_events": 0,
        "mem_peak": 49, "bat_start": 16.5, "bat_end": 15.5, "bat_min": 15.3,
        "vertices": 68400, "faces": 136500,
        "anomalies": [
            "GPS fix loss at t=67.2s — 3D fix recovered after 8.4s",
            "GPS fix loss at t=134.9s — 3D fix recovered after 5.1s",
            "EKF2 innovation spike 0.31m at t=67.8s (threshold 0.25m)",
            "EKF2 innovation spike 0.28m at t=135.2s",
            "HDOP max 4.2 — poor satellite geometry during fix loss windows",
        ],
        "health_rows": 7,
        "stage_timings": {"BagReader": 8.4, "MLSSmoother": 9.0, "GroundClassifier": 5.9, "DTMBuilder": 1.5, "DSMBuilder": 12.7, "MeshMerger": 0.5, "TextureProjection": 36.1, "Publisher": 0.8},
        "processing_time_s": 75.0,
        "bottleneck": "DSMBuilder",
    },
    {
        "num": 9, "label": "Nominal — best flight of batch",
        "duration_s": 234.6, "points_final": 1820000, "drift": 0.07,
        "loops": 6, "end_reason": "AUTO.LAND",
        "cpu_base": 52, "cpu_peak": 63, "throttle_events": 0,
        "mem_peak": 44, "bat_start": 16.9, "bat_end": 15.7, "bat_min": 15.6,
        "vertices": 128900, "faces": 257500,
        "anomalies": [],
        "health_rows": 9,
        "stage_timings": {"BagReader": 7.2, "MLSSmoother": 9.6, "GroundClassifier": 6.3, "DTMBuilder": 1.7, "DSMBuilder": 14.2, "MeshMerger": 0.6, "TextureProjection": 41.5, "Publisher": 1.0},
        "processing_time_s": 82.0,
        "bottleneck": "DSMBuilder",
    },
    {
        "num": 10, "label": "ANOMALY: Battery sag + memory pressure",
        "duration_s": 244.8, "points_final": 1380000, "drift": 0.19,
        "loops": 3, "end_reason": "LOW_BATTERY",  # <<< ended early
        "cpu_base": 57, "cpu_peak": 74, "throttle_events": 1,
        "mem_peak": 78,                            # <<< memory pressure
        "bat_start": 16.4, "bat_end": 14.6, "bat_min": 14.2,  # <<< sag
        "vertices": 98600, "faces": 197000,
        "anomalies": [
            "Flight ended early — LOW_BATTERY trigger at t=244.8s",
            "Battery minimum voltage 14.2V — below 14.8V caution threshold",
            "Battery sag 2.2V (start 16.4V → end 14.6V) — above normal 1.5V",
            "Memory usage peaked at 78% at t=198.3s — swap pressure observed",
            "1 CPU throttle event at t=201.1s coincident with memory peak",
        ],
        "health_rows": 10,
        "stage_timings": {"BagReader": 7.0, "MLSSmoother": 10.1, "GroundClassifier": 6.8, "DTMBuilder": 1.8, "DSMBuilder": 15.3, "MeshMerger": 0.6, "TextureProjection": 67.4, "Publisher": 1.1},
        "processing_time_s": 110.0,
        "bottleneck": "TextureProjection",
    },
]


# ── Writers ───────────────────────────────────────────────────────────────────

BASE_DT = datetime(2026, 5, 4, 8, 0, 0, tzinfo=timezone.utc)


def flight_record(p, arm_time):
    return {
        "flight_number":    p["num"],
        "session_id":       session_id(p, arm_time),
        "script_name":      "main.py",
        "flight_type":      "AUTONOMOUS",
        "context":          "mission",
        "arm_time_iso":     arm_time.isoformat(),
        "arm_time_unix":    arm_time.timestamp(),
        "duration_s":       p["duration_s"],
        "end_reason":       p["end_reason"],
        "bag_path":         f"/mnt/ssd/rosbags/{session_id(p, arm_time)}",
        "bag_closed_cleanly": True,
        "postprocess_ran":  True,
    }


def bag_summary(p):
    raw = int(p["points_final"] * random.uniform(1.8, 2.2))
    sor = int(raw * random.uniform(0.005, 0.015))
    return (
        "point_count_raw,point_count_final,drift_estimate_m,loop_closures,"
        "mls_iterations,sor_outliers_removed,processing_duration_s,bag_path\n"
        f"{raw},{p['points_final']},{p['drift']},{p['loops']},"
        f"{random.randint(3,7)},{sor},{round(random.uniform(38,65),1)},"
        f"/mnt/ssd/rosbags/scan\n"
    )


def health_log(p, duration):
    rows = ["timestamp_s,cpu_temp_c,throttle_flags,memory_used_pct"]
    n = p["health_rows"]
    step = duration / (n - 1)
    for i in range(n):
        t = round(i * step, 1)
        # Ramp cpu up then back down; inject peak near middle
        frac = i / (n - 1)
        warmup = p["cpu_base"] + (p["cpu_peak"] - p["cpu_base"]) * (
            4 * frac * (1 - frac)  # parabola peaking at midpoint
        )
        cpu = round(warmup + random.uniform(-1.5, 1.5), 1)
        # Throttle flag: 1 = under-voltage, 2 = arm frequency, 4 = temp
        # Only set near peak temps
        near_peak = abs(frac - 0.5) < 0.2
        throttle = 0
        if near_peak and p["throttle_events"] > 0:
            throttle = 6  # freq + temp flags
        elif p["throttle_events"] == 1 and frac > 0.8:
            throttle = 2
        mem = round(
            p["mem_peak"] * (0.85 + 0.15 * (4 * frac * (1 - frac)))
            + random.uniform(-2, 2), 1
        )
        rows.append(f"{t},{cpu},{throttle},{mem}")
    return "\n".join(rows) + "\n"


def report_md(p, arm_time):
    sid = session_id(p, arm_time)
    anm_section = ""
    if p["anomalies"]:
        items = "\n".join(f"- {a}" for a in p["anomalies"])
        anm_section = items
    else:
        anm_section = "None detected. All systems operated within normal parameters."

    quality = "Excellent" if p["drift"] < 0.10 else \
              "Good"      if p["drift"] < 0.20 else \
              "Degraded"  if p["drift"] < 0.35 else "Poor"

    bat_delta = round(p["bat_start"] - p["bat_end"], 1)
    bat_status = "normal" if bat_delta < 1.7 else "elevated — review battery health"

    rec_lines = []
    if p["drift"] > 0.25:
        rec_lines.append("- Inspect IMU calibration and vibration damping before next flight")
    if p["cpu_peak"] > 78:
        rec_lines.append("- Check RPi 5 cooling — consider active cooling or reduced pipeline load")
    if p["throttle_events"] > 2:
        rec_lines.append("- Reduce SLAM pipeline concurrency to prevent thermal throttling")
    if p["bat_min"] < 14.8:
        rec_lines.append("- Battery approaching end-of-life — measure internal resistance")
    if p["mem_peak"] > 70:
        rec_lines.append("- Memory usage critical — profile pipeline and reduce buffer sizes")
    if not rec_lines:
        rec_lines.append("- Continue standard pre-flight checks")
        rec_lines.append("- No corrective action required")

    recs = "\n".join(rec_lines)

    return f"""## Summary

Flight {p['num']:02d} ({p['label']}) completed in {p['duration_s']:.1f}s with end reason `{p['end_reason']}`. \
The SLAM pipeline produced {p['points_final']:,} points after filtering with a drift estimate of {p['drift']}m \
({quality.lower()} quality). \
{"No anomalies were detected." if not p['anomalies'] else f"{len(p['anomalies'])} anomalies were recorded — see below."}

## Anomalies

{anm_section}

## SLAM Quality

- **Point count (final):** {p['points_final']:,}
- **Drift estimate:** {p['drift']}m ({quality})
- **Loop closures:** {p['loops']}
- **Quality verdict:** {quality}

## GPS Quality

{"HDOP remained below 2.0 throughout. No fix-loss events." if "GPS" not in str(p['anomalies']) else "See anomalies — GPS fix loss detected during flight. EKF2 relied on SLAM odometry during outage windows."}

## System Health

- **CPU temp peak:** {p['cpu_peak']}°C {"⚠ throttling occurred" if p['throttle_events'] > 0 else "✓ within limits"}
- **Throttle events:** {p['throttle_events']}
- **Memory peak:** {p['mem_peak']}% {"⚠ pressure detected" if p['mem_peak'] > 70 else "✓ within limits"}
- **Battery:** {p['bat_start']}V → {p['bat_end']}V (min {p['bat_min']}V, Δ{bat_delta}V — {bat_status})

## Mesh Output

- **Vertices:** {p['vertices']:,}
- **Faces:** {p['faces']:,}
- **Session:** `{sid}`

## Recommendations

{recs}
"""


def metadata(p):
    st = p.get("stage_timings", {})
    return {
        "vertex_count":      p["vertices"],
        "face_count":        p["faces"],
        "processing_time_s": p.get("processing_time_s", 0),
        "stage_timings_s": {
            "BagReader":         st.get("BagReader", 0),
            "MLSSmoother":       st.get("MLSSmoother", 0),
            "GroundClassifier":  st.get("GroundClassifier", 0),
            "DTMBuilder":        st.get("DTMBuilder", 0),
            "DSMBuilder":        st.get("DSMBuilder", 0),
            "MeshMerger":        st.get("MeshMerger", 0),
            "TextureProjection": st.get("TextureProjection", 0),
            "Publisher":         st.get("Publisher", 0),
        },
        "bottleneck_stage":  p.get("bottleneck", "DSMBuilder"),
        "algorithms": {
            "smoother":   "MLS",
            "classifier": "SMRF",
            "dtm":        "Delaunay2.5D",
            "dsm":        "BPA",
        },
    }


def session_id(p, arm_time):
    return f"scan_{arm_time.strftime('%Y%m%d_%H%M%S')}"


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Generate test flight cache")
    parser.add_argument("--cache-dir", default=None)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    cache = get_cache_dir(args.cache_dir)
    print(f"Cache dir: {cache}")

    arm_time = BASE_DT
    created = []

    for p in PROFILES:
        sid = session_id(p, arm_time)
        dest = cache / sid

        if args.dry_run:
            print(f"  [dry-run] would create {dest}")
        else:
            dest.mkdir(parents=True, exist_ok=True)

            # flight_record.json
            (dest / "flight_record.json").write_text(
                json.dumps(flight_record(p, arm_time), indent=2), encoding="utf-8")

            # bag_summary.csv
            (dest / "bag_summary.csv").write_text(
                bag_summary(p), encoding="utf-8")

            # health_log.csv
            (dest / "health_log.csv").write_text(
                health_log(p, p["duration_s"]), encoding="utf-8")

            # metadata.json
            (dest / "metadata.json").write_text(
                json.dumps(metadata(p)), encoding="utf-8")

            # report.md
            (dest / "report.md").write_text(
                report_md(p, arm_time), encoding="utf-8")

            created.append(sid)
            anm_count = len(p["anomalies"])
            flag = "  *** ANOMALY FLIGHT ***" if anm_count else ""
            print(f"  Created {sid}  ({p['label']})  anomalies={anm_count}{flag}")

        # Space flights ~25 min apart with slight jitter
        arm_time += timedelta(minutes=random.randint(22, 32))

    if not args.dry_run:
        print(f"\nDone — {len(created)} sessions written to {cache}")
        print("\nVerify with:")
        print("  python -c \"from ground_station.db import FlightDatabase; "
              "db=FlightDatabase(); print(db.get_stats())\"")
        print("\nThen open the ground station and refresh flights.")


if __name__ == "__main__":
    main()
