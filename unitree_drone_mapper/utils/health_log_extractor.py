"""
mesh_tools/health_log_extractor.py — Extract /rpi/health from rosbag to CSV.

Reads std_msgs/String messages on /rpi/health from the MCAP bag recorded
during flight, deserializes the JSON payload, and writes health_log.csv to
the session folder.

This file is consumed by the ground station ArtifactFetcher, parsed by
HealthLogParser on the laptop, and stored in FlightDatabase for the CPU
temp trend chart and throttle detection dashboard card.

Architecture
------------
Single public method: extract(bag_path, output_path) → int (row count).
Non-fatal by contract — any failure logs a warning and returns 0.
Never raises. Never called by production code other than postprocess_mesh.py.
No main() — not a standalone script.

Column schema (must match ground station HealthLogParser exactly)
-----------------------------------------------------------------
  timestamp       Unix seconds (float) from ROS message stamp
  cpu_percent     psutil.cpu_percent()              0–100
  cpu_temp        /sys/class/thermal or vcgencmd    °C
  cpu_freq_mhz    psutil.cpu_freq()                 MHz
  mem_percent     psutil.virtual_memory()           0–100
  mem_used_mb     psutil.virtual_memory()           MB
  mem_total_mb    psutil.virtual_memory()           MB
  disk_percent    psutil.disk_usage("/")            0–100
  throttled       vcgencmd get_throttled            bool
  throttle_bits   vcgencmd get_throttled            hex string e.g. "0x0"
  load_avg_1m     os.getloadavg()                   float
  load_avg_5m     os.getloadavg()                   float

Message format
--------------
/rpi/health is published as std_msgs/String whose .data field is a JSON
string produced by rpi_health_node.py collect_metrics(). The rosbags
library deserializes std_msgs/String via deserialize_cdr(), exposing a
.data attribute. json.loads(.data) yields the metrics dict.

The JSON keys from collect_metrics() map directly to the CSV columns above.

Compatibility
-------------
Bags recorded before rpi_health_node was deployed will not have the
/rpi/health topic. extract() detects this and returns 0 with a log line.
No error is raised — the health_log.csv is simply absent for that session,
which the ground station handles gracefully.

References
----------
- rpi_health_node.py collect_metrics() — defines the JSON schema
- rosbags deserialize_cdr: https://ternaris.gitlab.io/rosbags/topics/serde.html
- HealthLogParser (ground_station/report_generator.py) — CSV consumer
"""

import csv
import json
from pathlib import Path
from typing import Union


# CSV columns in the exact order expected by the ground station HealthLogParser.
_FIELDNAMES = [
    "timestamp",
    "cpu_percent",
    "cpu_temp",
    "cpu_freq_mhz",
    "mem_percent",
    "mem_used_mb",
    "mem_total_mb",
    "disk_percent",
    "throttled",
    "throttle_bits",
    "load_avg_1m",
    "load_avg_5m",
]


class HealthLogExtractor:
    """
    Extracts /rpi/health topic messages from a rosbag and writes health_log.csv.

    Usage (called by postprocess_mesh.py after BagReader)
    -----------------------------------------------------
        extractor = HealthLogExtractor()
        n_rows = extractor.extract(bag_path, bag_path / "health_log.csv")
        # n_rows == 0 → topic absent or bag unreadable (non-fatal)

    The extractor is stateless — a single instance may be reused across
    multiple bags, though the postprocess pipeline creates one per run.
    """

    def extract(self,
                bag_path:    Union[Path, str],
                output_path: Union[Path, str]) -> int:
        """
        Read /rpi/health messages from bag and write health_log.csv.

        Parameters
        ----------
        bag_path    : Path to the rosbag MCAP directory.
        output_path : Destination path for health_log.csv.

        Returns
        -------
        int : Number of rows written (0 if topic absent or any error).
        """
        bag_path    = Path(bag_path)
        output_path = Path(output_path)

        rows = self._read_bag(bag_path)
        if not rows:
            return 0

        self._write_csv(rows, output_path)
        return len(rows)

    # ── Private ───────────────────────────────────────────────────────────────

    def _read_bag(self, bag_path: Path) -> list:
        """
        Open the bag, find /rpi/health connections, deserialize messages.

        Returns list of dicts (one per message), or [] on any error.
        """
        try:
            from rosbags.rosbag2 import Reader
            from rosbags.serde   import deserialize_cdr
        except ImportError as exc:
            print(f"  [HealthLog] rosbags not available — skipping: {exc}")
            return []

        rows = []
        try:
            with Reader(bag_path) as reader:
                connections = [
                    c for c in reader.connections
                    if c.topic == "/rpi/health"
                ]
                if not connections:
                    print("  [HealthLog] /rpi/health topic not in bag — skipping")
                    return []

                for conn, timestamp_ns, rawdata in reader.messages(connections=connections):
                    try:
                        # Deserialize std_msgs/String — yields object with .data
                        msg = deserialize_cdr(rawdata, conn.msgtype)
                        # .data is the JSON string from collect_metrics()
                        metrics = json.loads(msg.data)
                        row = self._build_row(metrics, timestamp_ns)
                        rows.append(row)
                    except Exception as exc:
                        # Single bad message — log and continue
                        print(f"  [HealthLog] Skipped malformed message: {exc}")
                        continue

        except Exception as exc:
            print(f"  [HealthLog] Bag read failed: {exc}")
            return []

        return rows

    def _build_row(self, metrics: dict, timestamp_ns: int) -> dict:
        """
        Map collect_metrics() JSON keys to CSV column names.

        Uses the ROS message timestamp (nanoseconds → seconds) rather than
        the timestamp field in the JSON payload. This keeps the CSV time
        axis aligned with other topics in the bag.

        All fields default to empty string if absent — the laptop parser
        handles missing columns with its own defaults.
        """
        return {
            "timestamp":    round(timestamp_ns / 1e9, 3),
            "cpu_percent":  metrics.get("cpu_percent", ""),
            "cpu_temp":     metrics.get("cpu_temp",    ""),
            "cpu_freq_mhz": metrics.get("cpu_freq_mhz", ""),
            "mem_percent":  metrics.get("mem_percent",  ""),
            "mem_used_mb":  metrics.get("mem_used_mb",  ""),
            "mem_total_mb": metrics.get("mem_total_mb", ""),
            "disk_percent": metrics.get("disk_percent", ""),
            "throttled":    metrics.get("throttled",    ""),
            "throttle_bits":metrics.get("throttle_bits",""),
            "load_avg_1m":  metrics.get("load_avg_1m",  ""),
            "load_avg_5m":  metrics.get("load_avg_5m",  ""),
        }

    def _write_csv(self, rows: list, output_path: Path) -> None:
        """Write rows to CSV. Parent directory must already exist."""
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=_FIELDNAMES,
                extrasaction="ignore",   # drop any extra keys gracefully
            )
            writer.writeheader()
            writer.writerows(rows)
        print(f"  [HealthLog] Wrote {len(rows)} rows → {output_path.name}")
