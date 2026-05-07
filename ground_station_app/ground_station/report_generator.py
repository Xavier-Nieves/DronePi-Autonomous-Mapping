"""
ground_station/report_generator.py — Post-flight narrative report generator.

Responsibilities
----------------
1. Accept the artifact paths dict from ArtifactFetcher.
2. Run FlightSamplesParser, BagSummaryParser, HealthLogParser on their files.
3. Merge all parser outputs into the structured JSON intermediate.
4. Build the system prompt and user prompt.
5. Call OllamaClient.chat() and return the markdown report string.

The LLM is a writer, not a pilot. It receives only structured data and
produces only a markdown narrative. No flight decisions are involved.

Data sources
------------
  flight_samples.csv   → FlightSamplesParser
      modes, rc_channel_7_events, ekf2_innovations (pos_error proxy),
      angvel anomalies, start_time, duration_s

  bag_summary.csv      → BagSummaryParser
      slam.point_count_final, drift_estimate_m, loop_closures

  health_log.csv       → HealthLogParser
      rpi_health.cpu_temp_max_c, throttling_events, memory_peak_pct

  flight_record.json   → direct JSON read
      start_time fallback, duration_s fallback

  metadata.json        → direct JSON read
      mesh.vertex_count, mesh.face_count

GPS quality is absent from this pipeline — /mavros/gpsstatus/gps1/raw is not
yet in BAG_TOPICS_REQUIRED. The gps field is populated with safe defaults and
Ollama is instructed to omit GPS sections when data is absent.

Structured JSON intermediate schema
------------------------------------
{
  "flight_id":    str,
  "start_time":   str,
  "duration_s":   float,
  "modes":        [{t, mode}],
  "ekf2_innovations": {max, mean, spikes},
  "gps":          {hdop_max, hdop_mean, fix_loss_events},
  "slam":         {point_count_final, drift_estimate_m, loop_closures},
  "battery":      {start_v, end_v, min_v},
  "rc_channel_7_events": [float],
  "rpi_health":   {cpu_temp_max_c, throttling_events, memory_peak_pct},
  "anomalies":    [str],
  "mesh":         {vertex_count, face_count}
}
"""

import json
import logging
from pathlib import Path
from typing import Optional

from ground_station.bag_summary_parser      import BagSummaryParser
from ground_station.flight_samples_parser   import FlightSamplesParser
from ground_station.health_log_parser       import HealthLogParser
from ground_station.ollama_client           import OllamaClient, OllamaError

log = logging.getLogger(__name__)

# ── System prompt ─────────────────────────────────────────────────────────────

SYSTEM_PROMPT = (
    "You are a flight log analyst for an autonomous LiDAR mapping hexacopter "
    "(DronePi). Generate a structured post-flight report in markdown with "
    "sections: **Summary**, **Anomalies**, **SLAM Quality**, **GPS Quality**, "
    "**System Health**, **Recommendations**. Be concise. Cite specific "
    "timestamps and values from the data. Do not speculate beyond what the "
    "data shows. If a section has no data (e.g. GPS), state that data was "
    "not available for this flight rather than inventing values."
)


# ══════════════════════════════════════════════════════════════════════════════
# ReportGenerator
# ══════════════════════════════════════════════════════════════════════════════

class ReportGenerator:
    """
    Orchestrates parsers → structured JSON → Ollama → markdown report.

    Parameters
    ----------
    ollama_client : OllamaClient
        Injected so the caller (orchestrator) can share one client instance
        and control model selection centrally.

    Usage
    -----
        generator = ReportGenerator(ollama_client=OllamaClient())
        report_md = generator.generate(session_id, artifact_paths)
        # report_md is a markdown string, or None on failure
    """

    def __init__(self, ollama_client: Optional[OllamaClient] = None) -> None:
        self._ollama = ollama_client or OllamaClient()

    # ── Public ────────────────────────────────────────────────────────────────

    def generate(
        self,
        session_id: str,
        artifact_paths: dict[str, Optional[Path]],
    ) -> Optional[str]:
        """
        Run full pipeline: parse → assemble → prompt → LLM → markdown.

        Returns markdown string on success, None on any unrecoverable failure.
        OllamaError is caught here — the daemon logs and moves on.
        """
        structured = self.build_structured(session_id, artifact_paths)
        return self._call_ollama(structured)

    def build_structured(
        self,
        session_id: str,
        artifact_paths: dict[str, Optional[Path]],
    ) -> dict:
        """
        Run all parsers and assemble the structured JSON intermediate.

        Public so test scripts can validate the intermediate without a
        live Ollama instance.
        """
        # ── Flight samples (telemetry: modes, RC, pos error, angvel) ──────
        samples_csv = artifact_paths.get("flight_samples_csv")
        samples_data = FlightSamplesParser(
            samples_csv or "/nonexistent"
        ).parse()

        # ── Bag summary (SLAM quality) ─────────────────────────────────────
        bag_csv  = artifact_paths.get("bag_summary_csv")
        bag_data = BagSummaryParser(bag_csv or "/nonexistent").parse()

        # ── Health log (RPi system health) ────────────────────────────────
        health_csv  = artifact_paths.get("health_log_csv")
        health_data = HealthLogParser(health_csv or "/nonexistent").parse()

        # ── Metadata (mesh output stats) ──────────────────────────────────
        mesh = {"vertex_count": 0, "face_count": 0}
        meta_path = artifact_paths.get("metadata_json")
        if meta_path and Path(meta_path).exists():
            try:
                raw = json.loads(Path(meta_path).read_text(encoding="utf-8"))
                mesh["vertex_count"] = raw.get("vertex_count", 0)
                mesh["face_count"]   = raw.get("face_count",   0)
            except Exception as exc:
                log.warning(f"[Generator] metadata.json parse failed: {exc}")

        # ── Flight record (arm time, duration fallback, end reason) ───────
        start_time = samples_data.get("start_time", "unknown")
        duration_s = samples_data.get("duration_s", 0.0)
        end_reason = "unknown"

        fr_path = artifact_paths.get("flight_record_json")
        if fr_path and Path(fr_path).exists():
            try:
                fr = json.loads(Path(fr_path).read_text(encoding="utf-8"))
                # Flight record arm_time is more precise — use it if samples
                # start_time is missing
                if start_time == "unknown":
                    start_time = fr.get("arm_time_iso", "unknown")
                # duration_s from flight_record is the actual arm→disarm time;
                # samples duration is t_flight_s of last row which may be slightly
                # shorter if the sampler stopped early. Use whichever is larger.
                fr_duration = fr.get("duration_s", 0.0) or 0.0
                if fr_duration > duration_s:
                    duration_s = fr_duration
                end_reason = fr.get("end_reason", "unknown") or "unknown"
            except Exception as exc:
                log.warning(f"[Generator] flight_record.json parse failed: {exc}")

        # ── Assemble ──────────────────────────────────────────────────────
        structured = {
            "flight_id":   session_id,
            "start_time":  start_time,
            "duration_s":  duration_s,
            "end_reason":  end_reason,
            "modes":       samples_data.get("modes", []),
            "ekf2_innovations": samples_data.get("ekf2_innovations", {
                "max": 0.0, "mean": 0.0, "spikes": []
            }),
            # GPS not yet available — placeholder so schema is complete
            "gps": {
                "hdop_max":        None,
                "hdop_mean":       None,
                "fix_loss_events": [],
                "available":       False,
            },
            "slam": {
                "point_count_final": bag_data.get("point_count_final", 0),
                "drift_estimate_m":  bag_data.get("drift_estimate_m"),
                "loop_closures":     bag_data.get("loop_closures", 0),
            },
            # Battery not measured — placeholder
            "battery": {
                "start_v": None,
                "end_v":   None,
                "min_v":   None,
                "available": False,
            },
            "rc_channel_7_events": samples_data.get("rc_channel_7_events", []),
            "rpi_health": {
                "cpu_temp_max_c":    health_data.get("cpu_temp_max_c",    0.0),
                "throttling_events": health_data.get("throttling_events", 0),
                "memory_peak_pct":   health_data.get("memory_peak_pct",   0.0),
            },
            "anomalies": samples_data.get("anomalies", []),
            "mesh": mesh,
        }

        log.info(
            f"[Generator] Structured data assembled for {session_id}: "
            f"duration={duration_s:.1f}s, end_reason={end_reason}, "
            f"points={structured['slam']['point_count_final']}, "
            f"modes={len(structured['modes'])}, "
            f"anomalies={len(structured['anomalies'])}"
        )
        return structured

    # ── Private ───────────────────────────────────────────────────────────────

    def _call_ollama(self, structured: dict) -> Optional[str]:
        """Build the user prompt and call Ollama. Returns markdown or None."""
        user_prompt = (
            f"Flight data:\n```json\n{json.dumps(structured, indent=2)}\n```\n\n"
            "Generate the post-flight report."
        )
        try:
            report = self._ollama.chat(SYSTEM_PROMPT, user_prompt)
            log.info(
                f"[Generator] Report generated for {structured['flight_id']} "
                f"({len(report)} chars)"
            )
            return report
        except OllamaError as exc:
            log.error(
                f"[Generator] Ollama failed for {structured['flight_id']}: {exc}"
            )
            return None
