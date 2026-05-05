"""
ground_station/report_generator.py — Post-flight narrative report generator.

Responsibilities
----------------
1. Accept the artifact paths dict from ArtifactFetcher.
2. Run ULogParser, BagSummaryParser, HealthLogParser on their respective files.
3. Merge all parser outputs into the structured JSON intermediate defined in
   the handoff document (§8).
4. Build the system prompt and user prompt.
5. Call OllamaClient.chat() and return the markdown report string.

The LLM is a writer, not a pilot. It receives only structured data and
produces only a markdown narrative. No flight decisions are involved.

Structured JSON intermediate schema (matches handoff §8)
---------------------------------------------------------
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

System prompt (fixed, from handoff §8)
---------------------------------------
"You are a flight log analyst for an autonomous LiDAR mapping hexacopter
(DronePi). Generate a structured post-flight report in markdown with sections:
Summary, Anomalies, SLAM Quality, GPS Quality, System Health, Recommendations.
Be concise. Cite specific timestamps and values from the data. Do not speculate
beyond what the data shows."

References
----------
- Handoff document §8: structured JSON intermediate and system prompt.
- ULogParser, BagSummaryParser, HealthLogParser: this module.
- OllamaClient: ground_station/ollama_client.py.
"""

import json
import logging
from pathlib import Path
from typing import Optional

from ground_station.bag_summary_parser import BagSummaryParser
from ground_station.health_log_parser import HealthLogParser
from ground_station.ollama_client import OllamaClient, OllamaError
from ground_station.ulog_parser import ULogParser, ULogParseError

log = logging.getLogger(__name__)

# ── System prompt — fixed, from handoff §8 ────────────────────────────────────

SYSTEM_PROMPT = (
    "You are a flight log analyst for an autonomous LiDAR mapping hexacopter "
    "(DronePi). Generate a structured post-flight report in markdown with "
    "sections: **Summary**, **Anomalies**, **SLAM Quality**, **GPS Quality**, "
    "**System Health**, **Recommendations**. Be concise. Cite specific "
    "timestamps and values from the data. Do not speculate beyond what the "
    "data shows."
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

        Public so test_report_generator.py can validate the intermediate
        without a live Ollama instance.
        """
        # ── ULog ──────────────────────────────────────────────────────────
        ulg_path = artifact_paths.get("ulg_path")
        ulog_data: dict = {}
        if ulg_path and Path(ulg_path).exists():
            try:
                ulog_data = ULogParser(ulg_path).parse()
                log.info(f"[Generator] ULog parsed for {session_id}")
            except ULogParseError as exc:
                log.warning(f"[Generator] ULog parse failed: {exc}")

        # ── Bag summary ───────────────────────────────────────────────────
        bag_csv = artifact_paths.get("bag_summary_csv")
        bag_data = BagSummaryParser(bag_csv or "/nonexistent").parse()

        # ── Health log ────────────────────────────────────────────────────
        health_csv = artifact_paths.get("health_log_csv")
        health_data = HealthLogParser(health_csv or "/nonexistent").parse()

        # ── Metadata (mesh stats) ─────────────────────────────────────────
        mesh = {"vertex_count": 0, "face_count": 0}
        meta_path = artifact_paths.get("metadata_json")
        if meta_path and Path(meta_path).exists():
            try:
                raw = json.loads(Path(meta_path).read_text(encoding="utf-8"))
                mesh["vertex_count"] = raw.get("vertex_count", 0)
                mesh["face_count"]   = raw.get("face_count", 0)
            except Exception as exc:
                log.warning(f"[Generator] metadata.json parse failed: {exc}")

        # ── Flight record (session metadata) ─────────────────────────────
        start_time = ulog_data.get("start_time", "unknown")
        duration_s = ulog_data.get("duration_s", 0.0)
        fr_path = artifact_paths.get("flight_record_json")
        if fr_path and Path(fr_path).exists():
            try:
                fr = json.loads(Path(fr_path).read_text(encoding="utf-8"))
                if start_time == "unknown":
                    start_time = fr.get("arm_time_iso", "unknown")
                if duration_s == 0.0:
                    duration_s = fr.get("duration_s", 0.0) or 0.0
            except Exception as exc:
                log.warning(f"[Generator] flight_record.json parse failed: {exc}")

        # ── Assemble ──────────────────────────────────────────────────────
        structured = {
            "flight_id":   session_id,
            "start_time":  start_time,
            "duration_s":  duration_s,
            "modes":       ulog_data.get("modes", []),
            "ekf2_innovations": ulog_data.get("ekf2_innovations", {
                "max": 0.0, "mean": 0.0, "spikes": []
            }),
            "gps": ulog_data.get("gps", {
                "hdop_max": 0.0, "hdop_mean": 0.0, "fix_loss_events": []
            }),
            "slam": {
                "point_count_final": bag_data.get("point_count_final", 0),
                "drift_estimate_m":  bag_data.get("drift_estimate_m"),
                "loop_closures":     bag_data.get("loop_closures", 0),
            },
            "battery": ulog_data.get("battery", {
                "start_v": 0.0, "end_v": 0.0, "min_v": 0.0
            }),
            "rc_channel_7_events": ulog_data.get("rc_channel_7_events", []),
            "rpi_health": {
                "cpu_temp_max_c":    health_data.get("cpu_temp_max_c", 0.0),
                "throttling_events": health_data.get("throttling_events", 0),
                "memory_peak_pct":   health_data.get("memory_peak_pct", 0.0),
            },
            "anomalies": ulog_data.get("anomalies", []),
            "mesh": mesh,
        }

        log.info(
            f"[Generator] Structured data assembled for {session_id}: "
            f"duration={duration_s}s, "
            f"points={structured['slam']['point_count_final']}, "
            f"anomalies={len(structured['anomalies'])}"
        )
        return structured

    # ── Private ───────────────────────────────────────────────────────────────

    def _call_ollama(self, structured: dict) -> Optional[str]:
        """
        Build the user prompt and call Ollama. Returns markdown or None.
        """
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
            log.error(f"[Generator] Ollama failed for {structured['flight_id']}: {exc}")
            return None
