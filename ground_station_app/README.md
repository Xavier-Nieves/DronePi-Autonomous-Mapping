# DronePi Ground Station

Post-flight analysis dashboard for the DronePi autonomous LiDAR mapping drone.
Runs entirely on a Windows laptop — no cloud, no internet required after setup.

---

## What It Does

- **Discovers** completed flights by polling the Raspberry Pi 5 over the local Wi-Fi hotspot
- **Downloads** sensor artifacts (SLAM metrics, health logs, flight records) to a local cache
- **Generates** post-flight reports using a locally hosted AI model (Ollama + qwen2.5:7b)
- **Stores** all flight metrics in a local SQLite database
- **Displays** a browser dashboard with fleet analytics, trend charts, anomaly digests, and per-flight reports

The dashboard is accessible at `http://localhost:8765` from any browser on the machine.

---

## Quick Setup (Fresh Machine)

```powershell
# From inside ground_station_app/
powershell -ExecutionPolicy Bypass -File setup.ps1
```

The setup script handles everything: Python package install, Ollama install, model download, desktop shortcut, and test data generation.

**Requirements before running setup:**
- Windows 10 or 11
- Python 3.11+ — [python.org/downloads](https://python.org/downloads)
- Internet connection for pip and Ollama model download (~4.5 GB one-time)

---

## Manual Setup

If you prefer to run each step yourself:

```powershell
# 1. Install the Python package and all dependencies
cd ground_station_app
pip install -e .

# 2. Install Ollama  (skip if already installed)
winget install Ollama.Ollama

# 3. Download the AI model  (~4.5 GB, one-time)
ollama pull qwen2.5:7b

# 4. Create the desktop shortcut
python ground_station\cli.py install

# 5. Generate test flights for offline testing  (optional)
python gen_test_flights.py

# 6. Start the app
python ground_station\cli.py start
```

After step 4 you can double-click **DronePi Ground Station** on the desktop instead of using the terminal.

---

## Python Dependencies

All declared in `pyproject.toml`. Installed automatically by `pip install -e .`

| Package | Purpose |
|---|---|
| `typer` | CLI framework — all `ground-station` commands |
| `rich` | Coloured terminal output |
| `httpx` | HTTP client — downloads artifacts from RPi, calls Ollama API |
| `pyulog` | Parses PX4 `.ulg` flight controller logs |
| `pandas` | CSV parsing for bag summary and health log files |

**Not in pyproject.toml** (installed separately):

| Tool | Purpose | Install |
|---|---|---|
| Ollama | Runs the AI model locally | `winget install Ollama.Ollama` |
| qwen2.5:7b | The language model used for reports | `ollama pull qwen2.5:7b` |

Everything else (`sqlite3`, `json`, `http.server`, `threading`, `pathlib`) is Python standard library — no install needed.

---

## CLI Commands

```powershell
ground-station start              # Start daemon + UI server in background
ground-station stop               # Stop both background processes
ground-station status             # Show daemon / Ollama / RPi health
ground-station logs               # Print daemon log
ground-station logs -f            # Follow daemon log in real time
ground-station replay <session>   # Re-run analysis for one flight
ground-station install            # Create desktop shortcut + .pyw launcher
ground-station uninstall          # Remove shortcut, stop processes
```

---

## Dashboard

Open `http://localhost:8765` in any browser after starting the app.

| Element | Description |
|---|---|
| **KPI strip** | Total flights · Avg duration · Anomaly count · Reports — always visible |
| **Sidebar** | Full flight list. Click any flight to open its report |
| **⇅ Sync DB** | Scans the local cache and indexes any sessions missing from the database |
| **Dashboard tab** | Fleet Summary (AI) · 4 KPI cards · 4 trend charts · Pipeline bottleneck · Anomaly digest |
| **Report tab** | Per-flight metrics strip · Full AI-generated markdown report |
| **🗺 Mesh Viewer** | Opens the RPi 3D mesh viewer. Links to the selected flight when one is active |

### Sync DB Button

The **⇅ Sync DB** button in the sidebar header is the manual sync trigger. Click it whenever:
- You ran `gen_test_flights.py` to generate test data
- Flights exist in the cache but are missing from the dashboard
- The fleet summary shows wrong counts or "no anomalies" despite anomalies being visible

The sync reads every session folder in `~/.dronepi-ground/cache/` and upserts anything missing into the database. It is safe to run repeatedly — existing records are never overwritten.

---

## File Layout

```
ground_station_app/
├── ground_station/
│   ├── cli.py                # CLI entry point and .pyw launcher generator
│   ├── orchestrator.py       # FlightReportDaemon background loop
│   ├── flight_watcher.py     # Polls RPi /api/flights for new sessions
│   ├── artifact_fetcher.py   # Downloads files from RPi to local cache
│   ├── report_generator.py   # Parses artifacts → structured JSON → Ollama → markdown
│   ├── ollama_client.py      # HTTP client for Ollama REST API
│   ├── report_publisher.py   # Saves report locally + PUTs to RPi
│   ├── db.py                 # FlightDatabase SQLite backend
│   ├── ui_server.py          # HTTP server for dashboard API
│   ├── dashboard.html        # Browser SPA — served from disk
│   └── process_manager.py    # Spawn / stop / PID tracking for both processes
├── gen_test_flights.py       # Generates 10 synthetic flights for offline testing
├── setup.ps1                 # One-shot Windows setup script
├── pyproject.toml            # Package metadata and dependencies
└── README.md                 # This file
```

---

## Local Data Storage

Everything the app writes lives in `%USERPROFILE%\.dronepi-ground\`:

```
~/.dronepi-ground/
├── dronepi.db          # SQLite database — all flight metrics and reports
├── daemon.pid          # Daemon process ID (written on start, deleted on stop)
├── ui.pid              # UI server process ID
├── daemon.log          # Daemon log output
├── state.json          # Set of already-processed session IDs
└── cache/
    └── scan_YYYYMMDD_HHMMSS/
        ├── flight_record.json   # Session metadata (arm time, duration, end reason)
        ├── bag_summary.csv      # SLAM metrics (points, drift, loop closures)
        ├── health_log.csv       # RPi health samples (CPU temp, throttle, memory)
        ├── metadata.json        # Mesh stats + pipeline stage timings
        └── report.md            # Ollama-generated markdown report
```

---

## Environment Variables

All parameters have working defaults. Override by setting environment variables before launching.

| Variable | Default | Description |
|---|---|---|
| `DRONEPI_RPI_HOST` | `10.42.0.1` | RPi IP on the Wi-Fi hotspot |
| `DRONEPI_RPI_PORT` | `8080` | RPi serve.py HTTP port |
| `DRONEPI_RPI_UPLOAD_TOKEN` | `dronepi` | Bearer token for report upload |
| `DRONEPI_OLLAMA_HOST` | `http://localhost:11434` | Ollama REST API base URL |
| `DRONEPI_OLLAMA_MODEL` | `qwen2.5:7b` | Model tag for every API call |
| `DRONEPI_OLLAMA_TIMEOUT_S` | `90` | Ollama read timeout in seconds |
| `DRONEPI_POLL_INTERVAL_S` | `20` | Daemon poll cycle in seconds |
| `DRONEPI_CACHE_DIR` | `~/.dronepi-ground/cache` | Local artifact cache root |
| `DRONEPI_DB_PATH` | `~/.dronepi-ground/dronepi.db` | SQLite database path |
| `DRONEPI_UI_PORT` | `8765` | Dashboard HTTP port |

---

## RPi Connection

The ground station connects to the RPi over the hotspot the RPi creates at `10.42.0.1`. The RPi must be running `serve.py` on port 8080.

The **RPi** pill in the dashboard header shows the connection state:
- 🟢 Green — RPi reachable, live data available
- 🟡 Yellow — RPi unreachable, dashboard uses local cache only
- 🔴 Red — daemon has errors

The app is fully functional offline — all analysis runs on locally cached files. Only artifact downloads and report uploads require the RPi connection.

---

## Troubleshooting

**Dashboard shows 0 flights after adding test data**
→ Click **⇅ Sync DB** in the sidebar. The database is only populated when the sync runs.

**Fleet summary says "no anomalies" but anomalies are visible**
→ Click **⇅ Sync DB** — the AI summary reads from the database, not the sidebar list.

**Ollama pill is red**
→ Run `ollama serve` in a terminal, or check that Ollama is installed: `ollama --version`

**Report generation times out**
→ Increase the timeout: set `DRONEPI_OLLAMA_TIMEOUT_S=180` before starting

**Daemon log shows "RPi unreachable"**
→ Normal when not connected to the drone hotspot. Cache-only mode works fine.

**"No module named ground_station"**
→ Run `pip install -e .` from inside `ground_station_app/`

---

## Development

```powershell
# Install with dev dependencies (adds pytest)
pip install -e ".[dev]"

# Run the pipeline test suite (requires RPi or mock server)
python tests\test_pipeline.py

# Replay a specific flight without starting the full daemon
python ground_station\cli.py replay scan_20260504_092800

# Watch daemon logs in real time
python ground_station\cli.py logs -f
```

---

*DronePi — Autonomous LiDAR Mapping Drone — UPRM Capstone 2026*
