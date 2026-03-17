# Pi HTTP Server & Automated Mesh Viewer
## How It Works, Why It Works on Ubuntu, and Full Deployment Guide
**Stack:** Raspberry Pi 5 · Ubuntu 24.04 · Python 3 · Three.js · systemd

---

## Table of Contents

1. [What "the Pi serving files" actually means](#1-what-the-pi-serving-files-actually-means)
2. [How Python's HTTP server works internally](#2-how-pythons-http-server-works-internally)
3. [Does it work on Ubuntu 24.04](#3-does-it-work-on-ubuntu-2404)
4. [The CORS problem and why it matters here](#4-the-cors-problem-and-why-it-matters-here)
5. [The custom CORS-enabled server script](#5-the-custom-cors-enabled-server-script)
6. [Running as a systemd service (auto-start on boot)](#6-running-as-a-systemd-service-auto-start-on-boot)
7. [How the auto-loading viewer fits in](#7-how-the-auto-loading-viewer-fits-in)
8. [The postprocess_mesh.py manifest trigger](#8-the-postprocess_meshpy-manifest-trigger)
9. [Full end-to-end automated flow](#9-full-end-to-end-automated-flow)
10. [Network access — WiFi, SSH tunnel, and laptop setup](#10-network-access--wifi-ssh-tunnel-and-laptop-setup)
11. [Troubleshooting common issues](#11-troubleshooting-common-issues)
12. [Security considerations for field use](#12-security-considerations-for-field-use)

---

## 1. What "the Pi serving files" actually means

When we say "the Pi serves the viewer and PLY files," this is what is physically happening:

```
LAPTOP BROWSER                        RASPBERRY PI 5
─────────────────                     ──────────────────────────────
Chrome tab                            Ubuntu 24.04 process running
  │                                   Python HTTP server on port 8080
  │  HTTP GET /meshview.html           │
  │ ──────────────────────────────────>│
  │                                    │  reads /mnt/ssd/maps/meshview.html
  │  200 OK + HTML file content        │  from the NVMe SSD
  │ <──────────────────────────────────│
  │                                    │
  │  HTTP GET /latest.json             │
  │ ──────────────────────────────────>│
  │                                    │  reads /mnt/ssd/maps/latest.json
  │  200 OK + JSON manifest            │
  │ <──────────────────────────────────│
  │                                    │
  │  HTTP GET /flight_001_textured.ply │
  │ ──────────────────────────────────>│
  │                                    │  reads PLY from SSD, streams bytes
  │  200 OK + binary PLY data          │
  │ <──────────────────────────────────│
  │                                    │
  Three.js parses the PLY              │
  WebGL renders the mesh               │
  (all on laptop GPU)                  │
```

**The Pi's only job is reading files from disk and sending their bytes over the network.**
It does zero rendering. All 3D processing happens on your laptop's GPU via WebGL.
The Pi is acting as a very simple **file server** — the same concept as a web server,
just without any dynamic logic.

---

## 2. How Python's HTTP server works internally

Python ships with a built-in HTTP server in its standard library — no installation needed.

### The basic command
```bash
python3 -m http.server 8080 --directory /mnt/ssd/maps
```

Breaking this down:
- `python3 -m http.server` — runs the `http.server` module directly as a script
- `8080` — the TCP port to listen on (any port above 1024 works without root)
- `--directory /mnt/ssd/maps` — serve files from this directory as the root

### What it does when a browser requests a file
1. Browser opens a TCP connection to `<pi-ip>:8080`
2. Browser sends an HTTP GET request: `GET /meshview.html HTTP/1.1`
3. Python maps that path to `/mnt/ssd/maps/meshview.html` on disk
4. Python reads the file and sends it back with appropriate headers
5. TCP connection closes (or is reused for the next request)

### What Python's http.server handles automatically
- Serving any file type by extension (`.html`, `.json`, `.ply`, `.js`)
- Setting correct `Content-Type` headers (`text/html`, `application/json`, `application/octet-stream`)
- Directory listings if no `index.html` is present
- Streaming large files in chunks (important for large PLY files)
- Multiple concurrent connections

### What it does NOT handle by default
- CORS headers (critical — explained in section 4)
- HTTPS/TLS encryption
- Authentication
- Compression

For a local network drone field deployment, the missing HTTPS and auth are acceptable.
CORS must be added manually — that is the only required fix.

---

## 3. Does it work on Ubuntu 24.04?

**Yes, completely and natively.** Here is why there are zero compatibility issues:

### Python 3 is pre-installed on Ubuntu 24.04
```bash
# Verify on your Pi
python3 --version
# Python 3.12.x  — ships with Ubuntu 24.04, no install needed
```

The `http.server` module is part of Python's standard library — it installs with
Python itself. There is nothing to `apt install`, no virtual environment needed,
no pip packages. It works identically on Ubuntu 24.04 arm64 (your Pi) as it does
on any other Linux platform.

### Ubuntu 24.04 specifics that are relevant
- Python 3.12 is the default — `http.server` is fully supported
- The `--directory` flag (required to serve from a specific path) was added in
  Python 3.7, so it works on any modern Ubuntu
- Ubuntu uses **systemd** as its init system — the correct way to auto-start the
  server on boot (covered in section 6)
- UFW firewall may block port 8080 by default — one command to open it (below)

### Open port 8080 in Ubuntu's firewall
```bash
# Allow incoming connections on port 8080
sudo ufw allow 8080/tcp
sudo ufw status  # verify
```

If you prefer to restrict access to your local WiFi subnet only:
```bash
# Allow only from 192.168.1.0/24 (adjust to your network)
sudo ufw allow from 192.168.1.0/24 to any port 8080
```

---

## 4. The CORS problem and why it matters here

This is the single most important technical detail to understand about the server setup.

### What CORS is
Cross-Origin Resource Sharing (CORS) is an HTTP-header based mechanism that allows a server to indicate any origins (domain, scheme, or port) other than its own from which a browser should permit loading resources.

### Why it blocks your viewer
When your laptop browser loads `meshview.html` from `http://192.168.1.100:8080`,
the page's **origin** is `http://192.168.1.100:8080`.

When the JavaScript inside that page then does `fetch('http://192.168.1.100:8080/latest.json')`,
the browser checks: is this the same origin? Yes — same IP, same port. No CORS needed.

**However**, if you ever open the HTML file directly from your laptop's filesystem
(`file:///home/xavier/meshview.html`) and it tries to fetch from the Pi, the
browser sees two different origins and blocks the request with:

```
Access to fetch at 'http://192.168.1.100:8080/latest.json' from origin 'null'
has been blocked by CORS policy: No 'Access-Control-Allow-Origin' header
is present on the requested resource.
```

The `Access-Control-Allow-Origin` header must be set by the server to tell the browser which origins are allowed to access the resource.

### The practical rule for your deployment
As long as you always open the viewer by navigating to
`http://<pi-ip>:8080/meshview.html` in the browser (not opening a local file),
CORS is not an issue. The origin matches.

But to make the setup robust — so it works from any access pattern — the server
should add CORS headers. Python's default `http.server` does not add them.
The fix is a small custom server script.

---

## 5. The custom CORS-enabled server script

Replace the bare `python3 -m http.server` command with this script.
It extends Python's built-in server with CORS headers and proper PLY MIME type.

Save this to the Pi at `/mnt/ssd/maps/serve.py`:

```python
#!/usr/bin/env python3
"""
drone_mesh_server.py
CORS-enabled HTTP file server for the drone mesh viewer.
Serves .ply files, latest.json manifest, and meshview.html
from /mnt/ssd/maps/ on port 8080.

Ubuntu 24.04 compatible — uses only Python 3 standard library.
"""

import sys
import os
from http.server import SimpleHTTPRequestHandler, HTTPServer

SERVE_DIR  = '/mnt/ssd/maps'
PORT       = 8080
BIND_ADDR  = '0.0.0.0'  # listen on all network interfaces


class CORSRequestHandler(SimpleHTTPRequestHandler):
    """
    Extends SimpleHTTPRequestHandler to add:
    - Access-Control-Allow-Origin: * (allow any browser origin)
    - Correct Content-Type for .ply files
    - Cache-Control: no-store for latest.json (always fresh)
    - Suppressed access log noise for polling requests
    """

    def end_headers(self):
        # CORS — allow any origin to fetch these files
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

        # Prevent browser from caching latest.json
        # so the polling loop always sees fresh data
        if self.path.startswith('/latest.json'):
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        else:
            # PLY files are large and don't change — allow caching
            self.send_header('Cache-Control', 'public, max-age=3600')

        super().end_headers()

    def do_OPTIONS(self):
        # Handle CORS preflight requests
        self.send_response(200)
        self.end_headers()

    def guess_type(self, path):
        # Add .ply MIME type — not in Python's default mimetype table
        if str(path).endswith('.ply'):
            return 'application/octet-stream'
        return super().guess_type(path)

    def log_message(self, format, *args):
        # Suppress repetitive polling noise (/latest.json every 10s)
        # but keep PLY transfer logs visible
        if '/latest.json' in args[0] if args else False:
            return  # silent
        super().log_message(format, *args)


def main():
    os.chdir(SERVE_DIR)
    server = HTTPServer((BIND_ADDR, PORT), CORSRequestHandler)
    print(f'[drone-mesh-server] Serving {SERVE_DIR} on http://0.0.0.0:{PORT}')
    print(f'[drone-mesh-server] Open on laptop: http://<pi-ip>:{PORT}/meshview.html')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n[drone-mesh-server] Stopped.')
        server.server_close()


if __name__ == '__main__':
    main()
```

### Test it manually first
```bash
python3 /mnt/ssd/maps/serve.py
# Output:
# [drone-mesh-server] Serving /mnt/ssd/maps on http://0.0.0.0:8080
# Open on laptop: http://192.168.1.100:8080/meshview.html
```

Open `http://<pi-ip>:8080/meshview.html` on your laptop — it should load.
`Ctrl+C` to stop.

---

## 6. Running as a systemd service (auto-start on boot)

On Ubuntu 24.04, **systemd** is the init system — it manages everything that
starts when the Pi boots. By default, Python's HTTP server won't start automatically if your system reboots or if the server crashes. systemd services fix this by making sure the server starts on boot and restarts if it crashes.

### Create the service unit file

```bash
sudo nano /etc/systemd/system/drone-mesh-server.service
```

Paste this content:

```ini
[Unit]
Description=Drone Mesh Viewer HTTP Server
# Start after network is up — required so the Pi has an IP before listening
After=network.target
# Optional: also wait for the SSD to be mounted
After=mnt-ssd.mount

[Service]
# Run as your user, not root — safer and has access to /mnt/ssd
User=dronepi
Group=dronepi

# The server script
ExecStart=/usr/bin/python3 /mnt/ssd/maps/serve.py

# Working directory — files are served relative to this
WorkingDirectory=/mnt/ssd/maps

# Restart automatically if it crashes
# on-failure = restart only on non-zero exit code (not on clean stop)
Restart=on-failure
RestartSec=5

# Log output to journald (viewable with journalctl)
StandardOutput=journal
StandardError=journal

[Install]
# Start during normal multi-user boot (not just single-user recovery mode)
WantedBy=multi-user.target
```

### Enable and start the service

```bash
# Reload systemd to recognize the new file
sudo systemctl daemon-reload

# Enable = auto-start on every boot
sudo systemctl enable drone-mesh-server.service

# Start it right now without rebooting
sudo systemctl start drone-mesh-server.service

# Verify it is running
sudo systemctl status drone-mesh-server.service
```

Expected output:
```
● drone-mesh-server.service - Drone Mesh Viewer HTTP Server
     Loaded: loaded (/etc/systemd/system/drone-mesh-server.service; enabled)
     Active: active (running) since ...
   Main PID: 12345 (python3)
```

### View logs
```bash
# Full log history
journalctl -u drone-mesh-server.service

# Follow live (useful during testing)
journalctl -u drone-mesh-server.service -f

# Last 50 lines
journalctl -u drone-mesh-server.service -n 50
```

### Control commands
```bash
sudo systemctl stop    drone-mesh-server.service  # stop
sudo systemctl restart drone-mesh-server.service  # restart
sudo systemctl disable drone-mesh-server.service  # remove from boot
```

### Why systemd and not crontab or rc.local
systemd is the correct approach on Ubuntu 24.04 for three reasons:
- `@reboot` crontab has no dependency ordering — it may run before the SSD is mounted
- `rc.local` is deprecated on systemd-based systems
- systemd can automatically restart your daemon when it exits with a non-zero exit code, which crontab and rc.local cannot do

---

## 7. How the auto-loading viewer fits in

The viewer HTML file (`meshview.html`) lives on the Pi at `/mnt/ssd/maps/`.
When you open it in a browser, it immediately starts **polling** the Pi for
`latest.json` every 10 seconds.

```
Browser loads meshview.html from Pi
         │
         ▼
JavaScript starts poll loop (every 10s)
         │
         ├── fetch http://<pi-ip>:8080/latest.json
         │        │
         │        ├── 404 Not Found  →  "WAITING FOR OUTPUT" screen stays,
         │        │                      try again in 10s
         │        │
         │        └── 200 OK + JSON  →  read manifest.file field
         │                               │
         │                               ├── same file as last load?
         │                               │   → do nothing, already showing it
         │                               │
         │                               └── new file?
         │                                   → fetch the .ply, render it
         │
         └── repeat every 10 seconds
```

This means:
- You can open the viewer **before** the flight even lands
- It will sit on the "waiting" screen automatically
- The moment `postprocess_mesh.py` finishes and writes `latest.json`,
  the viewer detects it within 10 seconds and loads the mesh with no
  action required from you

### Manual override is still available
The `OPEN PLY` button and drag-and-drop still work — useful for loading
an older flight's output without triggering a new processing run.
The `↺ RELOAD` button resets the loaded file and restarts the poll loop.

---

## 8. The postprocess_mesh.py manifest trigger

Add this block at the **very end** of `postprocess_mesh.py`, after the PLY
has been written successfully. This is what the viewer polls for.

```python
import json
import time
from pathlib import Path

# --- Add at the end of run_pipeline(), after o3d.io.write_triangle_mesh() ---

MAPS_DIR = Path('/mnt/ssd/maps')
MAPS_DIR.mkdir(parents=True, exist_ok=True)

# Copy the output PLY into the maps directory so the server can serve it
import shutil
output_filename = f'{flight_name}_textured.ply'
served_ply = MAPS_DIR / output_filename
shutil.copy2(str(output_ply), str(served_ply))

# Write the manifest — this is what triggers the browser to auto-load
manifest = {
    'file': output_filename,          # filename relative to server root
    'flight': flight_name,            # e.g. "flight_001"
    'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
    'points': len(pcd.points),        # point count for display
    'faces': 0,                       # update if you have face count
    'status': 'complete'
}

manifest_path = MAPS_DIR / 'latest.json'
with open(manifest_path, 'w') as f:
    json.dump(manifest, f, indent=2)

print(f'[postprocess] Manifest written → {manifest_path}')
print(f'[postprocess] PLY served at → http://<pi-ip>:8080/{output_filename}')
print(f'[postprocess] Viewer will auto-load within 10 seconds.')
```

### What latest.json looks like
```json
{
  "file": "flight_001_textured.ply",
  "flight": "flight_001",
  "timestamp": "2026-03-16T14:32:07",
  "points": 3241856,
  "faces": 6483200,
  "status": "complete"
}
```

### Directory layout after a complete flight
```
/mnt/ssd/maps/
├── serve.py                      ← CORS server script
├── meshview.html                 ← auto-loading Three.js viewer
├── latest.json                   ← written by postprocess_mesh.py
├── flight_001_textured.ply       ← served to browser automatically
├── flight_002_textured.ply       ← previous flights remain available
└── flight_003_textured.ply
```

---

## 9. Full end-to-end automated flow

```
1. Pi boots
   └── systemd starts drone-mesh-server.service automatically
       └── Python server listens on port 8080 serving /mnt/ssd/maps/

2. You open laptop browser → http://<pi-ip>:8080/meshview.html
   └── Viewer loads, shows "WAITING FOR OUTPUT" screen
   └── Poll loop starts: checks latest.json every 10 seconds

3. Flight executes (QGC → OFFBOARD → scanning → landing)
   └── Rosbag recorded to /mnt/ssd/rosbags/flight_003/

4. postprocess_mesh.py runs on Pi after landing
   Step 1: Load PCD from rosbag/saved file
   Step 2: Voxel downsample
   Step 3: Estimate normals
   Step 4: Poisson reconstruction
   Step 5: Trim floaters
   Step 6: Texture projection from IMX477 images
   Step 7: Save PLY to /mnt/ssd/maps/flight_003_textured.ply
   Step 8: Write /mnt/ssd/maps/latest.json  ← TRIGGER

5. Browser polls → latest.json exists → new file detected
   └── Header bar updates: flight name, timestamp, point count
   └── Progress bar fills as PLY downloads from Pi to laptop
   └── Three.js parses binary PLY
   └── Textured mesh renders on laptop GPU
   └── Foxglove bridge still running: full ROS data available separately
```

**Zero manual steps required after landing.** Open the browser before the
flight, walk away, and the mesh appears automatically.

---

## 10. Network access — WiFi, SSH tunnel, and laptop setup

### Same WiFi network (standard field setup)
```bash
# Find Pi's IP on your network
# On Pi:
ip addr show wlan0 | grep 'inet '
# e.g.: inet 192.168.1.100/24

# On laptop browser:
http://192.168.1.100:8080/meshview.html

# Or use Pi's hostname if mDNS is working:
http://dronepi.local:8080/meshview.html
```

### Passing the Pi IP to the viewer via URL parameter
The viewer reads a `?host=` URL parameter so you do not need to edit the HTML:
```
http://192.168.1.100:8080/meshview.html?host=192.168.1.100:8080
```

### SSH tunnel (if not on same WiFi — e.g. remote debugging)
```bash
# On laptop — forward Pi's port 8080 to localhost:8080
ssh -NL 8080:localhost:8080 dronepi@<pi-ip>

# Then open on laptop:
http://localhost:8080/meshview.html
```

The `-N` flag means no shell, just the tunnel. Run it in a background terminal.

### mDNS hostname setup (optional quality-of-life)
Ubuntu 24.04 supports Avahi mDNS — lets you use `dronepi.local` instead of
memorizing the IP:
```bash
# On Pi — set hostname
sudo hostnamectl set-hostname dronepi

# Install avahi if not present
sudo apt install avahi-daemon

# On laptop:
http://dronepi.local:8080/meshview.html
```

---

## 11. Troubleshooting common issues

### Browser shows "This site can't be reached"
```bash
# 1. Check server is running
sudo systemctl status drone-mesh-server.service

# 2. Check it is listening on the right port
ss -tlnp | grep 8080

# 3. Check firewall
sudo ufw status
sudo ufw allow 8080/tcp

# 4. Check Pi IP
ip addr show wlan0
```

### latest.json returns 404 (viewer stays on waiting screen forever)
```bash
# Check the file exists
ls -la /mnt/ssd/maps/latest.json

# Check postprocess_mesh.py completed without error
# It only writes latest.json on success — check for exceptions in output
```

### PLY loads but mesh is invisible / all black
This means the PLY has no vertex colors (no RGB data from texture projection).
The viewer will apply a depth colormap automatically as fallback. Check that
`postprocess_mesh.py` texture projection step ran correctly.

### CORS error in browser console
If you see `blocked by CORS policy` in Chrome DevTools:
```bash
# Make sure you are using serve.py, not the bare python3 -m http.server command
# The bare server has no CORS headers

# Restart with the correct script
sudo systemctl restart drone-mesh-server.service
journalctl -u drone-mesh-server -n 20
```

### SSD not mounted when server starts on boot
Add `After=mnt-ssd.mount` to the service file (already included in section 6).
Also verify your `/etc/fstab` has the `nofail` option for the SSD mount so
a missing SSD does not prevent boot entirely.

### Port 8080 already in use
```bash
# Find what is using port 8080
sudo ss -tlnp | grep 8080
# or
sudo lsof -i :8080

# Kill it or change the port in serve.py and the service file
```

---

## 12. Security considerations for field use

The server as configured is intentionally simple and open — appropriate for a
controlled field deployment on a trusted local network. Be aware of the following:

### What is exposed
- All files in `/mnt/ssd/maps/` are publicly readable by anyone on the WiFi
- This includes all historical PLY files from previous flights
- Directory listing is enabled (Python's default) — anyone can browse the file list

### For the capstone demo (acceptable risk)
The field network is controlled (your own hotspot or university WiFi during demo).
No sensitive data is in `/mnt/ssd/maps/` beyond the survey mesh output.
The `Access-Control-Allow-Origin: *` header is safe here because there are no
authentication cookies involved — CORS wildcard + credentials is the dangerous
combination, not CORS wildcard alone.

### If you want to harden it slightly
```python
# In serve.py — disable directory listing
class CORSRequestHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        # Block directory listing requests
        if os.path.isdir(self.translate_path(self.path)):
            self.send_error(403, 'Directory listing disabled')
            return
        super().do_GET()
```

```bash
# Restrict firewall to specific laptop IP only
sudo ufw allow from 192.168.1.50 to any port 8080  # replace with laptop IP
```

### What does NOT need to be secured for this use case
- HTTPS is not needed on a local LAN — TLS adds complexity with no benefit
  when the network is physically controlled
- Authentication is not needed — the data is non-sensitive survey output

---

## Summary — Why This Works on Ubuntu 24.04

| Requirement | How it is met |
|---|---|
| Python 3 available | Pre-installed on Ubuntu 24.04, version 3.12 |
| `http.server` module available | Standard library — zero install |
| `--directory` flag | Supported since Python 3.7, works on 3.12 |
| Auto-start on boot | systemd service with `WantedBy=multi-user.target` |
| Auto-restart on crash | `Restart=on-failure` in service file |
| CORS for browser fetch | Custom `CORSRequestHandler` adds headers |
| Firewall | UFW `allow 8080/tcp` — one command |
| SSD dependency | `After=mnt-ssd.mount` in service file |
| Logs | `journalctl -u drone-mesh-server` |

The entire server stack is **zero external dependencies** — Python standard
library only. Nothing to install, nothing to version-manage, nothing that can
break due to an Ubuntu package update. This makes it appropriate for an
embedded system that may not have reliable internet access in the field.

---

*Last updated: March 2026*
