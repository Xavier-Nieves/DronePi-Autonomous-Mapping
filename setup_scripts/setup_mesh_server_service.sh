#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# setup_mesh_server_service.sh
# Installs serve.py and dronepi_viewer_ultimate.html to /mnt/ssd/maps/ and registers
# the drone-mesh-server systemd service to auto-start on boot.
#
# Run once with sudo:
#   sudo bash setup_mesh_server_service.sh
# ─────────────────────────────────────────────────────────────────────────────

set -e

MAPS_DIR="/mnt/ssd/maps"
SERVICE="drone-mesh-server"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "======================================================="
echo "  Drone Mesh Server Service Setup"
echo "======================================================="

# ── 1. Verify SSD is mounted ──────────────────────────────────────────────────
echo ""
echo "[1/5] Checking SSD mount..."
if ! mountpoint -q /mnt/ssd; then
    echo "  [FAIL] /mnt/ssd is not mounted."
    echo "  Mount the SSD first: sudo mount -a"
    exit 1
fi
echo "  [OK] SSD mounted at /mnt/ssd"

# ── 2. Create maps directory and copy files ───────────────────────────────────
echo ""
echo "[2/5] Setting up /mnt/ssd/maps/..."
mkdir -p "$MAPS_DIR"

# Copy serve.py
if [ -f "$SCRIPT_DIR/serve.py" ]; then
    cp "$SCRIPT_DIR/serve.py" "$MAPS_DIR/serve.py"
    chmod +x "$MAPS_DIR/serve.py"
    echo "  [OK] serve.py copied"
else
    echo "  [FAIL] serve.py not found at $SCRIPT_DIR/serve.py"
    exit 1
fi

# Copy dronepi_viewer_ultimate.html (unified ground station)
if [ -f "$SCRIPT_DIR/dronepi_viewer_ultimate.html" ]; then
    cp "$SCRIPT_DIR/dronepi_viewer_ultimate.html" "$MAPS_DIR/dronepi_viewer_ultimate.html"
    echo "  [OK] dronepi_viewer_ultimate.html copied"
else
    echo "  [FAIL] dronepi_viewer_ultimate.html not found at $SCRIPT_DIR/dronepi_viewer_ultimate.html"
    exit 1
fi

# Set ownership
chown -R dronepi:dronepi "$MAPS_DIR"
echo "  [OK] Ownership set to dronepi"

# ── 3. Open firewall port ─────────────────────────────────────────────────────
echo ""
echo "[3/5] Configuring firewall..."
if command -v ufw &>/dev/null; then
    ufw allow 8080/tcp 2>/dev/null || true
    echo "  [OK] UFW: port 8080 allowed"
else
    echo "  [SKIP] UFW not found — skipping firewall config"
fi

# ── 4. Create systemd service ─────────────────────────────────────────────────
echo ""
echo "[4/5] Creating systemd service..."

cat > /etc/systemd/system/${SERVICE}.service << UNIT
[Unit]
Description=Drone Mesh HTTP Server (port 8080)
Documentation=http://10.42.0.1:8080/dronepi_viewer_ultimate.html
# Wait for SSD to be mounted and hotspot to be up
After=mnt-ssd.mount dronepi-hotspot.service
Wants=mnt-ssd.mount dronepi-hotspot.service

[Service]
Type=simple
User=dronepi
WorkingDirectory=/mnt/ssd/maps
ExecStart=/usr/bin/python3 /mnt/ssd/maps/serve.py
Restart=on-failure
RestartSec=5

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=drone-mesh-server

[Install]
WantedBy=multi-user.target
UNIT

echo "  [OK] Service file written"

# ── 5. Enable and start ───────────────────────────────────────────────────────
echo ""
echo "[5/5] Enabling and starting service..."
systemctl daemon-reload
systemctl enable ${SERVICE}.service
systemctl start ${SERVICE}.service

sleep 2

STATUS=$(systemctl is-active ${SERVICE}.service)
if [ "$STATUS" = "active" ]; then
    echo "  [OK] Service is active"
else
    echo "  [WARN] Service status: $STATUS"
    echo "  Check: sudo journalctl -u ${SERVICE} -n 30"
fi

echo ""
echo "======================================================="
echo "  Mesh server setup complete."
echo "======================================================="
echo ""
echo "  Serving from : $MAPS_DIR"
echo "  Viewer URL   : http://10.42.0.1:8080/dronepi_viewer_ultimate.html"
echo ""
echo "  Service commands:"
echo "    sudo systemctl status ${SERVICE}"
echo "    sudo systemctl restart ${SERVICE}"
echo "    sudo journalctl -u ${SERVICE} -f"
echo ""
echo "  To test the viewer:"
echo "    1. Connect laptop to dronepi-ap Wi-Fi"
echo "    2. Open: http://10.42.0.1:8080/dronepi_viewer_ultimate.html"
echo "    3. Viewer shows 'WAITING FOR OUTPUT' on 3D tab"
echo "    4. Copy a PLY + write latest.json to trigger auto-load"
echo ""
