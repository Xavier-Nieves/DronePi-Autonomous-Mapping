#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# setup_hotspot_service.sh
# Configures the Pi Wi-Fi hotspot (dronepi-ap) as a persistent systemd service
# backed by hotspot_watchdog.py — monitors wlan0 every 10s and auto-recovers
# if the hotspot drops. Replaces the old oneshot service.
#
# Run once with sudo:
#   sudo bash setup_hotspot_service.sh
#
# After running:
#   - Hotspot SSID : dronepi-ap
#   - Password     : dronepi123
#   - Pi IP        : 10.42.0.1
#   - SSH from laptop: ssh dronepi@10.42.0.1
# ─────────────────────────────────────────────────────────────────────────────

set -e

SSID="dronepi-ap"
PASSWORD="dronepi123"
IFACE="wlan0"
SERVICE="dronepi-hotspot"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATCHDOG_SRC="$SCRIPT_DIR/hotspot_watchdog.py"
WATCHDOG_DST="/home/dronepi/unitree_lidar_project/rpi_server/hotspot_watchdog.py"

echo "======================================================="
echo "  DronePi Hotspot Service Setup (Watchdog)"
echo "======================================================="

# ── 1. Confirm interface exists ───────────────────────────────────────────────
echo ""
echo "[1/5] Checking Wi-Fi interface..."
if ! ip link show "$IFACE" &>/dev/null; then
    echo "  [FAIL] Interface $IFACE not found."
    echo "  Available interfaces:"
    ip link show | grep -E "^[0-9]+:" | awk '{print "  " $2}'
    echo "  Edit IFACE= in this script and re-run."
    exit 1
fi
echo "  [OK] $IFACE found"

# ── 2. Ensure WPA2 hotspot connection profile exists ─────────────────────────
echo ""
echo "[2/5] Ensuring WPA2 hotspot connection profile..."

# Delete any legacy hotspot profiles that may conflict
for OLD in "Hotspot" "dronepi-ap"; do
    if nmcli connection show "$OLD" &>/dev/null 2>&1; then
        nmcli connection delete "$OLD" 2>/dev/null || true
        echo "  [INFO] Deleted legacy profile: $OLD"
    fi
done

# Create fresh WPA2 profile if it does not exist
if ! nmcli connection show "$SERVICE" &>/dev/null 2>&1; then
    nmcli connection add \
        type wifi ifname "$IFACE" mode ap \
        con-name "$SERVICE" ssid "$SSID" \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$PASSWORD" \
        wifi-sec.proto rsn \
        wifi-sec.pairwise ccmp \
        wifi-sec.group ccmp \
        ipv4.method shared
    echo "  [OK] WPA2 hotspot profile created: $SERVICE"
else
    echo "  [OK] Profile already exists: $SERVICE"
fi

# ── 3. Install watchdog script ────────────────────────────────────────────────
echo ""
echo "[3/5] Installing hotspot_watchdog.py..."
if [ ! -f "$WATCHDOG_SRC" ]; then
    echo "  [FAIL] hotspot_watchdog.py not found at $WATCHDOG_SRC"
    exit 1
fi
cp "$WATCHDOG_SRC" "$WATCHDOG_DST"
chown dronepi:dronepi "$WATCHDOG_DST"
chmod 755 "$WATCHDOG_DST"
echo "  [OK] Watchdog installed at $WATCHDOG_DST"

# ── 4. Create persistent systemd service ─────────────────────────────────────
echo ""
echo "[4/5] Creating persistent systemd service..."

cat > /etc/systemd/system/${SERVICE}.service << UNIT
[Unit]
Description=DronePi Wi-Fi Hotspot (dronepi-ap) — persistent watchdog
After=NetworkManager.service network.target
Wants=NetworkManager.service

[Service]
Type=simple
User=root
ExecStart=/usr/bin/python3 ${WATCHDOG_DST}
Restart=always
RestartSec=5

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=dronepi-hotspot

[Install]
WantedBy=multi-user.target
UNIT

echo "  [OK] Service file written to /etc/systemd/system/${SERVICE}.service"

# ── 5. Enable and start ───────────────────────────────────────────────────────
echo ""
echo "[5/5] Enabling and starting service..."
systemctl daemon-reload
systemctl enable ${SERVICE}.service

# Stop old instance if running before starting new one
systemctl stop ${SERVICE}.service 2>/dev/null || true
sleep 1
systemctl start ${SERVICE}.service

# Wait for watchdog to bring hotspot up
sleep 5

# ── Verify ────────────────────────────────────────────────────────────────────
STATUS=$(systemctl is-active ${SERVICE}.service)
if [ "$STATUS" = "active" ]; then
    echo "  [OK] Watchdog service is active"
else
    echo "  [WARN] Service status: $STATUS"
    echo "  Check with: sudo journalctl -u ${SERVICE} -n 20"
fi

IP=$(ip addr show ${IFACE} 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d/ -f1)
if [ -n "$IP" ]; then
    echo "  [OK] Pi IP on hotspot: $IP"
else
    echo "  [WARN] IP not yet assigned — watchdog may still be initializing"
    echo "  Run: ip addr show ${IFACE}"
fi

echo ""
echo "======================================================="
echo "  Hotspot service setup complete."
echo "======================================================="
echo ""
echo "  SSID     : $SSID"
echo "  Password : $PASSWORD"
echo "  Pi IP    : 10.42.0.1"
echo ""
echo "  Watchdog : checks wlan0 every 10s, auto-recovers on drop"
echo ""
echo "  On your laptop:"
echo "    1. Connect to Wi-Fi: $SSID"
echo "    2. SSH: ssh dronepi@10.42.0.1"
echo "    3. Browser: http://10.42.0.1:8080/meshview.html"
echo ""
echo "  Service commands:"
echo "    sudo systemctl status ${SERVICE}"
echo "    sudo systemctl restart ${SERVICE}"
echo "    sudo journalctl -u ${SERVICE} -f"
echo ""
