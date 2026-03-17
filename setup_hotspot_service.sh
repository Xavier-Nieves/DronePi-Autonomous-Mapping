#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# setup_hotspot_service.sh
# Configures the Pi Wi-Fi hotspot (dronepi-ap) as a systemd service
# that starts automatically on every boot before anything else.
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

echo "======================================================="
echo "  DronePi Hotspot Service Setup"
echo "======================================================="

# ── 1. Confirm interface exists ───────────────────────────────────────────────
echo ""
echo "[1/4] Checking Wi-Fi interface..."
if ! ip link show "$IFACE" &>/dev/null; then
    echo "  [FAIL] Interface $IFACE not found."
    echo "  Available interfaces:"
    ip link show | grep -E "^[0-9]+:" | awk '{print "  " $2}'
    echo "  Edit IFACE= in this script and re-run."
    exit 1
fi
echo "  [OK] $IFACE found"

# ── 2. Create systemd service ─────────────────────────────────────────────────
echo ""
echo "[2/4] Creating systemd service..."

cat > /etc/systemd/system/${SERVICE}.service << UNIT
[Unit]
Description=DronePi Wi-Fi Hotspot (dronepi-ap)
# Start after NetworkManager is running
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/bin/nmcli device wifi hotspot \
    ifname ${IFACE} \
    ssid ${SSID} \
    password ${PASSWORD}
ExecStop=/usr/bin/nmcli connection down ${SSID}
# Retry up to 3 times if it fails at boot
Restart=on-failure
RestartSec=5
StartLimitIntervalSec=30
StartLimitBurst=3

[Install]
WantedBy=multi-user.target
UNIT

echo "  [OK] Service file written to /etc/systemd/system/${SERVICE}.service"

# ── 3. Enable and start ───────────────────────────────────────────────────────
echo ""
echo "[3/4] Enabling and starting service..."
systemctl daemon-reload
systemctl enable ${SERVICE}.service
systemctl start ${SERVICE}.service

# Wait a moment for hotspot to come up
sleep 3

# ── 4. Verify ─────────────────────────────────────────────────────────────────
echo ""
echo "[4/4] Verifying hotspot..."
STATUS=$(systemctl is-active ${SERVICE}.service)
if [ "$STATUS" = "active" ]; then
    echo "  [OK] Service is active"
else
    echo "  [WARN] Service status: $STATUS"
    echo "  Check with: sudo journalctl -u ${SERVICE} -n 20"
fi

# Check if hotspot interface has the expected IP
IP=$(ip addr show ${IFACE} 2>/dev/null | grep 'inet ' | awk '{print $2}' | cut -d/ -f1)
if [ -n "$IP" ]; then
    echo "  [OK] Pi IP on hotspot: $IP"
else
    echo "  [WARN] Could not read IP — hotspot may still be initializing"
    echo "  Run: ip addr show ${IFACE}"
fi

echo ""
echo "======================================================="
echo "  Hotspot service setup complete."
echo "======================================================="
echo ""
echo "  SSID     : $SSID"
echo "  Password : $PASSWORD"
echo "  Pi IP    : 10.42.0.1 (standard nmcli hotspot default)"
echo ""
echo "  On your laptop:"
echo "    1. Connect to Wi-Fi: $SSID"
echo "    2. SSH: ssh dronepi@10.42.0.1"
echo ""
echo "  Service commands:"
echo "    sudo systemctl status ${SERVICE}"
echo "    sudo systemctl restart ${SERVICE}"
echo "    sudo journalctl -u ${SERVICE} -f"
echo ""
