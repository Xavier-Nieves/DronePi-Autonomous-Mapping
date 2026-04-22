#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# setup_foxglove_service.sh
# Registers foxglove_bridge as a systemd service that starts automatically
# on boot after the hotspot is up.
#
# Run once with sudo:
#   sudo bash setup_foxglove_service.sh
#
# After running:
#   Connect Foxglove Studio to: ws://10.42.0.1:8765
# ─────────────────────────────────────────────────────────────────────────────

set -e

SERVICE="foxglove-bridge"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

echo "======================================================="
echo "  Foxglove Bridge Service Setup"
echo "======================================================="

# ── 1. Verify foxglove_bridge is installed ────────────────────────────────────
echo ""
echo "[1/4] Checking foxglove_bridge installation..."
if ! bash -c "source $ROS_SETUP && ros2 pkg list 2>/dev/null | grep -q foxglove_bridge"; then
    echo "  [FAIL] foxglove_bridge not found."
    echo "  Install with: sudo apt install ros-jazzy-foxglove-bridge"
    exit 1
fi
echo "  [OK] foxglove_bridge found"

# ── 2. Create systemd service ─────────────────────────────────────────────────
echo ""
echo "[2/4] Creating systemd service..."

cat > /etc/systemd/system/${SERVICE}.service << UNIT
[Unit]
Description=Foxglove Bridge — ROS 2 WebSocket server (port 8765)
Documentation=https://foxglove.dev
# Start after hotspot is up so laptop can connect immediately
After=dronepi-hotspot.service network.target
Wants=dronepi-hotspot.service

[Service]
Type=simple
User=dronepi
Environment="HOME=/home/dronepi"
ExecStart=/bin/bash -c "source ${ROS_SETUP} && \
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    port:=8765 \
    address:=0.0.0.0 \
    send_buffer_limit:=100000000 \
    max_qos_depth:=10"
Restart=on-failure
RestartSec=5

# Logging
StandardOutput=journal
StandardError=journal
SyslogIdentifier=foxglove-bridge

[Install]
WantedBy=multi-user.target
UNIT

echo "  [OK] Service file written"

# ── 3. Open firewall port ─────────────────────────────────────────────────────
echo ""
echo "[3/4] Configuring firewall..."
if command -v ufw &>/dev/null; then
    ufw allow 8765/tcp 2>/dev/null || true
    echo "  [OK] UFW: port 8765 allowed"
else
    echo "  [SKIP] UFW not found"
fi

# ── 4. Enable and start ───────────────────────────────────────────────────────
echo ""
echo "[4/4] Enabling and starting service..."
systemctl daemon-reload
systemctl enable ${SERVICE}.service
systemctl start ${SERVICE}.service

sleep 3

STATUS=$(systemctl is-active ${SERVICE}.service)
if [ "$STATUS" = "active" ]; then
    echo "  [OK] Service is active"
else
    echo "  [WARN] Service status: $STATUS"
    echo "  Check: sudo journalctl -u ${SERVICE} -n 30"
fi

echo ""
echo "======================================================="
echo "  Foxglove bridge service setup complete."
echo "======================================================="
echo ""
echo "  WebSocket URL : ws://10.42.0.1:8765"
echo ""
echo "  In Foxglove Studio:"
echo "    Open connection → Foxglove WebSocket → ws://10.42.0.1:8765"
echo ""
echo "  Service commands:"
echo "    sudo systemctl status ${SERVICE}"
echo "    sudo systemctl restart ${SERVICE}"
echo "    sudo journalctl -u ${SERVICE} -f"
echo ""
