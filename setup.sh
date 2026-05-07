#!/bin/bash
# =============================================================================
# DronePi — Master Setup Script
# Fresh Ubuntu 24.04 (arm64) on Raspberry Pi 5 → fully operational
#
# Usage:
#   sudo bash setup_scripts/setup.sh
#
# What this does (in order):
#   [1]  System update + base build tools + gdal-bin + cv-bridge
#   [2]  ROS 2 Jazzy (native)
#   [3]  MAVROS + foxglove-bridge + cv-bridge ROS package
#   [4]  Unitree L1 LiDAR driver + colcon symlink fix
#   [5]  Point-LIO SLAM
#   [6]  Miniforge (Conda) + dronepi environment + all Python deps
#   [7]  udev rules (ttyPixhawk, ttyUSB0, Hailo group, gpio group)
#   [8]  Build ROS 2 workspace
#   [9]  CPU governor → performance
#   [10] Systemd services (hotspot, mesh-server, foxglove, mavros,
#         drone-watchdog, dronepi-main, pointlio-standby,
#         slam-bridge, rpi-health, led, camera-stream, meshview-telemetry)
#
# Run as root (sudo). The script will not proceed without it.
# Re-running is safe — each step checks before acting.
#
# Excluded (run separately if needed):
#   Hailo hailo_inference_env  →  requires hailort wheel from developer.hailo.ai
#   Foxglove Agent             →  requires FOXGLOVE_DEVICE_TOKEN; run install_foxglove_agent.sh
#   SSD /etc/fstab entry       →  UUID is hardware-specific; add manually
# =============================================================================

set -euo pipefail

# ── Guards ────────────────────────────────────────────────────────────────────
if [[ $EUID -ne 0 ]]; then
    echo "[ERROR] Run with sudo: sudo bash setup_scripts/setup.sh"
    exit 1
fi

UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "unknown")
if [[ "$UBUNTU_VERSION" != "24.04" ]]; then
    echo "[WARN] Expected Ubuntu 24.04, detected $UBUNTU_VERSION. Proceeding anyway."
fi

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_WS="$PROJECT_DIR/RPI5/ros2_ws"
FLIGHT_DIR="$PROJECT_DIR/unitree_drone_mapper/flight"
RPI_SERVER_DIR="$PROJECT_DIR/rpi_server"
DRONEPI_USER="dronepi"
DRONEPI_HOME="/home/$DRONEPI_USER"
CONDA_PREFIX="$DRONEPI_HOME/miniforge3"
CONDA_ENV="dronepi"

# ── Colour helpers ────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

step()  { echo -e "\n${CYAN}${BOLD}[$1] $2${NC}"; }
ok()    { echo -e "  ${GREEN}OK${NC}  $1"; }
warn()  { echo -e "  ${YELLOW}WARN${NC} $1"; }
fail()  { echo -e "  ${RED}FAIL${NC} $1"; exit 1; }
skip()  { echo -e "  ${YELLOW}SKIP${NC} $1 (already done)"; }

# ── Banner ────────────────────────────────────────────────────────────────────
echo -e "${BOLD}"
echo "  ╔══════════════════════════════════════════════════════╗"
echo "  ║   DronePi — Master Setup                            ║"
echo "  ║   Ubuntu 24.04 · Raspberry Pi 5 · ROS 2 Jazzy      ║"
echo "  ╚══════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo "  Project : $PROJECT_DIR"
echo "  User    : $DRONEPI_USER"
echo "  ROS2 WS : $ROS2_WS"
echo ""
read -rp "  Continue? (y/N) " REPLY
[[ "$REPLY" =~ ^[Yy]$ ]] || { echo "Aborted."; exit 0; }

# =============================================================================
# [1] System update + base tools
# =============================================================================
step "1/10" "System update + base tools"

apt-get update -qq
apt-get upgrade -y -qq
apt-get install -y -qq \
    git curl wget lsb-release gnupg2 \
    build-essential cmake ninja-build \
    python3-pip python3-venv \
    software-properties-common \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libpcl-dev \
    can-utils \
    net-tools \
    htop \
    rsync \
    gdal-bin

ok "Base tools installed"

# =============================================================================
# [2] ROS 2 Jazzy
# =============================================================================
step "2/10" "ROS 2 Jazzy"

if dpkg -l ros-jazzy-ros-base &>/dev/null; then
    skip "ros-jazzy-ros-base already installed"
else
    add-apt-repository universe -y
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | tee /etc/apt/sources.list.d/ros2.list > /dev/null

    apt-get update -qq
    apt-get install -y -qq \
        ros-jazzy-ros-base \
        ros-jazzy-ros-dev-tools \
        ros-jazzy-pcl-ros \
        ros-jazzy-pcl-conversions \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-visualization-msgs \
        python3-colcon-common-extensions \
        python3-rosdep

    rosdep init 2>/dev/null || true
    sudo -u "$DRONEPI_USER" rosdep update

    ok "ROS 2 Jazzy installed"
fi

# =============================================================================
# [3] MAVROS + foxglove-bridge + cv-bridge
# =============================================================================
step "3/10" "MAVROS + foxglove-bridge + cv-bridge"

if dpkg -l ros-jazzy-mavros &>/dev/null; then
    skip "ros-jazzy-mavros already installed"
else
    apt-get install -y -qq \
        ros-jazzy-mavros \
        ros-jazzy-mavros-msgs \
        ros-jazzy-mavros-extras

    # GeographicLib datasets — required by MAVROS EKF fusion
    /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh || \
        warn "GeographicLib install failed — run manually: /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh"

    ok "MAVROS installed"
fi

if dpkg -l ros-jazzy-foxglove-bridge &>/dev/null; then
    skip "foxglove-bridge already installed"
else
    apt-get install -y -qq ros-jazzy-foxglove-bridge
    ok "foxglove-bridge installed"
fi

if dpkg -l ros-jazzy-cv-bridge &>/dev/null; then
    skip "ros-jazzy-cv-bridge already installed"
else
    apt-get install -y -qq ros-jazzy-cv-bridge python3-cv-bridge
    ok "cv-bridge installed"
fi

# =============================================================================
# [4] Unitree L1 LiDAR driver + colcon symlink fix
# =============================================================================
step "4/10" "Unitree L1 LiDAR driver"

mkdir -p "$ROS2_WS/src"
UNILIDAR_DEST="$ROS2_WS/src/unilidar_sdk"
UNILIDAR_LINK="$ROS2_WS/src/unitree_lidar_ros2"

if [[ ! -d "$UNILIDAR_DEST" ]]; then
    sudo -u "$DRONEPI_USER" git clone \
        https://github.com/unitreerobotics/unilidar_sdk.git \
        "$UNILIDAR_DEST"
    ok "unilidar_sdk cloned"
else
    skip "unilidar_sdk already present"
fi

# Colcon symlink fix — package.xml is nested two levels deep inside the SDK.
PACKAGE_DEEP="$UNILIDAR_DEST/unitree_lidar_ros2/src/unitree_lidar_ros2"
if [[ ! -L "$UNILIDAR_LINK" ]]; then
    sudo -u "$DRONEPI_USER" ln -s "$PACKAGE_DEEP" "$UNILIDAR_LINK"
    ok "colcon symlink created"
else
    skip "colcon symlink already exists"
fi

# =============================================================================
# [5] Point-LIO SLAM
# =============================================================================
step "5/10" "Point-LIO SLAM"

POINTLIO_DIR="$ROS2_WS/src/point_lio_ros2"
if [[ ! -d "$POINTLIO_DIR" ]]; then
    sudo -u "$DRONEPI_USER" git clone \
        https://github.com/dfloreaa/point_lio_ros2.git \
        "$POINTLIO_DIR"
    ok "point_lio_ros2 cloned"
else
    skip "point_lio_ros2 already present"
fi

# =============================================================================
# [6] Miniforge (Conda) + dronepi environment + Python deps
# =============================================================================
step "6/10" "Miniforge + dronepi Conda environment"

# Install Miniforge if not present
if [[ ! -f "$CONDA_PREFIX/bin/conda" ]]; then
    MINIFORGE_INSTALLER="/tmp/Miniforge3-Linux-aarch64.sh"
    curl -fsSL \
        https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh \
        -o "$MINIFORGE_INSTALLER"
    sudo -u "$DRONEPI_USER" bash "$MINIFORGE_INSTALLER" -b -p "$CONDA_PREFIX"
    rm -f "$MINIFORGE_INSTALLER"
    ok "Miniforge installed at $CONDA_PREFIX"
else
    skip "Miniforge already installed"
fi

CONDA_BIN="$CONDA_PREFIX/bin/conda"
CONDA_RUN="sudo -u $DRONEPI_USER $CONDA_PREFIX/bin/conda run -n $CONDA_ENV"

# Create dronepi environment if not present
if ! sudo -u "$DRONEPI_USER" "$CONDA_BIN" env list | grep -q "^$CONDA_ENV "; then
    sudo -u "$DRONEPI_USER" "$CONDA_BIN" create -n "$CONDA_ENV" \
        python=3.12 -y -q
    ok "dronepi Conda environment created"
else
    skip "dronepi Conda environment already exists"
fi

# conda-forge packages (no aarch64 pip wheels for these)
echo "  Installing conda-forge packages..."
sudo -u "$DRONEPI_USER" "$CONDA_BIN" install -n "$CONDA_ENV" -y -q \
    -c conda-forge \
    "open3d>=0.19" \
    "pdal>=3.5" \
    pymeshlab \
    numpy scipy pyyaml psutil

ok "conda-forge packages installed"

# pip packages inside the Conda env
# empy must be pinned to 3.3.4 — ROS 2 Jazzy requires the legacy em module
# API that was removed in empy 4.x. Installing any newer version breaks colcon.
echo "  Installing pip packages inside dronepi env..."
$CONDA_RUN pip install -q \
    "empy==3.3.4" \
    catkin_pkg \
    lark \
    colcon-common-extensions \
    colcon-ros \
    rosdep \
    osrf-pycommon \
    trimesh \
    rosbags \
    websocket-client \
    websockets \
    rpi-lgpio \
    picamera2 \
    piexif \
    "opencv-python-headless>=4.9" \
    "Pillow>=10.0"

ok "pip packages installed inside dronepi env"

# Add conda activate to .bashrc if not already there
BASHRC="$DRONEPI_HOME/.bashrc"
CONDA_INIT_LINE="source $CONDA_PREFIX/etc/profile.d/conda.sh"
CONDA_ACTIVATE_LINE="conda activate $CONDA_ENV"

if ! grep -qF "$CONDA_INIT_LINE" "$BASHRC" 2>/dev/null; then
    {
        echo ""
        echo "# Conda — added by DronePi setup"
        echo "$CONDA_INIT_LINE"
        echo "$CONDA_ACTIVATE_LINE"
    } >> "$BASHRC"
    chown "$DRONEPI_USER:$DRONEPI_USER" "$BASHRC"
    ok "Conda activate added to .bashrc"
else
    skip "Conda activate already in .bashrc"
fi

# Add ROS 2 source lines to .bashrc
ROS_SOURCE="source /opt/ros/jazzy/setup.bash"
WS_SOURCE="source $ROS2_WS/install/setup.bash"

for line in "$ROS_SOURCE" "$WS_SOURCE"; do
    if ! grep -qF "$line" "$BASHRC" 2>/dev/null; then
        echo "$line" >> "$BASHRC"
    fi
done

# =============================================================================
# [7] udev rules
# =============================================================================
step "7/10" "udev rules"

UDEV_FILE="/etc/udev/rules.d/99-dronepi.rules"

tee "$UDEV_FILE" > /dev/null <<'EOF'
# Pixhawk 6X — persistent symlink at /dev/ttyPixhawk
SUBSYSTEM=="tty", KERNEL=="ttyACM*", MODE="0666", GROUP="dialout", SYMLINK+="ttyPixhawk"

# Unitree L1 LiDAR — CP210x USB-to-UART (Silicon Labs, 10c4:ea60)
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", MODE="0666", GROUP="dialout", \
    SYMLINK+="ttyLidar", TAG+="systemd", ENV{SYSTEMD_WANTS}="pointlio-standby.service"

# Hailo-8 PCIe AI HAT — allow dronepi user read access without root
SUBSYSTEM=="hailo_chardev", MODE="0660", GROUP="hailo"
EOF

udevadm control --reload-rules
udevadm trigger

# Groups
usermod -aG dialout "$DRONEPI_USER"
groupadd -f hailo
usermod -aG hailo "$DRONEPI_USER"
groupadd -f gpio
usermod -aG gpio "$DRONEPI_USER"

ok "udev rules written and reloaded"

# =============================================================================
# [8] Build ROS 2 workspace
# =============================================================================
step "8/10" "Build ROS 2 workspace"

mkdir -p "$ROS2_WS"
chown -R "$DRONEPI_USER:$DRONEPI_USER" "$ROS2_WS"

# Build inside the dronepi Conda env so colcon uses the correct Python
# with catkin_pkg, empy==3.3.4, and colcon-ros already installed.
sudo -u "$DRONEPI_USER" bash -c "
    source $CONDA_PREFIX/etc/profile.d/conda.sh
    conda activate $CONDA_ENV
    source /opt/ros/jazzy/setup.bash
    cd $ROS2_WS
    colcon build \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        2>&1 | tail -20
"
ok "ROS 2 workspace built"

# =============================================================================
# [9] CPU governor → performance
# =============================================================================
step "9/10" "CPU governor"

if [[ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]]; then
    echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null
    RC_LOCAL="/etc/rc.local"
    GOV_LINE="echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null"
    if [[ ! -f "$RC_LOCAL" ]]; then
        printf '#!/bin/bash\n%s\nexit 0\n' "$GOV_LINE" > "$RC_LOCAL"
        chmod +x "$RC_LOCAL"
    elif ! grep -qF "$GOV_LINE" "$RC_LOCAL"; then
        sed -i "/^exit 0/i $GOV_LINE" "$RC_LOCAL"
    fi
    ok "CPU governor set to performance (persistent)"
else
    warn "cpufreq not available — skipping governor"
fi

# =============================================================================
# [10] Systemd services
# =============================================================================
step "10/10" "Systemd services"

SCRIPTS="$SCRIPT_DIR"

run_sub() {
    local sub="$1"
    local label="$2"
    if [[ -f "$SCRIPTS/$sub" ]]; then
        echo "  Running $sub..."
        bash "$SCRIPTS/$sub"
        ok "$label"
    else
        warn "$sub not found at $SCRIPTS/$sub — skipping"
    fi
}

# Sub-scripts that write their own service files
run_sub "setup_hotspot_service.sh"      "dronepi-hotspot service"
run_sub "setup_mesh_server_service.sh"  "drone-mesh-server service"
run_sub "setup_foxglove_service.sh"     "foxglove-bridge service"
run_sub "setup_mavros_service.sh"       "mavros service"
run_sub "setup_watchdog_service.sh"     "drone-watchdog service"
run_sub "setup_main_service.sh"         "dronepi-main service"

# ── Services whose .service files live in the flight directory ────────────────
# These are copied directly from source rather than having their own sub-script.

install_service() {
    local name="$1"
    local src="$2"
    local dst="/etc/systemd/system/${name}.service"
    if [[ -f "$src" ]]; then
        cp "$src" "$dst"
        ok "${name}.service installed"
    else
        warn "${name}.service not found at $src — skipping"
    fi
}

install_service "pointlio-standby"    "$FLIGHT_DIR/pointlio-standby.service"
install_service "slam-bridge"         "$FLIGHT_DIR/slam-bridge.service"
install_service "led"                 "$FLIGHT_DIR/led.service"

# ── rpi_server service files + script permissions ─────────────────────────────
RPI_HEALTH_SERVICE="$RPI_SERVER_DIR/rpi-health.service"
if [[ -f "$RPI_HEALTH_SERVICE" ]]; then
    cp "$RPI_HEALTH_SERVICE" "/etc/systemd/system/rpi-health.service"
    chown "$DRONEPI_USER:$DRONEPI_USER" "$RPI_SERVER_DIR/rpi_health_node.py" 2>/dev/null || true
    chmod 755 "$RPI_SERVER_DIR/rpi_health_node.py" 2>/dev/null || true
    ok "rpi-health.service installed"
else
    warn "rpi-health.service not found at $RPI_HEALTH_SERVICE — skipping"
fi

install_service "camera-stream"       "$RPI_SERVER_DIR/camera-stream.service"
install_service "meshview-telemetry"  "$RPI_SERVER_DIR/meshview-telemetry.service"

# ── Reload daemon and enable all services ─────────────────────────────────────
systemctl daemon-reload

for svc in \
    dronepi-hotspot \
    drone-mesh-server \
    foxglove-bridge \
    mavros \
    drone-watchdog \
    dronepi-main \
    slam-bridge \
    rpi-health \
    led \
    camera-stream \
    meshview-telemetry; do
    if systemctl list-unit-files "${svc}.service" &>/dev/null; then
        systemctl enable "${svc}.service" 2>/dev/null || true
    fi
done

# pointlio-standby is triggered by udev (WantedBy=dev-ttyUSB0.device), not boot
if systemctl list-unit-files "pointlio-standby.service" &>/dev/null; then
    systemctl enable "pointlio-standby.service" 2>/dev/null || true
fi

ok "All services enabled"

# =============================================================================
# Summary
# =============================================================================
echo ""
echo -e "${GREEN}${BOLD}"
echo "  ╔══════════════════════════════════════════════════════╗"
echo "  ║   Setup complete                                     ║"
echo "  ╚══════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo "  Next steps (do these in order):"
echo ""
echo "  1. Log out and back in (group membership takes effect)"
echo "  2. Verify udev symlinks:     ls -l /dev/ttyPixhawk"
echo "  3. Source environment:       source ~/.bashrc"
echo "  4. Run preflight check:      bash setup_scripts/preflight_check.sh"
echo "  5. Reboot for services:      sudo reboot"
echo ""
echo "  After reboot, verify all services:"
echo "    sudo systemctl status mavros drone-watchdog slam-bridge \\"
echo "      pointlio-standby foxglove-bridge drone-mesh-server rpi-health \\"
echo "      led camera-stream meshview-telemetry"
echo ""
echo "  If SSD not yet configured:"
echo "    sudo blkid | grep -i ssd"
echo "    # Add:  UUID=<uuid>  /mnt/ssd  exfat  defaults,nofail  0  0"
echo "    # to /etc/fstab, then: sudo mkdir -p /mnt/ssd && sudo mount -a"
echo ""
echo "  Optional (run separately when ready):"
echo "    Foxglove Agent:  FOXGLOVE_DEVICE_TOKEN=fox_dt_... bash setup_scripts/install_foxglove_agent.sh"
echo "    Hailo venv:      see docs/hardware_setup.md — requires hailort wheel from developer.hailo.ai"
echo ""
