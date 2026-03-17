#!/bin/bash
# ============================================================
#  DRONE PRE-FLIGHT SYSTEM CHECK
#  Platform : Raspberry Pi 5 — Ubuntu 24.04
#  Project  : Autonomous Texture-Mapping Drone (UPRM Capstone)
#  Author   : Xavier
#  Usage    : bash preflight_check.sh   (no sudo needed)
#             sudo bash preflight_check.sh  (enables PMIC voltage read)
# ============================================================

# --- Colour codes ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

PASS=0
FAIL=0
WARN=0

pass() { echo -e "  ${GREEN}[PASS]${NC} $1"; ((PASS++)); }
fail() { echo -e "  ${RED}[FAIL]${NC} $1"; ((FAIL++)); }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $1"; ((WARN++)); }
section() { echo -e "\n${CYAN}${BOLD}━━━  $1  ━━━${NC}"; }

# ============================================================
# Resolve the actual user's home directory even when run via sudo
# ============================================================
if [ -n "$SUDO_USER" ]; then
    REAL_USER="$SUDO_USER"
    REAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    REAL_USER="$USER"
    REAL_HOME="$HOME"
fi

# Source ROS2 if not already sourced
if ! command -v ros2 > /dev/null 2>&1; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null
fi
if [ -f "$REAL_HOME/ros2_ws/install/setup.bash" ]; then
    source "$REAL_HOME/ros2_ws/install/setup.bash" 2>/dev/null
fi

# ============================================================
section "1. POWER RAIL & USB CURRENT OVERRIDE"
# ============================================================

if grep -q "usb_max_current_enable=1" /boot/firmware/config.txt 2>/dev/null; then
    pass "usb_max_current_enable=1 found in /boot/firmware/config.txt"
else
    fail "usb_max_current_enable=1 NOT found — USB ports may be limited to 600mA"
fi

THROTTLE=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
if [ "$THROTTLE" = "0x0" ]; then
    pass "No throttling or under-voltage events detected (throttled=0x0)"
else
    warn "Throttle flags active: $THROTTLE"
    echo "       Bit ref: 0x1=under-voltage now  0x4=throttled now  0x10000=under-voltage occurred"
fi

# Read voltage — strip everything except the numeric value
V5_RAW=$(vcgencmd pmic_read_adc EXT5V_V 2>/dev/null)
V5=$(echo "$V5_RAW" | grep -oP '[0-9]+\.[0-9]+' | head -1)
if [ -n "$V5" ]; then
    # Pure numeric awk comparison — no unit suffix passed in
    VCHECK=$(awk -v v="$V5" 'BEGIN{ print (v+0 >= 4.8) ? "ok" : "low" }')
    if [ "$VCHECK" = "ok" ]; then
        pass "5V rail voltage: ${V5}V (above 4.8V threshold)"
    else
        fail "5V rail voltage: ${V5}V — BELOW 4.8V threshold, risk of instability"
    fi
else
    warn "Could not read EXT5V_V — run as root for PMIC access (sudo bash preflight_check.sh)"
fi

# ============================================================
section "2. STORAGE — SanDisk Extreme SSD (/mnt/ssd)"
# ============================================================

SSD_MOUNT="/mnt/ssd"

if mountpoint -q "$SSD_MOUNT"; then
    pass "SSD mounted at $SSD_MOUNT"

    LABEL=$(lsblk -o LABEL,MOUNTPOINT | grep "$SSD_MOUNT" | awk '{print $1}')
    if [ "$LABEL" = "flight_data" ]; then
        pass "SSD filesystem label: $LABEL"
    else
        warn "SSD label is '${LABEL:-none}' (expected 'flight_data')"
    fi

    FREE_GB=$(df -BG "$SSD_MOUNT" | tail -1 | awk '{gsub("G",""); print $4}')
    if [ "${FREE_GB:-0}" -ge 20 ] 2>/dev/null; then
        pass "SSD free space: ${FREE_GB}GB"
    else
        warn "SSD free space LOW: ${FREE_GB}GB — clear old rosbags before flight"
    fi

    echo "  Running write speed test (256MB) — please wait..."
    WRITE_OUT=$(dd if=/dev/zero of="$SSD_MOUNT/.speed_test.tmp" \
        bs=1M count=256 oflag=direct 2>&1)
    rm -f "$SSD_MOUNT/.speed_test.tmp"
    WRITE_SPEED=$(echo "$WRITE_OUT" | grep -oP '[0-9.]+ [MG]B/s' | tail -1)
    if [ -n "$WRITE_SPEED" ]; then
        SPEED_VAL=$(echo "$WRITE_SPEED" | grep -oP '[0-9.]+')
        SPEED_UNIT=$(echo "$WRITE_SPEED" | grep -oP '[MG]B/s')
        SPEED_OK=$(awk -v v="$SPEED_VAL" -v u="$SPEED_UNIT" \
            'BEGIN{ print (u=="GB/s" || (u=="MB/s" && v+0>=150)) ? "ok" : "low" }')
        if [ "$SPEED_OK" = "ok" ]; then
            pass "SSD write speed: $WRITE_SPEED (sufficient for rosbag recording)"
        else
            warn "SSD write speed: $WRITE_SPEED — may be insufficient for concurrent rosbag streams"
        fi
    else
        warn "Could not measure SSD write speed"
    fi

    if lsusb | grep -qi "sandisk\|western digital"; then
        pass "SanDisk SSD detected on USB bus"
    else
        warn "SanDisk not positively identified on USB bus (may enumerate under WD ID)"
    fi

else
    fail "SSD NOT mounted at $SSD_MOUNT"
    echo "       If SSD is connected, check: sudo mount -a"
    echo "       If not connected, this is expected — connect before flight"
fi

# ============================================================
section "3. PIXHAWK 6X — MAVROS SERIAL"
# ============================================================

if [ -L /dev/ttyPixhawk ]; then
    DEVICE="/dev/ttyPixhawk"
    pass "Pixhawk udev symlink: /dev/ttyPixhawk → $(readlink /dev/ttyPixhawk)"
elif [ -e /dev/ttyACM0 ]; then
    DEVICE="/dev/ttyACM0"
    pass "Pixhawk serial device: /dev/ttyACM0"
    warn "Consider adding a udev symlink for stable device naming across reboots"
    echo "       See README for udev rule instructions"
else
    DEVICE=""
    fail "Pixhawk serial device NOT found at /dev/ttyACM0 or /dev/ttyPixhawk"
fi

if [ -n "$DEVICE" ]; then
    if [ -r "$DEVICE" ]; then
        pass "$DEVICE is readable"
    else
        fail "$DEVICE not readable — add user to dialout group"
        echo "       Fix: sudo usermod -aG dialout $REAL_USER"
    fi
fi

if id -nG "$REAL_USER" 2>/dev/null | grep -q "dialout"; then
    pass "User '$REAL_USER' is in dialout group"
else
    warn "User '$REAL_USER' NOT in dialout group"
    echo "       Fix: sudo usermod -aG dialout $REAL_USER  (then log out and back in)"
fi

if pgrep -f "mavros" > /dev/null 2>&1; then
    pass "MAVROS process is running"
else
    warn "MAVROS not currently running — launch before arming"
    echo "       Launch: ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:921600"
fi

# ============================================================
section "4. HAILO-8 AI ACCELERATOR HAT"
# ============================================================

if lspci 2>/dev/null | grep -qi "hailo"; then
    HAILO_INFO=$(lspci | grep -i hailo | head -1)
    pass "Hailo-8 on PCIe bus: $HAILO_INFO"
elif [ -e /dev/hailo0 ]; then
    pass "Hailo device node /dev/hailo0 present (PCIe enumeration may need lspci update)"
else
    fail "Hailo-8 NOT detected — check HAT seating on PCIe FPC connector"
fi

if [ -e /dev/hailo0 ]; then
    pass "/dev/hailo0 device node exists"
    if [ -r /dev/hailo0 ]; then
        pass "/dev/hailo0 is accessible by current user"
    else
        warn "/dev/hailo0 not accessible — check hailo group: sudo usermod -aG hailo $REAL_USER"
    fi
else
    warn "/dev/hailo0 not found — HailoRT driver may not be loaded"
    echo "       Check: lsmod | grep hailo"
fi

# Resolve venv under real user home, not root home
HAILO_VENV="$REAL_HOME/hailo_inference_env"
if [ -d "$HAILO_VENV" ]; then
    pass "hailo_inference_env venv found at $HAILO_VENV"
    HAILO_IMPORT=$("$HAILO_VENV/bin/python3" -c \
        "import hailo_platform; print('ok')" 2>/dev/null)
    if [ "$HAILO_IMPORT" = "ok" ]; then
        pass "hailo_platform imports successfully"
    else
        fail "hailo_platform import FAILED in hailo_inference_env"
        echo "       Check HailoRT installation: $HAILO_VENV/bin/pip list | grep hailo"
    fi
else
    fail "hailo_inference_env NOT found at $HAILO_VENV"
    echo "       Expected location: $HAILO_VENV"
fi

# ============================================================
section "5. CAMERA — Raspberry Pi HQ IMX477"
# ============================================================

CAM_OK=false

# Try libcamera (may need full path on Ubuntu 24.04)
for BIN in libcamera-hello /usr/bin/libcamera-hello /usr/local/bin/libcamera-hello; do
    if command -v "$BIN" > /dev/null 2>&1 || [ -x "$BIN" ]; then
        CAM_DETECT=$("$BIN" --list-cameras 2>&1)
        if echo "$CAM_DETECT" | grep -qi "imx477"; then
            pass "IMX477 HQ Camera detected via libcamera"
            echo "       $(echo "$CAM_DETECT" | grep -i 'imx477' | head -1 | xargs)"
            CAM_OK=true
        elif echo "$CAM_DETECT" | grep -qi "available\|camera"; then
            warn "Camera found by libcamera but model not confirmed as IMX477"
            echo "       Output: $(echo "$CAM_DETECT" | head -2 | xargs)"
            CAM_OK=true
        fi
        break
    fi
done

# v4l2 fallback
if [ "$CAM_OK" = false ]; then
    if v4l2-ctl --list-devices 2>/dev/null | grep -qi "unicam\|imx477\|camera\|rp1"; then
        pass "Camera detected via v4l2"
        CAM_OK=true
    fi
fi

# rpicam fallback
if [ "$CAM_OK" = false ]; then
    if rpicam-still --list-cameras 2>/dev/null | grep -qi "imx477\|available"; then
        pass "IMX477 detected via rpicam-still"
        CAM_OK=true
    fi
fi

if [ "$CAM_OK" = false ]; then
    fail "Camera NOT detected — verify CSI ribbon cable seating and lock tab"
    echo "       Both ends of ribbon must be fully seated with lock tabs closed"
    echo "       Tamron C-mount lens: ensure C-CS spacer ring is REMOVED for infinity focus"
fi

# Calibration file
CAL_FOUND=false
for CAL_PATH in \
    "$(pwd)/config/camera_calibration.yaml" \
    "$REAL_HOME/config/camera_calibration.yaml" \
    "/mnt/ssd/config/camera_calibration.yaml"; do
    if [ -f "$CAL_PATH" ]; then
        pass "Calibration file: $CAL_PATH"
        CAL_FOUND=true
        break
    fi
done
[ "$CAL_FOUND" = false ] && warn "camera_calibration.yaml not found in expected locations"

# ============================================================
section "6. LIDAR — Unitree 4D L1 (CP210x)"
# ============================================================

if lsusb | grep -qi "cp210\|10c4:ea60"; then
    LIDAR_USB=$(lsusb | grep -i "cp210\|10c4:ea60" | head -1)
    pass "LiDAR CP210x USB adapter detected: $LIDAR_USB"
elif lsusb | grep -qi "ch340\|ch341\|ftdi\|silabs"; then
    LIDAR_USB=$(lsusb | grep -i "ch340\|ch341\|ftdi\|silabs" | head -1)
    warn "USB serial adapter found but not CP210x (expected for L1): $LIDAR_USB"
elif ls /dev/ttyUSB* > /dev/null 2>&1; then
    warn "ttyUSB devices present but vendor unconfirmed: $(ls /dev/ttyUSB*)"
else
    fail "No LiDAR USB serial device detected — check USB connection and LiDAR power"
fi

if id -nG "$REAL_USER" 2>/dev/null | grep -q "dialout"; then
    pass "User has dialout access for LiDAR serial port"
else
    warn "dialout group needed for /dev/ttyUSB* access"
fi

if ros2 pkg list 2>/dev/null | grep -qi "point_lio\|pointlio"; then
    pass "Point-LIO ROS2 package available"
else
    warn "Point-LIO not found in current ROS2 environment"
    echo "       Fix: source $REAL_HOME/ros2_ws/install/setup.bash"
fi

# ============================================================
section "7. WI-FI DONGLE — TP-Link AC600 (RTL8821AU)"
# ============================================================

if lsusb | grep -qiE "2357:011e|2357:0115|tp-link.*rtl88"; then
    TP_INFO=$(lsusb | grep -iE "2357:011e|2357:0115|tp-link" | head -1)
    pass "TP-Link AC600 RTL8821AU detected: $TP_INFO"
else
    warn "TP-Link AC600 not confirmed on USB bus (check: lsusb | grep 2357)"
fi

# Strip trailing colon from interface name if present
WLAN_IF=$(ip link show | grep -oP '(?<=\d: )wlan[^:@\s]+' | head -1 | tr -d ':')
if [ -n "$WLAN_IF" ]; then
    pass "Wireless interface active: $WLAN_IF"
    SSID=$(iwgetid "$WLAN_IF" -r 2>/dev/null)
    if [ -n "$SSID" ]; then
        pass "Connected to Wi-Fi SSID: $SSID"
        GW=$(ip route | grep default | awk '{print $3}' | head -1)
        if ping -c 1 -W 2 "$GW" > /dev/null 2>&1; then
            pass "Gateway reachable: $GW"
        else
            warn "Gateway $GW not responding — check AP and signal strength"
        fi
    else
        warn "$WLAN_IF present but not connected — expected if flying without GCS link"
    fi
else
    fail "No wireless interface found — check 8821au driver: lsmod | grep 8821au"
fi

# ============================================================
section "8. ROS 2 JAZZY ENVIRONMENT"
# ============================================================

if command -v ros2 > /dev/null 2>&1; then
    ROS_VER=$(ros2 --version 2>/dev/null | head -1)
    pass "ROS 2 available: $ROS_VER"

    for PKG in "mavros" "rosbridge_server" "sensor_msgs" "geometry_msgs"; do
        if ros2 pkg list 2>/dev/null | grep -q "^${PKG}$"; then
            pass "ROS2 package: $PKG"
        else
            warn "ROS2 package missing: $PKG"
        fi
    done
else
    fail "ROS 2 not in PATH — sourcing failed"
    echo "       Manual fix: source /opt/ros/jazzy/setup.bash"
    echo "       Permanent fix (for this script): add to /root/.bashrc AND $REAL_HOME/.bashrc"
fi

# ============================================================
section "9. SYSTEM HEALTH"
# ============================================================

CPU_TEMP=$(vcgencmd measure_temp 2>/dev/null | grep -oP '[0-9.]+')
if [ -n "$CPU_TEMP" ]; then
    TEMP_STATUS=$(awk -v t="$CPU_TEMP" 'BEGIN{
        if (t+0 < 70) print "ok"
        else if (t+0 < 80) print "warn"
        else print "crit"
    }')
    if   [ "$TEMP_STATUS" = "ok"   ]; then pass "CPU temperature: ${CPU_TEMP}°C (nominal)"
    elif [ "$TEMP_STATUS" = "warn" ]; then warn "CPU temperature: ${CPU_TEMP}°C (elevated — verify active cooling)"
    else                                   fail "CPU temperature: ${CPU_TEMP}°C — CRITICAL, do not fly"
    fi
fi

FREE_RAM=$(free -m | awk '/^Mem:/{print $7}')
RAM_STATUS=$(awk -v r="${FREE_RAM:-0}" 'BEGIN{
    if (r+0 >= 1024) print "ok"
    else if (r+0 >= 512) print "warn"
    else print "crit"
}')
if   [ "$RAM_STATUS" = "ok"   ]; then pass "Available RAM: ${FREE_RAM}MB"
elif [ "$RAM_STATUS" = "warn" ]; then warn "Available RAM: ${FREE_RAM}MB — close unused processes before flight"
else                                   fail "Available RAM critically low: ${FREE_RAM}MB"
fi

# CPU clock — fixed formatting
CPU_HZ=$(vcgencmd measure_clock arm 2>/dev/null | grep -oP '[0-9]+$')
if [ -n "$CPU_HZ" ]; then
    CPU_GHZ=$(awk -v h="$CPU_HZ" 'BEGIN{ printf "%.2f", h/1000000000 }')
    pass "CPU clock: ${CPU_GHZ} GHz"
fi

ROOT_DEV=$(findmnt -n -o SOURCE / | sed 's/p\?[0-9]*$//')
if echo "$ROOT_DEV" | grep -q "mmcblk"; then
    warn "Root filesystem is on SD card — confirm all rosbag writes target /mnt/ssd"
else
    pass "Root filesystem on: $ROOT_DEV (not SD card)"
fi

# ============================================================
section "10. OUTPUT DIRECTORIES"
# ============================================================

for DIR in "/mnt/ssd/flights" "/mnt/ssd/rosbags" "/mnt/ssd/maps"; do
    if [ -d "$DIR" ]; then
        pass "Directory exists: $DIR"
    elif mountpoint -q "/mnt/ssd"; then
        warn "Creating missing directory: $DIR"
        mkdir -p "$DIR" 2>/dev/null && pass "Created: $DIR" || \
            fail "Could not create $DIR"
    else
        warn "Skipping $DIR — SSD not mounted"
    fi
done

# ============================================================
echo -e "\n${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BOLD}  PRE-FLIGHT SYSTEM CHECK — RESULTS${NC}"
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "  ${GREEN}PASS${NC} : $PASS"
echo -e "  ${YELLOW}WARN${NC} : $WARN"
echo -e "  ${RED}FAIL${NC} : $FAIL"
echo ""
if   [ "$FAIL" -gt 0 ]; then
    echo -e "  ${RED}${BOLD}▶ STATUS: NOT READY FOR FLIGHT — resolve all FAIL items above${NC}"
elif [ "$WARN" -gt 0 ]; then
    echo -e "  ${YELLOW}${BOLD}▶ STATUS: READY WITH WARNINGS — review WARN items before flight${NC}"
else
    echo -e "  ${GREEN}${BOLD}▶ STATUS: ALL SYSTEMS GO ✓${NC}"
fi
echo ""
