#!/bin/bash
# Record ROS 2 bag file with all relevant topics

set -e

# Configuration
OUTPUT_DIR="${HOME}/drone_bags"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="flight_${TIMESTAMP}"

# Topics to record
TOPICS=(
    "/cloud_registered"
    "/odometry"
    "/path"
    "/lidar/points"
    "/imu/data"
    "/camera/image_raw"
    "/camera/camera_info"
    "/detections"
    "/semantic_cloud"
    "/mavlink/state"
    "/mavlink/battery"
    "/health/status"
    "/tf"
    "/tf_static"
)

# Create output directory
mkdir -p "$OUTPUT_DIR"

echo "=================================="
echo "🎥 Starting ROS 2 Bag Recording"
echo "=================================="
echo "Output: $OUTPUT_DIR/$BAG_NAME"
echo "Topics: ${#TOPICS[@]}"
echo ""
echo "Recording topics:"
for topic in "${TOPICS[@]}"; do
    echo "  - $topic"
done
echo ""
echo "Press Ctrl+C to stop recording"
echo "=================================="

# Record bag
ros2 bag record \
    -o "$OUTPUT_DIR/$BAG_NAME" \
    -s mcap \
    "${TOPICS[@]}"

# Print summary
if [ $? -eq 0 ]; then
    echo ""
    echo "=================================="
    echo "✅ Recording completed!"
    echo "=================================="
    echo "Bag file: $OUTPUT_DIR/$BAG_NAME"
    echo ""
    echo "To replay:"
    echo "  ros2 bag play $OUTPUT_DIR/$BAG_NAME"
    echo ""
    echo "To inspect:"
    echo "  ros2 bag info $OUTPUT_DIR/$BAG_NAME"
else
    echo "❌ Recording failed!"
    exit 1
fi