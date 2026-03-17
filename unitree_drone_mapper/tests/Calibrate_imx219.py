#!/usr/bin/env python3
"""IMX477 + Tamron 6mm intrinsic calibration — 9x6 inner corners, 22mm squares."""

import cv2
import numpy as np
import glob
import json
import yaml
import os

# === YOUR BOARD ===
CHECKERBOARD = (9, 6)    # inner corners
SQUARE_SIZE  = 0.022     # 22mm in meters

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

obj_points = []
img_points = []

images = sorted(glob.glob(os.path.expanduser("~/calibration/images/*.jpg")))
print(f"Found {len(images)} images\n")

gray_shape = None
for fname in images:
    img  = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_shape = gray.shape[::-1]

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        obj_points.append(objp)
        img_points.append(corners2)
        print(f"  ✓ {os.path.basename(fname)}")
    else:
        print(f"  ✗ {os.path.basename(fname)} — not detected, skipping")

print(f"\n{len(obj_points)}/{len(images)} images usable")
if len(obj_points) < 5:
    print("ERROR: Need at least 5 valid images. Capture more.")
    exit(1)

print("Calibrating...")
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, gray_shape, None, None
)

print(f"\n{'='*50}")
print(f"RMS reprojection error: {ret:.4f}")
print(f"  < 0.5 = excellent  |  < 1.0 = acceptable  |  > 1.0 = redo")
print(f"{'='*50}")
print(f"\nCamera matrix K:")
print(f"  fx = {K[0,0]:.2f}")
print(f"  fy = {K[1,1]:.2f}")
print(f"  cx = {K[0,2]:.2f}")
print(f"  cy = {K[1,2]:.2f}")
print(f"\nDistortion: {dist.ravel()}")

# === Save JSON ===
calib_json = {
    "camera_matrix": K.tolist(),
    "dist_coeffs": dist.tolist(),
    "rms_error": ret,
    "image_size": list(gray_shape),
    "board": {"inner_corners": list(CHECKERBOARD), "square_size_m": SQUARE_SIZE},
    "lens": "Tamron 4-12mm set to ~6mm",
    "sensor": "IMX477 (RPi HQ Camera)",
}
json_path = os.path.expanduser("~/calibration/imx477_calibration.json")
with open(json_path, "w") as f:
    json.dump(calib_json, f, indent=2)
print(f"\nSaved JSON → {json_path}")

# === Save YAML (matches test_texture_live.py pipeline) ===
calib_yaml = {
    "camera": {
        "image_width": gray_shape[0],
        "image_height": gray_shape[1],
        "fx": float(K[0, 0]),
        "fy": float(K[1, 1]),
        "cx": float(K[0, 2]),
        "cy": float(K[1, 2]),
        "dist_coeffs": dist.ravel().tolist(),
    },
    "extrinsic": {
        "translation": [0.0, 0.0, 0.0],
        "rotation_rpy": [0.0, 0.0, 0.0],
        "note": "PLACEHOLDER — measure LiDAR-to-camera offset on drone frame",
    },
}
yaml_path = os.path.expanduser("~/calibration/camera_calibration.yaml")
with open(yaml_path, "w") as f:
    yaml.dump(calib_yaml, f, default_flow_style=False, sort_keys=False)
print(f"Saved YAML → {yaml_path}")
print(f"\nNext step — copy to your project:")
print(f"  cp {yaml_path} ~/unitree_lidar_project/unitree_drone_mapper/config/camera_calibration.yaml")