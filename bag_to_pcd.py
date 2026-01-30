#!/usr/bin/env python3
"""
ROS2 Bag to PCD Converter
Converts Point-LIO point clouds from ROS2 bag files to PCD format
"""

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import struct
import os

def read_bag(bag_path):
    """Read all point clouds from bag file"""
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    all_points = []
    msg_count = 0
    
    print("📖 Reading bag file...")
    print(f"   Path: {bag_path}")
    print("")
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == '/cloud_registered':
            msg = deserialize_message(data, PointCloud2)
            
            # Read points from the message
            for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "intensity")):
                all_points.append(point)
            
            msg_count += 1
            if msg_count % 10 == 0:
                print(f"   Processed {msg_count} messages, {len(all_points):,} points...")
    
    print(f"\n✅ Reading complete!")
    print(f"   Total messages: {msg_count}")
    print(f"   Total points: {len(all_points):,}")
    
    return all_points

def save_pcd(points, filename):
    """Save points to PCD file in ASCII format"""
    print(f"\n💾 Saving to PCD file...")
    print(f"   Output: {filename}")
    
    with open(filename, 'w') as f:
        # Write PCD header
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write('VERSION 0.7\n')
        f.write('FIELDS x y z intensity\n')
        f.write('SIZE 4 4 4 4\n')
        f.write('TYPE F F F F\n')
        f.write('COUNT 1 1 1 1\n')
        f.write(f'WIDTH {len(points)}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {len(points)}\n')
        f.write('DATA ascii\n')
        
        # Write point data
        print(f"   Writing points...")
        for i, p in enumerate(points):
            f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[3]:.6f}\n')
            
            # Progress indicator
            if (i + 1) % 50000 == 0:
                progress = (i + 1) / len(points) * 100
                print(f"   Progress: {progress:.1f}% ({i + 1:,}/{len(points):,} points)")
    
    # Get file size
    file_size = os.path.getsize(filename)
    if file_size > 1024 * 1024:
        size_str = f"{file_size / (1024 * 1024):.2f} MB"
    elif file_size > 1024:
        size_str = f"{file_size / 1024:.2f} KB"
    else:
        size_str = f"{file_size} bytes"
    
    print(f"\n✅ Save complete!")
    print(f"   File size: {size_str}")

def main():
    """Main conversion function"""
    print("=" * 60)
    print("🎯 ROS2 Bag to PCD Converter")
    print("=" * 60)
    print("")
    
    # Configuration
    bag_path = '/root/data/my_scan'
    output_file = '/root/data/scans.pcd'
    
    # Check if bag exists
    if not os.path.exists(bag_path):
        print(f"❌ Error: Bag file not found at {bag_path}")
        print(f"   Make sure you recorded a bag first with:")
        print(f"   ros2 bag record /cloud_registered -o my_scan")
        return
    
    print(f"Input:  {bag_path}")
    print(f"Output: {output_file}")
    print("")
    
    try:
        # Read the bag
        points = read_bag(bag_path)
        
        if len(points) == 0:
            print("\n❌ No points found in bag file!")
            print("   Make sure:")
            print("   1. Point-LIO was running when you recorded")
            print("   2. You moved the LiDAR around during recording")
            print("   3. The /cloud_registered topic had data")
            return
        
        # Save to PCD
        save_pcd(points, output_file)
        
        print("\n" + "=" * 60)
        print("🎉 SUCCESS!")
        print("=" * 60)
        print(f"\nYour point cloud is ready at:")
        print(f"   {output_file}")
        print(f"\nTo access from Windows:")
        print(f"   C:\\Users\\nieve\\unitree_lidar_project\\data\\scans.pcd")
        print(f"\nNext steps:")
        print(f"   1. Exit the Docker container")
        print(f"   2. Open CloudCompare on Windows")
        print(f"   3. Load the scans.pcd file")
        print(f"   4. Start creating your 3D model!")
        print("")
        
    except Exception as e:
        print(f"\n❌ Error during conversion:")
        print(f"   {str(e)}")
        print(f"\nIf you see 'No module named rosbag2_py', run:")
        print(f"   pip3 install rosbag2_py")

if __name__ == '__main__':
    main()
