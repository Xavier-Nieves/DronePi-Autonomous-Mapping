#!/usr/bin/env python3
"""
Automated Point Cloud to Mesh Pipeline
Integrates with ROS2 to automatically process LiDAR scans

This script:
1. Subscribes to /cloud_registered topic
2. Accumulates points while you scan
3. On Ctrl+C, processes the point cloud:
   - Statistical outlier removal
   - Normal estimation
   - Poisson surface reconstruction
4. Saves both PCD and mesh (OBJ) files
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
from datetime import datetime
import os
import signal
import sys

class AutoMeshGenerator(Node):
    def __init__(self):
        super().__init__('auto_mesh_generator')
        
        # Configuration
        self.output_dir = '/root/data'
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Point cloud accumulator
        self.all_points = []
        self.point_count = 0
        self.frame_count = 0
        
        # Processing parameters
        self.sor_neighbors = 20
        self.sor_std_ratio = 2.0
        self.normal_radius = 0.1
        self.normal_max_nn = 30
        self.poisson_depth = 9
        
        # Subscribe to point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.cloud_callback,
            10)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎯 Auto Mesh Generator Started!')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info(f'Session ID: {self.timestamp}')
        self.get_logger().info('')
        self.get_logger().info('📡 Listening to /cloud_registered topic...')
        self.get_logger().info('🚶 Walk around and scan your area')
        self.get_logger().info('⏸️  Press Ctrl+C when done to process and save')
        self.get_logger().info('')
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def cloud_callback(self, msg):
        """Accumulate point clouds from topic"""
        # Extract points from ROS message
        points_list = []
        for point in pc2.read_points(msg, skip_nans=True, 
                                     field_names=("x", "y", "z", "intensity")):
            points_list.append([point[0], point[1], point[2]])
        
        if points_list:
            self.all_points.extend(points_list)
            self.point_count = len(self.all_points)
            self.frame_count += 1
            
            # Log progress every 10 frames
            if self.frame_count % 10 == 0:
                self.get_logger().info(
                    f'📊 Frames: {self.frame_count} | Points: {self.point_count:,}'
                )
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('⏹️  Stopping scan collection...')
        self.get_logger().info('=' * 60)
        
        if self.point_count == 0:
            self.get_logger().warn('❌ No points collected!')
            self.get_logger().info('Make sure Point-LIO and LiDAR driver are running.')
            sys.exit(0)
        
        self.get_logger().info(f'✅ Collected {self.point_count:,} points from {self.frame_count} frames')
        self.get_logger().info('')
        
        # Process the point cloud
        self.process_pointcloud()
        
        sys.exit(0)
    
    def process_pointcloud(self):
        """Main processing pipeline"""
        self.get_logger().info('🔄 Starting processing pipeline...')
        self.get_logger().info('')
        
        # Convert to numpy array
        points_array = np.array(self.all_points, dtype=np.float32)
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_array)
        
        self.get_logger().info(f'📦 Point cloud created: {len(pcd.points):,} points')
        
        # Step 1: Save raw point cloud
        self.save_raw_pcd(pcd)
        
        # Step 2: Statistical outlier removal
        pcd_clean = self.remove_outliers(pcd)
        
        # Step 3: Downsample if too dense
        pcd_clean = self.downsample_if_needed(pcd_clean)
        
        # Step 4: Estimate normals
        pcd_clean = self.estimate_normals(pcd_clean)
        
        # Step 5: Save cleaned point cloud
        self.save_clean_pcd(pcd_clean)
        
        # Step 6: Poisson surface reconstruction
        mesh = self.create_mesh(pcd_clean)
        
        # Step 7: Post-process mesh
        mesh = self.clean_mesh(mesh)
        
        # Step 8: Save mesh
        self.save_mesh(mesh)
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎉 SUCCESS! Processing complete!')
        self.get_logger().info('=' * 60)
        self.print_summary()
    
    def save_raw_pcd(self, pcd):
        """Save raw unprocessed point cloud"""
        filename = f"{self.output_dir}/scan_raw_{self.timestamp}.pcd"
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f'💾 Raw PCD saved: {filename}')
    
    def remove_outliers(self, pcd):
        """Statistical Outlier Removal"""
        self.get_logger().info('')
        self.get_logger().info('🧹 Step 1: Removing outliers...')
        self.get_logger().info(f'   Parameters: neighbors={self.sor_neighbors}, std_ratio={self.sor_std_ratio}')
        
        pcd_clean, ind = pcd.remove_statistical_outlier(
            nb_neighbors=self.sor_neighbors,
            std_ratio=self.sor_std_ratio
        )
        
        removed = len(pcd.points) - len(pcd_clean.points)
        percent = (removed / len(pcd.points)) * 100
        
        self.get_logger().info(f'   ✅ Removed {removed:,} outliers ({percent:.1f}%)')
        self.get_logger().info(f'   Remaining: {len(pcd_clean.points):,} points')
        
        return pcd_clean
    
    def downsample_if_needed(self, pcd, threshold=500000):
        """Downsample if point cloud is too large"""
        if len(pcd.points) > threshold:
            self.get_logger().info('')
            self.get_logger().info(f'📉 Point cloud large ({len(pcd.points):,} points), downsampling...')
            
            voxel_size = 0.02  # 2cm voxels
            pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
            
            self.get_logger().info(f'   ✅ Downsampled to {len(pcd_down.points):,} points (voxel={voxel_size}m)')
            return pcd_down
        
        return pcd
    
    def estimate_normals(self, pcd):
        """Estimate surface normals"""
        self.get_logger().info('')
        self.get_logger().info('🧭 Step 2: Computing normals...')
        self.get_logger().info(f'   Parameters: radius={self.normal_radius}m, max_nn={self.normal_max_nn}')
        
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.normal_radius,
                max_nn=self.normal_max_nn
            )
        )
        
        # Orient normals consistently
        pcd.orient_normals_consistent_tangent_plane(k=15)
        
        self.get_logger().info('   ✅ Normals computed and oriented')
        
        return pcd
    
    def save_clean_pcd(self, pcd):
        """Save cleaned point cloud with normals"""
        filename = f"{self.output_dir}/scan_clean_{self.timestamp}.pcd"
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f'💾 Clean PCD saved: {filename}')
    
    def create_mesh(self, pcd):
        """Poisson surface reconstruction"""
        self.get_logger().info('')
        self.get_logger().info('🏗️  Step 3: Creating mesh (Poisson reconstruction)...')
        self.get_logger().info(f'   Depth: {self.poisson_depth} (higher = more detail)')
        self.get_logger().info('   ⏳ This may take 5-30 minutes...')
        
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd,
            depth=self.poisson_depth,
            width=0,
            scale=1.1,
            linear_fit=False
        )
        
        self.get_logger().info(f'   ✅ Mesh created: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} triangles')
        
        # Remove low-density vertices (artifacts)
        vertices_to_remove = densities < np.quantile(densities, 0.01)
        mesh.remove_vertices_by_mask(vertices_to_remove)
        
        removed = vertices_to_remove.sum()
        self.get_logger().info(f'   ✅ Removed {removed:,} low-density vertices')
        
        return mesh
    
    def clean_mesh(self, mesh):
        """Post-process mesh"""
        self.get_logger().info('')
        self.get_logger().info('🧽 Step 4: Cleaning mesh...')
        
        # Remove disconnected components
        original_triangles = len(mesh.triangles)
        
        triangle_clusters, cluster_n_triangles, cluster_area = (
            mesh.cluster_connected_triangles()
        )
        triangle_clusters = np.asarray(triangle_clusters)
        cluster_n_triangles = np.asarray(cluster_n_triangles)
        
        # Keep largest component
        largest_cluster_idx = cluster_n_triangles.argmax()
        triangles_to_remove = triangle_clusters != largest_cluster_idx
        mesh.remove_triangles_by_mask(triangles_to_remove)
        
        removed = triangles_to_remove.sum()
        self.get_logger().info(f'   ✅ Removed {removed:,} isolated triangles')
        
        # Remove degenerate triangles
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_non_manifold_edges()
        
        self.get_logger().info(f'   ✅ Final mesh: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} triangles')
        
        return mesh
    
    def save_mesh(self, mesh):
        """Save mesh in multiple formats"""
        self.get_logger().info('')
        self.get_logger().info('💾 Step 5: Saving mesh...')
        
        # Save as OBJ (best for texturing)
        obj_file = f"{self.output_dir}/model_{self.timestamp}.obj"
        o3d.io.write_triangle_mesh(obj_file, mesh, write_ascii=True)
        obj_size = os.path.getsize(obj_file) / (1024 * 1024)  # MB
        self.get_logger().info(f'   ✅ OBJ saved: {obj_file} ({obj_size:.2f} MB)')
        
        # Save as PLY (preserves more data)
        ply_file = f"{self.output_dir}/model_{self.timestamp}.ply"
        o3d.io.write_triangle_mesh(ply_file, mesh, write_ascii=False)
        ply_size = os.path.getsize(ply_file) / (1024 * 1024)  # MB
        self.get_logger().info(f'   ✅ PLY saved: {ply_file} ({ply_size:.2f} MB)')
        
        # Save as STL (for 3D printing)
        stl_file = f"{self.output_dir}/model_{self.timestamp}.stl"
        o3d.io.write_triangle_mesh(stl_file, mesh)
        stl_size = os.path.getsize(stl_file) / (1024 * 1024)  # MB
        self.get_logger().info(f'   ✅ STL saved: {stl_file} ({stl_size:.2f} MB)')
    
    def print_summary(self):
        """Print final summary"""
        self.get_logger().info('')
        self.get_logger().info('📊 Summary:')
        self.get_logger().info(f'   Session: {self.timestamp}')
        self.get_logger().info(f'   Frames processed: {self.frame_count}')
        self.get_logger().info(f'   Total points collected: {self.point_count:,}')
        self.get_logger().info('')
        self.get_logger().info('📁 Output files:')
        self.get_logger().info(f'   {self.output_dir}/scan_raw_{self.timestamp}.pcd')
        self.get_logger().info(f'   {self.output_dir}/scan_clean_{self.timestamp}.pcd')
        self.get_logger().info(f'   {self.output_dir}/model_{self.timestamp}.obj')
        self.get_logger().info(f'   {self.output_dir}/model_{self.timestamp}.ply')
        self.get_logger().info(f'   {self.output_dir}/model_{self.timestamp}.stl')
        self.get_logger().info('')
        self.get_logger().info('🪟 Windows paths:')
        self.get_logger().info(f'   C:\\Users\\nieve\\unitree_lidar_project\\data\\')
        self.get_logger().info('')
        self.get_logger().info('🎨 Next steps:')
        self.get_logger().info('   1. Open .obj file in CloudCompare or Blender')
        self.get_logger().info('   2. Add photo textures in Blender/Meshroom')
        self.get_logger().info('   3. Use .stl for 3D printing')
        self.get_logger().info('')

def main(args=None):
    # Install dependencies check
    try:
        import open3d
    except ImportError:
        print("❌ Open3D not installed!")
        print("Install with: pip3 install open3d")
        print("Then run this script again.")
        return
    
    rclpy.init(args=args)
    
    try:
        node = AutoMeshGenerator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
