#!/usr/bin/env python3
"""
Optimized Auto Mesh Generator for 3D Reconstruction
Streamlined version - only creates files needed for photorealistic texturing

Outputs:
- Clean PCD (for backup/reprocessing)
- OBJ mesh (for texturing in Meshroom/Blender)
- That's it! No unnecessary files.
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

class OptimizedMeshGenerator(Node):
    def __init__(self):
        super().__init__('optimized_mesh_generator')
        
        # Configuration
        self.output_dir = '/root/data'
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Point cloud accumulator
        self.all_points = []
        self.point_count = 0
        self.frame_count = 0
        
        # Processing parameters (optimized for speed + quality)
        self.sor_neighbors = 20
        self.sor_std_ratio = 2.0
        self.normal_radius = 0.1
        self.normal_max_nn = 30
        self.poisson_depth = 9  # Good balance - change to 8 for faster, 10 for better
        
        # Subscribe to point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.cloud_callback,
            10)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('🎯 Optimized Mesh Generator for 3D Reconstruction')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'📁 Output: {self.output_dir}')
        self.get_logger().info(f'🆔 Session: {self.timestamp}')
        self.get_logger().info(f'⚙️  Poisson depth: {self.poisson_depth} (8=fast, 9=balanced, 10=quality)')
        self.get_logger().info('')
        self.get_logger().info('📡 Listening to /cloud_registered...')
        self.get_logger().info('🚶 Walk around and scan your area')
        self.get_logger().info('⏸️  Press Ctrl+C when done')
        self.get_logger().info('')
        self.get_logger().info('Will create:')
        self.get_logger().info('  • Clean PCD (backup/reprocessing)')
        self.get_logger().info('  • OBJ mesh (for Meshroom/Blender texturing)')
        self.get_logger().info('')
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def cloud_callback(self, msg):
        """Accumulate point clouds from topic"""
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
                    f'📊 Frames: {self.frame_count:>4} | Points: {self.point_count:>8,}'
                )
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('⏹️  Scan stopped')
        self.get_logger().info('=' * 70)
        
        if self.point_count == 0:
            self.get_logger().warn('❌ No points collected!')
            self.get_logger().info('')
            self.get_logger().info('Troubleshooting:')
            self.get_logger().info('  1. Is Point-LIO running?')
            self.get_logger().info('  2. Is LiDAR driver running?')
            self.get_logger().info('  3. Check: ros2 topic hz /cloud_registered')
            sys.exit(0)
        
        self.get_logger().info(f'✅ Collected {self.point_count:,} points from {self.frame_count} frames')
        self.get_logger().info('')
        
        # Process the point cloud
        self.process_pointcloud()
        
        sys.exit(0)
    
    def process_pointcloud(self):
        """Main processing pipeline - streamlined for 3D reconstruction"""
        self.get_logger().info('🔄 Processing for 3D reconstruction...')
        self.get_logger().info('')
        
        # Convert to numpy array
        points_array = np.array(self.all_points, dtype=np.float32)
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_array)
        
        self.get_logger().info(f'📦 Point cloud created: {len(pcd.points):,} points')
        
        # Step 1: Statistical outlier removal
        pcd_clean = self.remove_outliers(pcd)
        
        # Step 2: Downsample if too dense (optional but recommended)
        pcd_clean = self.downsample_if_needed(pcd_clean)
        
        # Step 3: Estimate normals (CRITICAL for meshing)
        pcd_clean = self.estimate_normals(pcd_clean)
        
        # Step 4: Save cleaned point cloud
        self.save_clean_pcd(pcd_clean)
        
        # Step 5: Poisson surface reconstruction
        mesh = self.create_mesh(pcd_clean)
        
        # Step 6: Clean mesh
        mesh = self.clean_mesh(mesh)
        
        # Step 7: Save mesh (OBJ only - what you actually need!)
        self.save_mesh(mesh)
        
        # Done!
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('🎉 SUCCESS!')
        self.get_logger().info('=' * 70)
        self.print_summary()
    
    def remove_outliers(self, pcd):
        """Statistical Outlier Removal"""
        self.get_logger().info('')
        self.get_logger().info('🧹 Step 1/4: Removing noise...')
        
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
            self.get_logger().info(f'📉 Point cloud large ({len(pcd.points):,}), downsampling...')
            
            voxel_size = 0.02  # 2cm voxels
            pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
            
            self.get_logger().info(f'   ✅ Downsampled to {len(pcd_down.points):,} points')
            return pcd_down
        
        return pcd
    
    def estimate_normals(self, pcd):
        """Estimate surface normals - CRITICAL for good meshing"""
        self.get_logger().info('')
        self.get_logger().info('🧭 Step 2/4: Computing normals...')
        
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
        """Save cleaned point cloud for backup/reprocessing"""
        filename = f"{self.output_dir}/scan_clean_{self.timestamp}.pcd"
        o3d.io.write_point_cloud(filename, pcd)
        
        file_size = os.path.getsize(filename) / (1024 * 1024)  # MB
        self.get_logger().info(f'💾 Clean PCD saved: {file_size:.2f} MB')
    
    def create_mesh(self, pcd):
        """Poisson surface reconstruction"""
        self.get_logger().info('')
        self.get_logger().info('🏗️  Step 3/4: Creating mesh...')
        self.get_logger().info(f'   Poisson depth: {self.poisson_depth}')
        
        # Estimate time
        if self.poisson_depth <= 8:
            time_est = "3-8 minutes"
        elif self.poisson_depth == 9:
            time_est = "8-15 minutes"
        else:
            time_est = "15-30 minutes"
        
        self.get_logger().info(f'   ⏳ Estimated time: {time_est}')
        self.get_logger().info('   (Go grab coffee... ☕)')
        
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd,
            depth=self.poisson_depth,
            width=0,
            scale=1.1,
            linear_fit=False
        )
        
        self.get_logger().info(f'   ✅ Mesh created: {len(mesh.vertices):,} vertices')
        
        # Remove low-density artifacts
        vertices_to_remove = densities < np.quantile(densities, 0.01)
        mesh.remove_vertices_by_mask(vertices_to_remove)
        
        removed = vertices_to_remove.sum()
        if removed > 0:
            self.get_logger().info(f'   ✅ Cleaned {removed:,} low-density vertices')
        
        return mesh
    
    def clean_mesh(self, mesh):
        """Post-process mesh for better quality"""
        self.get_logger().info('')
        self.get_logger().info('🧽 Step 4/4: Cleaning mesh...')
        
        original_triangles = len(mesh.triangles)
        
        # Remove disconnected components (keep only largest)
        triangle_clusters, cluster_n_triangles, cluster_area = (
            mesh.cluster_connected_triangles()
        )
        triangle_clusters = np.asarray(triangle_clusters)
        cluster_n_triangles = np.asarray(cluster_n_triangles)
        
        largest_cluster_idx = cluster_n_triangles.argmax()
        triangles_to_remove = triangle_clusters != largest_cluster_idx
        mesh.remove_triangles_by_mask(triangles_to_remove)
        
        removed = triangles_to_remove.sum()
        if removed > 0:
            self.get_logger().info(f'   ✅ Removed {removed:,} isolated triangles')
        
        # Remove degenerate geometry
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_non_manifold_edges()
        
        self.get_logger().info(f'   ✅ Final: {len(mesh.vertices):,} vertices, {len(mesh.triangles):,} triangles')
        
        return mesh
    
    def save_mesh(self, mesh):
        """Save mesh - OBJ only (perfect for Meshroom/Blender)"""
        self.get_logger().info('')
        self.get_logger().info('💾 Saving mesh...')
        
        obj_file = f"{self.output_dir}/model_{self.timestamp}.obj"
        
        # Save as OBJ (ASCII format, best compatibility)
        o3d.io.write_triangle_mesh(obj_file, mesh, write_ascii=True)
        
        obj_size = os.path.getsize(obj_file) / (1024 * 1024)  # MB
        self.get_logger().info(f'   ✅ OBJ: {obj_size:.2f} MB')
        
        # Check if MTL file was created (material file)
        mtl_file = obj_file.replace('.obj', '.mtl')
        if os.path.exists(mtl_file):
            self.get_logger().info(f'   ✅ MTL: material file created')
    
    def print_summary(self):
        """Print final summary"""
        self.get_logger().info('')
        self.get_logger().info('📊 Summary:')
        self.get_logger().info(f'   Session ID: {self.timestamp}')
        self.get_logger().info(f'   Frames: {self.frame_count}')
        self.get_logger().info(f'   Points: {self.point_count:,}')
        self.get_logger().info('')
        self.get_logger().info('📁 Output Files:')
        self.get_logger().info(f'   • scan_clean_{self.timestamp}.pcd')
        self.get_logger().info(f'   • model_{self.timestamp}.obj')
        self.get_logger().info(f'   • model_{self.timestamp}.mtl')
        self.get_logger().info('')
        self.get_logger().info('🪟 Windows Path:')
        self.get_logger().info(f'   C:\\Users\\nieve\\unitree_lidar_project\\data\\')
        self.get_logger().info('')
        self.get_logger().info('🎨 Next Steps:')
        self.get_logger().info('   1. Take 50-100 photos of the scanned area')
        self.get_logger().info('      • 70% overlap between photos')
        self.get_logger().info('      • Even lighting')
        self.get_logger().info('      • Multiple angles/heights')
        self.get_logger().info('')
        self.get_logger().info('   2. Install Meshroom (free):')
        self.get_logger().info('      https://alicevision.org/#meshroom')
        self.get_logger().info('')
        self.get_logger().info('   3. Import to Meshroom:')
        self.get_logger().info('      • Add your photos')
        self.get_logger().info(f'      • Import model_{self.timestamp}.obj as mesh')
        self.get_logger().info('      • Disable Meshing node')
        self.get_logger().info('      • Run Texturing')
        self.get_logger().info('')
        self.get_logger().info('   4. Get photorealistic 3D model! 🎉')
        self.get_logger().info('')
        self.get_logger().info('💡 Tip: See PHOTOREALISTIC_3D_RECONSTRUCTION.md for details')
        self.get_logger().info('')

def main(args=None):
    # Dependency check
    try:
        import open3d
    except ImportError:
        print('')
        print('=' * 70)
        print('❌ Open3D not installed!')
        print('=' * 70)
        print('')
        print('Install it with:')
        print('  pip3 install open3d')
        print('')
        print('Then run this script again.')
        print('')
        return
    
    rclpy.init(args=args)
    
    try:
        node = OptimizedMeshGenerator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
