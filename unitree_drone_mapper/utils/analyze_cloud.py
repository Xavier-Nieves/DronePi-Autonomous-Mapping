#!/usr/bin/env python3
"""
analyze_cloud.py — Point cloud analysis and statistics export.

Analyzes a point cloud and exports statistics to CSV/Excel for inspection.
Useful for understanding Z distribution, setting ground height thresholds,
and debugging classification issues.

Output folder: {bag_path}/analysis/   (NOT debug/ to avoid conflicts)
    - {session}_stats.csv           Summary statistics
    - {session}_z_histogram.csv     Z value distribution (for plotting)
    - {session}_z_percentiles.csv   Percentile values (for threshold selection)
    - {session}_sample_points.csv   Sample of actual points (for inspection)
    - {session}_ground_analysis.csv Ground detection recommendations
    - {session}_analysis.xlsx       Combined Excel workbook (if openpyxl installed)

Usage:
    python analyze_cloud.py --bag C:\\path\\to\\scan_20260319_230358
    python analyze_cloud.py --ply C:\\path\\to\\combined_cloud.ply
    python analyze_cloud.py --bag C:\\path\\to\\bag --output C:\\custom\\output\\path
"""

import argparse
import sys
from pathlib import Path
from datetime import datetime

import numpy as np

try:
    import pandas as pd
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False


def load_points_from_ply(ply_path: Path) -> np.ndarray:
    """Load points from a PLY file."""
    try:
        import trimesh
        cloud = trimesh.load(str(ply_path))
        if hasattr(cloud, 'vertices'):
            return np.asarray(cloud.vertices, dtype=np.float32)
        else:
            raise ValueError("PLY file does not contain vertices")
    except Exception as e:
        print(f"[ERROR] Failed to load PLY: {e}")
        sys.exit(1)


def load_points_from_bag(bag_path: Path, max_frames: int = None) -> np.ndarray:
    """Load points from a ROS bag."""
    sys.path.insert(0, str(Path(__file__).parent))
    try:
        from mesh_tools.bag_reader import BagReader
        reader = BagReader(bag_path, max_frames=max_frames)
        return reader.extract()
    except Exception as e:
        print(f"[ERROR] Failed to read bag: {e}")
        sys.exit(1)


def analyze_points(pts: np.ndarray, output_dir: Path, session_id: str):
    """
    Analyze point cloud and export statistics.
    """
    print(f"\n{'='*60}")
    print(f"  Point Cloud Analysis")
    print(f"  Session: {session_id}")
    print(f"  Points:  {len(pts):,}")
    print(f"  Output:  {output_dir}")
    print(f"{'='*60}")

    # ── Basic Statistics ──────────────────────────────────────────────────────
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    
    x_range = x.max() - x.min()
    y_range = y.max() - y.min()
    z_range = z.max() - z.min()
    volume = x_range * y_range * z_range
    density = len(pts) / volume if volume > 0 else 0
    
    stats = {
        'Metric': [
            'Total Points',
            'X Min (m)', 'X Max (m)', 'X Range (m)', 'X Mean (m)', 'X Std (m)',
            'Y Min (m)', 'Y Max (m)', 'Y Range (m)', 'Y Mean (m)', 'Y Std (m)',
            'Z Min (m)', 'Z Max (m)', 'Z Range (m)', 'Z Mean (m)', 'Z Std (m)',
            'Bounding Box Volume (m³)',
            'Point Density (pts/m³)',
            'Analysis Date',
        ],
        'Value': [
            f"{len(pts):,}",
            f"{x.min():.4f}", f"{x.max():.4f}", f"{x_range:.4f}", f"{x.mean():.4f}", f"{x.std():.4f}",
            f"{y.min():.4f}", f"{y.max():.4f}", f"{y_range:.4f}", f"{y.mean():.4f}", f"{y.std():.4f}",
            f"{z.min():.4f}", f"{z.max():.4f}", f"{z_range:.4f}", f"{z.mean():.4f}", f"{z.std():.4f}",
            f"{volume:.2f}",
            f"{density:.2f}",
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        ]
    }
    
    # ── Z Percentiles (for ground threshold selection) ────────────────────────
    percentiles = [1, 2, 5, 10, 15, 20, 25, 30, 40, 50, 60, 70, 75, 80, 85, 90, 95, 98, 99]
    z_percentiles = {
        'Percentile (%)': percentiles,
        'Z Value (m)': [f"{np.percentile(z, p):.4f}" for p in percentiles],
        'Points Below': [f"{int(len(z) * p / 100):,}" for p in percentiles],
        'Points Above': [f"{int(len(z) * (100-p) / 100):,}" for p in percentiles],
    }
    
    print("\n  Z-Value Percentiles:")
    print("  " + "-"*55)
    print(f"  {'Percentile':>12} {'Z Value':>12} {'Points Below':>15}")
    print("  " + "-"*55)
    for p in [5, 10, 20, 25, 30, 50, 75, 90, 95]:
        z_val = np.percentile(z, p)
        pts_below = int(len(z) * p / 100)
        print(f"  {p:>10}th  {z_val:>10.3f}m  {pts_below:>15,}")
    print("  " + "-"*55)
    
    # ── Z Histogram (for visualization) ───────────────────────────────────────
    bin_width = 0.20  # 20cm bins
    z_min_hist, z_max_hist = z.min(), z.max()
    n_bins = max(10, min(100, int((z_max_hist - z_min_hist) / bin_width)))
    
    hist, bin_edges = np.histogram(z, bins=n_bins)
    bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
    cumulative = np.cumsum(hist)
    
    z_histogram = {
        'Bin Center (m)': [f"{c:.4f}" for c in bin_centers],
        'Bin Min (m)': [f"{bin_edges[i]:.4f}" for i in range(len(bin_edges)-1)],
        'Bin Max (m)': [f"{bin_edges[i+1]:.4f}" for i in range(len(bin_edges)-1)],
        'Point Count': hist.tolist(),
        'Percentage (%)': [f"{100*h/len(z):.2f}" for h in hist],
        'Cumulative Count': cumulative.tolist(),
        'Cumulative (%)': [f"{100*c/len(z):.2f}" for c in cumulative],
    }
    
    # ── Sample Points (random sample for inspection) ──────────────────────────
    n_sample = min(1000, len(pts))
    sample_idx = np.random.choice(len(pts), n_sample, replace=False)
    sample_pts = pts[sample_idx]
    
    # Sort by Z for easier reading
    sort_idx = np.argsort(sample_pts[:, 2])
    sample_pts = sample_pts[sort_idx]
    
    sample_points = {
        'Index': list(range(n_sample)),
        'X (m)': [f"{p[0]:.4f}" for p in sample_pts],
        'Y (m)': [f"{p[1]:.4f}" for p in sample_pts],
        'Z (m)': [f"{p[2]:.4f}" for p in sample_pts],
        'Distance from Origin (m)': [f"{np.linalg.norm(p):.4f}" for p in sample_pts],
    }
    
    # ── Nearest Neighbor Statistics (for BPA radius) ──────────────────────────
    nn_stats = None
    print("\n  Computing nearest neighbor distances...")
    try:
        from scipy.spatial import KDTree
        nn_sample_size = min(5000, len(pts))
        nn_idx = np.random.choice(len(pts), nn_sample_size, replace=False)
        nn_pts = pts[nn_idx]
        tree = KDTree(nn_pts)
        dists, _ = tree.query(nn_pts, k=2)
        nn_dists = dists[:, 1]
        
        nn_stats = {
            'Metric': [
                'Sample Size',
                'NN Min (m)', 'NN Max (m)', 'NN Mean (m)', 'NN Median (m)', 'NN Std (m)',
                'Suggested BPA Radius (median × 2.5)',
                'Suggested BPA Radius Clamped (0.03-0.50m)',
            ],
            'Value': [
                f"{nn_sample_size:,}",
                f"{nn_dists.min():.4f}",
                f"{nn_dists.max():.4f}",
                f"{nn_dists.mean():.4f}",
                f"{np.median(nn_dists):.4f}",
                f"{nn_dists.std():.4f}",
                f"{np.median(nn_dists) * 2.5:.4f}",
                f"{max(0.03, min(0.50, np.median(nn_dists) * 2.5)):.4f}",
            ]
        }
        
        suggested_radius = np.median(nn_dists) * 2.5
        clamped_radius = max(0.03, min(0.50, suggested_radius))
        print(f"    Median NN distance: {np.median(nn_dists):.4f}m")
        print(f"    Suggested BPA radius: {suggested_radius:.4f}m")
        if clamped_radius != suggested_radius:
            print(f"    Clamped BPA radius: {clamped_radius:.4f}m")
        
    except ImportError:
        print("    [SKIP] scipy not available for NN calculation")
    except Exception as e:
        print(f"    [SKIP] NN calculation failed: {e}")
    
    # ── Ground Detection Analysis ─────────────────────────────────────────────
    # Find the densest Z region (likely floor)
    peak_idx = np.argmax(hist)
    floor_estimate = bin_centers[peak_idx]
    
    # Also find lowest significant bin (>1% of points)
    threshold_count = len(z) * 0.01
    lowest_significant = None
    for i, count in enumerate(hist):
        if count >= threshold_count:
            lowest_significant = bin_centers[i]
            break
    
    ground_analysis = {
        'Metric': [
            'Z Minimum (m)',
            'Z 5th Percentile (m)',
            'Z 10th Percentile (m)',
            'Z 20th Percentile (m)',
            'Densest Z Bin (m)',
            'Lowest Significant Bin (>1%) (m)',
            'Recommended --ground-threshold',
            'Recommended --ground-cell-size',
        ],
        'Value': [
            f"{z.min():.4f}",
            f"{np.percentile(z, 5):.4f}",
            f"{np.percentile(z, 10):.4f}",
            f"{np.percentile(z, 20):.4f}",
            f"{floor_estimate:.4f}",
            f"{lowest_significant:.4f}" if lowest_significant else "N/A",
            "0.3 to 0.5",
            "0.5 to 1.0",
        ],
        'Notes': [
            'Absolute minimum (may include outliers)',
            'Robust floor estimate',
            'Conservative floor estimate',
            'Original Z-percentile method',
            'Where most low points cluster',
            'Lowest bin with significant points',
            'Height above local ground = ground',
            'Smaller = more precise, slower',
        ]
    }
    
    # ── Export to CSV/Excel ───────────────────────────────────────────────────
    output_dir.mkdir(parents=True, exist_ok=True)
    
    if HAS_PANDAS:
        df_stats = pd.DataFrame(stats)
        df_percentiles = pd.DataFrame(z_percentiles)
        df_histogram = pd.DataFrame(z_histogram)
        df_sample = pd.DataFrame(sample_points)
        df_ground = pd.DataFrame(ground_analysis)
        
        # Save individual CSVs
        df_stats.to_csv(output_dir / f"{session_id}_stats.csv", index=False)
        df_percentiles.to_csv(output_dir / f"{session_id}_z_percentiles.csv", index=False)
        df_histogram.to_csv(output_dir / f"{session_id}_z_histogram.csv", index=False)
        df_sample.to_csv(output_dir / f"{session_id}_sample_points.csv", index=False)
        df_ground.to_csv(output_dir / f"{session_id}_ground_analysis.csv", index=False)
        
        # Save combined Excel file
        try:
            excel_path = output_dir / f"{session_id}_analysis.xlsx"
            with pd.ExcelWriter(excel_path, engine='openpyxl') as writer:
                df_stats.to_excel(writer, sheet_name='Summary', index=False)
                df_ground.to_excel(writer, sheet_name='Ground Analysis', index=False)
                df_percentiles.to_excel(writer, sheet_name='Z Percentiles', index=False)
                df_histogram.to_excel(writer, sheet_name='Z Histogram', index=False)
                df_sample.to_excel(writer, sheet_name='Sample Points', index=False)
                if nn_stats:
                    pd.DataFrame(nn_stats).to_excel(writer, sheet_name='NN Stats', index=False)
            print(f"\n  [OK] Excel: {excel_path.name}")
        except ImportError:
            print(f"\n  [WARN] openpyxl not installed - Excel export skipped")
            print(f"         Install with: pip install openpyxl")
        except Exception as e:
            print(f"\n  [WARN] Excel export failed: {e}")
        
        print(f"  [OK] CSVs saved to: {output_dir}")
        
    else:
        # Manual CSV export without pandas
        print("\n  [WARN] pandas not installed - using basic CSV export")
        print("         Install with: pip install pandas")
        
        def write_csv(filepath, data_dict):
            with open(filepath, 'w') as f:
                headers = list(data_dict.keys())
                f.write(','.join(headers) + '\n')
                n_rows = len(list(data_dict.values())[0])
                for i in range(n_rows):
                    row = [str(data_dict[h][i]) for h in headers]
                    f.write(','.join(row) + '\n')
        
        write_csv(output_dir / f"{session_id}_stats.csv", stats)
        write_csv(output_dir / f"{session_id}_z_percentiles.csv", z_percentiles)
        write_csv(output_dir / f"{session_id}_z_histogram.csv", z_histogram)
        write_csv(output_dir / f"{session_id}_sample_points.csv", sample_points)
        write_csv(output_dir / f"{session_id}_ground_analysis.csv", ground_analysis)
        
        print(f"\n  [OK] CSVs saved to: {output_dir}")
    
    # ── Print Summary ─────────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"  Analysis Complete")
    print(f"{'='*60}")
    print(f"  Output files in: {output_dir}")
    print(f"    - {session_id}_stats.csv")
    print(f"    - {session_id}_z_percentiles.csv")
    print(f"    - {session_id}_z_histogram.csv")
    print(f"    - {session_id}_sample_points.csv")
    print(f"    - {session_id}_ground_analysis.csv")
    if HAS_PANDAS:
        print(f"    - {session_id}_analysis.xlsx (combined)")
    
    # ── Recommendations ───────────────────────────────────────────────────────
    z_5 = np.percentile(z, 5)
    
    print(f"\n  Recommendations:")
    print(f"  " + "-"*55)
    print(f"    Floor estimate (densest bin): {floor_estimate:.3f}m")
    print(f"    Floor estimate (5th percentile): {z_5:.3f}m")
    print(f"")
    print(f"    Suggested command:")
    print(f"      python postprocess_mesh.py --bag <path> \\")
    print(f"        --ground-threshold 0.3 --ground-cell-size 0.5")
    print(f"  " + "-"*55)


def main():
    parser = argparse.ArgumentParser(
        description="Analyze point cloud and export statistics to CSV/Excel",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python analyze_cloud.py --bag C:\\path\\to\\scan_20260319_230358
  python analyze_cloud.py --ply C:\\path\\to\\combined_cloud.ply
  python analyze_cloud.py --bag C:\\path\\to\\bag --output C:\\my_analysis
        """)
    parser.add_argument("--bag", type=str, default=None,
                        help="Path to ROS bag directory")
    parser.add_argument("--ply", type=str, default=None,
                        help="Path to PLY file")
    parser.add_argument("--max-frames", type=int, default=None,
                        help="Max bag frames to read (for faster analysis)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output directory (default: {bag}/analysis/ or {ply_dir}/analysis/)")
    args = parser.parse_args()
    
    if not args.bag and not args.ply:
        print("[ERROR] Must specify --bag or --ply")
        parser.print_help()
        sys.exit(1)
    
    # Load points
    if args.ply:
        ply_path = Path(args.ply)
        print(f"\n  Loading PLY: {ply_path.name}")
        pts = load_points_from_ply(ply_path)
        session_id = ply_path.stem
        base_dir = ply_path.parent
    else:
        bag_path = Path(args.bag)
        print(f"\n  Loading bag: {bag_path.name}")
        pts = load_points_from_bag(bag_path, args.max_frames)
        session_id = bag_path.name
        base_dir = bag_path
    
    # Determine output directory - use 'analysis' subfolder (NOT 'debug')
    if args.output:
        output_dir = Path(args.output)
    else:
        output_dir = base_dir / "analysis"
    
    # Analyze
    analyze_points(pts, output_dir, session_id)


if __name__ == "__main__":
    main()
