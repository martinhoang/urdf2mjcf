#!/usr/bin/env python3

import glob
import os
import open3d as o3d
from multiprocessing import Pool, cpu_count
from functools import partial


def process_mesh(stlpath, outdirpath, visualize=False):
    """
    Process a single mesh file to generate collision mesh.
    
    Args:
        stlpath: Path to input STL file
        outdirpath: Output directory path
        visualize: Whether to show visualization (disabled for parallel processing)
    """
    print(f"Processing {os.path.basename(stlpath)}")
    
    try:
        # Read and process mesh
        mesh = o3d.io.read_triangle_mesh(stlpath)
        mesh.compute_vertex_normals()
        
        # Generate point cloud and convex hull
        pcl = mesh.sample_points_poisson_disk(number_of_points=5000)
        hull, _ = pcl.compute_convex_hull()
        hull.orient_triangles()
        
        # Visualization (only in single-threaded mode)
        if visualize:
            o3d.visualization.draw_geometries([hull])
        
        hull.compute_vertex_normals()
        
        # Write output
        output_path = os.path.join(outdirpath, os.path.basename(stlpath))
        o3d.io.write_triangle_mesh(output_path, hull)
        
        print(f"Completed {os.path.basename(stlpath)}")
        return True
        
    except Exception as e:
        print(f"Error processing {os.path.basename(stlpath)}: {str(e)}")
        return False


def main():
    """Main entry point for command-line usage."""
    indirpath = "meshes"
    outdirpath = "meshes/collision"

    # Get absolute paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)
    mesh_pattern = os.path.join(parent_dir, indirpath, "*.stl")
    stlpaths = glob.glob(mesh_pattern)
    
    full_outdirpath = os.path.join(parent_dir, outdirpath)
    os.makedirs(full_outdirpath, exist_ok=True)

    if not stlpaths:
        print(f"No STL files found in {mesh_pattern}")
        exit(1)

    print(f"Found {len(stlpaths)} STL files to process")
    
    # Determine number of processes (use all cores, but cap at number of files)
    num_processes = min(cpu_count(), len(stlpaths))
    print(f"Using {num_processes} parallel processes")

    # Create partial function with fixed outdirpath
    process_func = partial(process_mesh, outdirpath=full_outdirpath, visualize=False)

    # Process files in parallel
    with Pool(processes=num_processes) as pool:
        results = pool.map(process_func, stlpaths)
    
    # Summary
    successful = sum(results)
    total = len(stlpaths)
    print(f"\nProcessing complete: {successful}/{total} files processed successfully")
    
    if successful < total:
        print(f"Warning: {total - successful} files failed to process")
    else:
        print("All files processed successfully!")


if __name__ == "__main__":
    main()
