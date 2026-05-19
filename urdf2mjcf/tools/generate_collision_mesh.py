#!/usr/bin/env python3

import argparse
import glob
import os
import trimesh
from multiprocessing import Pool, cpu_count
from functools import partial
import numpy as np

# Try to import CoACD for advanced convex decomposition
try:
    import coacd
    COACD_AVAILABLE = True
except ImportError:
    COACD_AVAILABLE = False
    coacd = None

# Try to import tqdm for progress bar
try:
    from tqdm import tqdm
    TQDM_AVAILABLE = True
except ImportError:
    TQDM_AVAILABLE = False
    tqdm = None


def process_mesh_coacd(mesh_path, outdirpath, threshold=0.05, max_convex_hull=-1, visualize=False, verbose=False):
    """
    Process a single mesh file using CoACD (Collision-Aware Convex Decomposition).
    This method provides better collision approximation with fewer components.
    
    Args:
        mesh_path: Path to input mesh file (STL, OBJ, PLY, OFF, etc.)
        outdirpath: Output directory path
        threshold: Concavity threshold for decomposition (0.01~1, default: 0.05)
        max_convex_hull: Max number of convex hulls (-1 for no limit, default: -1)
        visualize: Whether to show visualization (disabled for parallel processing)
        verbose: Whether to print verbose output
    
    Returns:
        True if successful, False otherwise
    """
    if not COACD_AVAILABLE:
        print("Error: CoACD not available. Install with: pip install coacd trimesh")
        return False
    
    if verbose:
        print(f"Processing {os.path.basename(mesh_path)} with CoACD")
    
    try:
        # Load mesh with trimesh
        mesh = trimesh.load(mesh_path, force="mesh")
        
        # Convert to CoACD mesh format
        coacd_mesh = coacd.Mesh(mesh.vertices, mesh.faces)
        
        # Run CoACD decomposition
        parts = coacd.run_coacd(
            coacd_mesh,
            threshold=threshold,
            max_convex_hull=max_convex_hull,
            preprocess_mode="auto",  # Automatically check manifoldness
            preprocess_resolution=50,
            mcts_iterations=100,
            mcts_max_depth=3,
            mcts_nodes=20,
            resolution=2000
        )
        
        if not parts:
            print(f"Warning: CoACD produced no parts for {os.path.basename(mesh_path)}")
            return False
        
        if verbose:
            print(f"  Generated {len(parts)} convex components")
        
        # Visualization (only in single-threaded mode)
        if visualize:
            meshes = []
            for part in parts:
                if isinstance(part, trimesh.Trimesh):
                    meshes.append(part)
                else:
                    meshes.append(trimesh.Trimesh(vertices=part[0], faces=part[1]))
            trimesh.Scene(meshes).show()
        
        # Save output
        base_name = os.path.splitext(os.path.basename(mesh_path))[0]
        
        if len(parts) == 1:
            # Single component - save directly
            output_path = os.path.join(outdirpath, f"{base_name}.stl")
            # CoACD returns trimesh objects
            part_mesh = parts[0] if isinstance(parts[0], trimesh.Trimesh) else trimesh.Trimesh(vertices=parts[0][0], faces=parts[0][1])
            part_mesh.export(output_path)
        else:
            # Multiple components - save as separate files or combined
            # Option 1: Save as a single combined mesh
            combined_vertices = []
            combined_faces = []
            vertex_offset = 0
            
            for part in parts:
                # CoACD returns trimesh objects
                if isinstance(part, trimesh.Trimesh):
                    vertices = part.vertices
                    faces = part.faces
                else:
                    # If it's a tuple of (vertices, faces)
                    vertices = part[0]
                    faces = part[1]
                
                combined_vertices.append(vertices)
                combined_faces.append(faces + vertex_offset)
                vertex_offset += len(vertices)
            
            combined_vertices = np.vstack(combined_vertices)
            combined_faces = np.vstack(combined_faces)
            
            output_path = os.path.join(outdirpath, f"{base_name}.stl")
            mesh_out = trimesh.Trimesh(vertices=combined_vertices, faces=combined_faces)
            mesh_out.export(output_path)
            
            # Option 2: Also save individual components
            parts_dir = os.path.join(outdirpath, f"{base_name}_parts")
            os.makedirs(parts_dir, exist_ok=True)
            for i, part in enumerate(parts):
                part_path = os.path.join(parts_dir, f"{base_name}_part_{i:03d}.stl")
                if isinstance(part, trimesh.Trimesh):
                    part.export(part_path)
                else:
                    mesh_part = trimesh.Trimesh(vertices=part[0], faces=part[1])
                    mesh_part.export(part_path)
        
        if verbose:
            print(f"Completed {os.path.basename(mesh_path)}")
        return True
        
    except Exception as e:
        print(f"Error processing {os.path.basename(mesh_path)} with CoACD: {str(e)}")
        return False


def process_mesh_convex_hull(mesh_path, outdirpath, num_points=5000, visualize=False, verbose=False):
    """
    Process a single mesh file to generate collision mesh using simple convex hull.
    This is the fast method - produces a single convex approximation.

    Args:
        mesh_path: Path to input mesh file (STL, OBJ, PLY, OFF, etc.)
        outdirpath: Output directory path
        num_points: Unused; kept for API compatibility.
        visualize: Whether to show visualization (disabled for parallel processing)
        verbose: Whether to print verbose output

    Returns:
        True if successful, False otherwise
    """
    if verbose:
        print(f"Processing {os.path.basename(mesh_path)} with convex hull")

    try:
        mesh = trimesh.load(mesh_path, force="mesh")
        # Compute convex hull from all vertices to guarantee the true maximal hull.
        # (Sampling-based approaches can miss extremal vertices in small triangles.)
        hull = mesh.convex_hull

        if visualize:
            hull.show()

        output_path = os.path.join(outdirpath, os.path.basename(mesh_path))
        hull.export(output_path)

        if verbose:
            print(f"Completed {os.path.basename(mesh_path)}")
        return True

    except Exception as e:
        print(f"Error processing {os.path.basename(mesh_path)}: {str(e)}")
        return False


def process_mesh(mesh_path, outdirpath, visualize=False, verbose=False):
    """Generate a collision mesh using the convex hull method (default for pipeline use)."""
    return process_mesh_convex_hull(mesh_path, outdirpath, visualize=visualize, verbose=verbose)


def main():
    """Main entry point for command-line usage."""
    parser = argparse.ArgumentParser(
        description="Generate collision meshes from 3D mesh files using convex hull or CoACD approximation.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Supported Formats:
  STL, OBJ, DAE (COLLADA) - case-insensitive: .stl, .STL, .obj, .OBJ, .dae, .DAE

Methods:
  convex-hull : Simple convex hull (fast, single convex shape)
  coacd       : Collision-Aware Convex Decomposition (better accuracy, multiple convex shapes)

Examples:
  # Process all meshes in a directory (default method)
  %(prog)s /path/to/meshes

  # Process a single mesh file
  %(prog)s /path/to/mesh/robot.stl

  # Process multiple files via shell glob
  %(prog)s *.stl

  # Use CoACD for better collision approximation
  %(prog)s /path/to/meshes -m coacd

  # Customize CoACD threshold and max hulls
  %(prog)s *.stl -m coacd -t 0.03 -c 32

  # Custom output directory
  %(prog)s *.obj -o collision_meshes/
        """
    )
    parser.add_argument(
        "inputs",
        type=str,
        nargs="+",
        help="Input directory, single mesh file, or multiple mesh files (STL, OBJ, DAE). "
             "Shell globs like *.stl are accepted."
    )
    parser.add_argument(
        "-o", "--output-dir",
        type=str,
        default=None,
        help="Output directory for collision meshes (default: <input_dir>/collision)"
    )
    parser.add_argument(
        "-m", "--method",
        type=str,
        choices=["convex-hull", "coacd"],
        default="convex-hull",
        help="Method to use for collision mesh generation (default: convex-hull)"
    )
    
    # Convex hull specific parameters
    parser.add_argument(
        "-n", "--num-points",
        type=int,
        default=5000,
        help="[Convex-hull] Number of points to sample for convex hull generation (default: 5000)"
    )
    
    # CoACD specific parameters
    parser.add_argument(
        "-t", "--threshold",
        type=float,
        default=0.05,
        help="[CoACD] Concavity threshold for decomposition, 0.01~1 (default: 0.05). Lower = more detailed"
    )
    parser.add_argument(
        "-c", "--max-convex-hull",
        type=int,
        default=-1,
        help="[CoACD] Max number of convex hulls, -1 for no limit (default: -1)"
    )
    
    # General parameters
    parser.add_argument(
        "-j", "--jobs",
        type=int,
        default=None,
        help="Number of parallel processes (default: all CPU cores)"
    )
    parser.add_argument(
        "-v", "--visualize",
        action="store_true",
        help="Visualize meshes (disables parallel processing)"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose output (show detailed processing information)"
    )
    
    args = parser.parse_args()
    
    # Check if CoACD is available when requested
    if args.method == "coacd" and not COACD_AVAILABLE:
        print("Error: CoACD method requested but not available.")
        print("Install with: pip install coacd trimesh")
        exit(1)
    
    # Set up paths
    inputs = [os.path.abspath(p) for p in args.inputs]
    supported_extensions = {'stl', 'obj', 'dae'}

    if len(inputs) == 1 and os.path.isdir(inputs[0]):
        # Directory mode
        indirpath = inputs[0]
        case_insensitive_patterns = [
            '*.[sS][tT][lL]',
            '*.[oO][bB][jJ]',
            '*.[dD][aA][eE]',
        ]
        mesh_paths = []
        for pat in case_insensitive_patterns:
            mesh_paths.extend(glob.glob(os.path.join(indirpath, pat)))

        if not mesh_paths:
            print(f"No mesh files found in {indirpath}")
            print("Supported formats: stl, obj, dae (case-insensitive)")
            exit(1)
    else:
        # One or more explicit file paths (including shell-expanded globs)
        mesh_paths = []
        for p in inputs:
            if not os.path.isfile(p):
                print(f"Error: Path does not exist: {p}")
                exit(1)
            file_ext = os.path.splitext(p)[1][1:].lower()
            if file_ext not in supported_extensions:
                print(f"Error: Unsupported file format '{file_ext}'")
                print(f"Supported formats: {', '.join(sorted(supported_extensions))}")
                exit(1)
            mesh_paths.append(p)
        indirpath = os.path.dirname(mesh_paths[0])

    # Set up output directory
    if args.output_dir is None:
        outdirpath = os.path.join(indirpath, "collision")
    else:
        outdirpath = args.output_dir
    
    outdirpath = os.path.abspath(outdirpath)
    
    os.makedirs(outdirpath, exist_ok=True)

    # Print summary
    if args.verbose or args.visualize:
        print(f"Found {len(mesh_paths)} mesh files to process")
        print(f"Input directory: {indirpath}")
        print(f"Output directory: {outdirpath}")
        print(f"Method: {args.method}")
        
        if args.method == "convex-hull":
            print(f"Number of sample points: {args.num_points}")
        elif args.method == "coacd":
            print(f"CoACD threshold: {args.threshold}")
            print(f"CoACD max convex hulls: {args.max_convex_hull}")
    else:
        print(f"Processing {len(mesh_paths)} mesh files ({args.method})...")
    
    # Process with or without parallelization
    if args.visualize:
        if args.verbose:
            print("Visualization enabled - processing sequentially")
        results = []
        
        # Use tqdm if available and not in verbose mode
        iterator = tqdm(mesh_paths, desc="Processing", unit="mesh") if TQDM_AVAILABLE and not args.verbose else mesh_paths
        
        for mesh_path in iterator:
            if args.method == "coacd":
                result = process_mesh_coacd(
                    mesh_path, outdirpath, 
                    threshold=args.threshold,
                    max_convex_hull=args.max_convex_hull,
                    visualize=True,
                    verbose=args.verbose
                )
            else:
                result = process_mesh_convex_hull(
                    mesh_path, outdirpath, 
                    num_points=args.num_points,
                    visualize=True,
                    verbose=args.verbose
                )
            results.append(result)
    else:
        # Determine number of processes
        if args.jobs is not None:
            num_processes = min(args.jobs, len(mesh_paths))
        else:
            num_processes = min(cpu_count(), len(mesh_paths))
        
        if args.verbose:
            print(f"Using {num_processes} parallel processes")

        # Create partial function with fixed parameters
        if args.method == "coacd":
            process_func = partial(
                process_mesh_coacd,
                outdirpath=outdirpath,
                threshold=args.threshold,
                max_convex_hull=args.max_convex_hull,
                visualize=False,
                verbose=args.verbose
            )
        else:
            process_func = partial(
                process_mesh_convex_hull,
                outdirpath=outdirpath,
                num_points=args.num_points,
                visualize=False,
                verbose=args.verbose
            )

        # Process files in parallel with progress bar
        with Pool(processes=num_processes) as pool:
            if TQDM_AVAILABLE and not args.verbose:
                # Use tqdm with imap for real-time progress updates
                results = list(tqdm(
                    pool.imap(process_func, mesh_paths),
                    total=len(mesh_paths),
                    desc="Processing",
                    unit="mesh"
                ))
            else:
                results = pool.map(process_func, mesh_paths)
    
    # Summary
    successful = sum(results)
    total = len(mesh_paths)
    
    if args.verbose:
        print(f"\nProcessing complete: {successful}/{total} files processed successfully")
        if successful < total:
            print(f"Warning: {total - successful} files failed to process")
        else:
            print("All files processed successfully!")
    else:
        # Concise summary
        if successful == total:
            print(f"✓ Successfully processed all {total} files")
        else:
            print(f"⚠ Processed {successful}/{total} files ({total - successful} failed)")


if __name__ == "__main__":
    main()
