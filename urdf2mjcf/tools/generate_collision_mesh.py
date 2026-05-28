#!/usr/bin/env python3

import argparse
import glob
import os
import trimesh
from multiprocessing import Pool, cpu_count
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


def _resolve_inputs(raw_inputs, output_dir_override, supported_extensions):
    """Resolve CLI inputs to a list of (mesh_path, outdirpath) pairs.

    Each input arg is handled in order:
      1. Existing directory  → find mesh files directly inside it (non-recursive).
      2. Existing file       → use it directly.
      3. Anything else       → treat as a glob pattern; if no matches, retry
                               recursively (case-insensitive) so that ``*.stl``
                               passed from a directory whose STL files live in
                               subdirectories (or are uppercase ``.STL``) is still
                               found.

    When *output_dir_override* is None each output file is placed in a
    ``collision/`` sub-directory **next to its source file**, so files from
    different directories each get their own ``collision/`` folder.
    """
    # Case-insensitive recursive patterns for each supported extension
    _CI_RECURSIVE = {
        'stl': '**/*.[sS][tT][lL]',
        'obj': '**/*.[oO][bB][jJ]',
        'dae': '**/*.[dD][aA][eE]',
    }
    _CI_DIRECT = {
        'stl': '*.[sS][tT][lL]',
        'obj': '*.[oO][bB][jJ]',
        'dae': '*.[dD][aA][eE]',
    }

    def _ext_ok(path):
        return os.path.splitext(path)[1][1:].lower() in supported_extensions

    def _make_pair(f):
        outdir = output_dir_override or os.path.join(os.path.dirname(f), 'collision')
        return (f, os.path.abspath(outdir))

    def _recursive_from(search_dir, base_pat):
        """Find files matching base_pat (e.g. '*.stl') recursively under search_dir,
        using case-insensitive patterns for all supported extensions."""
        ext = os.path.splitext(base_pat)[1][1:].lower()
        ci_pats = [_CI_RECURSIVE[e] for e in supported_extensions] if not ext or ext not in supported_extensions \
            else [_CI_RECURSIVE[ext]] if ext in _CI_RECURSIVE else [os.path.join('**', base_pat)]
        found = []
        for pat in ci_pats:
            found.extend(glob.glob(os.path.join(search_dir, pat), recursive=True))
        return sorted(set(found))

    pairs = []
    for raw in raw_inputs:
        abs_p = os.path.abspath(raw)

        if os.path.isdir(abs_p):
            # Directory: find mesh files non-recursively (case-insensitive)
            found = []
            for pat in _CI_DIRECT.values():
                found.extend(glob.glob(os.path.join(abs_p, pat)))
            if not found:
                print(f"No mesh files found in directory: {abs_p}")
                exit(1)
            for f in sorted(found):
                pairs.append(_make_pair(f))

        elif os.path.isfile(abs_p):
            ext = os.path.splitext(abs_p)[1][1:].lower()
            if ext not in supported_extensions:
                print(f"Error: Unsupported file format '{ext}' for {abs_p}")
                print(f"Supported formats: {', '.join(sorted(supported_extensions))}")
                exit(1)
            pairs.append(_make_pair(abs_p))

        else:
            # Not a file or directory — treat as a glob pattern.
            # Step 1: literal glob (works when shell expands it, or cwd has matches)
            matches = glob.glob(raw)

            if not matches:
                # Step 2: recursive case-insensitive search from the pattern's
                # directory component.
                # For "*.stl"            → search cwd recursively
                # For "/abs/path/*.stl"  → search /abs/path recursively (never cwd)
                dirpart = os.path.dirname(raw)
                base_pat = os.path.basename(raw)
                search_dir = os.path.abspath(dirpart) if dirpart else os.getcwd()

                if os.path.isdir(search_dir):
                    matches = _recursive_from(search_dir, base_pat)

            if not matches:
                print(f"Error: No files found matching pattern: {raw}")
                exit(1)

            for f in sorted(os.path.abspath(m) for m in matches):
                if _ext_ok(f):
                    pairs.append(_make_pair(f))

            if not pairs:
                print(f"Error: No supported mesh files found matching pattern: {raw}")
                print(f"Supported formats: {', '.join(sorted(supported_extensions))}")
                exit(1)

    return pairs


def _process_pair(args_tuple):
    """Top-level worker for multiprocessing — unpacks (mesh_path, outdirpath, kwargs)."""
    mesh_path, outdirpath, kwargs = args_tuple
    os.makedirs(outdirpath, exist_ok=True)
    method = kwargs.pop('method')
    if method == 'coacd':
        return process_mesh_coacd(mesh_path, outdirpath, **kwargs)
    return process_mesh_convex_hull(mesh_path, outdirpath, **kwargs)


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

  # Glob pattern — works even when the shell cannot expand it
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
        help="Input directory, single mesh file, or glob pattern (STL, OBJ, DAE). "
             "Unmatched globs are searched recursively in subdirectories."
    )
    parser.add_argument(
        "-o", "--output-dir",
        type=str,
        default=None,
        help="Output directory for all collision meshes. "
             "Default: a 'collision/' sub-directory next to each source file."
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

    supported_extensions = {'stl', 'obj', 'dae'}
    output_dir_override = os.path.abspath(args.output_dir) if args.output_dir else None

    pairs = _resolve_inputs(args.inputs, output_dir_override, supported_extensions)
    total = len(pairs)

    if args.verbose or args.visualize:
        print(f"Found {total} mesh file(s) to process (method: {args.method})")
        unique_outdirs = sorted({outdir for _, outdir in pairs})
        for d in unique_outdirs:
            print(f"  Output dir: {d}")
    else:
        print(f"Processing {total} mesh file(s) ({args.method})...")

    # Build per-file kwargs
    if args.method == "coacd":
        extra = dict(method='coacd', threshold=args.threshold,
                     max_convex_hull=args.max_convex_hull, visualize=False, verbose=args.verbose)
    else:
        extra = dict(method='convex-hull', num_points=args.num_points,
                     visualize=False, verbose=args.verbose)

    work_items = [(mesh_path, outdirpath, dict(extra)) for mesh_path, outdirpath in pairs]

    if args.visualize:
        if args.verbose:
            print("Visualization enabled — processing sequentially")
        results = []
        iterator = (tqdm(work_items, desc="Processing", unit="mesh")
                    if TQDM_AVAILABLE and not args.verbose else work_items)
        for mesh_path, outdirpath, kwargs in iterator:
            os.makedirs(outdirpath, exist_ok=True)
            kwargs['visualize'] = True
            method = kwargs.pop('method')
            if method == 'coacd':
                results.append(process_mesh_coacd(mesh_path, outdirpath, **kwargs))
            else:
                results.append(process_mesh_convex_hull(mesh_path, outdirpath, **kwargs))
    else:
        num_processes = min(args.jobs or cpu_count(), total)
        if args.verbose:
            print(f"Using {num_processes} parallel process(es)")

        with Pool(processes=num_processes) as pool:
            if TQDM_AVAILABLE and not args.verbose:
                results = list(tqdm(
                    pool.imap(_process_pair, work_items),
                    total=total,
                    desc="Processing",
                    unit="mesh"
                ))
            else:
                results = pool.map(_process_pair, work_items)

    # Summary
    successful = sum(results)
    if args.verbose:
        print(f"\nProcessing complete: {successful}/{total} files processed successfully")
        if successful < total:
            print(f"Warning: {total - successful} file(s) failed")
        else:
            print("All files processed successfully!")
    else:
        if successful == total:
            print(f"✓ Successfully processed all {total} files")
        else:
            print(f"⚠ Processed {successful}/{total} files ({total - successful} failed)")


if __name__ == "__main__":
    main()
