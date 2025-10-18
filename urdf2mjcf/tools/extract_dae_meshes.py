#!/usr/bin/env python3

import argparse
import json
import os
import sys
import numpy as np

# Try to import required libraries
try:
    import collada
    COLLADA_AVAILABLE = True
except ImportError:
    COLLADA_AVAILABLE = False
    collada = None

try:
    import trimesh
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False
    trimesh = None

try:
    from tqdm import tqdm
    TQDM_AVAILABLE = True
except ImportError:
    TQDM_AVAILABLE = False
    tqdm = None


def extract_meshes_from_dae(dae_file_path, output_dir, output_format='stl', verbose=False, progress_callback=None, scale_factor=None, separate_meshes=False):
    """
    Extracts all meshes from a DAE (COLLADA) file and saves them as individual files or a single combined file.
    
    This function can be imported and used in other modules (cli.py, converter.py, mesh_ops.py).
    Progress tracking is handled via an optional callback, making it suitable for both 
    CLI usage (with tqdm) and programmatic usage.
    
    Args:
        dae_file_path (str): Path to the input DAE file.
        output_dir (str): Directory to save the extracted mesh files.
        output_format (str): Output format ('stl', 'obj', 'ply'). Default is 'stl'.
        verbose (bool): Whether to print verbose output.
        progress_callback (callable, optional): Callback function called after each mesh is processed.
                                               Signature: callback(current, total)
                                               Use this for progress tracking in CLI or GUI applications.
        scale_factor (float, optional): Scale factor to apply to meshes. If None, auto-detect from DAE units
                                       and convert to meters. If DAE has no unit info, defaults to 0.001 (mm to m).
        separate_meshes (bool): If True (default), extract each mesh/material as a separate file with color info.
                               If False, combine all meshes into a single file (legacy/backward compatible behavior).
    
    Returns:
        tuple: (success_count, total_count, mesh_info) where:
               - success_count (int): Number of successfully extracted meshes
               - total_count (int): Total number of meshes found (or 1 if separate_meshes=False)
               - mesh_info (dict): Dictionary mapping mesh names to their metadata:
                   {
                       'mesh_name': {
                           'file': str,           # Output filename
                           'geometry': str,       # Geometry ID from DAE
                           'vertices': int,       # Vertex count
                           'faces': int,          # Face count
                           'material': str,       # Material name (if available)
                           'rgba': [r,g,b,a],    # RGBA color (if available)
                           'color': {...}         # Detailed color info (if available)
                       }
                   }
    
    Example:
        # Extract multiple meshes with colors (default)
        success, total, info = extract_meshes_from_dae('model.dae', 'output/')
        
        # Legacy mode: combine all into single mesh
        success, total, info = extract_meshes_from_dae('model.dae', 'output/', separate_meshes=False)
        
        # With explicit scale factor (e.g., mm to m)
        success, total, info = extract_meshes_from_dae('model.dae', 'output/', scale_factor=0.001)
        
        # With progress tracking
        def on_progress(current, total):
            print(f"Processing {current}/{total}")
        success, total, info = extract_meshes_from_dae('model.dae', 'output/', 
                                                        progress_callback=on_progress)
    """
    if not COLLADA_AVAILABLE:
        print("Error: pycollada not available. Install with: pip install pycollada")
        return 0, 0, {}
    
    if not TRIMESH_AVAILABLE:
        print("Error: trimesh not available. Install with: pip install trimesh")
        return 0, 0, {}
    
    if verbose:
        print(f"Loading DAE file: {dae_file_path}")
    
    try:
        # Load the DAE file
        dae = collada.Collada(dae_file_path)
        
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Check coordinate system up-axis
        up_axis = 'Z_UP'  # Default COLLADA convention
        if hasattr(dae, 'assetInfo') and dae.assetInfo is not None:
            if hasattr(dae.assetInfo, 'upaxis'):
                up_axis = dae.assetInfo.upaxis
        
        if verbose:
            print(f"DAE coordinate system: {up_axis}")
        
        # Detect unit and calculate scale factor if not explicitly provided
        if scale_factor is None:
            # Get unit from DAE asset
            unit_meter = 1.0  # Default to meter
            unit_name = "meter"
            
            if hasattr(dae, 'assetInfo') and dae.assetInfo is not None:
                if hasattr(dae.assetInfo, 'unitmeter') and dae.assetInfo.unitmeter is not None:
                    unit_meter = float(dae.assetInfo.unitmeter)
                if hasattr(dae.assetInfo, 'unitname') and dae.assetInfo.unitname is not None:
                    unit_name = dae.assetInfo.unitname
            
            # Calculate scale factor to convert to meters
            if unit_meter != 1.0:
                scale_factor = unit_meter
                if verbose:
                    print(f"DAE unit detected: {unit_name} (meter scale: {unit_meter})")
                    print(f"Applying scale factor: {scale_factor} to convert to meters")
            else:
                # No unit info found - check if unit_name suggests non-meter units
                unit_name_lower = unit_name.lower()
                if 'millimeter' in unit_name_lower or 'mm' == unit_name_lower:
                    scale_factor = 0.001
                    if verbose:
                        print(f"DAE unit detected: {unit_name}, applying scale factor: {scale_factor} (mm to m)")
                elif 'centimeter' in unit_name_lower or 'cm' == unit_name_lower:
                    scale_factor = 0.01
                    if verbose:
                        print(f"DAE unit detected: {unit_name}, applying scale factor: {scale_factor} (cm to m)")
                elif 'inch' in unit_name_lower or 'in' == unit_name_lower:
                    scale_factor = 0.0254
                    if verbose:
                        print(f"DAE unit detected: {unit_name}, applying scale factor: {scale_factor} (inch to m)")
                else:
                    # Default to millimeter conversion if no unit info
                    scale_factor = 0.001
                    if verbose:
                        print(f"No unit info in DAE, defaulting to scale factor: {scale_factor} (assuming mm to m)")
        else:
            if verbose:
                print(f"Using explicit scale factor: {scale_factor}")
        
        if verbose:
            print(f"Found {len(dae.geometries)} geometries in the DAE file")
        
        # Count total extractable primitives (only supported types)
        total_meshes = 0
        for geometry in dae.geometries:
            for primitive in geometry.primitives:
                # Only count primitives we can actually extract
                if isinstance(primitive, (collada.polylist.Polylist, collada.triangleset.TriangleSet)):
                    total_meshes += 1
        
        if total_meshes == 0:
            print(f"Warning: No extractable meshes found in {dae_file_path}")
            return 0, 0, {}
        
        success_count = 0
        mesh_index = 0
        mesh_info = {}
        
        # If not separating meshes, collect all meshes for combination
        combined_meshes = [] if not separate_meshes else None
        
        # Iterate through all geometries in the DAE file
        for geom_idx, geometry in enumerate(dae.geometries):
            geometry_name = geometry.id if geometry.id else f"geometry_{geom_idx}"
            
            for prim_idx, primitive in enumerate(geometry.primitives):
                try:
                    # Check if primitive is a supported type
                    if isinstance(primitive, (collada.polylist.Polylist, collada.triangleset.TriangleSet)):
                        # Extract vertices and indices
                        vertices = primitive.vertex.reshape(-1, 3)
                        
                        # Get the vertex indices
                        if hasattr(primitive, 'vertex_index'):
                            indices = primitive.vertex_index.reshape(-1, 3)
                        else:
                            # For some primitive types, we need to generate indices
                            indices = None
                        
                        # Create a trimesh object
                        if indices is not None:
                            mesh = trimesh.Trimesh(vertices=vertices, faces=indices)
                        else:
                            # If no indices, assume sequential triangulation
                            n_vertices = len(vertices)
                            if n_vertices % 3 == 0:
                                indices = [[i, i+1, i+2] for i in range(0, n_vertices, 3)]
                                mesh = trimesh.Trimesh(vertices=vertices, faces=indices)
                            else:
                                if verbose:
                                    print(f"Warning: Cannot triangulate mesh {geometry_name}_{prim_idx} (vertices: {n_vertices})")
                                continue
                        
                        # Apply scale factor to convert units (e.g., mm to meters)
                        if scale_factor != 1.0:
                            mesh.apply_scale(scale_factor)
                        
                        # Handle coordinate system conversion
                        # pycollada converts Z-up (COLLADA default) to Y-up automatically.
                        # To restore the original Z-up orientation, we apply a +90 degree rotation around the X-axis.
                        if up_axis == 'Z_UP':
                            # Rotate from Y-up back to Z-up: rotate +90° around X-axis
                            rotation_matrix = trimesh.transformations.rotation_matrix(
                                angle=np.radians(90),
                                direction=[1, 0, 0]
                            )
                            mesh.apply_transform(rotation_matrix)
                        
                        # Extract material/color information
                        material_info = {}
                        material_name = None
                        rgba = None
                        
                        # Get material from primitive
                        if hasattr(primitive, 'material') and primitive.material is not None:
                            try:
                                # Get the material symbol/ID
                                material_symbol = primitive.material
                                
                                # The material is actually a string reference, need to look it up
                                # Try to get the actual material from the DAE's material library
                                if isinstance(material_symbol, str):
                                    material_name = material_symbol
                                    # Look up in the DAE's materials
                                    if material_symbol in dae.materials:
                                        material_obj = dae.materials[material_symbol]
                                        effect = material_obj.effect
                                    else:
                                        effect = None
                                else:
                                    # Sometimes material is already the object
                                    material_name = getattr(material_symbol, 'id', str(material_symbol))
                                    effect = getattr(material_symbol, 'effect', None)
                                
                                # Extract colors from effect if available
                                if effect is not None:
                                    # Get diffuse color (most important for URDF)
                                    if hasattr(effect, 'diffuse') and effect.diffuse is not None:
                                        diffuse = effect.diffuse
                                        if isinstance(diffuse, (tuple, list)) and len(diffuse) >= 3:
                                            rgba = list(diffuse[:4]) if len(diffuse) >= 4 else list(diffuse[:3]) + [1.0]
                                            material_info['rgba'] = rgba
                                        
                                    # Get ambient color
                                    if hasattr(effect, 'ambient') and effect.ambient is not None:
                                        ambient = effect.ambient
                                        if isinstance(ambient, (tuple, list)) and len(ambient) >= 3:
                                            material_info['ambient'] = list(ambient[:4]) if len(ambient) >= 4 else list(ambient[:3]) + [1.0]
                                    
                                    # Get specular color
                                    if hasattr(effect, 'specular') and effect.specular is not None:
                                        specular = effect.specular
                                        if isinstance(specular, (tuple, list)) and len(specular) >= 3:
                                            material_info['specular'] = list(specular[:4]) if len(specular) >= 4 else list(specular[:3]) + [1.0]
                                    
                                    # Get emission color
                                    if hasattr(effect, 'emission') and effect.emission is not None:
                                        emission = effect.emission
                                        if isinstance(emission, (tuple, list)) and len(emission) >= 3:
                                            material_info['emission'] = list(emission[:4]) if len(emission) >= 4 else list(emission[:3]) + [1.0]
                                    
                                    # Get shininess
                                    if hasattr(effect, 'shininess') and effect.shininess is not None:
                                        material_info['shininess'] = float(effect.shininess)
                                        
                            except Exception as e:
                                if verbose:
                                    print(f"Warning: Could not extract material info: {e}")
                        
                        # Create mesh name
                        if material_name:
                            mesh_name = f"{geometry_name}_{material_name}_{prim_idx}"
                        else:
                            mesh_name = f"{geometry_name}_{prim_idx}"
                        
                        # Clean mesh name (remove invalid characters)
                        mesh_name = mesh_name.replace('/', '_').replace('\\', '_').replace(' ', '_')
                        
                        if separate_meshes:
                            # Save each mesh as a separate file (default behavior)
                            # Define output filename
                            output_filename = os.path.join(output_dir, f"{mesh_name}.{output_format}")
                            
                            # Save the mesh
                            mesh.export(output_filename)
                            success_count += 1
                            
                            # Store mesh info
                            mesh_data = {
                                'file': f"{mesh_name}.{output_format}",
                                'geometry': geometry_name,
                                'vertices': len(vertices),
                                'faces': len(mesh.faces)
                            }
                            
                            if material_name:
                                mesh_data['material'] = material_name
                            
                            # Add RGBA for easy URDF integration
                            if rgba:
                                mesh_data['rgba'] = rgba
                            
                            # Add detailed color info if available
                            if material_info:
                                mesh_data['color'] = material_info
                            
                            mesh_info[mesh_name] = mesh_data
                            
                            if verbose:
                                print(f"Saved: {output_filename} ({len(vertices)} vertices, {len(mesh.faces)} faces)")
                                if material_name:
                                    print(f"  Material: {material_name}")
                                if rgba:
                                    print(f"  RGBA: [{rgba[0]:.3f}, {rgba[1]:.3f}, {rgba[2]:.3f}, {rgba[3]:.3f}]")
                                if material_info and len(material_info) > 1:  # More than just rgba
                                    print(f"  Additional properties: {list(material_info.keys())}")
                        else:
                            # Combine mode: collect meshes for later combination
                            combined_meshes.append(mesh)
                            if verbose:
                                print(f"Added mesh to combined output: {mesh_name} ({len(vertices)} vertices, {len(mesh.faces)} faces)")
                        
                    else:
                        # Skip unsupported primitive types (e.g., LineSet, Points)
                        if verbose:
                            print(f"Skipping unsupported primitive type: {type(primitive).__name__}")
                        continue
                
                except Exception as e:
                    print(f"Error processing mesh {geometry_name}_{prim_idx}: {e}")
                    continue
                
                finally:
                    # Only increment for supported mesh types
                    if isinstance(primitive, (collada.polylist.Polylist, collada.triangleset.TriangleSet)):
                        mesh_index += 1
                        # Call progress callback if provided
                        if progress_callback:
                            progress_callback(mesh_index, total_meshes)
        
        # Handle combined mesh mode
        if not separate_meshes and combined_meshes:
            if verbose:
                print(f"\nCombining {len(combined_meshes)} meshes into a single file...")
            
            try:
                # Combine all meshes using trimesh
                combined_mesh = trimesh.util.concatenate(combined_meshes)
                
                # Generate output filename based on DAE filename
                dae_basename = os.path.splitext(os.path.basename(dae_file_path))[0]
                output_filename = os.path.join(output_dir, f"{dae_basename}.{output_format}")
                
                # Save the combined mesh
                combined_mesh.export(output_filename)
                success_count = 1
                total_meshes = 1
                
                # Store combined mesh info
                mesh_info = {
                    dae_basename: {
                        'file': f"{dae_basename}.{output_format}",
                        'geometry': 'combined',
                        'vertices': len(combined_mesh.vertices),
                        'faces': len(combined_mesh.faces),
                        'source_meshes': len(combined_meshes)
                    }
                }
                
                if verbose:
                    print(f"Saved combined mesh: {output_filename}")
                    print(f"  Total vertices: {len(combined_mesh.vertices)}")
                    print(f"  Total faces: {len(combined_mesh.faces)}")
                    print(f"  Source meshes: {len(combined_meshes)}")
                
            except Exception as e:
                print(f"Error combining meshes: {e}")
                return 0, 1, {}
        
        return success_count, total_meshes, mesh_info
    
    except Exception as e:
        print(f"Error loading DAE file: {e}")
        import traceback
        traceback.print_exc()
        return 0, 0, {}


def main():
    parser = argparse.ArgumentParser(
        description="Extract individual meshes from a DAE (COLLADA) file and save as separate mesh files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Extract all meshes from a DAE file to STL format
  urdf2mjcf-extract-dae model.dae -o output_meshes/
  
  # Extract with verbose output and custom JSON output
  urdf2mjcf-extract-dae model.dae -o output_meshes/ --verbose -j meshes.json
  
  # Extract to OBJ format (JSON info saved automatically)
  urdf2mjcf-extract-dae model.dae -o output_meshes/ -f obj
  
  # Extract to a specific directory with custom format
  urdf2mjcf-extract-dae robot.dae -o meshes/extracted/ -f ply

Note: A JSON file with mesh information (including colors/materials) is 
automatically saved to <output_dir>/mesh_info.json unless specified with -j.
"""
    )
    
    parser.add_argument(
        "input",
        type=str,
        help="Path to the input DAE file"
    )
    
    parser.add_argument(
        "-o", "--output",
        type=str,
        default="extracted_meshes",
        help="Output directory for extracted meshes (default: extracted_meshes)"
    )
    
    parser.add_argument(
        "-f", "--format",
        type=str,
        choices=['stl', 'obj', 'ply', 'off'],
        default='stl',
        help="Output mesh format (default: stl)"
    )
    
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output with detailed information"
    )
    
    parser.add_argument(
        "-j", "--json",
        type=str,
        default=None,
        help="Save mesh information to a JSON file (default: <output_dir>/mesh_info.json)"
    )
    
    parser.add_argument(
        "-s", "--scale",
        type=float,
        default=None,
        help="Scale factor to apply to meshes. If not specified, auto-detects from DAE units and converts to meters. "
             "If DAE has no unit info, defaults to 0.001 (mm to m). Examples: 0.001 for mm->m, 0.01 for cm->m"
    )
    
    parser.add_argument(
        "-c", "--combine",
        action="store_true",
        help="Combine all meshes into a single file (legacy/backward compatible mode). "
             "By default, each mesh with different materials is saved separately to preserve colors."
    )
    
    args = parser.parse_args()
    
    # Check if input file exists
    if not os.path.exists(args.input):
        print(f"Error: Input file '{args.input}' not found")
        sys.exit(1)
    
    # Check if input file is a DAE file
    if not args.input.lower().endswith('.dae'):
        print(f"Error: Input file must be a DAE file (got: {args.input})")
        sys.exit(1)
    
    # Check dependencies
    if not COLLADA_AVAILABLE:
        print("Error: pycollada is required but not installed.")
        print("Install with: pip install pycollada")
        sys.exit(1)
    
    if not TRIMESH_AVAILABLE:
        print("Error: trimesh is required but not installed.")
        print("Install with: pip install trimesh")
        sys.exit(1)
    
    print(f"Extracting meshes from: {args.input}")
    print(f"Output directory: {args.output}")
    print(f"Output format: {args.format.upper()}")
    print()
    
    # Setup progress bar for CLI
    progress_bar = None
    if TQDM_AVAILABLE and not args.verbose:
        progress_bar = tqdm(total=0, desc="Extracting meshes", unit="mesh")
        
        def update_progress(current, total):
            if progress_bar.total != total:
                progress_bar.total = total
                progress_bar.refresh()
            progress_bar.update(1)
        
        progress_callback = update_progress
    else:
        progress_callback = None
    
    # Extract meshes
    success_count, total_count, mesh_info = extract_meshes_from_dae(
        args.input,
        args.output,
        args.format,
        args.verbose,
        progress_callback,
        args.scale,
        separate_meshes=not args.combine  # Invert: combine=True means separate_meshes=False
    )
    
    # Close progress bar if it was created
    if progress_bar:
        progress_bar.close()
    
    # Print summary
    print()
    print(f"Extraction complete: {success_count}/{total_count} meshes successfully extracted")
    
    # Save mesh info to JSON
    if success_count > 0:
        json_path = args.json if args.json else os.path.join(args.output, "mesh_info.json")
        
        # Create summary with source file
        output_data = {
            'source': os.path.basename(args.input),
            'total_meshes': total_count,
            'extracted_meshes': success_count,
            'meshes': mesh_info
        }
        
        try:
            with open(json_path, 'w') as f:
                json.dump(output_data, f, indent=2)
            print(f"Mesh information saved to: {json_path}")
        except Exception as e:
            print(f"Warning: Could not save JSON file: {e}")
    
    if success_count < total_count:
        print(f"Warning: {total_count - success_count} meshes failed to extract")
        sys.exit(1)
    elif success_count == 0:
        print("Error: No meshes were extracted")
        sys.exit(1)
    else:
        print(f"All meshes saved to: {os.path.abspath(args.output)}")
        sys.exit(0)


if __name__ == "__main__":
    main()
