#!/usr/bin/env python3

import argparse
import json
import logging
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


DAE_UP_AXIS_CHOICES = ("auto", "x", "y", "z")


def _resolve_dae_up_axis(
    declared_up_axis,
    dae_up_axis="auto",
    blender_legacy_axes=False,
    scene_geometry_available=False,
):
    """Return the source up-axis that should be converted to MuJoCo Z-up."""
    requested_axis = (dae_up_axis or "auto").lower()
    if requested_axis not in DAE_UP_AXIS_CHOICES:
        raise ValueError(
            f"Invalid DAE up-axis '{dae_up_axis}'. "
            f"Expected one of: {', '.join(DAE_UP_AXIS_CHOICES)}"
        )

    if requested_axis != "auto":
        return f"{requested_axis.upper()}_UP"

    # Bound scene primitives already include each COLLADA node's transform.
    # Exporters commonly encode their Y-up -> robot-frame conversion there,
    # even when the asset header still says Y_UP.
    if scene_geometry_available:
        return "Z_UP"

    if blender_legacy_axes:
        return "Y_UP"

    normalized_declared = str(declared_up_axis or "Y_UP").upper()
    if normalized_declared not in {"X_UP", "Y_UP", "Z_UP"}:
        return "Y_UP"
    return normalized_declared


def _up_axis_rotation_matrix(source_up_axis):
    """Return a homogeneous transform from a COLLADA up-axis to Z-up."""
    if source_up_axis == "Y_UP":
        return trimesh.transformations.rotation_matrix(
            angle=np.radians(90), direction=[1, 0, 0]
        )
    if source_up_axis == "X_UP":
        return trimesh.transformations.rotation_matrix(
            angle=np.radians(-90), direction=[0, 1, 0]
        )
    return None


def _collect_dae_primitives(dae):
    """Collect scene-bound primitives, falling back to raw geometry."""
    supported_types = (
        collada.polylist.Polylist,
        collada.polylist.BoundPolylist,
        collada.triangleset.TriangleSet,
        collada.triangleset.BoundTriangleSet,
    )

    bound_geometries = (
        list(dae.scene.objects("geometry")) if dae.scene is not None else []
    )
    entries = []
    geometry_instance_counts = {}
    for bound_geometry in bound_geometries:
        geometry_id = bound_geometry.original.id or "geometry"
        geometry_instance_counts[geometry_id] = (
            geometry_instance_counts.get(geometry_id, 0) + 1
        )

    seen_instances = {}
    for bound_geometry in bound_geometries:
        geometry_id = bound_geometry.original.id or "geometry"
        instance_index = seen_instances.get(geometry_id, 0)
        seen_instances[geometry_id] = instance_index + 1
        geometry_name = geometry_id
        if geometry_instance_counts[geometry_id] > 1:
            geometry_name = f"{geometry_id}_instance_{instance_index}"

        for primitive in bound_geometry.primitives():
            if isinstance(primitive, supported_types):
                entries.append((geometry_name, primitive))

    if entries:
        return entries, True

    for geom_idx, geometry in enumerate(dae.geometries):
        geometry_name = geometry.id if geometry.id else f"geometry_{geom_idx}"
        for primitive in geometry.primitives:
            if isinstance(primitive, supported_types):
                entries.append((geometry_name, primitive))
    return entries, False


def extract_meshes_from_dae(
    dae_file_path,
    output_dir,
    output_format="stl",
    verbose=False,
    progress_callback=None,
    scale_factor=None,
    separate_meshes=False,
    skip_degenerate=False,
    name_prefix="",
    dae_up_axis="auto",
):
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
        skip_degenerate (bool): If True, silently skip coplanar (flat) meshes that MuJoCo cannot use for
                               volume-based inertia computation. Default is False — flat meshes such as
                               surface logos are valid for visual rendering and should be kept.
        name_prefix (str): Optional prefix prepended to every extracted mesh filename. Use the URDF link
                          name as prefix (e.g. 'dg5fs_right_link_base_') so that identical DAE files used
                          by different links (left vs right hand) produce unique output filenames and never
                          overwrite each other in a shared output directory.
        dae_up_axis (str): Source up-axis handling: "auto" reads COLLADA metadata, while
                          "x", "y", or "z" force the source axis. Extracted meshes are
                          rotated to MuJoCo's Z-up coordinate system.
    
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

    # Suppress trimesh's verbose logging (cache-clear notices, negative-volume
    # warnings, etc.) that would otherwise pollute the user's terminal.
    logging.getLogger('trimesh').setLevel(logging.ERROR)

    if verbose:
        print(f"Loading DAE file: {dae_file_path}")
    
    try:
        # Load the DAE file
        dae = collada.Collada(dae_file_path)
        
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Check coordinate system up-axis
        up_axis = "Y_UP"  # COLLADA default when <up_axis> is omitted
        if hasattr(dae, 'assetInfo') and dae.assetInfo is not None:
            if hasattr(dae.assetInfo, 'upaxis') and dae.assetInfo.upaxis:
                up_axis = dae.assetInfo.upaxis
        
        if verbose:
            print(f"DAE coordinate system: {up_axis}")
        
        blender_legacy_axes = False

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
            
            # Calculate scale factor to convert to meters.
            # unit_meter is the DAE unit expressed in meters (e.g. 0.001 for mm).
            # When unit_meter == 1.0 the file is already in meters; only override
            # that if the unitname explicitly names a sub-meter unit.
            #
            # blender_legacy_axes: set True when the Blender 2.x unit bug is
            # detected (declared meter=1 but vertices are in mm).  The SAME
            # Blender exporter also declares 'Z_UP' for meshes that are actually
            # in Y_UP orientation, so we must also apply the Y→Z rotation when
            # this flag is set (even though the header says Z_UP).
            if unit_meter != 1.0:
                scale_factor = unit_meter
                if verbose:
                    print(f"DAE unit detected: {unit_name} (meter scale: {unit_meter})")
                    print(f"Applying scale factor: {scale_factor} to convert to meters")
            else:
                # unit_meter == 1.0: the file declares metre-scale units.
                # Only override if the unit name explicitly says otherwise.
                unit_name_lower = unit_name.lower()
                if 'millimeter' in unit_name_lower or unit_name_lower == 'mm':
                    scale_factor = 0.001
                    if verbose:
                        print(f"DAE unit detected: {unit_name}, applying scale factor: {scale_factor} (mm to m)")
                elif 'centimeter' in unit_name_lower or unit_name_lower == 'cm':
                    scale_factor = 0.01
                    if verbose:
                        print(f"DAE unit detected: {unit_name}, applying scale factor: {scale_factor} (cm to m)")
                elif 'inch' in unit_name_lower or unit_name_lower == 'in':
                    scale_factor = 0.0254
                    if verbose:
                        print(f"DAE unit detected: {unit_name}, applying scale factor: {scale_factor} (inch to m)")
                else:
                    # 'meter' or any unknown name with unitmeter==1.0.
                    #
                    # Some DAE exporters (notably Blender 2.x used by Universal
                    # Robots) declare meter="1" but store vertices in millimetres.
                    # Detect this by sampling the raw vertex magnitudes from the
                    # first available primitive: if any coordinate exceeds
                    # _METER_UNIT_MAX_M (2 m) in a file that claims meters, the
                    # data is almost certainly in mm.
                    _METER_UNIT_MAX_M = 2.0
                    scale_factor = 1.0
                    try:
                        for _g in dae.geometries:
                            for _p in _g.primitives:
                                if isinstance(_p, (collada.polylist.Polylist, collada.triangleset.TriangleSet)):
                                    _verts = np.asarray(_p.vertex)
                                    _max_coord = float(np.max(np.abs(_verts)))
                                    if _max_coord > _METER_UNIT_MAX_M:
                                        scale_factor = 0.001
                                        blender_legacy_axes = True
                                        if verbose:
                                            print(
                                                f"DAE declares meter units but max vertex coord "
                                                f"= {_max_coord:.1f} > {_METER_UNIT_MAX_M} m; "
                                                f"assuming Blender legacy export (mm + Y_UP) — "
                                                f"applying scale_factor=0.001 and Y→Z rotation"
                                            )
                                    else:
                                        if verbose:
                                            print(
                                                f"DAE unit detected: {unit_name} (unitmeter=1.0); "
                                                f"max vertex coord = {_max_coord:.4f} m; no scaling applied"
                                            )
                                    break  # only need the first primitive
                            else:
                                continue
                            break
                    except Exception:
                        pass  # keep scale_factor=1.0 on any sampling error
        else:
            if verbose:
                print(f"Using explicit scale factor: {scale_factor}")
        
        if verbose:
            print(f"Found {len(dae.geometries)} geometries in the DAE file")

        primitive_entries, using_scene_geometry = _collect_dae_primitives(dae)
        source_up_axis = _resolve_dae_up_axis(
            up_axis,
            dae_up_axis,
            blender_legacy_axes,
            scene_geometry_available=using_scene_geometry,
        )
        rotation_matrix = _up_axis_rotation_matrix(source_up_axis)
        if verbose:
            if dae_up_axis == "auto" and using_scene_geometry:
                mode_description = "using COLLADA scene-node transforms"
            elif dae_up_axis == "auto" and not blender_legacy_axes:
                mode_description = f"detected from COLLADA metadata ({up_axis})"
            else:
                mode_description = f"selected by mode '{dae_up_axis}'"
            if rotation_matrix is None:
                print(f"DAE up-axis: {source_up_axis}, {mode_description}; no rotation needed")
            else:
                print(
                    f"DAE up-axis: {source_up_axis}, {mode_description}; "
                    "rotating mesh to Z_UP"
                )
        
        # Count total extractable primitives (only supported types)
        total_meshes = len(primitive_entries)
        
        if total_meshes == 0:
            print(f"Warning: No extractable meshes found in {dae_file_path}")
            return 0, 0, {}
        
        success_count = 0
        mesh_index = 0
        mesh_info = {}

        # Build a symbol -> Material mapping from the scene graph's bind_material nodes.
        # COLLADA bind_material maps a user-visible symbol (e.g. "defaultMaterial") to the
        # actual material id (e.g. "m0DefaultMaterial").  pycollada's dae.materials dict is
        # keyed by id, so a direct primitive.material lookup fails when symbol != id.
        symbol_to_material = {}
        if dae.scene:
            def _collect_material_bindings(nodes):
                for node in nodes:
                    for child in getattr(node, 'children', []):
                        if hasattr(child, 'materials'):
                            for mat_node in child.materials:
                                if mat_node.symbol and mat_node.target is not None:
                                    symbol_to_material[mat_node.symbol] = mat_node.target
                        if hasattr(child, 'children'):
                            _collect_material_bindings([child])
            _collect_material_bindings(dae.scene.nodes)

        # If not separating meshes, collect all meshes for combination
        combined_meshes = [] if not separate_meshes else None
        
        # Iterate through scene-bound primitives so node transforms are preserved.
        for prim_idx, (geometry_name, primitive) in enumerate(primitive_entries):
            try:
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

                # Handle coordinate system conversion.
                # pycollada reads vertices as-is — it does NOT reinterpret axes.
                # MuJoCo uses Z-up, so we rotate for Y-up DAE files.
                # Auto mode follows COLLADA metadata and handles a known
                # Blender legacy export bug. The caller may also force the
                # source axis when a file has incorrect metadata.
                if rotation_matrix is not None:
                    mesh.apply_transform(rotation_matrix)

                # Skip degenerate (flat / coplanar) meshes.
                # MuJoCo rejects them with "mesh volume is too small" when it
                # tries to compute volume-based inertia.  We detect coplanarity
                # via SVD: if the smallest singular value of the centred vertex
                # matrix is negligible relative to the largest, all vertices lie
                # in a plane.  This avoids convex_hull (which raises exceptions
                # on meshes with inverted normals and also prints trimesh noise).
                try:
                    if len(mesh.vertices) >= 4:
                        centered = mesh.vertices - mesh.vertices.mean(axis=0)
                        sv = np.linalg.svd(centered, compute_uv=False)
                        is_degenerate = (sv[0] > 0 and sv[-1] < 1e-6 * sv[0])
                    else:
                        is_degenerate = True  # too few vertices for a solid
                except Exception:
                    is_degenerate = False  # conservative: keep on error

                if is_degenerate:
                    if skip_degenerate:
                        if verbose:
                            print(
                                f"Warning: Skipping coplanar (flat) mesh "
                                f"{geometry_name}_{prim_idx} ({len(mesh.faces)} faces) "
                                f"— MuJoCo cannot compute volume inertia for flat geometry."
                            )
                        continue
                    elif verbose:
                        print(
                            f"Note: coplanar (flat) mesh {geometry_name}_{prim_idx} "
                            f"({len(mesh.faces)} faces) kept for visual rendering "
                            f"(use skip_degenerate=True to exclude)."
                        )

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
                            # Look up in the DAE's materials by id (direct)
                            if material_symbol in dae.materials:
                                material_obj = dae.materials[material_symbol]
                                effect = material_obj.effect
                            elif material_symbol in symbol_to_material:
                                # Resolve via scene graph symbol→material binding
                                material_obj = symbol_to_material[material_symbol]
                                material_name = getattr(material_obj, 'id', material_symbol)
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

                # Create mesh name (scoped by caller-supplied prefix so that
                # identical DAE files in different links don't collide)
                if material_name:
                    mesh_name = f"{name_prefix}{geometry_name}_{material_name}_{prim_idx}"
                else:
                    mesh_name = f"{name_prefix}{geometry_name}_{prim_idx}"

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
                        'faces': len(mesh.faces),
                        'source_up_axis': source_up_axis,
                        'target_up_axis': 'Z_UP',
                        'axis_rotation_applied': rotation_matrix is not None,
                        'scene_transforms_applied': using_scene_geometry,
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


            except Exception as e:
                print(f"Error processing mesh {geometry_name}_{prim_idx}: {e}")
                continue

            finally:
                mesh_index += 1
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
                        'source_meshes': len(combined_meshes),
                        'source_up_axis': source_up_axis,
                        'target_up_axis': 'Z_UP',
                        'axis_rotation_applied': rotation_matrix is not None,
                        'scene_transforms_applied': using_scene_geometry,
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
        "--dae-up-axis",
        choices=DAE_UP_AXIS_CHOICES,
        default="auto",
        help=(
            "Source DAE up-axis. 'auto' applies COLLADA scene-node transforms "
            "and falls back to metadata; x/y/z force an additional source-axis "
            "conversion to MuJoCo Z-up (default: auto)."
        ),
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
        separate_meshes=not args.combine,  # Invert: combine=True means separate_meshes=False
        dae_up_axis=args.dae_up_axis,
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
