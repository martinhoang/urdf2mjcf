import os
import shutil
import traceback
from _utils import print_base, print_info, print_warning, print_error, print_confirm

try:
    import pymeshlab

    PYMESHLAB_AVAILABLE = True
except ImportError:
    PYMESHLAB_AVAILABLE = False

# Import mesh tools
try:
    from urdf2mjcf.tools.calculate_inertia import calculate_inertia, print_urdf_inertia

    CALCULATE_INERTIA_AVAILABLE = True
except ImportError as e:
    print_warning(
        f"Failed importing calculate_inertia tool: {e}\n{traceback.format_exc()}"
    )
    import sys
    sys.exit(1)
    CALCULATE_INERTIA_AVAILABLE = False

try:
    from urdf2mjcf.tools.generate_collision_mesh import process_mesh as generate_collision_mesh

    GENERATE_COLLISION_AVAILABLE = True
except ImportError:
    GENERATE_COLLISION_AVAILABLE = False

try:
    from urdf2mjcf.tools.simplify_mesh import simplify_mesh as simplify_mesh_tool

    SIMPLIFY_MESH_TOOL_AVAILABLE = True
except ImportError:
    SIMPLIFY_MESH_TOOL_AVAILABLE = False


def simplify_mesh(input_file, output_file, target_reduction=0.95, combine_meshes=False):
    """
    Simplify one mesh file. Will require pymeshlab.

    Args:
            input_file (str): Path to input mesh file.
            output_file (str): Path to save simplified mesh file. Auto create if not exists.
            target_reduction (float): Fraction of original faces to remove (0.0-1.0).

    """
    print(f"Simplifying: {input_file}")
    try:
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(input_file)

        target_percentage = 1.0 - target_reduction

        if len(ms) > 1:
            combine_mesh_str = (
                " Combining all meshes into one before simplification."
                if combine_meshes
                else "Convert them separately unless --combine-meshes is set."
            )
            print_warning(f"Multiple meshes found in {input_file}.{combine_mesh_str}")

        for i in range(len(ms)):
            ms.set_current_mesh(i)
            ms.apply_filter(
                "meshing_decimation_quadric_edge_collapse",
                targetperc=target_percentage,
                preservenormal=True,
            )
            output_dir = os.path.dirname(output_file)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)

            modified_output_file = output_file
            if len(ms) > 1 and not combine_meshes:
                base, ext = os.path.splitext(output_file)
                modified_output_file = f"{base}_{i}{ext}"
            ms.save_current_mesh(modified_output_file)

            print(f"Saved simplified mesh to: {modified_output_file}")
    except Exception as e:
        print_error(f"Failed to process {input_file}: {e}")
        raise RuntimeError(f"Mesh simplification failed for {input_file}.")


def count_mesh_faces(mesh_file):
    """
    Count the number of faces in a mesh file.
    Supports STL (binary and ASCII) and OBJ formats.

    Args:
            mesh_file (str): Path to mesh file

    Returns:
            int: Number of faces in the mesh, or -1 if unable to determine
    """
    if not os.path.exists(mesh_file):
        print_warning(f"Mesh file not found: {mesh_file}")
        return -1

    ext = os.path.splitext(mesh_file)[1].lower()

    try:
        if ext == ".stl":
            # Try reading as binary STL first
            with open(mesh_file, "rb") as f:
                # Skip 80-byte header
                f.read(80)
                # Read number of triangles (uint32)
                data = f.read(4)
                if len(data) == 4:
                    num_triangles = int.from_bytes(
                        data, byteorder="little", signed=False
                    )
                    # Validate it's actually binary (check file size)
                    expected_size = (
                        80 + 4 + (num_triangles * 50)
                    )  # header + count + (triangle data)
                    actual_size = os.path.getsize(mesh_file)

                    if abs(actual_size - expected_size) < 100:  # Allow small tolerance
                        return num_triangles

                # If binary read failed, try ASCII
                f.seek(0)
                content = f.read().decode("utf-8", errors="ignore")
                # Count 'facet normal' occurrences in ASCII STL
                face_count = content.lower().count("facet normal")
                if face_count > 0:
                    return face_count

        elif ext == ".obj":
            # Count faces in OBJ file (lines starting with 'f')
            face_count = 0
            with open(mesh_file, "r") as f:
                for line in f:
                    if line.strip().startswith("f "):
                        face_count += 1
            return face_count

        elif ext == ".dae":
            # DAE files are complex, use pymeshlab if available
            if PYMESHLAB_AVAILABLE:
                ms = pymeshlab.MeshSet()
                ms.load_new_mesh(mesh_file)
                total_faces = sum(ms.mesh(i).face_number() for i in range(len(ms)))
                return total_faces
            else:
                print_warning(
                    f"Cannot count faces in DAE file without pymeshlab: {mesh_file}"
                )
                return -1

        else:
            print_warning(f"Unsupported file format for face counting: {ext}")
            return -1

    except Exception as e:
        print_warning(f"Error counting faces in {mesh_file}: {e}")
        return -1


def validate_and_fix_mesh_faces(
    mesh_file, max_faces=200000, target_reduction_ratio=0.5
):
    """
    Check if a mesh exceeds MuJoCo's face limit and automatically simplify if needed.
    MuJoCo has a hard limit of 200,000 faces per mesh file.

    Args:
            mesh_file (str): Path to mesh file to validate
            max_faces (int): Maximum allowed faces (MuJoCo limit is 200,000)
            target_reduction_ratio (float): How aggressively to reduce faces (0.0-1.0)
                    0.5 = reduce to 50% of max_faces, 0.75 = reduce to 75% of max_faces

    Returns:
            tuple: (needs_fix, face_count, suggested_target)
                    needs_fix (bool): True if mesh exceeds limit
                    face_count (int): Current number of faces
                    suggested_target (int): Suggested target face count if reduction needed
    """
    if not os.path.exists(mesh_file):
        return False, 0, 0

    face_count = count_mesh_faces(mesh_file)

    if face_count < 0:
        # Unable to determine face count
        return False, -1, 0

    if face_count > max_faces:
        # Calculate target: reduce to a safe margin below the limit
        suggested_target = int(max_faces * target_reduction_ratio)
        return True, face_count, suggested_target

    return False, face_count, 0


def validate_all_meshes_in_directory(mesh_dir, max_faces=200000):
    """
    Scan all mesh files in a directory and report which ones exceed MuJoCo's face limit.

    Args:
            mesh_dir (str): Directory containing mesh files
            max_faces (int): Maximum allowed faces per mesh

    Returns:
            list: List of tuples (mesh_file, face_count, suggested_target) for meshes that need fixing
    """
    problematic_meshes = []

    if not os.path.exists(mesh_dir):
        return problematic_meshes

    supported_extensions = [".stl", ".obj", ".dae"]

    for root, dirs, files in os.walk(mesh_dir):
        for filename in files:
            ext = os.path.splitext(filename)[1].lower()
            if ext in supported_extensions:
                mesh_path = os.path.join(root, filename)
                needs_fix, face_count, suggested_target = validate_and_fix_mesh_faces(
                    mesh_path, max_faces
                )

                if needs_fix:
                    problematic_meshes.append((mesh_path, face_count, suggested_target))

    return problematic_meshes


def fix_oversized_meshes(
    mesh_dir, max_faces=200000, target_reduction_ratio=0.5, backup=True
):
    """
    Automatically fix all meshes in a directory that exceed MuJoCo's face limit.

    Args:
            mesh_dir (str): Directory containing mesh files
            max_faces (int): Maximum allowed faces per mesh (default: 200000)
            target_reduction_ratio (float): Target face count as ratio of max_faces (default: 0.5)
            backup (bool): Create .bak backup of original files before modifying

    Returns:
            tuple: (fixed_count, failed_count)
    """
    if not SIMPLIFY_MESH_TOOL_AVAILABLE:
        print_error(
            "Cannot fix oversized meshes: simplify_mesh tool not available. Install pymeshlab."
        )
        return 0, 0

    problematic_meshes = validate_all_meshes_in_directory(mesh_dir, max_faces)

    if not problematic_meshes:
        return 0, 0

    print_warning(
        f"Found {len(problematic_meshes)} mesh(es) exceeding MuJoCo's {max_faces:,} face limit:"
    )
    for mesh_path, face_count, suggested_target in problematic_meshes:
        print_warning(
            f"  - {os.path.basename(mesh_path)}: {face_count:,} faces (target: {suggested_target:,})"
        )

    fixed_count = 0
    failed_count = 0

    for mesh_path, face_count, suggested_target in problematic_meshes:
        try:
            # Create backup if requested
            if backup:
                backup_path = mesh_path + ".bak"
                if not os.path.exists(backup_path):
                    shutil.copy2(mesh_path, backup_path)
                    print_info(f"Created backup: {os.path.basename(backup_path)}")

            print_info(
                f"Simplifying {os.path.basename(mesh_path)}: {face_count:,} → {suggested_target:,} faces"
            )

            # Use simplify_mesh_tool with target_faces parameter
            simplify_mesh_tool(
                mesh_path,
                mesh_path,  # Overwrite original
                target_reduction=None,
                target_faces=suggested_target,
            )

            # Verify the fix
            new_face_count = count_mesh_faces(mesh_path)
            if new_face_count > 0 and new_face_count <= max_faces:
                print_confirm(
                    f"✓ Successfully reduced {os.path.basename(mesh_path)} to {new_face_count:,} faces"
                )
                fixed_count += 1
            else:
                print_warning(
                    f"⚠ {os.path.basename(mesh_path)} may still have issues (detected: {new_face_count:,} faces)"
                )
                fixed_count += 1  # Count as fixed even if verification is uncertain

        except Exception as e:
            print_error(f"✗ Failed to simplify {os.path.basename(mesh_path)}: {e}")
            failed_count += 1

    return fixed_count, failed_count


def copy_mesh_files(
    absolute_mesh_paths,
    output_dir,
    mesh_dir=None,
    mesh_reduction=0.9,
    calculate_inertia_flag=False,
    link_properties=None,
    generate_collision=False,
    simplify_meshes=False,
    simplify_params=None,
    raise_on_error=True,
):
    """
    Copy mesh files to output dir.
    Support STL/OBJ. Convert DAE->STL via pymeshlab or extract multiple meshes from DAE with materials.

    Args:
            absolute_mesh_paths: Dictionary of mesh paths organized by link
            output_dir: Output directory for meshes
            mesh_dir: Subdirectory name for meshes within output_dir
            mesh_reduction: Reduction factor for DAE->STL conversion (legacy single-mesh mode)
            calculate_inertia_flag: Whether to calculate and print inertia for meshes
            link_properties: Dict with link properties: {'link_name': {'mass': float, 'mesh_scales': {'visual': [floats], 'collision': [floats]}}}
            generate_collision: Whether to generate collision meshes using convex hulls
            simplify_meshes: Whether to simplify meshes using the simplify tool
            simplify_params: Dict with optional 'reduction' (0.0-1.0), 'target_faces' (int), 'translation', 'scale' for mesh simplification
            raise_on_error: Whether to raise exceptions on errors

    Returns:
            dict: Material information for extracted visual meshes only, organized by link name:
                    {
                            'link_name': {
                                    'visual': [
                                            {
                                                    'file': 'mesh_name.stl',
                                                    'material': 'material_name',
                                                    'rgba': [r, g, b, a]
                                            },
                                            ...
                                    ]
                            }
                    }
                    Note: Only visual meshes have material info. Collision meshes don't need materials.
    """
    print_info("Copying mesh files...")
    if not absolute_mesh_paths:
        print_base("-> No meshes to copy.")
        return {}

    # Material information to be returned
    material_info = {}
    # Inertia data to be returned
    inertia_data = {}

    # Check tool availability and warn if requested but not available
    if calculate_inertia_flag and not CALCULATE_INERTIA_AVAILABLE:
        print_warning(
            "Inertia calculation requested but trimesh library not available. Install with: pip install trimesh[easy]"
        )

    if generate_collision and not GENERATE_COLLISION_AVAILABLE:
        print_warning(
            "Collision mesh generation requested but open3d library not available. Install with: pip install open3d"
        )

    if simplify_meshes and not SIMPLIFY_MESH_TOOL_AVAILABLE:
        print_warning(
            "Mesh simplification requested but pymeshlab not available. Install with: pip install pymeshlab"
        )

    output_mesh_dir = os.path.join(output_dir, mesh_dir) if mesh_dir else output_dir
    os.makedirs(output_mesh_dir, exist_ok=True)

    SUPPORTED_FORMATS = {".stl", ".obj"}
    CONVERTIBLE_FORMATS = {".dae"}

    copied_count = 0
    converted_count = 0
    ignored_count = 0

    def _copy_with_conflict_check(mesh_type, srcs, dsts, link_name):
        """Copy or convert meshes.

        Note: DAE files are already extracted in urdf_preprocess, so here we just copy
        the extracted STL files. Material information is already in the URDF.

        Returns: List of dicts with material info for each mesh (empty for now).
        """
        nonlocal copied_count, converted_count, ignored_count, material_info

        is_valid = isinstance(srcs, list) and isinstance(dsts, list)
        is_valid |= isinstance(srcs, str) and isinstance(dsts, str)

        if not is_valid:
            raise RuntimeError(
                f"Expected either both src and dest to be str or both to be list. Got src: {srcs} and dest: {dsts}"
            )

        if isinstance(srcs, str):
            srcs = [srcs]
            dsts = [dsts]

        mesh_materials = []  # Track material info for each mesh processed

        for src, dst in zip(srcs, dsts):
            src_name = os.path.basename(src)
            src_ext = os.path.splitext(src_name)[1].lower()
            dest_name = os.path.basename(dst)
            dest_ext = os.path.splitext(dest_name)[1].lower()

            if not os.path.exists(src):
                print_warning(f"Source mesh file '{src}' does not exist. Ignoring.")
                if raise_on_error:
                    raise RuntimeError(f"Source mesh file '{src}' does not exist.")
                else:
                    ignored_count += 1
                return mesh_materials

            if dest_ext not in SUPPORTED_FORMATS:
                raise RuntimeError(
                    f"Destination mesh format '{dest_ext}' not supported for '{src}'->'{dst}'."
                )

            if src_ext in SUPPORTED_FORMATS:
                # Direct copy of STL/OBJ files (including already-extracted DAE meshes)
                modified_dest = os.path.join(output_mesh_dir, dest_name)
                try:
                    shutil.copy2(src, modified_dest)
                    copied_count += 1
                    mesh_materials.append(
                        {"file": dest_name, "material": None, "rgba": None}
                    )
                    print_base(
                        f"Copied '{mesh_type}' mesh '{src_name}' to '{dest_name}'."
                    )
                except Exception as e:
                    print_warning(f"Could not copy mesh from '{src}'. Error: {e}")
                    raise RuntimeError(
                        f"Failed to copy mesh from '{src}' to '{modified_dest}'."
                    )

            elif src_ext in CONVERTIBLE_FORMATS:
                # DAE files that weren't extracted (fallback case)
                # This shouldn't happen if preprocessing worked correctly
                if not PYMESHLAB_AVAILABLE:
                    print_error(
                        f"pymeshlab not available. Cannot convert '{src}' from DAE to STL. Ignoring."
                    )
                    raise RuntimeError(
                        "pymeshlab is required for DAE to STL conversion but is not installed."
                    )
                try:
                    modified_dest = os.path.join(output_mesh_dir, dest_name)
                    simplify_mesh(src, modified_dest, mesh_reduction)
                    print_base(
                        f"-> Converted mesh:\n\tFrom: '{src}' ({src_ext.upper()})\n\tTo: '{modified_dest}' ({dest_ext.upper()})"
                    )
                    converted_count += 1
                    mesh_materials.append(
                        {"file": dest_name, "material": None, "rgba": None}
                    )
                    print_base(
                        f"Copied '{mesh_type}' mesh '{src_name}' to '{dest_name}'."
                    )
                except Exception as e:
                    print_warning(
                        f"Could not convert mesh from '{src}' (DAE to STL). Error: {e}"
                    )
                    if raise_on_error:
                        raise RuntimeError(
                            f"Failed to convert mesh from '{src}' to '{modified_dest}'."
                        )
                    else:
                        ignored_count += 1
            else:
                print_warning(
                    f"Unsupported mesh format '{src_ext}' for file '{os.path.basename(src)}'. Ignoring."
                )
                if raise_on_error:
                    raise RuntimeError(
                        f"Unsupported mesh format '{src_ext}' for file '{os.path.basename(src)}'."
                    )
                else:
                    ignored_count += 1

        return mesh_materials

    def _apply_mesh_tools(mesh_file_path, link_name, mesh_type_name):
        """Apply optional mesh tools to processed mesh files."""
        if not os.path.exists(mesh_file_path):
            return

        # Apply mesh simplification if requested
        if simplify_meshes and SIMPLIFY_MESH_TOOL_AVAILABLE:
            try:
                params = simplify_params or {}
                reduction = params.get("reduction", None)
                target_faces = params.get("target_faces", None)
                translation = params.get("translation", None)
                scale = params.get("scale", None)

                print_info(
                    f"Simplifying {mesh_type_name} mesh for link '{link_name}': {os.path.basename(mesh_file_path)}"
                )
                simplify_mesh_tool(
                    mesh_file_path,
                    mesh_file_path,
                    reduction,
                    target_faces,
                    translation,
                    scale,
                )
                print_confirm(
                    f"-> Simplified {mesh_type_name} mesh: {os.path.basename(mesh_file_path)}"
                )
            except Exception as e:
                print_warning(
                    f"Failed to simplify {mesh_type_name} mesh '{mesh_file_path}': {e}"
                )

        # Generate collision mesh if requested and this is a visual mesh
        if (
            generate_collision
            and mesh_type_name == "visual"
            and GENERATE_COLLISION_AVAILABLE
        ):
            try:
                collision_dir = os.path.join(output_mesh_dir, "collision")
                os.makedirs(collision_dir, exist_ok=True)
                collision_file = os.path.join(
                    collision_dir, os.path.basename(mesh_file_path)
                )

                print_info(
                    f"Generating collision mesh for link '{link_name}': {os.path.basename(mesh_file_path)}"
                )
                success = generate_collision_mesh(
                    mesh_file_path, collision_dir, visualize=False
                )
                if success:
                    print_confirm(
                        f"-> Generated collision mesh: {os.path.basename(collision_file)}"
                    )
                else:
                    print_warning(
                        f"Failed to generate collision mesh for '{mesh_file_path}'"
                    )
            except Exception as e:
                print_warning(
                    f"Failed to generate collision mesh for '{mesh_file_path}': {e}"
                )

        # Calculate inertia if requested and link has mass
        if calculate_inertia_flag and CALCULATE_INERTIA_AVAILABLE and link_properties:
            link_props = link_properties.get(link_name, {})
            link_mass = link_props.get("mass")
            if link_mass is not None:
                try:
                    # Get scale from URDF mesh element, default to 1.0 if not found
                    scale = 1.0
                    mesh_scales = link_props.get("mesh_scales", {})
                    mesh_type_scales = mesh_scales.get(mesh_type_name, [])
                    if mesh_type_scales:
                        # Use the first scale value (assuming one mesh per link for inertia calc)
                        scale = mesh_type_scales[0]
                    
                    translation = None
                    orientation = None

                    print_info(
                        f"Calculating inertia for {mesh_type_name} mesh of link '{link_name}': {os.path.basename(mesh_file_path)} (scale: {scale})"
                    )
                    calculated_inertia = calculate_inertia(
                        mesh_file_path, link_mass, translation, orientation, scale
                    )

                    if calculated_inertia:
                        # Store inertia data for this link
                        inertia_data[link_name] = calculated_inertia
                        print_confirm(
                            f"-> Calculated inertia for link '{link_name}' (mass: {link_mass} kg):"
                        )
                        print_urdf_inertia(calculated_inertia)
                    else:
                        print_warning(
                            f"Failed to calculate inertia for '{mesh_file_path}'"
                        )
                except Exception as e:
                    print_warning(
                        f"Failed to calculate inertia for '{mesh_file_path}': {e}"
                    )
            else:
                print_warning(
                    f"No mass found for link '{link_name}'. Skipping inertia calculation for its meshes."
                )
        elif calculate_inertia_flag and not CALCULATE_INERTIA_AVAILABLE:
            print_warning(
                f"Inertia calculation requested but trimesh library not available. Skipping for link '{link_name}'."
            )
        elif calculate_inertia_flag and not link_properties:
            print_warning(
                f"Inertia calculation requested but no link properties provided. Skipping for link '{link_name}'."
            )

    for link, mesh_path in absolute_mesh_paths.items():
        # Some meshes dont have visual or collision, skip those
        if len(mesh_path) == 0:
            continue

        # Initialize material info for this link
        if link not in material_info:
            material_info[link] = {}

        if "visual" in mesh_path:
            visual_src = mesh_path["visual"]["from"]
            visual_dst = mesh_path["visual"]["to"]
            print_base(
                f"Processing 'visual' mesh for link '{link}':\n\tFrom: {visual_src}\n\tTo: {visual_dst}"
            )
            visual_materials = _copy_with_conflict_check(
                "visual", srcs=visual_src, dsts=visual_dst, link_name=link
            )

            # Store material info for visual meshes
            if visual_materials:
                material_info[link]["visual"] = visual_materials

            # Apply mesh tools to visual meshes
            if isinstance(visual_dst, list):
                for dst_file in visual_dst:
                    final_dst = os.path.join(
                        output_mesh_dir, os.path.basename(dst_file)
                    )
                    _apply_mesh_tools(final_dst, link, "visual")
            else:
                final_dst = os.path.join(output_mesh_dir, os.path.basename(visual_dst))
                _apply_mesh_tools(final_dst, link, "visual")

        if "collision" in mesh_path:
            collision_src = mesh_path["collision"]["from"]
            collision_dst = mesh_path["collision"]["to"]
            print_base(
                f"Processing 'collision' mesh for link '{link}':\n\tFrom: {collision_src}\n\tTo: {collision_dst}"
            )
            # Collision meshes don't need material info, so we don't store the return value
            _copy_with_conflict_check(
                "collision", srcs=collision_src, dsts=collision_dst, link_name=link
            )

            # Apply mesh tools to collision meshes (excluding collision generation since it's already collision)
            if isinstance(collision_dst, list):
                for dst_file in collision_dst:
                    final_dst = os.path.join(
                        output_mesh_dir, os.path.basename(dst_file)
                    )
                    _apply_mesh_tools(final_dst, link, "collision")
            else:
                final_dst = os.path.join(
                    output_mesh_dir, os.path.basename(collision_dst)
                )
                _apply_mesh_tools(final_dst, link, "collision")

    total_processed = copied_count + converted_count + ignored_count
    summary_parts = []
    if copied_count > 0:
        summary_parts.append(f"{copied_count} copied")
    if converted_count > 0:
        summary_parts.append(f"{converted_count} converted (DAE→STL)")
    if ignored_count > 0:
        summary_parts.append(f"{ignored_count} ignored")
    if summary_parts:
        if ignored_count > 0:
            print_warning(
                f"-> Processed {total_processed} mesh files: {', '.join(summary_parts)}. Output: '{output_mesh_dir}'"
            )
        else:
            print_confirm(
                f"-> Processed {total_processed} mesh files: {', '.join(summary_parts)}"
            )
    else:
        print_base(f"-> No mesh files processed. Output: '{output_mesh_dir}'")

    return material_info, inertia_data
