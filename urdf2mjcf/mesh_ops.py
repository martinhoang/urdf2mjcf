import os
import shutil
from _utils import print_base, print_info, print_warning, print_error, print_confirm

try:
	import pymeshlab
	PYMESHLAB_AVAILABLE = True
except ImportError:
	PYMESHLAB_AVAILABLE = False

# Import mesh tools
try:
	from .tools.calculate_inertia import calculate_inertia, print_urdf_inertia
	CALCULATE_INERTIA_AVAILABLE = True
except ImportError:
	CALCULATE_INERTIA_AVAILABLE = False

try:
	from .tools.generate_collision_mesh import process_mesh as generate_collision_mesh
	GENERATE_COLLISION_AVAILABLE = True
except ImportError:
	GENERATE_COLLISION_AVAILABLE = False

try:
	from .tools.simplify_mesh import simplify_mesh as simplify_mesh_tool
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
			combine_mesh_str = " Combining all meshes into one before simplification." if combine_meshes else "Convert them separately unless --combine-meshes is set."
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

def copy_mesh_files(absolute_mesh_paths, output_dir, mesh_dir=None, mesh_reduction=0.9, 
					calculate_inertia_params=None, generate_collision=False, 
					simplify_meshes=False, simplify_params=None, raise_on_error=True):
	"""
	Copy mesh files to output dir.
	Support STL/OBJ. Convert DAE->STL via pymeshlab.
	
	Args:
		absolute_mesh_paths: Dictionary of mesh paths organized by link
		output_dir: Output directory for meshes
		mesh_dir: Subdirectory name for meshes within output_dir
		mesh_reduction: Reduction factor for DAE->STL conversion
		calculate_inertia_params: Dict with 'mass' (required) and optional 'translation', 'orientation', 'scale' for inertia calculation
		generate_collision: Whether to generate collision meshes using convex hulls
		simplify_meshes: Whether to simplify meshes using the simplify tool
		simplify_params: Dict with optional 'reduction' (0.0-1.0), 'target_faces' (int), 'translation', 'scale' for mesh simplification
	"""
	print_info("Copying mesh files...")
	if not absolute_mesh_paths:
		print_base("-> No meshes to copy.")
		return

	# Check tool availability and warn if requested but not available
	if calculate_inertia_params and not CALCULATE_INERTIA_AVAILABLE:
		print_warning("Inertia calculation requested but trimesh library not available. Install with: pip install trimesh[easy]")
	
	if generate_collision and not GENERATE_COLLISION_AVAILABLE:
		print_warning("Collision mesh generation requested but open3d library not available. Install with: pip install open3d")
	
	if simplify_meshes and not SIMPLIFY_MESH_TOOL_AVAILABLE:
		print_warning("Mesh simplification requested but pymeshlab not available. Install with: pip install pymeshlab")

	output_mesh_dir = os.path.join(output_dir, mesh_dir) if mesh_dir else output_dir
	os.makedirs(output_mesh_dir, exist_ok=True)

	SUPPORTED_FORMATS = {".stl", ".obj"}
	CONVERTIBLE_FORMATS = {".dae"}

	copied_count = 0
	converted_count = 0
	ignored_count = 0

	def _copy_with_conflict_check(mesh_type, srcs, dsts):
		nonlocal copied_count, converted_count, ignored_count

		is_valid = isinstance(srcs, list) and isinstance(dsts, list)
		is_valid |= isinstance(srcs, str) and isinstance(dsts, str)

		if not is_valid:
			raise RuntimeError(f"Expected either both src and dest to be str or both to be list. Got src: {srcs} and dest: {dsts}")

		if isinstance(srcs, str):
			srcs = [srcs]
			dsts = [dsts]

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
				return

			if dest_ext not in SUPPORTED_FORMATS:
				raise RuntimeError(f"Destination mesh format '{dest_ext}' not supported for '{src}'->'{dst}'.")
	
			if src_ext in SUPPORTED_FORMATS:
				modified_dest = os.path.join(output_mesh_dir, dest_name)
				try:
					shutil.copy2(src, modified_dest)
					copied_count += 1
				except Exception as e:
					print_warning(f"Could not copy mesh from '{src}'. Error: {e}")
					raise RuntimeError(f"Failed to copy mesh from '{src}' to '{modified_dest}'.")
	
			elif src_ext in CONVERTIBLE_FORMATS:
				if not PYMESHLAB_AVAILABLE:
					print_error(f"pymeshlab not available. Cannot convert '{src}' from DAE to STL. Ignoring.")
					raise RuntimeError("pymeshlab is required for DAE to STL conversion but is not installed.")
				try:
					modified_dest = os.path.join(output_mesh_dir, dest_name)
					simplify_mesh(src, modified_dest, mesh_reduction)
					print_base(f"-> Converted mesh:\n\tFrom: '{src}' ({src_ext.upper()})\n\tTo: '{modified_dest}' ({dest_ext.upper()})")
					converted_count += 1
				except Exception as e:
					print_warning(f"Could not convert mesh from '{src}' (DAE to STL). Error: {e}")
					if raise_on_error:
						raise RuntimeError(f"Failed to convert mesh from '{src}' to '{modified_dest}'.")
					else:
						ignored_count += 1
			else:
				print_warning(f"Unsupported mesh format '{src_ext}' for file '{os.path.basename(src)}'. Ignoring.")
				if raise_on_error:
					raise RuntimeError(f"Unsupported mesh format '{src_ext}' for file '{os.path.basename(src)}'.")
				else:
					ignored_count += 1
			
			print_base(f"Copied '{mesh_type}' mesh '{src_name}' to '{dest_name}'.")

	def _apply_mesh_tools(mesh_file_path, link_name, mesh_type_name):
		"""Apply optional mesh tools to processed mesh files."""
		if not os.path.exists(mesh_file_path):
			return
			
		# Apply mesh simplification if requested
		if simplify_meshes and SIMPLIFY_MESH_TOOL_AVAILABLE:
			try:
				params = simplify_params or {}
				reduction = params.get('reduction', None)
				target_faces = params.get('target_faces', None)
				translation = params.get('translation', None)
				scale = params.get('scale', None)
				
				print_info(f"Simplifying {mesh_type_name} mesh for link '{link_name}': {os.path.basename(mesh_file_path)}")
				simplify_mesh_tool(mesh_file_path, mesh_file_path, reduction, target_faces, translation, scale)
				print_confirm(f"-> Simplified {mesh_type_name} mesh: {os.path.basename(mesh_file_path)}")
			except Exception as e:
				print_warning(f"Failed to simplify {mesh_type_name} mesh '{mesh_file_path}': {e}")
		
		# Generate collision mesh if requested and this is a visual mesh
		if generate_collision and mesh_type_name == "visual" and GENERATE_COLLISION_AVAILABLE:
			try:
				collision_dir = os.path.join(output_mesh_dir, "collision")
				os.makedirs(collision_dir, exist_ok=True)
				collision_file = os.path.join(collision_dir, os.path.basename(mesh_file_path))
				
				print_info(f"Generating collision mesh for link '{link_name}': {os.path.basename(mesh_file_path)}")
				success = generate_collision_mesh(mesh_file_path, collision_dir, visualize=False)
				if success:
					print_confirm(f"-> Generated collision mesh: {os.path.basename(collision_file)}")
				else:
					print_warning(f"Failed to generate collision mesh for '{mesh_file_path}'")
			except Exception as e:
				print_warning(f"Failed to generate collision mesh for '{mesh_file_path}': {e}")
		
		# Calculate inertia if requested and parameters provided
		if calculate_inertia_params and CALCULATE_INERTIA_AVAILABLE:
			try:
				mass = calculate_inertia_params.get('mass')
				if mass is None:
					print_warning(f"Mass parameter required for inertia calculation. Skipping '{mesh_file_path}'.")
					return
					
				translation = calculate_inertia_params.get('translation', None) 
				orientation = calculate_inertia_params.get('orientation', None)
				scale = calculate_inertia_params.get('scale', 0.001)  # Default mm to m
				
				print_info(f"Calculating inertia for {mesh_type_name} mesh of link '{link_name}': {os.path.basename(mesh_file_path)}")
				inertia_data = calculate_inertia(mesh_file_path, mass, translation, orientation, scale)
				
				if inertia_data:
					print_confirm(f"-> Calculated inertia for link '{link_name}':")
					print_urdf_inertia(inertia_data)
				else:
					print_warning(f"Failed to calculate inertia for '{mesh_file_path}'")
			except Exception as e:
				print_warning(f"Failed to calculate inertia for '{mesh_file_path}': {e}")

	for link, mesh_path in absolute_mesh_paths.items():
		# Some meshes dont have visual or collision, skip those
		if len(mesh_path) == 0:
			continue
		
		if "visual" in mesh_path:
			visual_src = mesh_path["visual"]["from"]
			visual_dst = mesh_path["visual"]["to"]
			print_base(f"Processing 'visual' mesh for link '{link}':\n\tFrom: {visual_src}\n\tTo: {visual_dst}")
			_copy_with_conflict_check("visual", srcs=visual_src, dsts=visual_dst)
			
			# Apply mesh tools to visual meshes
			if isinstance(visual_dst, list):
				for dst_file in visual_dst:
					final_dst = os.path.join(output_mesh_dir, os.path.basename(dst_file))
					_apply_mesh_tools(final_dst, link, "visual")
			else:
				final_dst = os.path.join(output_mesh_dir, os.path.basename(visual_dst))
				_apply_mesh_tools(final_dst, link, "visual")
		
		if "collision" in mesh_path:
			collision_src = mesh_path["collision"]["from"]
			collision_dst = mesh_path["collision"]["to"]
			print_base(f"Processing 'collision' mesh for link '{link}':\n\tFrom: {collision_src}\n\tTo: {collision_dst}")
			_copy_with_conflict_check("collision", srcs=collision_src, dsts=collision_dst)
			
			# Apply mesh tools to collision meshes (excluding collision generation since it's already collision)
			if isinstance(collision_dst, list):
				for dst_file in collision_dst:
					final_dst = os.path.join(output_mesh_dir, os.path.basename(dst_file))
					_apply_mesh_tools(final_dst, link, "collision")
			else:
				final_dst = os.path.join(output_mesh_dir, os.path.basename(collision_dst))
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
			print_warning(f"-> Processed {total_processed} mesh files: {', '.join(summary_parts)}. Output: '{output_mesh_dir}'")
		else:
			print_confirm(f"-> Processed {total_processed} mesh files: {', '.join(summary_parts)}")
	else:
		print_base(f"-> No mesh files processed. Output: '{output_mesh_dir}'")