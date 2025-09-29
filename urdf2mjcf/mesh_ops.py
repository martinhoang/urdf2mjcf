import os
import shutil
from _utils import print_base, print_info, print_warning, print_error, print_confirm

try:
	import pymeshlab
	PYMESHLAB_AVAILABLE = True
except ImportError:
	PYMESHLAB_AVAILABLE = False

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
		print(f"Failed to process {input_file}: {e}")

def copy_mesh_files(absolute_mesh_paths, output_dir, mesh_dir=None, mesh_reduction=0.9):
	"""
	Copy mesh files to output dir.
	Support STL/OBJ. Convert DAE->STL via pymeshlab.
	"""
	print_info("Copying mesh files...")
	if not absolute_mesh_paths:
		print_base("-> No meshes to copy.")
		return

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
					ignored_count += 1
			else:
				print_warning(f"Unsupported mesh format '{src_ext}' for file '{os.path.basename(src)}'. Ignoring.")
				ignored_count += 1
			
			print_base(f"Copied '{mesh_type}' mesh '{src_name}' to '{dest_name}'.")

	for link, mesh_path in absolute_mesh_paths.items():
		# Some meshes dont have visual or collision, skip those
		if len(mesh_path) == 0:
			continue
		
		if "visual" in mesh_path:
			visual_src = mesh_path["visual"]["from"]
			visual_dst = mesh_path["visual"]["to"]
			print_base(f"Processing 'visual' mesh for link '{link}':\n\tFrom: {visual_src}\n\tTo: {visual_dst}")
			_copy_with_conflict_check("visual", srcs=visual_src, dsts=visual_dst)
		
		if "collision" in mesh_path:
			collision_src = mesh_path["collision"]["from"]
			collision_dst = mesh_path["collision"]["to"]
			print_base(f"Processing 'collision' mesh for link '{link}':\n\tFrom: {collision_src}\n\tTo: {collision_dst}")
			_copy_with_conflict_check("collision", srcs=collision_src, dsts=collision_dst)

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