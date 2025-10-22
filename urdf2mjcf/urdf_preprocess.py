import os
import re
import copy
import tempfile
import numpy as np
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from _utils import (
	print_base,
	print_info,
	print_warning,
	print_debug,
	print_confirm,
)

# Try to import DAE extraction tool
try:
	from .tools.extract_dae_meshes import extract_meshes_from_dae
	EXTRACT_DAE_AVAILABLE = True
except ImportError:
	EXTRACT_DAE_AVAILABLE = False

def rotation_matrix_from_rpy(roll, pitch, yaw):
	"""
	Create a 3x3 rotation matrix from roll, pitch, yaw (in radians).
	Uses ZYX Euler angle convention (yaw-pitch-roll).
	"""
	cr, sr = np.cos(roll), np.sin(roll)
	cp, sp = np.cos(pitch), np.sin(pitch)
	cy, sy = np.cos(yaw), np.sin(yaw)
	
	# Rotation matrices
	Rx = np.array([[1, 0, 0],
				   [0, cr, -sr],
				   [0, sr, cr]])
	
	Ry = np.array([[cp, 0, sp],
				   [0, 1, 0],
				   [-sp, 0, cp]])
	
	Rz = np.array([[cy, -sy, 0],
				   [sy, cy, 0],
				   [0, 0, 1]])
	
	# Combined rotation: R = Rz * Ry * Rx (ZYX convention)
	return Rz @ Ry @ Rx

def transform_inertia_tensor(inertia_matrix, rotation_matrix):
	"""
	Transform an inertia tensor by a rotation matrix.
	I' = R * I * R^T
	
	Args:
		inertia_matrix: 3x3 inertia tensor
		rotation_matrix: 3x3 rotation matrix
	
	Returns:
		Transformed 3x3 inertia tensor
	"""
	return rotation_matrix @ inertia_matrix @ rotation_matrix.T

def zero_inertial_orientation(link_node):
	"""
	Transform inertial properties to remove non-zero RPY orientation.
	The inertia tensor is rotated to the link frame with zero orientation.
	
	Args:
		link_node: ET.Element representing a <link> node
	
	Returns:
		True if transformation was applied, False otherwise
	"""
	inertial = link_node.find("inertial")
	if inertial is None:
		return False
	
	origin = inertial.find("origin")
	if origin is None:
		return False
	
	rpy_str = origin.get("rpy", "0 0 0")
	rpy_parts = rpy_str.strip().split()
	
	if len(rpy_parts) != 3:
		print_warning(f"Invalid RPY format in inertial origin: '{rpy_str}'")
		return False
	
	try:
		# Parse RPY values (may contain expressions like ${pi/2})
		roll_str, pitch_str, yaw_str = rpy_parts
		
		# Simple expression evaluator for common patterns
		def eval_expr(expr_str):
			# Replace ${...} with the expression inside
			expr_str = re.sub(r'\$\{([^}]+)\}', r'\1', expr_str)
			# Replace 'pi' with its value
			expr_str = expr_str.replace('pi', str(np.pi))
			# Evaluate the expression
			return float(eval(expr_str))
		
		roll = eval_expr(roll_str)
		pitch = eval_expr(pitch_str)
		yaw = eval_expr(yaw_str)
		
		# Check if RPY is already zero (within tolerance)
		if abs(roll) < 1e-9 and abs(pitch) < 1e-9 and abs(yaw) < 1e-9:
			return False  # Already zero, no transformation needed
		
		# Get the inertia values
		inertia_elem = inertial.find("inertia")
		if inertia_elem is None:
			return False
		
		ixx = float(inertia_elem.get("ixx", "0"))
		ixy = float(inertia_elem.get("ixy", "0"))
		ixz = float(inertia_elem.get("ixz", "0"))
		iyy = float(inertia_elem.get("iyy", "0"))
		iyz = float(inertia_elem.get("iyz", "0"))
		izz = float(inertia_elem.get("izz", "0"))
		
		# Build the inertia tensor
		inertia_tensor = np.array([[ixx, ixy, ixz],
									[ixy, iyy, iyz],
									[ixz, iyz, izz]])
		
		# Create rotation matrix from RPY
		rot_matrix = rotation_matrix_from_rpy(roll, pitch, yaw)
		
		# Transform the inertia tensor
		inertia_transformed = transform_inertia_tensor(inertia_tensor, rot_matrix)
		
		# Update the inertia values in the XML
		inertia_elem.set("ixx", f"{inertia_transformed[0, 0]:.10g}")
		inertia_elem.set("ixy", f"{inertia_transformed[0, 1]:.10g}")
		inertia_elem.set("ixz", f"{inertia_transformed[0, 2]:.10g}")
		inertia_elem.set("iyy", f"{inertia_transformed[1, 1]:.10g}")
		inertia_elem.set("iyz", f"{inertia_transformed[1, 2]:.10g}")
		inertia_elem.set("izz", f"{inertia_transformed[2, 2]:.10g}")
		
		# Set RPY to zero
		origin.set("rpy", "0 0 0")
		
		link_name = link_node.get("name", "unknown")
		print_debug(f"   -> Zeroed inertial orientation for link '{link_name}' (was: {roll:.4f} {pitch:.4f} {yaw:.4f})")
		
		return True
		
	except Exception as e:
		link_name = link_node.get("name", "unknown")
		print_warning(f"Failed to transform inertial for link '{link_name}': {e}")
		return False

def resolve_path(path):
	"""
	Resolve a path that can be a package URI, a file URI, a relative path, or an absolute path.
	Also supports environment variable syntax: ${env:VAR}
	Returns the absolute path if resolvable, otherwise None.
	"""
	if not path:
		return None

	# Helper function to resolve $(find package) syntax
	def _resolve_find_syntax(path_str, context=""):
		find_match = re.match(r"\$\(find\s+([^)]+)\)(.*)", path_str)
		if find_match:
			package_name = find_match.group(1)
			relative_path = find_match.group(2).lstrip("/")
			print_info(f"Resolving for package '{package_name}' with relative path '{relative_path}'")
			try:
				pkg_share = get_package_share_directory(package_name)
				return os.path.abspath(os.path.join(pkg_share, relative_path))
			except PackageNotFoundError:
				context_msg = f" in {context}" if context else ""
				print_warning(f"Could not resolve package '{package_name}'{context_msg}.")
				return None
		return None

	# Helper function to resolve ${env:VAR} syntax
	def _resolve_env_syntax(path_str, context=""):
		"""Resolve environment variables in the format ${env:VAR_NAME}"""
		env_pattern = r"\$\{env:([^}]+)\}"
		
		def replace_env_var(match):
			var_name = match.group(1)
			env_value = os.environ.get(var_name)
			if env_value is not None:
				print_info(f"Resolved environment variable '{var_name}' to '{env_value}'")
				return env_value
			else:
				context_msg = f" in {context}" if context else ""
				print_warning(f"Environment variable '{var_name}' not found{context_msg}.")
				return match.group(0)  # Return original string if not found
		
		return re.sub(env_pattern, replace_env_var, path_str)

	# Resolve environment variables first
	if "${env:" in path:
		path = _resolve_env_syntax(path)

	if path.startswith("package://"):
		parts = path[len("package://") :].split("/", 1)
		package_name = parts[0]
		relative_path = parts[1] if len(parts) > 1 else ""
		try: 	
			pkg_share = get_package_share_directory(package_name)
			return os.path.abspath(os.path.join(pkg_share, relative_path))
		except PackageNotFoundError:
			print_warning(f"Could not resolve package '{package_name}'")
			return None
	
	elif path.startswith("file://"):
		path = path[len("file://") :]
		resolved = None
		# Resolve environment variables in file URI
		if "${env:" in path:
			path = _resolve_env_syntax(path, "file URI")
		# Try to resolve $(find package) syntax first
		if "$(find" in path:
			resolved = _resolve_find_syntax(path, "file URI")
		elif os.path.exists(path):
			resolved = os.path.abspath(path)
		if resolved is not None:
			return resolved
		# Fallback to regular file path
		return os.path.abspath(path.lstrip("/"))
	
	# Handle $(find package) syntax in regular paths
	resolved = _resolve_find_syntax(path)
	if resolved is not None:
		return resolved
	
	# Handle regular file paths (relative or absolute)
	return os.path.abspath(path)

def _merge_nodes_recursively(nodes):
	"""Recursively merge multiple XML nodes with the same tag, only merging truly equivalent elements."""
	if not nodes:
		return None
	
	if len(nodes) == 1:
		return copy.deepcopy(nodes[0])
	
	# Start with the first node as base
	merged = copy.deepcopy(nodes[0])
	
	# For each additional node, merge its children
	for node in nodes[1:]:
		for child in node:
			# Find if there's an equivalent child in merged node
			equivalent_child = None
			for existing_child in merged:
				# Check if elements are equivalent (same tag, attributes, and text)
				if (existing_child.tag == child.tag and 
					existing_child.attrib == child.attrib and 
					(existing_child.text or "").strip() == (child.text or "").strip()):
					equivalent_child = existing_child
					break
			
			if equivalent_child is not None:
				# Found equivalent child - merge them recursively
				if len(list(child)) > 0 or len(list(equivalent_child)) > 0:
					# Has children, merge recursively
					merged_child = _merge_nodes_recursively([equivalent_child, child])
					# Replace the existing child with merged version
					merged.remove(equivalent_child)
					merged.append(merged_child)
				# If no children, the elements are already equivalent, no action needed
			else:
				# No equivalent child found - add as new child
				merged.append(copy.deepcopy(child))
	
	return merged

def preprocess_urdf(urdf_path, compiler_options, default_mesh_dir, separate_dae_meshes=False, append_mesh_type=False, zero_inertial_rpy=True):
	"""Pre-process URDF for MuJoCo compatibility."""
	print_base("Pre-processing URDF...")
	urdf_tree = ET.parse(urdf_path)
	root = urdf_tree.getroot()

	###########################
	# Final mapping for copy/convert: desired_dest_basename -> {"src": abs_path}
	###########################
	absolute_mesh_paths = {}
	
	# Temporary directory for DAE extraction during preprocessing
	temp_dae_extract_dir = tempfile.mkdtemp(prefix="urdf2mjcf_dae_")

	# Find all <link> and their mesh elements
	link_nodes = root.findall(".//link")
	
	# First pass: Zero inertial orientations if requested
	if zero_inertial_rpy:
		print_info("Zeroing inertial orientations (transforming inertia tensors)...")
		transformed_count = 0
		for link_node in link_nodes:
			if zero_inertial_orientation(link_node):
				transformed_count += 1
		if transformed_count > 0:
			print_confirm(f"-> Transformed {transformed_count} inertial orientation(s) to zero RPY")
	
	for link_node in link_nodes:
		link_name = link_node.get("name")
		link_dict = {}
		
		# Process visual meshes (with DAE expansion)
		visual_elements = link_node.findall("visual")
		for visual_idx, visual_elem in enumerate(visual_elements):
			geometry = visual_elem.find("geometry")
			if geometry is None:
				continue
			
			mesh_elem = geometry.find("mesh")
			if mesh_elem is None:
				continue
			
			file_path = mesh_elem.get("filename", "")
			src_path = resolve_path(file_path)
			
			if not src_path or not os.path.isfile(src_path):
				raise FileNotFoundError(f"Mesh file at {file_path} not found or could not be resolved.")
			
			base_name = os.path.basename(src_path)
			file_name = os.path.splitext(base_name)[0]
			file_ext = os.path.splitext(base_name)[1].lower()
			
			# Check if it's a DAE file that needs expansion
			if file_ext == ".dae" and EXTRACT_DAE_AVAILABLE:
				print_info(f"Expanding DAE file '{base_name}' for link '{link_name}' into multiple visual elements...")
				
				try:
					# Extract meshes from DAE
					success_count, total_count, dae_mesh_info = extract_meshes_from_dae(
						src_path,
						temp_dae_extract_dir,
						output_format='stl',
						verbose=False,
						separate_meshes=separate_dae_meshes
					)
					
					if success_count > 0 and dae_mesh_info:
						print_confirm(f"-> Extracted {success_count} meshes from '{base_name}'")
						
						# Get the parent of visual element (should be the link)
						parent = link_node
						visual_insert_idx = list(parent).index(visual_elem)
						
						# Remove the original visual element
						parent.remove(visual_elem)
						
						# Create new visual elements for each extracted mesh
						for mesh_idx, (mesh_name, mesh_data) in enumerate(dae_mesh_info.items()):
							# Create a new visual element (copy attributes from original)
							new_visual = ET.Element("visual")
							
							# Copy origin if it exists
							origin = visual_elem.find("origin")
							if origin is not None:
								new_origin = copy.deepcopy(origin)
								new_visual.append(new_origin)
							
							# Create geometry with the extracted mesh
							new_geometry = ET.SubElement(new_visual, "geometry")
							new_mesh = ET.SubElement(new_geometry, "mesh")
							
							extracted_file = mesh_data['file']
							
							# Determine the destination filename (with suffix if needed)
							if append_mesh_type:
								fname, fext = os.path.splitext(extracted_file)
								dest_file = f"{fname}_visual{fext}"
							else:
								dest_file = extracted_file
							
							new_mesh.set("filename", dest_file)
							
							# Add material if RGBA is available
							rgba = mesh_data.get('rgba')
							if rgba:
								material_elem = ET.SubElement(new_visual, "material")
								material_name = f"{link_name}_{mesh_name}_mat"
								material_elem.set("name", material_name)
								
								# Add color sub-element with RGBA
								color_elem = ET.SubElement(material_elem, "color")
								rgba_str = f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"
								color_elem.set("rgba", rgba_str)
								
								print_debug(f"   -> Added visual '{mesh_name}' with material (RGBA: {rgba_str})")
							else:
								print_debug(f"   -> Added visual '{mesh_name}' (no material info)")
							
							# Insert the new visual element
							parent.insert(visual_insert_idx + mesh_idx, new_visual)
							
							# Track the mesh for copying (source is original name, dest has suffix)
							temp_mesh_path = os.path.join(temp_dae_extract_dir, extracted_file)
							if "visual" not in link_dict:
								link_dict["visual"] = {"from": [], "to": []}
							link_dict["visual"]["from"].append(temp_mesh_path)
							link_dict["visual"]["to"].append(dest_file)
					else:
						print_warning(f"Failed to extract meshes from '{base_name}', using fallback single-mesh conversion")
						# Fallback: treat as single mesh
						dest_path = f"{file_name}.stl"
						if append_mesh_type:
							dest_path = f"{file_name}_visual.stl"
						mesh_elem.set("filename", dest_path)
						if "visual" not in link_dict:
							link_dict["visual"] = {"from": [], "to": []}
						link_dict["visual"]["from"].append(src_path)
						link_dict["visual"]["to"].append(dest_path)
						
				except Exception as e:
					print_warning(f"Error extracting DAE '{base_name}': {e}. Using fallback single-mesh conversion.")
					# Fallback: treat as single mesh
					dest_path = f"{file_name}.stl"
					if append_mesh_type:
						dest_path = f"{file_name}_visual.stl"
					mesh_elem.set("filename", dest_path)
					if "visual" not in link_dict:
						link_dict["visual"] = {"from": [], "to": []}
					link_dict["visual"]["from"].append(src_path)
					link_dict["visual"]["to"].append(dest_path)
			
			else:
				# Not a DAE or extraction not available - handle normally
				if file_ext == ".dae":
					print_warning(f"DAE extraction not available (install pycollada and trimesh). Using fallback single-mesh conversion for '{base_name}'.")
					dest_path = f"{file_name}_visual.stl" if append_mesh_type else f"{file_name}.stl"
				else:
					if append_mesh_type:
						fname, fext = os.path.splitext(base_name)
						dest_path = f"{fname}_visual{fext}"
					else:
						dest_path = base_name
				
				mesh_elem.set("filename", dest_path)
				if "visual" not in link_dict:
					link_dict["visual"] = {"from": [], "to": []}
				link_dict["visual"]["from"].append(src_path)
				link_dict["visual"]["to"].append(dest_path)
		
		# Process collision meshes (no DAE expansion needed)
		collision_mesh_nodes = link_node.findall(".//collision/geometry/mesh")
		for mesh_elem in collision_mesh_nodes:
			file_path = mesh_elem.get("filename", "")
			src_path = resolve_path(file_path)
			
			if not src_path or not os.path.isfile(src_path):
				raise FileNotFoundError(f"Collision mesh file at {file_path} not found or could not be resolved.")
			
			base_name = os.path.basename(src_path)
			file_name = os.path.splitext(base_name)[0]
			file_ext = os.path.splitext(base_name)[1].lower()
			
			if file_ext == ".dae":
				dest_path = f"{file_name}_collision.stl"
			else:
				if append_mesh_type:
					fname, fext = os.path.splitext(base_name)
					dest_path = f"{fname}_collision{fext}"
				else:
					dest_path = base_name
			
			mesh_elem.set("filename", dest_path)
			if "collision" not in link_dict:
				link_dict["collision"] = {"from": [], "to": []}
			link_dict["collision"]["from"].append(src_path)
			link_dict["collision"]["to"].append(dest_path)

		####################################################################### 
		# NOTE: DISCLAIMER - This approach is still really heuristic and does not 
		# handle well non-standard cases:
		# 1) Where there are repeating 'visual' meshes which share the same source
		# path and whose destination path collides with one or more collision meshes.
		# 2) Where a 'visual' mesh destination collides with another 'visual' mesh  
		# but they have different source paths
		# If this is the case, manual copy is required for now.
		####################################################################### 
		# Prevent collision of dest_path names if src_path are different
		if 'visual' in link_dict and 'collision' in link_dict:
			vis_src_paths = link_dict['visual']['from']
			vis_dest_paths = link_dict['visual']['to']
			col_src_paths = link_dict['collision']['from']
			col_dest_paths = link_dict['collision']['to']
			
			for i in range(len(vis_dest_paths)):
				dest_visual = vis_dest_paths[i]
				src_visual = vis_src_paths[i]
				
				# Find all collision meshes that have the same destination path but a different source path
				collided_indices = [
					j for j, dest_collision in enumerate(col_dest_paths)
					if dest_collision == dest_visual and col_src_paths[j] != src_visual
				]
				
				if len(collided_indices) > 0:
					# Rename visual mesh
					vis_filename, visual_ext = os.path.splitext(os.path.basename(dest_visual))
					new_dest_visual = f"{vis_filename}_visual{visual_ext}"
					vis_dest_paths[i] = new_dest_visual
					
					# Find corresponding XML element and update it
					# This is tricky because we don't have a direct link back to the element.
					# We rely on the fact that findall will return elements in document order.
					# This is a fragile part of the original script's logic.
					visual_mesh_elements = link_node.findall(".//visual/geometry/mesh")
					if i < len(visual_mesh_elements):
						visual_mesh_elements[i].set("filename", new_dest_visual)

					# Rename collided collision meshes
					for idx in collided_indices:
						col_dest_path = col_dest_paths[idx]
						col_filename, collision_ext = os.path.splitext(os.path.basename(col_dest_path))
						new_col_dest_path = f"{col_filename}_collision{collision_ext}"
						col_dest_paths[idx] = new_col_dest_path

						collision_mesh_elements = link_node.findall(".//collision/geometry/mesh")
						if idx < len(collision_mesh_elements):
							collision_mesh_elements[idx].set("filename", new_col_dest_path)

		if len(link_dict) == 0:
			continue

		absolute_mesh_paths[link_name] = link_dict
	
	# Clean up temporary DAE extraction directory
	# Note: We keep the extracted meshes because they're tracked in absolute_mesh_paths
	# and will be copied to the final output directory by mesh_ops.copy_mesh_files()
	# The temp directory will be cleaned up automatically by the OS eventually,
	# but we could also manually clean it up here if needed.
	# For now, we leave it since the files are still referenced.

	absolute_mesh_paths_str = "Absolute mesh paths found:"
	for link_name, value in absolute_mesh_paths.items():
		absolute_mesh_paths_str += f"\n=> \"{link_name}:\""
		for mesh_type, src_path in value.items():
			absolute_mesh_paths_str += f"\n- {mesh_type}:"
			src_path_from_str = "\n\t".join(src_path['from'])
			src_path_to_str = "\n\t".join(src_path['to'])
			absolute_mesh_paths_str += f"\n{' '*3}from:\n\t{src_path_from_str}"
			absolute_mesh_paths_str += f"\n{' '*3}to:\n\t{src_path_to_str}"
		absolute_mesh_paths_str += "\n"
	print_info(absolute_mesh_paths_str)

	###########################
	# Process <mujoco> tags
	###########################
	all_mujoco_nodes = root.findall("mujoco")
	custom_mujoco_elements = []
	compiler_node = None

	if all_mujoco_nodes:
		print_info(f"-> Found {len(all_mujoco_nodes)} <mujoco> tags in URDF. Merging them.")
		# Merge all mujoco nodes recursively
		mujoco_node = _merge_nodes_recursively(all_mujoco_nodes)
		
		# Remove all existing mujoco nodes 
		for node_to_remove in all_mujoco_nodes:
			root.remove(node_to_remove)
		if mujoco_node is not None:
			root.insert(0, mujoco_node)
	else:
		print_warning("No <mujoco> tag found in URDF. Creating one.")
		mujoco_node = ET.Element("mujoco")
		root.insert(0, mujoco_node)

	compiler_node = mujoco_node.find("compiler")
	# Find compiler node, if not create one
	if compiler_node is None:
		compiler_node = ET.Element("compiler")
		mujoco_node.append(compiler_node)

	for child in list(mujoco_node):
		if child.tag != "compiler":
			custom_mujoco_elements.append(child)
			mujoco_node.remove(child)

	urdf_plugins = []
	for elem in custom_mujoco_elements:
		if elem.tag == "plugin":
			urdf_plugins.append(elem)
	custom_mujoco_elements = [e for e in custom_mujoco_elements if e.tag != "plugin"]

	if custom_mujoco_elements:
		tags = [e.tag for e in custom_mujoco_elements]
		print_base(f"-> Found {len(custom_mujoco_elements)} custom MuJoCo elements to inject: {', '.join(tags)}")
		# Debug: show each element's attributes
		for i, elem in enumerate(custom_mujoco_elements):
			attrs_str = ", ".join([f"{k}='{v}'" for k, v in elem.attrib.items()])
			print_debug(f"   Element {i+1}: <{elem.tag} {attrs_str}>")
		
	final_compiler_attrs = {
		"meshdir": default_mesh_dir,
		"balanceinertia": "false",
		"discardvisual": "false",
		"fusestatic": "false",
		"inertiafromgeom": "false",
	}
	final_compiler_attrs.update(compiler_node.attrib)

	if compiler_options:
		for option in compiler_options:
			if "=" in option:
				key, value = option.split("=", 1)
				final_compiler_attrs[key] = value
			else:
				print_warning(
					f"Malformed compiler option '{option}'. Should be in KEY=VALUE format. Skipping."
				)

	for key, value in final_compiler_attrs.items():
		compiler_node.set(key, value)

	print_base(f"-> Set <compiler> tag attributes to: {final_compiler_attrs}")

	###########################
	# Process mimic joints
	###########################
	mimic_joints = {}
	for joint_node in root.findall(".//joint"):
		mimic_tag = joint_node.find("mimic")
		if mimic_tag is not None:
			joint_name = joint_node.get("name")
			mimic_joint_name = mimic_tag.get("joint")
			multiplier = mimic_tag.get("multiplier", "1.0")
			offset = mimic_tag.get("offset", "0.0")
			if joint_name and mimic_joint_name:
				mimic_joints[joint_name] = {
					"joint": mimic_joint_name,
					"multiplier": multiplier,
					"offset": offset,
				}
	if mimic_joints:
		print_base(f"-> Found mimic joints: {', '.join(mimic_joints.keys())}")

	###########################
	# Process ros2 control tags
	###########################
	ros2c_joint_map = {}
	for rc_node in root.findall(".//ros2_control"):
		for jnode in rc_node.findall("joint"):
			jname = jnode.get("name")
			if not jname:
				continue
			ifaces = set()
			for ci in jnode.findall("command_interface"):
				ci_name = ci.get("name") or (ci.text.strip() if ci.text else "")
				if ci_name:
					ifaces.add(ci_name)
			if ifaces:
				ros2c_joint_map[jname] = ifaces
	
	if ros2c_joint_map:
		for jname, ifaces in ros2c_joint_map.items():
			print_base(f"-> Joint '{jname}' has command interfaces: {', '.join(ifaces)}")

	return urdf_tree, absolute_mesh_paths, mimic_joints, custom_mujoco_elements, urdf_plugins, ros2c_joint_map