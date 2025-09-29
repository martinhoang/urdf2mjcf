import os
import re
import copy
import xml.etree.ElementTree as ET
from typing import Literal
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from _utils import (
	print_base,
	print_info,
	print_warning,
	print_debug,
)

def resolve_path(path):
	"""
	Resolve a path that can be a package URI, a file URI, a relative path, or an absolute path.
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

def preprocess_urdf(urdf_path, compiler_options, default_mesh_dir):
	"""Pre-process URDF for MuJoCo compatibility."""
	print_base("Pre-processing URDF...")
	urdf_tree = ET.parse(urdf_path)
	root = urdf_tree.getroot()

	###########################
	# Final mapping for copy/convert: desired_dest_basename -> {"src": abs_path}
	###########################
	absolute_mesh_paths = {}

	# Find all <link> and their <mesh> elements
	link_nodes = root.findall(".//link")
	
	def _get_mesh_path(elements : list, mesh_type: Literal["visual", "collision"]):
		output_dict = {}
		if len(elements) == 0:
			return output_dict
		
		for elem in elements:
			file_path = elem.get("filename", "")
			src_path = resolve_path(elem.get("filename", ""))
			
			if not src_path or not os.path.isfile(src_path):
				raise FileNotFoundError(f"Mesh file at {file_path} not found or could not be resolved.")
			
			base_name = os.path.basename(src_path)
			file_name = os.path.splitext(base_name)[0]
			file_ext = os.path.splitext(base_name)[1].lower()

			dest_path = base_name

			if file_ext == ".dae":
				print_warning(f"Mesh '{file_name}' is a DAE file. Will attempt to convert it to STL during processing. Set mesh reduction ratio on the command line to your preference.")
				dest_path = f"{file_name}.stl"

			# Add mesh path to output dict as a dict or if more than one meshes, append to a list
			if mesh_type not in output_dict:
				output_dict[mesh_type] = {"from" : [src_path], "to" : [dest_path]}
			else:
				# Assume it is already a list from now
				output_dict[mesh_type]["from"].append(src_path)
				output_dict[mesh_type]["to"].append(dest_path)

			#! This step is important
			# Copy back the absolute path to the URDF for mujoco compiler
			elem.set("filename", dest_path)

		return output_dict

	for link_node in link_nodes:
		visual_mesh_node = link_node.findall(".//visual/geometry/mesh")
		collision_mesh_node = link_node.findall(".//collision/geometry/mesh")
		link_name = link_node.get("name")

		link_dict = _get_mesh_path(visual_mesh_node, "visual")
		link_dict.update(_get_mesh_path(collision_mesh_node, "collision"))

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