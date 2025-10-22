import copy
import os
import re
import xml.etree.ElementTree as ET

from . import urdf_preprocess, xml_utils
from _utils import (
    print_base,
    print_debug,
    print_info,
    print_warning,
    print_confirm,
)

def post_process_damping_multiplier(root, damping_multiplier):
	"""Multiply all joint damping by a factor to stabilize simulation for some robots."""
	for joint in root.findall(".//joint"):
		if "damping" in joint.attrib:
				original_damping = float(joint.get("damping"))
				joint.set("damping", str(original_damping * damping_multiplier))
		print_base(f"-> Multiplied joint damping by a factor of {damping_multiplier}.")

def post_process_compiler_options(root):
	"""Apply compiler options captured from URDF into final MJCF."""
	# Capture <compiler> options if present
	if root is not None:
		compiler_node = root.find("compiler")

	if compiler_node is None:
		print_warning("No <compiler> tag found to apply post-processing to.")
		return
	
	final_compiler_node = root.find("compiler")
	if final_compiler_node is None:
		final_compiler_node = ET.Element("compiler")
		root.insert(0, final_compiler_node)
	attrs_to_copy = compiler_node.attrib
	for key, value in attrs_to_copy.items():
		final_compiler_node.set(key, value)
	print_base(f"-> Applied post-processing to <compiler> tag with attributes: {attrs_to_copy}")

def _parse_attr_string(attr_str, separator=" "):
	"""Parse attribute string with given separator into a dictionary.
	
	Examples:
	- "class='visual' group='2'" (separator=" ")
	- "class='visual';group='2'" (separator=";")  
	- "class='visual',group='2'" (separator=",")
	- "name:='value'" (ROS-style assignment with :=)
	- "pos='1.0 2.0 3.0' size='0.5 0.5 0.5'" (space-separated numbers in quotes)
	"""
	attrs = {}
	duplicate_keys = set()
	
	# Use regex to find all key=value or key:=value pairs, handling quoted values that may contain spaces
	# Supports both = and := syntax
	pattern = r"(\w+):?=(['\"])(.*?)\2"
	matches = re.findall(pattern, attr_str)
	
	for match in matches:
		key, quote, value = match
		if key in attrs:
			duplicate_keys.add(key)
			print_warning(f"Duplicate attribute '{key}' found in '{attr_str}'. Using last value: '{value}'")
		attrs[key] = value
	
	if not attrs:
		print_warning(f"Could not parse attribute string: '{attr_str}'. Expected format like key1='value1'{separator}key2='value2' or key:='value'")
	
	return attrs

def _parse_conditional_replacement(attr_string):
	"""Parse conditional replacement string like 'attr1=val1,attr2=val2:new_attr=new_val'.
	
	Returns a dictionary with:
	- 'conditions': list of (key, value) tuples that must all match
	- 'replacements': list of (key, value) tuples to apply if conditions match
	"""
	try:
		conditions_part, replacement_part = attr_string.split(':', 1)
		
		# Parse conditions (left side of colon)
		conditions = []
		condition_attrs = _parse_attr_string(conditions_part.strip(), separator=",")
		for key, value in condition_attrs.items():
			conditions.append((key, value))
		
		# Parse replacements (right side of colon)  
		replacements = []
		replacement_attrs = _parse_attr_string(replacement_part.strip(), separator=",")
		for key, value in replacement_attrs.items():
			replacements.append((key, value))
		
		if not conditions:
			print_warning(f"No valid conditions found in conditional replacement: {attr_string}")
			return None
			
		if not replacements:
			print_warning(f"No valid replacements found in conditional replacement: {attr_string}")
			return None
		
		return {'conditions': conditions, 'replacements': replacements}
		
	except ValueError:
		print_warning(f"Invalid conditional replacement format: {attr_string}")
		return None

def _parse_custom_syntax(elem):
	"""Parse custom syntax attributes and return operations to perform.
	
	Supported syntax:
	- inject_attr="key1='value1' key2='value2'"
	- inject_attrs="key1='value1';key2='value2'"  
	- replace_attrs="key1='value1',key2='value2'"
	- inject_children="key1='value1',key2='value2'"
	"""
	operations = []
	
	# Handle inject_attr (space-separated)
	inject_attr = elem.get("inject_attr")
	if inject_attr:
		attrs = _parse_attr_string(inject_attr, separator=" ")
		if attrs:
			operations.append(("inject", attrs))
	
	# Handle inject_attrs (semicolon-separated)
	inject_attrs = elem.get("inject_attrs")
	if inject_attrs:
		attrs = _parse_attr_string(inject_attrs, separator=";")
		if attrs:
			operations.append(("inject", attrs))
	
	# Handle replace_attrs (comma-separated or conditional replacement)
	replace_attrs = elem.get("replace_attrs")
	if replace_attrs:
		if ':' in replace_attrs:
			# Handle conditional replacement: "cond1='val1',cond2='val2':new_attr='new_val'"
			conditional_op = _parse_conditional_replacement(replace_attrs)
			if conditional_op:
				operations.append(("conditional_replace", conditional_op))
		else:
			# Handle simple replacement
			attrs = _parse_attr_string(replace_attrs, separator=",")
			if attrs:
				operations.append(("replace", attrs))
	
	# Handle inject_children (comma-separated attributes to match)
	inject_children = elem.get("inject_children")
	if inject_children:
		attrs = _parse_attr_string(inject_children, separator=",")
		if attrs:
			operations.append(("inject_children", attrs))
	
	return operations

def _apply_custom_operations(target_node, operations):
	"""Apply custom operations (inject/replace) to a target node."""
	for operation_type, attrs in operations:
		if operation_type == "inject":
			# Always add/overwrite attributes
			for attr_name, attr_value in attrs.items():
				old_value = target_node.get(attr_name)
				target_node.set(attr_name, attr_value)
				if old_value:
					print_debug(f"Injected attribute {attr_name}='{attr_value}' (overwrote '{old_value}') into <{target_node.tag}>")
				else:
					print_debug(f"Injected attribute {attr_name}='{attr_value}' into <{target_node.tag}>")
		
		elif operation_type == "replace":
			# Only replace if attribute already exists
			for attr_name, attr_value in attrs.items():
				if target_node.get(attr_name) is not None:
					old_value = target_node.get(attr_name)
					target_node.set(attr_name, attr_value)
					print_debug(f"Replaced attribute {attr_name}='{old_value}' with '{attr_value}' in <{target_node.tag}>")
				else:
					print_warning(f"Cannot replace non-existent attribute '{attr_name}' in <{target_node.tag}>. Use inject_attr(s) to add new attributes.")
		
		elif operation_type == "conditional_replace":
			# Check if all conditions match
			conditions = attrs['conditions']
			replacements = attrs['replacements']
			
			# Check if all conditions are satisfied
			all_conditions_match = True
			for cond_key, cond_value in conditions:
				current_value = target_node.get(cond_key)
				if current_value != cond_value:
					all_conditions_match = False
					break
			
			if all_conditions_match:
				# Remove the condition attributes and add replacement attributes
				condition_keys = [key for key, _ in conditions]
				replacement_info = []
				
				# Remove condition attributes
				for key in condition_keys:
					old_value = target_node.get(key)
					if old_value is not None:
						del target_node.attrib[key]
						replacement_info.append(f"removed {key}='{old_value}'")
				
				# Add replacement attributes
				for repl_key, repl_value in replacements:
					old_value = target_node.get(repl_key)
					target_node.set(repl_key, repl_value)
					if old_value:
						replacement_info.append(f"set {repl_key}='{repl_value}' (overwrote '{old_value}')")
					else:
						replacement_info.append(f"set {repl_key}='{repl_value}'")
				
				print_debug(f"Conditional replacement applied to <{target_node.tag}>: {', '.join(replacement_info)}")
			else:
				print_debug(f"Conditional replacement conditions not met for <{target_node.tag}>")
	
	# Log final result
	target_attrs_str = ", ".join([f"{k}='{v}'" for k, v in target_node.attrib.items()])
	print_base(f"Applied custom operations to element: <{target_node.tag} {target_attrs_str}>")

def post_process_inject_custom_mujoco_elements(root, elements):
	"""Inject elements from URDF <mujoco> into MJCF.
	Supports child element injection, attribute injection, and attribute replacement."""
	if not elements:
		print_warning("No custom MJCF elements found to inject from URDF.")
		return

	elements_str = "\n" + "\n".join([ET.tostring(elem, encoding="unicode") for elem in elements])
	print_info(f"-> Injecting custom MJCF elements from URDF: {elements_str}")
	
	def _process_element_recursively(elem, parent_context=None):
		"""Process an element and its children recursively."""
		# Parse custom syntax attributes
		custom_operations = _parse_custom_syntax(elem)
		
		# Debug: Log what operations were found
		elem_attrs_str = ", ".join([f"{k}='{v}'" for k, v in elem.attrib.items()])
		if custom_operations:
			ops_str = ", ".join([f"{op[0]}" for op in custom_operations])
			print_debug(f"Processing <{elem.tag} {elem_attrs_str}> with operations: [{ops_str}]")
		
		# Handle inject_children operation specially
		inject_children_op = next((op for op in custom_operations if op[0] == "inject_children"), None)
		if inject_children_op:
			_, match_attrs = inject_children_op
			
			print_debug(f"Found inject_children operation with match attributes: {match_attrs}")
			
			# Find all matching elements in the MJCF that match the specified attributes
			# We need to search for elements with the same tag as elem and matching attributes
			search_context = parent_context if parent_context is not None else root
			
			# Build XPath query to find matching elements
			xpath_parts = [elem.tag]
			for attr_name, attr_value in match_attrs.items():
				xpath_parts.append(f"[@{attr_name}='{attr_value}']")
			xpath_query = "".join(xpath_parts)
			
			print_debug(f"Searching with XPath: {xpath_query}")
			
			# Search for matching elements
			matching_targets = search_context.findall(f".//{xpath_query}")
			
			print_debug(f"Found {len(matching_targets)} matching target(s)")
			
			if matching_targets:
				# Inject all children of this element into each matching target
				children_to_inject = list(elem)  # Get all child elements
				
				for target_node in matching_targets:
					target_attrs_str = ", ".join([f"{k}='{v}'" for k, v in target_node.attrib.items()])
					print_info(f"Injecting {len(children_to_inject)} child element(s) into <{target_node.tag} {target_attrs_str}>")
					
					for child in children_to_inject:
						child_copy = copy.deepcopy(child)
						target_node.append(child_copy)
						child_attrs_str = ", ".join([f"{k}='{v}'" for k, v in child_copy.attrib.items()])
						print_confirm(f"  -> Injected <{child_copy.tag} {child_attrs_str}> into matching <{target_node.tag}>")
			else:
				match_attrs_str = ", ".join([f"{k}='{v}'" for k, v in match_attrs.items()])
				context_desc = f"within {parent_context.tag}" if parent_context is not None else "globally"
				print_warning(f"No matching <{elem.tag}> elements found with attributes [{match_attrs_str}] {context_desc}")
			
			# IMPORTANT: Always return here - do NOT add this element to the MJCF
			# The inject_children operation consumes the element entirely
			return
		
		# Handle other custom operations (inject_attr, replace_attrs, etc.)
		if custom_operations:
			# Remove custom syntax attributes from matching criteria
			matching_attrs = {k: v for k, v in elem.attrib.items() 
							if k not in ["inject_attr", "inject_attrs", "replace_attrs", "inject_children"]}
			elem_for_matching = ET.Element(elem.tag, matching_attrs)
			
			# Find matching nodes in the appropriate context
			if parent_context is not None:
				# Search within the parent context only
				matching_nodes = xml_utils.find_matching_elements(parent_context, elem_for_matching)
			else:
				# Global search
				matching_nodes = xml_utils.find_matching_elements(root, elem_for_matching)
			
			if matching_nodes:
				for target_node in matching_nodes:
					_apply_custom_operations(target_node, custom_operations)
			else:
				matching_attrs_str = ", ".join([f"{k}='{v}'" for k, v in matching_attrs.items()])
				context_desc = f"within {parent_context.tag}" if parent_context is not None else "globally"
				print_warning(f"No matching elements found for custom operations pattern <{elem.tag} {matching_attrs_str}> {context_desc}")
			
			# Process children of this element recursively (in case there are nested operations)
			for child in elem:
				_process_element_recursively(child, parent_context)
			return
		
		# Check if this element has children that need custom operations
		has_children_with_operations = any(_parse_custom_syntax(child) for child in elem)
		
		if has_children_with_operations:
			# Find matching parent element to provide context for children
			matching_parents = xml_utils.find_matching_elements(root, elem)
			
			if matching_parents:
				for parent_node in matching_parents:
					# Process children within this parent context
					for child in elem:
						child_operations = _parse_custom_syntax(child)
						if child_operations:
							# Process child with custom operations
							_process_element_recursively(child, parent_node)
						else:
							# Regular child injection - copy the child as-is into the parent
							child_copy = copy.deepcopy(child)
							parent_node.append(child_copy)
							print_debug(f"Injected regular child <{child_copy.tag}> into existing <{parent_node.tag}>.")
			else:
				attrs_str = ", ".join([f"{k}='{v}'" for k, v in elem.attrib.items()])
				print_warning(f"No matching parent element found for <{elem.tag} {attrs_str}> - cannot apply child operations")
			return
		
		# Standard child element injection (existing behavior)
		matching_nodes = xml_utils.find_matching_elements(root, elem)
		
		if matching_nodes:
			# Inject children and attributes into matching existing elements
			for target_node in matching_nodes:
				attrs_str = ", ".join([f"{k}='{v}'" for k, v in elem.attrib.items()])
				target_attrs_str = ", ".join([f"{k}='{v}'" for k, v in target_node.attrib.items()])
				print_info(f"Injecting element <{elem.tag} {attrs_str}> into existing element: <{target_node.tag} {target_attrs_str}>")
				
				# Copy attributes from source to target
				for attr_name, attr_value in elem.attrib.items():
					target_node.set(attr_name, attr_value)
					print_debug(f"Copied attribute {attr_name}='{attr_value}' to existing <{target_node.tag}>")
				
				# Copy children from the injected element to the target
				for child in elem:
					child_copy = copy.deepcopy(child)
					target_node.append(child_copy)
					print_debug(f"Injected <{child_copy.tag}> into existing <{target_node.tag}>.")
			return

		# No matching element found, create new element at root level
		target_node = root.find(elem.tag)
		if target_node is None:
			target_node = xml_utils.ensure_node_before_worldbody(root, elem.tag)
			print_debug(f"Created new <{elem.tag}> tag in MJCF.")
		
		# Copy attributes from source element to target
		for attr_name, attr_value in elem.attrib.items():
			target_node.set(attr_name, attr_value)
			print_debug(f"Copied attribute {attr_name}='{attr_value}' to <{elem.tag}>")
		
		# Copy children from source element to target
		for child in elem:
			child_copy = copy.deepcopy(child)
			target_node.append(child_copy)
			print_debug(f"Injected <{child_copy.tag}> into <{elem.tag}>.")
	
	# Process all root-level elements
	for elem in elements:
		_process_element_recursively(elem)


def post_process_transform_and_add_custom_plugin(root, urdf_plugin_node):
	"""Transform URDF-style <plugin> into MJCF <plugin> under <extension>."""
	plugin_name = urdf_plugin_node.get("filename")
	instance_name = urdf_plugin_node.get("name", plugin_name)
	if not plugin_name:
		print_warning("Skipping custom <plugin> tag with no 'filename' attribute.")
		return

	extension_node = xml_utils.ensure_extension_node(root)
	mjcf_plugin_node = ET.SubElement(extension_node, "plugin", {"plugin": plugin_name})
	instance_node = ET.SubElement(mjcf_plugin_node, "instance", {"name": instance_name})

	has_params = False
	for param_node in urdf_plugin_node:
		key = param_node.tag
		value = param_node.text.strip() if param_node.text else ""
		resolved_value = urdf_preprocess.resolve_path(value)

		if not resolved_value or resolved_value == '':
			print_warning(f"Could not resolve path for plugin parameter '{key}': {value}. Skipping this parameter.")
			continue

		ET.SubElement(instance_node, "config", {"key": key, "value": resolved_value})
		has_params = True

	if has_params:
		print_base(f"-> Transformed and added custom plugin '{plugin_name}' with parameters.")
	else:
		print_base(f"-> Transformed and added custom plugin '{plugin_name}'.")

def post_process_add_floor(root):
	"""Add a floor plane."""
	is_floor_exists = root.find(".//geom[@name='floor']")
	if is_floor_exists is not None:
		print_base("-> Floor plane already exists in the model; skipping addition.")
		return
	asset = root.find("asset")
	if asset is None:
		asset = xml_utils.ensure_node_before_worldbody(root, "asset")
	asset.append(
		ET.Element(
			"texture",
			{
				"name": "floor",
				"type": "2d",
				"builtin": "checker",
				"rgb1": "0.1 0.2 0.3",
				"rgb2": "0.2 0.3 0.4",
				"width": "300",
				"height": "300",
				"mark": "edge",
				"markrgb": "0.2 0.3 0.4",
			},
		)
	)
	asset.append(
		ET.Element(
			"material",
			{
				"name": "floor",
				"texture": "floor",
				"texrepeat": "10 10",
				"texuniform": "true",
			},
		)
	)
	worldbody = root.find("worldbody")
	if worldbody is not None:
		ET.SubElement(
			worldbody,
			"geom",
			{
				"name": "floor",
				"type": "plane",
				"size": "20 20 0.1",
				"material": "floor",
			},
		)

	print_base("-> Added a floor plane to the model.")

def post_process_add_light(root):
	"""Add a default light."""
	worldbody = root.find("worldbody")
	if worldbody is not None:
		ET.SubElement(
			worldbody,
			"light",
			{"diffuse": ".8 .8 .8", "pos": "0 0 5", "dir": "0 0 -1"},
		)
		print_base("-> Added a default light to the model.")

def post_process_add_clock_publisher_plugin(root):
	"""Add clock publisher plugin."""
	extension_node = xml_utils.ensure_extension_node(root)
	ET.SubElement(
		extension_node,
		"plugin",
		{
			"plugin": "MujocoRosUtils::ClockPublisher",
		},
	)

	worldbody = root.find("worldbody")
	if worldbody is None:
		print_warning("No <worldbody> found in the model. Cannot add clock publisher plugin.")
		return
	clock_publisher_node = ET.SubElement(
		worldbody,
		"plugin",
		{
			"plugin": "MujocoRosUtils::ClockPublisher",
		},
	)
	ET.SubElement(
		clock_publisher_node, "config", {"key": "topic_name", "value": "/clock"}
	)
	ET.SubElement(
		clock_publisher_node, "config", {"key": "publish_rate", "value": "100"}
	)
	ET.SubElement(
		clock_publisher_node, "config", {"key": "use_sim_time", "value": "true"}
	)
	print_base("-> Added 'MujocoRosUtils::ClockPublisher' plugin to the model.")

def post_process_add_ros2_control_plugin(root, default_ros2_control_instance, config_file=None):
	"""Add Ros2Control plugin."""
	extension_node = xml_utils.ensure_extension_node(root)
	plugin_extension_node = ET.SubElement(
		extension_node,
		"plugin",
		{"plugin": "MujocoRosUtils::Ros2Control"},
	)
	instance_node = ET.SubElement(
		plugin_extension_node, "instance", {"name": default_ros2_control_instance}
	)

	if config_file:
		ET.SubElement(
			instance_node,
			"config",
			{"key": "config_file", "value": config_file},
		)
	else:
		print_warning("Adding ROS2 control WITHOUT a config file! Ros2Control plugin will use default config file and it may not work as expected.")

	worldbody = root.find("worldbody")
	if worldbody is None:
		print_warning("No <worldbody> found in the model. Cannot add Ros2Control plugin.")
		return

	print_base("-> Added 'MujocoRosUtils::Ros2Control' plugin to the model.")

def post_process_group_ros_utils_plugins(root):
	"""Group all MujocoRosUtils plugins together under <extension> with comment markers."""
	extension_node = root.find("extension")
	if extension_node is None:
		return
	# Collect existing MujocoRosUtils plugins
	plugins = []
	for p in list(extension_node.findall("plugin")):
		pname = p.get("plugin") or ""
		if pname.startswith("MujocoRosUtils::"):
			plugins.append(p)
	if not plugins:
		return
	# Remove them from current positions
	for p in plugins:
		try:
			extension_node.remove(p)
		except ValueError:
			pass
	# Append grouped with comments
	for p in plugins:
		extension_node.append(p)


def post_process_make_base_floating(root, height_above_ground=0.0):
	"""Add free joint to root body."""
	worldbody = root.find("worldbody")
	if worldbody is not None:
		base_body = worldbody.find("body")
		if base_body is not None:
			if base_body.find('joint[@type="free"]') is None:
				ET.SubElement(base_body, "joint", {"name": "root", "type": "free"})
				print_base(
					f"-> Made the base link '{base_body.get('name')}' floating with a free joint."
				)
		
			if base_body.get("pos") is None:
				base_body.set("pos", f"0 0 {height_above_ground}")

def post_process_add_gravity_compensation(root):
	"""Set gravcomp=1 for bodies."""
	worldbody = root.find("worldbody")
	if worldbody is not None:
		bodies = worldbody.findall(".//body")
		for body in bodies:
			body.set("gravcomp", "1")
		print_base(f"-> Enabled gravity compensation for {len(bodies)} bodies.")

def post_process_set_joint_armature(root, armature_value):
	"""Set 'armature' for all joints."""
	worldbody = root.find("worldbody")
	if worldbody is not None:
		joints = worldbody.findall(".//joint")
		for joint in joints:
			joint.set("armature", str(armature_value))
		print_base(f"-> Set armature to '{armature_value}' for {len(joints)} joints.")

def post_process_set_simulation_options(root, solver=None, integrator=None):
	"""Set <option> solver/integrator."""
	if not solver and not integrator:
		return
	option_node = root.find("option")
	if option_node is None:
		option_node = xml_utils.ensure_child_after(root, "option", "compiler")
	options_set = []
	if solver:
		option_node.set("solver", solver)
		options_set.append(f"solver='{solver}'")
	if integrator:
		option_node.set("integrator", integrator)
		options_set.append(f"integrator='{integrator}'")
	if options_set:
		print_base(f"-> Set simulation options: {', '.join(options_set)}")


def post_process_add_actuators(root, default_ros2_control_instance, mimic_joints=None, add_ros_plugins=False, default_actuator_gains=[500.0, 1.0], ros2c_joint_map=None, force_actuator_tags=True):
	"""Add actuators per ros2_control interfaces.
	- Multiple interfaces on one joint are supported.
	- If a joint has both 'position' and 'velocity', create both actuators.
	- Actuator naming:
	  - single interface: <joint>
	  - multiple interfaces: <joint>_position and/or <joint>_velocity
	"""
	worldbody = root.find("worldbody")
	if worldbody is None:
		print_warning("No <worldbody> found in the model. Cannot add actuators.")
		return

	# Require ros2_control joints map
	if not ros2c_joint_map:
		print_base("-> No ros2_control joints found; skipping actuator generation.")
		return
	all_joints = worldbody.findall(".//joint")
	actuatable_joints = [j for j in all_joints if j.get("type") != "free"]

	# Filter to ros2_control-listed joints only
	names_set = set(ros2c_joint_map.keys())
	actuatable_joints = [j for j in actuatable_joints if j.get("name") in names_set]

	if not actuatable_joints:
		print_base("-> No actuatable joints found to create actuators for.")
		return

	actuator_node = root.find("actuator")
	if actuator_node is None:
		actuator_node = ET.Element("actuator")
		root_children = list(root)
		try:
			worldbody_index = root_children.index(worldbody)
			root.insert(worldbody_index + 1, actuator_node)
		except ValueError:
			root.append(actuator_node)

	if add_ros_plugins:
		extension_node = xml_utils.ensure_extension_node(root)
		plugin_defined = any(
			p.get("plugin") == "MujocoRosUtils::ActuatorCommand"
			for p in extension_node.findall("plugin")
		)
		if not plugin_defined:
			actuator_command_plugin = ET.SubElement(
				extension_node, "plugin", {"plugin": "MujocoRosUtils::ActuatorCommand"}
			)
			actuator_command_plugin.append(
				ET.Element("instance", {"name": default_ros2_control_instance})
			)
			print_base("-> Added 'MujocoRosUtils::ActuatorCommand' extension plugin.")

	joint_names = []
	plugin_joint_names = []
	for joint in actuatable_joints:
		joint_name = joint.get("name")
		if not joint_name:
			continue

		is_mimic = mimic_joints and joint_name in mimic_joints

		# Determine which actuator types to create from ros2_control interfaces
		ifaces = set(ros2c_joint_map.get(joint_name, set()))
		tags_to_add = []
		if "position" in ifaces:
			tags_to_add.append("position")
		if "velocity" in ifaces:
			tags_to_add.append("velocity")
		if not tags_to_add:
			continue

		# Use unique actuator names if multiple interfaces per joint
		use_suffix = len(tags_to_add) > 1 or force_actuator_tags

		for tag in tags_to_add:
			act_name = f"{joint_name}_{tag}" if use_suffix else joint_name
			actuator_attrs = {
				"name": act_name,
				"joint": joint_name,
			}
			# Gains: kp for position, kv for velocity
			if tag == "position":
				actuator_attrs["kp"] = str(default_actuator_gains[0])
			elif tag == "velocity":
				actuator_attrs["kv"] = str(default_actuator_gains[1])

			# ctrlrange from joint range
			if "range" in joint.attrib:
				actuator_attrs["ctrlrange"] = joint.get("range")

			# forcerange from joint actuatorfrcrange
			if "actuatorfrcrange" in joint.attrib:
				actuator_attrs["forcelimited"] = "true"
				actuator_attrs["forcerange"] = joint.get("actuatorfrcrange")

			ET.SubElement(actuator_node, tag, actuator_attrs)
			print_base(f"-> Added '{tag}' actuator for joint: {joint_name}")

		joint_names.append(joint_name)

		if add_ros_plugins and not is_mimic:
			ET.SubElement(
				actuator_node,
				"plugin",
				{
					"plugin": "MujocoRosUtils::ActuatorCommand",
					"joint": joint_name,
					"instance": default_ros2_control_instance,
				},
			)
			plugin_joint_names.append(joint_name)
			print_base(f"-> Added ROS plugin actuators for joint: {joint_name}")

	if joint_names:
		print_base(f"-> Added actuators for joints: {', '.join(joint_names)}")
	if plugin_joint_names:
		print_base(f"-> Added ROS plugin actuators for joints: {', '.join(plugin_joint_names)}")

def post_process_add_mimic_plugins(root, mimic_joints, default_actuator_gains):
	"""Add MimicJoint plugins and ensure a position actuator exists for each mimic joint."""
	if not mimic_joints:
		return

	extension_node = xml_utils.ensure_extension_node(root)
	mimic_plugin_defined = any(
		p.get("plugin") == "MujocoRosUtils::MimicJoint"
		for p in extension_node.findall("plugin")
	)
	if not mimic_plugin_defined:
		ET.SubElement(
			extension_node, "plugin", {"plugin": "MujocoRosUtils::MimicJoint"}
		)
		print_base("-> Added 'MujocoRosUtils::MimicJoint' extension plugin.")

	# Ensure <actuator> block exists (mimic requires a controllable actuator on the follower joint)
	actuator_node = root.find("actuator")
	if actuator_node is None:
		actuator_node = ET.Element("actuator")
		worldbody = root.find("worldbody")
		if worldbody is not None:
			root_children = list(root)
			try:
				widx = root_children.index(worldbody)
				root.insert(widx + 1, actuator_node)
			except ValueError:
				root.append(actuator_node)
		else:
			root.append(actuator_node)

	# Build a set of existing actuator names for uniqueness checks
	existing_actuator_names = set(a.get("name", "") for a in actuator_node)

	mimic_plugin_joint_names = []
	created_position_actuators = []

	for joint_name, mimic_info in mimic_joints.items():
		# Ensure a <position> actuator exists for the follower joint
		has_position = any(
			a.tag == "position" and a.get("joint") == joint_name
			for a in actuator_node
		)
		if not has_position:
			# Choose a unique actuator name
			candidate = joint_name
			if candidate in existing_actuator_names:
				candidate = f"{joint_name}_position"
			# Derive attributes similar to add_actuators()
			attr = {
				"name": candidate,
				"joint": joint_name,
				"kp": str(default_actuator_gains[0]),
			}
			# Copy ctrlrange/forcerange from the joint definition if available
			jnode = root.find(f".//joint[@name='{joint_name}']")
			if jnode is not None:
				if "range" in jnode.attrib:
					attr["ctrlrange"] = jnode.get("range")
				if "actuatorfrcrange" in jnode.attrib:
					attr["forcelimited"] = "true"
					attr["forcerange"] = jnode.get("actuatorfrcrange")
			ET.SubElement(actuator_node, "position", attr)
			existing_actuator_names.add(candidate)
			created_position_actuators.append(joint_name)
			print_base(f"-> Added 'position' actuator for mimic joint: '{joint_name}'.")

		# Add the MimicJoint plugin entry under <actuator>
		plugin_node = ET.SubElement(
			actuator_node,
			"plugin",
			{
				"plugin": "MujocoRosUtils::MimicJoint",
				"joint": joint_name,
			},
		)
		ET.SubElement(
			plugin_node,
			"config",
			{"key": "mimic_joint", "value": mimic_info["joint"]},
		)
		ET.SubElement(
			plugin_node,
			"config",
			{"key": "gear", "value": mimic_info["multiplier"]},
		)
		if float(mimic_info.get("offset", 0.0)) != 0.0:
			ET.SubElement(
				plugin_node,
				"config",
				{"key": "offset", "value": mimic_info["offset"]},
			)
		mimic_plugin_joint_names.append(joint_name)

	if created_position_actuators:
		print_base(f"-> Created missing position actuators for mimic joints: {', '.join(created_position_actuators)}")
	if mimic_plugin_joint_names:
		print_base(f"-> Added ROS mimic joint plugins for joints: {', '.join(mimic_plugin_joint_names)}")

def post_process_add_materials(root, material_info, mesh_dir="assets/"):
	"""Add material definitions to MJCF based on extracted mesh material information.
	
	Only processes visual meshes - collision meshes don't need material information.
	
	Args:
		root: MJCF root element
		material_info: Dictionary of material information organized by link:
			{
				'link_name': {
					'visual': [
						{'file': 'mesh.stl', 'material': 'mat_name', 'rgba': [r,g,b,a]},
						...
					]
				}
			}
		mesh_dir: Directory where meshes are stored (for matching mesh names to geoms)
	"""
	if not material_info:
		print_base("-> No material information to add to MJCF.")
		return
	
	# Find or create <asset> section
	asset_node = root.find("asset")
	if asset_node is None:
		asset_node = ET.Element("asset")
		# Insert after <compiler> if exists, otherwise at the beginning
		compiler_idx = list(root).index(root.find("compiler")) + 1 if root.find("compiler") is not None else 0
		root.insert(compiler_idx, asset_node)
	
	# Track unique materials to avoid duplicates
	existing_materials = {mat.get("name") for mat in asset_node.findall("material")}
	added_materials = set()
	material_count = 0
	geom_material_assignments = 0
	
	# Process each link's visual material info only
	for link_name, link_materials in material_info.items():
		# Only process visual meshes
		if 'visual' not in link_materials:
			continue
		
		for mesh_data in link_materials['visual']:
			rgba = mesh_data.get('rgba')
			material_name = mesh_data.get('material')
			mesh_file = mesh_data.get('file')
			
			# Skip meshes without color information
			if not rgba or not mesh_file:
				continue
			
			# Generate a unique material name
			# Use material name from DAE if available, otherwise derive from mesh file
			if material_name:
				mat_name = f"mat_{material_name}"
			else:
				mesh_base = os.path.splitext(mesh_file)[0]
				mat_name = f"mat_{mesh_base}"
			
			# Ensure unique name
			original_mat_name = mat_name
			counter = 1
			while mat_name in existing_materials or mat_name in added_materials:
				mat_name = f"{original_mat_name}_{counter}"
				counter += 1
			
			# Add material definition to <asset>
			material_attrib = {
				"name": mat_name,
				"rgba": f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"
			}
			ET.SubElement(asset_node, "material", material_attrib)
			added_materials.add(mat_name)
			material_count += 1
			
			# Now find the corresponding visual geom(s) and assign the material
			# MuJoCo stores mesh references in <mesh> elements in <asset>, and geoms reference them by name
			# The mesh name is typically the filename without extension
			mesh_ref_base = os.path.splitext(mesh_file)[0]
			
			# Find all geoms that reference this mesh
			# We need to match geoms in the worldbody/body hierarchy
			for geom in root.findall(".//geom[@mesh]"):
				geom_mesh = geom.get("mesh")
				
				# Try to match by checking if geom's mesh attribute matches our mesh name
				# This is somewhat heuristic since MuJoCo might modify mesh names during compilation
				if geom_mesh and (mesh_ref_base in geom_mesh or geom_mesh in mesh_ref_base):
					# Assign material to this geom
					geom.set("material", mat_name)
					geom_material_assignments += 1
					print_debug(f"-> Assigned material '{mat_name}' (RGBA: {rgba}) to geom with mesh '{geom_mesh}' in link '{link_name}'")
	
	if material_count > 0:
		print_confirm(f"-> Added {material_count} material definitions from DAE files")
		if geom_material_assignments > 0:
			print_confirm(f"-> Assigned materials to {geom_material_assignments} visual geoms")
		else:
			print_warning("-> Warning: Materials were created but no geoms were assigned (mesh name matching may have failed)")
	else:
		print_base("-> No materials added (no valid RGBA data found in visual meshes)")


