import fnmatch
import xml.etree.ElementTree as ET
from _utils import print_base, print_info, print_warning

def ensure_extension_node(root):
	"""Ensure <extension> exists. Insert after <compiler>, else before <worldbody>, else append."""
	extension_node = root.find("extension")
	if extension_node is not None:
		return extension_node
	extension_node = ET.Element("extension")
	compiler_node = root.find("compiler")
	if compiler_node is not None:
		root_children = list(root)
		compiler_index = root_children.index(compiler_node)
		root.insert(compiler_index + 1, extension_node)
		return extension_node
	worldbody_node = root.find("worldbody")
	if worldbody_node is not None:
		root_children = list(root)
		worldbody_index = root_children.index(worldbody_node)
		root.insert(worldbody_index, extension_node)
	else:
		root.append(extension_node)
	return extension_node

def ensure_child_after(root, child_tag, after_tag):
	"""Ensure child_tag node exists and is placed after after_tag if present, else at start."""
	node = root.find(child_tag)
	if node is not None:
		return node
	node = ET.Element(child_tag)
	after_node = root.find(after_tag)
	if after_node is not None:
		root_children = list(root)
		idx = root_children.index(after_node)
		root.insert(idx + 1, node)
	else:
		root.insert(0, node)
	return node

def ensure_node_before_worldbody(root, tag):
	"""Ensure tag exists; place before <worldbody> or append."""
	node = root.find(tag)
	if node is not None:
		return node
	node = ET.Element(tag)
	worldbody_node = root.find("worldbody")
	if worldbody_node is not None:
		root_children = list(root)
		worldbody_index = root_children.index(worldbody_node)
		root.insert(worldbody_index, node)
	else:
		root.append(node)
	return node

def find_matching_elements_with_wildcards(root, elem):
	"""Find matching elements using wildcard pattern matching for attribute values."""
	
	tag = elem.tag
	target_attrs = elem.attrib
	
	# Find all elements with the matching tag throughout the entire tree
	all_candidates = root.findall(f".//{tag}")
	matching_nodes = []
	
	for candidate in all_candidates:
		# Check if all attributes match (including wildcard patterns)
		matches_all = True
		for attr_name, pattern in target_attrs.items():
			candidate_value = candidate.get(attr_name)
			if candidate_value is None:
				# Candidate doesn't have this attribute
				matches_all = False
				break
			
			# Use fnmatch for wildcard matching
			if not fnmatch.fnmatch(candidate_value, pattern):
				matches_all = False
				break
		
		if matches_all:
			matching_nodes.append(candidate)
			# Log the successful match with more detail
			candidate_attrs = ", ".join([f"{k}='{v}'" for k, v in candidate.attrib.items()])
			pattern_attrs = ", ".join([f"{k}='{v}'" for k, v in target_attrs.items()])
			print_base(f"Matched pattern <{tag} {pattern_attrs}> with wildcard found: <{tag} {candidate_attrs}>")
	
	if matching_nodes:
		print_info(f"Found {len(matching_nodes)} matching elements for pattern <{tag} {', '.join([f'{k}={v}' for k, v in target_attrs.items()])}>")
	else:
		pattern_attrs = ", ".join([f"{k}='{v}'" for k, v in target_attrs.items()])
		print_warning(f"No matching elements found for pattern <{tag} {pattern_attrs}>")
	
	return matching_nodes

def find_matching_elements(root, elem):
	"""Find all elements in the XML tree that match the tag and all attributes of the given element.
	Supports wildcard matching with '*' character for attribute values."""
	if not elem.attrib:
		# No attributes to match, return empty list to fall back to default behavior
		return []
	
	tag = elem.tag
	
	# Check if any attribute values contain wildcards
	has_wildcards = any('*' in attr_value for attr_value in elem.attrib.values())
	
	if not has_wildcards:
		# No wildcards, use simple XPath matching
		attr_conditions = []
		for attr_name, attr_value in elem.attrib.items():
			attr_conditions.append(f"@{attr_name}='{attr_value}'")
		
		xpath_query = f".//{tag}[{' and '.join(attr_conditions)}]"
		
		try:
			matching_nodes = root.findall(xpath_query)
			return matching_nodes
		except Exception as e:
			print_warning(f"Error finding matching elements with XPath '{xpath_query}': {e}")
			return []
	else:
		# Has wildcards, need to use manual filtering
		return find_matching_elements_with_wildcards(root, elem)