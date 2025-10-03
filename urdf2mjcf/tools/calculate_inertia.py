#!/usr/bin/env python3

import argparse
import traceback
import trimesh

def calculate_inertia(mesh_path, mass, translation=None, orientation=None, scale=0.001):
	"""
	Calculates the moment of inertia for a given STL file and mass.
	
	Args:
		mesh_path (str): Path to the STL file.
		mass (float): Mass of the object in kilograms.
		translation (list, optional): The translation [x, y, z] of the new origin.
		orientation (list, optional): The orientation [roll, pitch, yaw] in degrees of the new origin.

	Returns:
		A dictionary with inertia data if successful, None otherwise.
	"""
	try:
		# Load the mesh from the STL file. Use force='mesh' to ensure we get a
		# single Trimesh object, not a Scene.
		mesh = trimesh.load_mesh(mesh_path, force='mesh')
  
		mesh.apply_scale(scale)

		# If the mesh is not watertight, it's not a closed volume,
		# so we can't calculate volume or inertia.
		if not mesh.is_watertight:
			print(f"Warning: Mesh '{mesh_path}' is not watertight. Trying to fix it.")
			if not mesh.fill_holes():
				print(f"Error: Could not make the mesh '{mesh_path}' watertight. Cannot calculate inertia.")
				return None

		# To set the mass, we must first calculate the density based on the
		# mesh's volume and the desired mass.
		if mesh.volume == 0:
			print(f"Error: Mesh '{mesh_path}' has zero volume. Cannot calculate inertia.")
			return None
		
		density = mass / mesh.volume
		mesh.density = density

		# If no new frame is specified, calculate inertia at the mesh's center of mass.
		if translation is None and orientation is None:
			inertia_tensor = mesh.moment_inertia
			center_of_mass = mesh.center_mass
		else:
			# A new frame is specified. Create the transformation matrix for this frame.
			# This matrix defines the pose of the new frame in the original coordinates.
			translation = translation or [0, 0, 0]
			if orientation:
				# Convert degrees to radians for euler_matrix
				orient_rad = [trimesh.transformations.math.radians(d) for d in orientation]
				rot_matrix = trimesh.transformations.euler_matrix(orient_rad[0], orient_rad[1], orient_rad[2], 'sxyz')
			else:
				rot_matrix = trimesh.transformations.identity_matrix()
			
			trans_matrix = trimesh.transformations.translation_matrix(translation)
			transform = trans_matrix @ rot_matrix

			# Calculate the moment of inertia with respect to the new frame.
			inertia_tensor = mesh.moment_inertia_frame(transform)

			# The center of mass must also be calculated relative to the new frame.
			# We do this by applying the inverse of the transform to the original center of mass.
			inverse_transform = trimesh.transformations.inverse_matrix(transform)
			center_of_mass = trimesh.transformations.transform_points([mesh.center_mass], inverse_transform)[0]
		
		# For URDF, we need the diagonal elements (ixx, iyy, izz)
		# and the off-diagonal elements (ixy, ixz, iyz).
		ixx = inertia_tensor[0, 0]
		iyy = inertia_tensor[1, 1]
		izz = inertia_tensor[2, 2]
		ixy = inertia_tensor[0, 1]
		ixz = inertia_tensor[0, 2]
		iyz = inertia_tensor[1, 2]

		# Get the center of mass. If a transform is provided, this will be the
		# center of mass relative to the new origin.
		if 'transform' in locals() and transform is not None:
			center_of_mass = trimesh.transformations.transform_points([mesh.center_mass], transform)[0]
		else:
			center_of_mass = mesh.center_mass

		return {
			"mass": mass,
			"center_of_mass": center_of_mass,
			"ixx": ixx,
			"iyy": iyy,
			"izz": izz,
			"ixy": ixy,
			"ixz": ixz,
			"iyz": iyz,
		}

	except Exception as e:
		print(f"An error occurred: {e}{traceback.format_exc()}")
		return None

def print_urdf_inertia(inertia_data):
	"""Prints the inertia data in URDF format."""
	if not inertia_data:
		return

	print("Inertia calculation results:")
	print(f"  Mass: {inertia_data['mass']}")
	print(f"  Center of Mass: {inertia_data['center_of_mass']}")
	print(f"  Ixx: {inertia_data['ixx']}")
	print(f"  Iyy: {inertia_data['iyy']}")
	print(f"  Izz: {inertia_data['izz']}")
	print(f"  Ixy: {inertia_data['ixy']}")
	print(f"  Ixz: {inertia_data['ixz']}")
	print(f"  Iyz: {inertia_data['iyz']}")

	print("\n--- URDF Inertial Tag ---")
	print('<inertial>')
	print(f'  <mass value="{inertia_data["mass"]}" />')
	print(f'  <origin xyz="{inertia_data["center_of_mass"][0]} {inertia_data["center_of_mass"][1]} {inertia_data["center_of_mass"][2]}" rpy="0 0 0" />')
	print(f'  <inertia ixx="{inertia_data["ixx"]}" ixy="{inertia_data["ixy"]}" ixz="{inertia_data["ixz"]}" iyy="{inertia_data["iyy"]}" iyz="{inertia_data["iyz"]}" izz="{inertia_data["izz"]}" />')
	print('</inertial>')
	print("-------------------------\n")


if __name__ == "__main__":
	try: 
		parser = argparse.ArgumentParser(
			description="Calculate the moment of inertia of an STL file given its mass.",
			epilog="This script requires the 'trimesh' library. Install it using: pip install trimesh[easy]"
		)
		parser.add_argument(
			type=str,
			dest="mesh",
			help="Path to the input STL mesh file."
		)
		parser.add_argument(
			type=float,
			dest="mass",
			help="Mass of the object in kilograms."
		)
		parser.add_argument(
			"-t",
			"--translation",
			type=float,
			nargs=3,
			default=None,
			help="The translation [x y z] of the new origin."
		)
		parser.add_argument(
			"-o",
			"--orientation",
			type=float,
			nargs=3,
			default=None,
			help="The orientation [roll pitch yaw] in degrees of the new origin."
		)
		parser.add_argument(
			"-s",
			"--scale",
			type=float,
			default=1.0,
			help="Scale factor to apply to the mesh (default: 1 m)."
		)

		args = parser.parse_args()

		inertia_data = calculate_inertia(args.mesh, args.mass, args.translation, args.orientation)

		if inertia_data:
			print(f"Successfully calculated inertia for '{args.mesh}'.")
			print_urdf_inertia(inertia_data)

	except Exception as e:
   
		print(f"An unexpected error occurred: {e}{traceback.format_exc()}")