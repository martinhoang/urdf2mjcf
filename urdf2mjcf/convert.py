#!/usr/bin/env python3

import os
import subprocess
import sys
import venv
from _utils import print_base, print_error, print_info


def setup_mujoco_venv():
	"""
	Checks for the 'mujoco' package. If not found, creates a local venv,
	installs mujoco, and re-launches the script within that venv.
	"""
	# Path to the virtual environment directory, located alongside the script
	venv_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".venv")

	# Check if we are running inside our target venv by comparing sys.prefix
	is_in_venv = os.path.abspath(sys.prefix) == os.path.abspath(venv_dir)

	python_exe_exists = (
		os.path.isfile(os.path.join(venv_dir, "bin", "python"))
		if not is_in_venv
		else False
	)

	try:
		import mujoco
		return  # mujoco is already available
	except ImportError:
		# If we are in the venv but the import fails, something is wrong.
		if is_in_venv and not python_exe_exists:
			print_error(
				f"Running in the virtual environment at '{venv_dir}' but "
				f"the 'mujoco' package is still not found.\n"
				f"Try deleting the '.venv' directory and running again."
			)
			sys.exit(1)
		elif python_exe_exists:
			print_info(
				f"Virtual environment found at '{venv_dir}'. Re-launching script within it."
			)
			os.execv(
				os.path.join(venv_dir, "bin", "python"),
				[os.path.join(venv_dir, "bin", "python")] + sys.argv,
			)

		# Not in venv and mujoco not found globally. Let's set it up.
		print_base("--- MuJoCo Dependency Setup ---")
		print_base(
			"Python package 'mujoco' not found. Setting up a local virtual environment."
		)

		# 1. Create venv if it doesn't exist
		if not os.path.isdir(venv_dir):
			print_base(f"Creating virtual environment in '{venv_dir}'...")
			try:
				venv.create(venv_dir, with_pip=True)
			except Exception as e:
				print_error(f"Could not create virtual environment.\n{e}")
				sys.exit(1)

		# 2. Get path to python/pip in venv
		if sys.platform == "win32":
			pip_exe = os.path.join(venv_dir, "Scripts", "pip.exe")
			python_exe = os.path.join(venv_dir, "Scripts", "python.exe")
		else:
			pip_exe = os.path.join(venv_dir, "bin", "pip")
			python_exe = os.path.join(venv_dir, "bin", "python")

		# 3. Install mujoco
		print_base("Installing 'mujoco' package...")
		try:
			subprocess.check_call([pip_exe, "install", "mujoco"])
		except subprocess.CalledProcessError as e:
			print_error(f"Failed to install 'mujoco' using pip.\n{e}")
			sys.exit(1)

		# 4. Relaunch script with the venv's python
		print_info("Installation successful. Relaunching script...")
		print_base("---------------------------------\n")
		os.execv(python_exe, [python_exe] + sys.argv)


if __name__ == "__main__":
	# Set up the environment and dependencies automatically
	setup_mujoco_venv()

	# Now that the venv is set up and the script is potentially re-launched,
	# we can safely import and run the main application.
	try:
		from . import cli
	except ImportError:
		print_error("Failed to import the new 'urdf2mjcf'. Make sure it is in the same directory.")
		sys.exit(1)
	
	cli.main()
