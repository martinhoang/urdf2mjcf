import os
import shutil
import subprocess
import tempfile
import json
import xml.etree.ElementTree as ET
import copy
import mujoco

from . import urdf_preprocess, mesh_ops, mjcf_postprocess
from _utils import (
    print_base,
    print_info,
    print_warning,
    print_error,
    print_confirm,
)

_DEFAULT_ROS2_CONTROL_INSTANCE = "ros2_control"
_DEFAULT_MESH_DIR = "assets/"

class URDFToMJCFConverter:
    """
    Main converter class that orchestrates the URDF to MJCF conversion process.
    """

    def __init__(self, args):
        self.args = args
        self.default_ros2_control_instance = _DEFAULT_ROS2_CONTROL_INSTANCE
        self.default_mesh_dir = _DEFAULT_MESH_DIR

    def convert(self):
        """Main conversion pipeline."""
        args = copy.deepcopy(self.args)
        
        tracking_progress = []

        # Resolve input and output paths
        input_path = urdf_preprocess.resolve_path(args.input)
        if not input_path or not os.path.exists(input_path):
            print_error(f"Input file not found at '{args.input}' (resolved to '{input_path}')")
            return
        else:
            print_info(f"Input file: {input_path}")

        if args.output:
            output_dir = urdf_preprocess.resolve_path(args.output)
            print_info(f"Using specified output directory: '{output_dir}'")
        else:
            print_warning("No output directory specified. Using input directory.")
            output_dir = os.path.dirname(input_path)

        base_name = os.path.basename(input_path)
        file_name_without_ext = os.path.splitext(os.path.splitext(base_name)[0])[0]

        output_dir = os.path.join(output_dir, file_name_without_ext)
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, f"{file_name_without_ext}.xml")
    
        # temp_urdf_path: raw xacro-expanded (or original) URDF
        temp_urdf_path = os.path.join(output_dir, f"{file_name_without_ext}.temp.urdf")
        # preprocessed_urdf_path: URDF after our preprocess_urdf() mutations
        preprocessed_urdf_path = os.path.join(output_dir, f"{file_name_without_ext}.preprocessed.urdf")
    
        print_base(f"Output directory set to: '{output_dir}'")
        print_base(f"Output file will be saved to: '{output_path}'")

        urdf_to_process = input_path

        # XACRO pre-processing
        if input_path.endswith(".xacro"):
            print_warning(f"Input file '{base_name}' is a xacro file.")
            print_base("-> Attempting to convert to URDF using the 'xacro' command...")

            if not shutil.which("xacro"):
                print_error("The 'xacro' command is not in your PATH. Please install ROS 2 or the 'xacro' package.")
                return

            try:
                xacro_command = ["xacro", input_path]
                if args.xacro_args:
                    print_base(f"-> Passing arguments to xacro: {' '.join(args.xacro_args)}")
                    xacro_command.extend(args.xacro_args)
                xacro_command.extend(["-o", temp_urdf_path])

                subprocess.run(xacro_command, check=True)
                urdf_to_process = temp_urdf_path
                print_info(f"-> Successfully converted xacro to temporary URDF: {urdf_to_process}")
            except (subprocess.CalledProcessError, FileNotFoundError) as e:
                print_error(f"Failed to run xacro processor.\n{e}")
                return

        root = None
        custom_mujoco_elements = []
        urdf_plugins = []
        try:
            tracking_progress.append({'name': 'Pre-process URDF'})
            (
                modified_urdf_tree,
                absolute_mesh_paths,
                mimic_joints,
                custom_mujoco_elements,
                urdf_plugins,
                ros2c_joint_map,
            ) = urdf_preprocess.preprocess_urdf(urdf_to_process, args.compiler_options, self.default_mesh_dir, args.separate_dae_meshes, args.append_mesh_type, args.zero_inertial_rpy)

            tracking_progress.append({'name': 'Convert & Copy Meshes'})
            if not args.no_copy_meshes:
                # Validate that only one simplification method is specified
                if args.simplify_reduction < 1.0 and args.simplify_target_faces is not None:
                    raise ValueError("Cannot specify both --simplify-reduction and --simplify-target-faces. Choose one.")
                
                # Prepare mesh tool parameters
                calculate_inertia_params = None
                if args.calculate_inertia and args.calculate_inertia_mass:
                    calculate_inertia_params = {
                        'mass': args.calculate_inertia_mass,
                        'scale': 0.001  # Default mm to m conversion
                    }
                
                # Simplification is enabled when simplify_reduction < 1.0 or target_faces is specified
                simplify_params = None
                simplify_meshes = args.simplify_reduction < 1.0 or args.simplify_target_faces is not None
                if simplify_meshes:
                    if args.simplify_target_faces is not None:
                        # Use target faces method
                        simplify_params = {
                            'target_faces': args.simplify_target_faces
                        }
                    else:
                        # Use reduction ratio method
                        simplify_params = {
                            'reduction': args.simplify_reduction
                        }
                
                # Copy mesh files (materials already added to URDF in preprocessing)
                mesh_ops.copy_mesh_files(
                    absolute_mesh_paths,
                    output_dir,
                    mesh_dir=self.default_mesh_dir,
                    mesh_reduction=args.mesh_reduction if args.mesh_reduction < 1.0 and args.mesh_reduction > 0.0 else 0.0,
                    calculate_inertia_params=calculate_inertia_params,
                    generate_collision=args.generate_collision_meshes,
                    simplify_meshes=simplify_meshes,
                    simplify_params=simplify_params,
                )

            ET.indent(modified_urdf_tree, space="\t")
            # Write the modified/preprocessed URDF to a distinct file
            modified_urdf_tree.write(preprocessed_urdf_path, encoding="unicode")

            if args.save_preprocessed:
                print_info(f"-> Saved pre-processed URDF to '{preprocessed_urdf_path}'")

            # Validate mesh face counts before MuJoCo import
            if args.validate_mesh_faces:
                tracking_progress.append({'name': 'Validate Mesh Face Counts'})
                print_info("Validating mesh files before MuJoCo import...")
                mesh_dir_full_path = os.path.join(output_dir, self.default_mesh_dir)
                
                if os.path.exists(mesh_dir_full_path):
                    problematic = mesh_ops.validate_all_meshes_in_directory(
                        mesh_dir_full_path,
                        max_faces=args.max_faces_limit
                    )
                    
                    if problematic:
                        print_warning(f"Found {len(problematic)} mesh(es) exceeding MuJoCo's {args.max_faces_limit:,} face limit:")
                        for mesh_path, face_count, suggested_target in problematic:
                            print_warning(f"  ✗ {os.path.basename(mesh_path)}: {face_count:,} faces → needs reduction to ~{suggested_target:,}")
                        
                        print_info("Attempting to automatically fix oversized meshes...")
                        fixed, failed = mesh_ops.fix_oversized_meshes(
                            mesh_dir_full_path,
                            max_faces=args.max_faces_limit,
                            target_reduction_ratio=0.5,  # Reduce to 50% of limit for safety margin
                            backup=True
                        )
                        
                        if fixed > 0:
                            print_confirm(f"✓ Successfully simplified {fixed} mesh(es)")
                        if failed > 0:
                            print_error(f"✗ Failed to simplify {failed} mesh(es) - MuJoCo import may fail")
                            if not args.traceback:
                                print_info("Use --traceback flag to see detailed error information")
                    else:
                        print_confirm("✓ All mesh files are within MuJoCo's face count limits")
                else:
                    print_info(f"No mesh directory found at '{mesh_dir_full_path}', skipping validation")

            print_info(f"Loading pre-processed URDF to Mujoco: {preprocessed_urdf_path}")
            tracking_progress.append({'name': 'Import URDF to Mujoco'})
            model = mujoco.MjModel.from_xml_path(preprocessed_urdf_path)

            with tempfile.NamedTemporaryFile(mode="r", delete=True, suffix=".xml", encoding="utf-8") as f:
                mujoco.mj_saveLastXML(f.name, model)
                xml_string = f.read()

            assert xml_string, "Failed to read back the generated MJCF XML"
            root = ET.fromstring(xml_string)

        except Exception as e:
            progress_str = ""
            if len(tracking_progress) > 0:
                progress_str = tracking_progress[-1]['name']
            print_error(f"Error during \"{progress_str}\"\n{e}", exc_info=True)
            return
        finally:
            # Clean up intermediates unless explicitly asked to keep preprocessed
            if not args.save_preprocessed:
                if os.path.exists(preprocessed_urdf_path):
                    os.remove(preprocessed_urdf_path)
                if os.path.exists(temp_urdf_path):
                    os.remove(temp_urdf_path)

        if root is None:
            print_error("MJCF root element not created. Aborting.")
            return
        
        print_confirm("Loaded URDF to MJCF successfully. Applying post-processing MJCF...")

        # Apply damping multiplier if specified
        if args.damping_multiplier != 1.0:
            mjcf_postprocess.post_process_damping_multiplier(root, args.damping_multiplier)

        # Inject custom plugins and mujoco elements from URDF
        for plugin_node in urdf_plugins:
            mjcf_postprocess.post_process_transform_and_add_custom_plugin(root, plugin_node)
        mjcf_postprocess.post_process_compiler_options(root)
        mjcf_postprocess.post_process_add_light(root)
        
        # Note: Materials from DAE files are already in the preprocessed URDF
        # and will be preserved by MuJoCo during import
        
        if args.add_clock_publisher:
            mjcf_postprocess.post_process_add_clock_publisher_plugin(root)
        if args.add_ros2_control:
            mjcf_postprocess.post_process_add_ros2_control_plugin(root, config_file=args.ros2_control_config)
        # Group MujocoRosUtils extension plugins together
        mjcf_postprocess.post_process_group_ros_utils_plugins(root)

        if args.floating_base:
            floating_base_args = [root]
            if args.height_above_floor > 0:
                floating_base_args.append(args.height_above_floor)
            mjcf_postprocess.post_process_make_base_floating(*floating_base_args)
        if args.add_floor:
            mjcf_postprocess.post_process_add_floor(root)
        if not args.no_actuators:
            mjcf_postprocess.post_process_add_actuators(
                root, 
                mimic_joints, 
                args.add_ros_plugins, 
                default_actuator_gains=args.default_actuator_gains, 
                ros2c_joint_map=ros2c_joint_map,
                force_actuator_tags=args.force_actuator_tags
            )
            if args.add_mimic_joints and mimic_joints:
                mjcf_postprocess.post_process_add_mimic_plugins(root, mimic_joints, args.default_actuator_gains)
            # Regroup MujocoRosUtils plugins after actuator/mimic insertions
            mjcf_postprocess.post_process_group_ros_utils_plugins(root)

        if args.gravity_compensation:
            mjcf_postprocess.post_process_add_gravity_compensation(root)
        if args.armature is not None:
            mjcf_postprocess.post_process_set_joint_armature(root, args.armature)
        if args.solver or args.integrator:
            mjcf_postprocess.post_process_set_simulation_options(root, solver=args.solver, integrator=args.integrator)

        # Final regroup to ensure MujocoRosUtils plugins are contiguous
        mjcf_postprocess.post_process_inject_custom_mujoco_elements(root, custom_mujoco_elements)
        mjcf_postprocess.post_process_group_ros_utils_plugins(root)
        

        try:
            tree = ET.ElementTree(root)
            ET.indent(tree, space="\t")
            tree.write(output_path, encoding="utf-8", xml_declaration=True)
            print_info(f"{'=' * 100}")
            print_info("Successfully converted URDF to MJCF.")
            print_base(f"Output saved to: {output_path}")
            print_warning("REMEMBER TO BUILD THE PACKAGE SO THE ASSET FILES ARE COPIED OR LINKED!")
            abs_path = os.path.abspath(output_path)
            # Clean path printing: use absolute path directly (no redundant cwd prefix)
            print_confirm(f"\nRun this to simulate with installed mujoco:\n\nsimulate {abs_path}\n\n")
        except Exception as e:
            print_error(f"Failed to save the final MJCF file to '{output_path}'")
            print_error(f"Error details: {e}")

        try:
            self.save_config(output_dir)
        except Exception as e:
            print_warning(f"Could not save arguments to config file. Error: {e}")
            pass

    def save_config(self, output_dir):
        """Save arguments to a JSON file, excluding specified keys."""
        args_to_save = vars(self.args).copy()
        
        # Exclude non-serializable or irrelevant args before saving
        keys_to_exclude = {
            "config_file"
        }
        for key in keys_to_exclude:
            args_to_save.pop(key, None)

        config_path = os.path.join(output_dir, "config.json")
        try:
            with open(config_path, "w") as f:
                json.dump(args_to_save, f, indent=4)
            print_base(f"-> Arguments saved to '{config_path}'")
        except Exception as e:
            print_warning(f"Could not save arguments to '{config_path}'. Error: {e}")