import argparse
import json
import os
import sys

from .converter import URDFToMJCFConverter
from _utils import set_log_level, print_confirm, print_warning, print_info


class ConfigLoader:
    def __init__(self, parser):
        self.parser = parser

    def load_config(self, config_file, current_args):
        """
        Apply JSON config to args, with CLI args taking precedence.
        The function loads the JSON file and manually updates the namespace,
        preserving CLI arguments that were explicitly provided.
        """
        if not config_file or not os.path.exists(config_file):
            print_warning(f"Config file '{config_file}' not found. Using default arguments and CLI overrides only.")
            return None

        # Load config from JSON file
        with open(config_file, "r") as f:
            config_data = json.load(f)
        
        # Get the current arguments that were explicitly set via CLI
        # We'll preserve these and only update the ones not set
        cli_provided_args = set()
        
        # Check which arguments were provided on command line
        for arg in sys.argv[1:]:
            if arg.startswith('-'):
                # Remove leading dashes and convert to dest format
                clean_arg = arg.lstrip('-').replace('-', '_')
                cli_provided_args.add(clean_arg)
        
        # Update namespace with config values, but don't override CLI args
        for key, value in config_data.items():
            # Convert key format (e.g., 'add-floor' -> 'add_floor')
            dest_key = key.replace('-', '_')
            
            # Only set if not provided via CLI (except for the special case of input)
            if dest_key not in cli_provided_args or dest_key == 'input':
                setattr(current_args, dest_key, value)
        
        print_confirm(f"Loaded configuration from '{config_file}'")
        
        return current_args

def main():
    parser = argparse.ArgumentParser(
        description="Convert a URDF file to a MuJoCo MJCF (XML) file using the <compiler> tag method.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    
    # Add an argument for the config file
    parser.add_argument(
        "-cf",
        "--config-file",
        type=str,
        metavar="PATH",
        help="Path to the JSON configuration file.",
    )

    parser.add_argument(
        "input", 
        type=str, 
        nargs='?',
        metavar="PATH",
        help="Path to the input URDF, xacro, or JSON config file."
    )

    # Organized argument groups
    core_group = parser.add_argument_group("Core Conversion Options")
    physics_group = parser.add_argument_group("Physics & Model Properties")
    ros_group = parser.add_argument_group("ROS Integration")
    advanced_group = parser.add_argument_group("Advanced & Debugging Options")

    # Core Conversion Options
    core_group.add_argument(
        "-af",
        "--add-floor",
        action="store_true",
        help="Add a ground plane to the simulation world.",
    )
    core_group.add_argument(
        "-fb",
        "--floating-base",
        action="store_true",
        help="Add a free joint to the root link to make the base float.",
    )
    core_group.add_argument(
        "-haf",
        "--height-above-floor",
        type=float,
        default=0.0,
        help="Set the height of the robot w.r.t the added floor",
    )
    core_group.add_argument(
        "-na",
        "--no-actuators",
        action="store_true",
        help="Do not add position and plugin actuators for non-fixed joints.",
    )
    core_group.add_argument(
        "-o",
        "--output",
        type=str,
        default="",
        metavar="DIR",
        help="The directory where the output MJCF file and assets will be saved. (default: A new directory in the package's 'output' folder)",
    )

    # Physics & Model Properties
    physics_group.add_argument(
        "-a",
        "--armature",
        type=float,
        metavar="VALUE",
        help="Set a global armature value for all joints.",
    )
    physics_group.add_argument(
        "-djs",
        "--default-actuator-gains",
        type=float,
        nargs=2,
        default=[500.0, 1.0],
        help="Set the default joint stiffness (kp) and damping (kv) for all position actuators.",
    )
    physics_group.add_argument(
        "-dm",
        "--damping-multiplier",
        type=float,
        default=1.0,
        metavar="FACTOR",
        help="Multiply all joint damping values by this factor. (default: 1.0)",
    )
    physics_group.add_argument(
        "-gc",
        "--gravity-compensation",
        action="store_true",
        help="Enable gravity compensation (gravcomp=1) for all bodies in the model.",
    )

    # ROS Integration
    ros_group.add_argument(
        "-arc",
        "--add-ros2-control",
        dest="add_ros2_control",
        action="store_true",
        help="Add the main MujocoRosUtils::Ros2Control plugin. Not added by default.",
    )
    ros_group.add_argument(
        "-arp",
        "--add-ros-plugins",
        action="store_true",
        help="Add MujocoRosUtils plugins for actuator commands, enabling ROS control.",
    )
    ros_group.add_argument(
        "-ncp",
        "--no-clock-publisher",
        dest="add_clock_publisher",
        action="store_false",
        help="Do not add the MujocoRosUtils::ClockPublisher plugin to publish simulation time.",
    )
    ros_group.add_argument(
        "-nmj",
        "--no-mimic-joints",
        dest="add_mimic_joints",
        action="store_false",
        help="Do not add MujocoRosUtils::MimicJoint plugins. Added by default.",
    )
    ros_group.add_argument(
        "-rcc",
        "--ros2-control-config",
        type=str,
        metavar="PATH",
        help="Path to the ros2_control configuration YAML file.",
    )

    # Advanced & Debugging Options
    advanced_group.add_argument(
        "-co",
        "--compiler-options",
        nargs="+",
        metavar="KEY=VALUE",
        help="Override or add compiler attributes, e.g., 'balanceinertia=true' 'discardvisual=false'",
    )
    advanced_group.add_argument(
        "-int",
        "--integrator",
        type=str,
        metavar="INTEGRATOR",
        help="Set the simulation integrator (e.g., 'Euler', 'RK4', 'implicitfast').",
    )
    advanced_group.add_argument(
        "-ncm",
        "--no-copy-meshes",
        action="store_true",
        help="Do not copy referenced mesh files to the output directory.",
    )
    advanced_group.add_argument(
        "-mr",
        "--mesh-reduction",
        default=0.9,
        type=float,
        metavar="RATIO",
        help="Set the mesh reduction ratio, 0.0 for no reduction and 1.0 for full reduction.",
    )
    advanced_group.add_argument(
        "-sp",
        "--save-preprocessed",
        action="store_true",
        help="Save the intermediate, pre-processed URDF file for debugging.",
    )
    advanced_group.add_argument(
        "-s",
        "--solver",
        type=str,
        metavar="SOLVER",
        help="Set the simulation solver (e.g., 'PGS', 'CG', 'Newton').",
    )
    advanced_group.add_argument(
        "-xa",
        "--xacro-args",
        nargs="*",
        metavar="KEY:=VALUE",
        default=["hangup:=true", "hardware_type:=sim_mujoco"],
        help="Arguments to pass to the xacro processor, e.g., 'param1:=true' 'param2:=false'",
    )
    advanced_group.add_argument(
        "-ll",
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Set the logging level for console output.",
    )
    advanced_group.add_argument(
        "-tb",
        "--traceback",
        action="store_true",
        help="Show full traceback on error.",
    )
    # Initial parse to get log level and config file path
    args, _ = parser.parse_known_args()
    
    # Auto-detect JSON config file from input argument
    config_file = args.config_file
    input_file = args.input
    
    if args.input and args.input.endswith('.json'):
        # If input is a JSON file, treat it as config file
        config_file = args.input
        input_file = None  # Will be set from config
        print_info(f"Auto-detected JSON config file: {config_file}")
    
    if config_file:
        config_loader = ConfigLoader(parser)
        loaded_args = config_loader.load_config(config_file, args)
        if loaded_args:
            args = loaded_args
            # If we auto-detected config, get input from the loaded config
            if input_file is None and hasattr(args, 'input') and args.input:
                input_file = args.input
    
    # Set the final input file
    if input_file:
        args.input = input_file
    
    if args.input is None:
        raise ValueError("Input URDF or xacro file must be specified either as argument or in config file.")
    else:
        if (input_ext := args.input.split(".")[-1]) not in ["urdf", "xacro"]:
            raise ValueError(f"Input file must be a URDF (.urdf) or xacro (.xacro) file, not {input_ext}")
        print_info(f"Input file: {args.input}")
    
    # Update log level again in case it was in the config
    set_log_level(args.log_level, args.traceback)

    converter = URDFToMJCFConverter(args)
    return converter.convert()