# URDF2MJCF Converter

A powerful command-line tool for converting URDF (Unified Robot Description Format) files to MuJoCo MJCF (XML) format with advanced features for robotics simulation.

## Features

- **Multiple Input Formats**: Supports URDF, Xacro, and JSON configuration files
- **ROS2 Integration**: Built-in support for ROS2 control plugins and MuJoCo-ROS utilities at my fork of AIST [MujocoRosUtils](https://github.com/martinhoang/MujocoRosUtils.git)
- **Physics Customization**: Configure joint properties, actuators, and simulation parameters
- **Mesh Processing**: Automatic mesh copying and optional mesh reduction
- **Flexible Configuration**: CLI arguments, JSON config files, or hybrid approaches
- **Auto-Detection**: Automatically detects JSON config files when passed as input

## Installation

### As a UV Tool (System-wide)

```bash
# Navigate to the urdf2mjcf directory
cd /path/to/umanoid_description_mujoco/urdf2mjcf

# Install as a system-wide tool
uv tool install .

# Or install in development mode for local changes
uv tool install -e .
```

### Verify Installation

```bash
urdf2mjcf --help
```

## Usage

### Basic Usage

```bash
# Convert a URDF file
urdf2mjcf robot.urdf

# Convert a Xacro file
urdf2mjcf robot.urdf.xacro

# Specify output directory
urdf2mjcf robot.urdf -o /path/to/output

# Add floating base and floor
urdf2mjcf robot.urdf -fb -af
```

### Configuration File Usage

#### Method 1: Explicit Config Flag
```bash
urdf2mjcf robot.urdf -cf config.json
```

#### Method 2: Auto-Detection (New!)
```bash
# Automatically detects and loads JSON config
urdf2mjcf config.json
```

### JSON Configuration Format

Create a `config.json` file with your desired settings:

```json
{
  "input": "path/to/robot.urdf.xacro",
  "output": "output_directory",
  "floating_base": true,
  "add_floor": true,
  "height_above_floor": 0.5,
  "add_ros_plugins": true,
  "add_ros2_control": true,
  "default_actuator_gains": [1000.0, 10.0],
  "armature": 0.01,
  "damping_multiplier": 2.0,
  "gravity_compensation": true,
  "xacro_args": ["param1:=value1", "param2:=value2"],
  "mesh_reduction": 0.8,
  "log_level": "INFO"
}
```

## Command Line Options

### Core Conversion Options
- `-af, --add-floor`: Add a ground plane to the simulation world
- `-fb, --floating-base`: Add a free joint to the root link to make the base float  
- `-haf, --height-above-floor`: Set the height of the robot w.r.t the added floor
- `-na, --no-actuators`: Do not add position and plugin actuators for non-fixed joints
- `-o, --output`: The directory where the output MJCF file and assets will be saved

### Physics & Model Properties
- `-a, --armature`: Set a global armature value for all joints
- `-djs, --default-actuator-gains`: Set the default joint stiffness (kp) and damping (kv)
- `-dm, --damping-multiplier`: Multiply all joint damping values by this factor
- `-gc, --gravity-compensation`: Enable gravity compensation for all bodies

### ROS Integration
- `-arc, --add-ros2-control`: Add the main MujocoRosUtils::Ros2Control plugin
- `-arp, --add-ros-plugins`: Add MujocoRosUtils plugins for actuator commands
- `-ncp, --no-clock-publisher`: Do not add the ClockPublisher plugin
- `-nmj, --no-mimic-joints`: Do not add MimicJoint plugins
- `-rcc, --ros2-control-config`: Path to the ros2_control configuration YAML file

### Advanced & Debugging Options
- `-co, --compiler-options`: Override or add compiler attributes
- `-int, --integrator`: Set the simulation integrator
- `-ncm, --no-copy-meshes`: Do not copy referenced mesh files to output directory
- `-mr, --mesh-reduction`: Set the mesh reduction ratio (0.0-1.0)
- `-nvmf, --no-validate-mesh-faces`: Disable automatic mesh face validation (enabled by default)
- `-mfl, --max-faces-limit`: Maximum faces per mesh file (MuJoCo limit: 200000)
- `-sr, --simplify-reduction`: Simplify meshes using pymeshlab (reduction ratio 0.0-1.0)
- `-stf, --simplify-target-faces`: Simplify meshes to target number of faces
- `-gcm, --generate-collision-meshes`: Generate convex hull collision meshes
- `-ci, --calculate-inertia`: Calculate and print inertia for meshes
- `-cim, --calculate-inertia-mass`: Mass (in kg) to use for inertia calculations
- `-sp, --save-preprocessed`: Save the intermediate, pre-processed URDF file
- `-sdm, --separate-dae-meshes`: Extract each mesh/material from DAE as separate STL files with colors (default: combine all into single STL)
- `-amt, --append-mesh-type`: Append '_visual' or '_collision' to mesh filenames for easy distinguishment
- `-s, --solver`: Set the simulation solver
- `-xa, --xacro-args`: Arguments to pass to the xacro processor
- `-ll, --log-level`: Set the logging level (DEBUG, INFO, WARNING, ERROR)
- `-tb, --traceback`: Show full traceback on error

## Examples

### Example 1: Basic Robot Conversion
```bash
urdf2mjcf my_robot.urdf.xacro -o my_robot_mujoco -fb -af
```

### Example 2: Advanced Configuration
```bash
urdf2mjcf robot.urdf \
  -fb \
  -af \
  -haf 0.3 \
  -arp \
  -arc \
  -djs 800.0 5.0 \
  -dm 1.5 \
  -gc \
  -sdm \
  -amt \
  -xa "use_gripper:=true" "hardware_type:=sim_mujoco" \
  -ll DEBUG
```

### Example 3: DAE Mesh Processing
```bash
# Extract separate meshes from DAE files with material preservation
urdf2mjcf robot.urdf.xacro -o output/ --separate-dae-meshes

# Append mesh type to filenames for easy identification
urdf2mjcf robot.urdf.xacro -o output/ --append-mesh-type
# Creates: base_visual.stl, wrist_collision.stl, etc.

# Combined: separate DAE meshes + append type tags
urdf2mjcf robot.urdf.xacro -o output/ --separate-dae-meshes --append-mesh-type
# Creates: base_JointGrey_visual.stl, base_Black_visual.stl, wrist2_collision.stl, etc.
```

### Example 4: Using JSON Configuration
Create `robot_config.json`:
```json
{
  "input": "my_robot.urdf.xacro",
  "output": "mujoco_output",
  "floating_base": true,
  "add_floor": true,
  "height_above_floor": 0.2,
  "add_ros_plugins": true,
  "add_ros2_control": true,
  "default_actuator_gains": [500.0, 1.0],
  "gravity_compensation": true,
  "xacro_args": ["hardware_type:=sim_mujoco", "use_sensors:=true"]
}
```

Then run:
```bash
urdf2mjcf robot_config.json
```

### Example 5: Override Config with CLI Arguments
```bash
# Load config but override specific settings
urdf2mjcf robot_config.json -ll DEBUG -o different_output
```

### Example 6: Mesh Face Validation (Auto-Fix)
```bash
# Automatic validation and fixing (enabled by default)
urdf2mjcf robot.urdf

# Disable validation if you don't want auto-fixing
urdf2mjcf robot.urdf -nvmf

# Custom face limit (if needed)
urdf2mjcf robot.urdf -mfl 150000
```

## Mesh Face Count Validation

MuJoCo has a hard limit of **200,000 faces per mesh file**. This converter now **automatically validates** all meshes before importing into MuJoCo and **auto-fixes** any oversized meshes.

### How It Works

1. **Automatic Detection**: Before MuJoCo import, all mesh files are scanned
2. **Face Counting**: Counts faces in STL, OBJ, and DAE files
3. **Auto-Fix**: Meshes exceeding the limit are automatically simplified to 50% of the limit (100,000 faces)
4. **Backup**: Original files are backed up as `.bak` before modification
5. **Verification**: Confirms successful reduction after simplification

### Features

- ✅ **Enabled by default** - no manual intervention needed
- ✅ **Supports STL (binary/ASCII), OBJ, DAE** formats
- ✅ **Creates automatic backups** before modifying files
- ✅ **Detailed logging** shows before/after face counts
- ✅ **Smart target reduction** to 50% of limit for safety margin

### Manual Control

```bash
# Disable validation (not recommended)
urdf2mjcf robot.urdf --no-validate-mesh-faces

# Custom face limit
urdf2mjcf robot.urdf --max-faces-limit 150000
```

### Example Output

```
Validating mesh files before MuJoCo import...
Found 2 mesh(es) exceeding MuJoCo's 200,000 face limit:
  ✗ BASE_visual.stl: 450,320 faces → needs reduction to ~100,000
  ✗ ARM_visual.stl: 220,150 faces → needs reduction to ~100,000
Attempting to automatically fix oversized meshes...
Created backup: BASE_visual.stl.bak
Simplifying BASE_visual.stl: 450,320 → 100,000 faces
✓ Successfully reduced BASE_visual.stl to 99,847 faces
✓ Successfully simplified 2 mesh(es)
```

## Output Structure

The converter creates the following output structure:
```
output_directory/
├── robot_name.xml          # Main MJCF file
├── assets/                 # Mesh files and textures
│   ├── mesh1.stl
│   ├── mesh2.stl
│   └── ...
├── config.json            # Configuration used for conversion
└── robot_preprocessed.urdf # (Optional) Preprocessed URDF file
```

## Configuration Precedence

When using both JSON config files and CLI arguments:

1. **Default values** (lowest priority)
2. **JSON configuration file** values
3. **CLI arguments** (highest priority)

CLI arguments will always override JSON configuration values.

## Troubleshooting

### Common Issues

1. **File not found errors**: Ensure all mesh files referenced in the URDF are accessible
2. **Xacro processing errors**: Check xacro arguments and file syntax
3. **Missing dependencies**: Install required packages (mujoco, ament-index-python)

### Debug Mode
```bash
urdf2mjcf robot.urdf -ll DEBUG -tb
```

### Save Preprocessed Files
```bash
urdf2mjcf robot.urdf -sp
```

## Dependencies

- Python ≥ 3.8
- MuJoCo
- ament-index-python (for ROS2 integration)
- Standard libraries: argparse, json, os, sys

## Development

### Local Development Installation
```bash
cd urdf2mjcf/
uv tool install -e .
```

### Running Tests
[TODO]

## TODO bug fixes and new features
Refer to [TODO](/doc/todo.md)
 
## License

See the main package [LICENSE](./LICENSE) file for details.

## Contributing

Please follow the project's contribution guidelines and ensure all changes are tested before submission.