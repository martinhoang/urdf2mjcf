# URDF2MJCF Converter

A powerful CLI tool for converting URDF/Xacro files to MuJoCo MJCF format with advanced mesh processing, physics customization, and ROS2 integration.

## Features

- **Smart Input**: Supports URDF, Xacro files, and JSON config with auto-detection
- **Automatic Inertia Calculation**: Calculate and inject inertia tensors from mesh geometry and link masses
- **Inertial Tensor Transformation**: Automatically transforms inertial frames to zero RPY (MuJoCo 3.2+ compatibility)
- **Mesh Processing**: Automatic validation, simplification, and DAE multi-mesh extraction with material preservation
- **ROS2 Integration**: Built-in support for ROS2 control and [MujocoRosUtils](https://github.com/martinhoang/MujocoRosUtils.git) plugins
- **Physics Tuning**: Configure actuators, damping, gravity compensation, and solver settings
- **Custom MJCF Injection**: Inject custom MuJoCo elements directly from URDF via `<mujoco>` tags

## Installation

```bash
# Install as UV tool (recommended)
cd urdf2mjcf
uv tool install -e .

# Verify installation
urdf2mjcf --help
```

## Quick Start

```bash
# Basic conversion
urdf2mjcf robot.urdf.xacro

# With floating base and floor
urdf2mjcf robot.urdf.xacro -fb -af -haf 0.3

# Using JSON config (auto-detected)
urdf2mjcf config.json

# Advanced: Full ROS2 setup with custom gains
urdf2mjcf robot.urdf.xacro -fb -af -arp -arc -djs 800.0 5.0 -ci
```

## Key Arguments

### Core Options
- `-o, --output PATH` - Output directory (default: `<input_name>/`)
- `-fb, --floating-base` - Add free joint for floating base
- `-af, --add-floor` - Add ground plane
- `-haf, --height-above-floor HEIGHT` - Set robot height above floor (default: 0.3)

### Physics & Dynamics
- `-djs, --default-actuator-gains Kp Kv` - Set joint stiffness and damping
- `-dm, --damping-multiplier FACTOR` - Multiply all joint damping values
- `-gc, --gravity-compensation` - Enable gravity compensation
- `-a, --armature VALUE` - Set global armature for joints
- `-s, --solver SOLVER` - Simulation solver (PGS, CG, Newton)
- `-int, --integrator TYPE` - Integrator (Euler, RK4, implicitfast)

### ROS2 Integration
- `-arp, --add-ros-plugins` - Add MujocoRosUtils actuator plugins
- `-arc, --add-ros2-control` - Add main Ros2Control plugin
- `-rcc, --ros2-control-config PATH` - Path to ros2_control YAML config
- `-ncp, --no-clock-publisher` - Disable clock publisher
- `-nmj, --no-mimic-joints` - Disable mimic joint plugins

### Mesh Processing
- `-ci, --calculate-inertia` - **NEW!** Calculate inertia from meshes using URDF link masses
- `-sr, --simplify-reduction RATIO` - Simplify meshes (0.0-1.0, values < 1.0 enable)
- `-stf, --simplify-target-faces COUNT` - Target face count for simplification
- `-sdm, --separate-dae-meshes` - Extract separate STL per mesh/material from DAE
- `-nmt, --no-append-mesh-type` - Don't append `_visual`/`_collision` to filenames
- `-nvmf, --no-validate-mesh-faces` - Disable auto mesh face validation
- `-mfl, --max-faces-limit COUNT` - Max faces per mesh (default: 200000)
- `-gcm, --generate-collision-meshes` - Generate convex hull collision meshes

### Advanced
- `-nzi, --no-zero-inertial-rpy` - Keep original inertial RPY (disable auto-transform)
- `-xa, --xacro-args KEY:=VALUE ...` - Pass args to xacro processor
- `-co, --compiler-options KEY=VALUE ...` - Override compiler attributes
- `-sp, --save-preprocessed` - Save intermediate URDF for debugging
- `-ll, --log-level LEVEL` - Logging level (DEBUG, INFO, WARNING, ERROR)
- `-tb, --traceback` - Show full error traceback

## JSON Configuration

Create `config.json`:
```json
{
  "input": "robot.urdf.xacro",
  "output": "mujoco_output",
  "floating_base": true,
  "add_floor": true,
  "height_above_floor": 0.3,
  "add_ros_plugins": true,
  "add_ros2_control": true,
  "calculate_inertia": true,
  "default_actuator_gains": [800.0, 5.0],
  "damping_multiplier": 1.5,
  "gravity_compensation": true,
  "separate_dae_meshes": true,
  "xacro_args": ["hardware_type:=sim_mujoco", "use_sensors:=true"],
  "log_level": "INFO"
}
```

Then run: `urdf2mjcf config.json` (auto-detected) or `urdf2mjcf robot.urdf -cf config.json`

**Precedence**: CLI arguments > JSON config > defaults

## New Features

### 🎯 Automatic Inertia Calculation & Injection

Calculate physically accurate inertia tensors from mesh geometry and automatically inject them into the URDF:

```bash
urdf2mjcf robot.urdf.xacro -ci
```

**How it works:**
1. Extracts link masses from URDF `<inertial><mass>` elements
2. Parses mesh scale from `<mesh scale="...">` attribute
3. Calculates inertia tensor using mesh geometry (via trimesh)
4. Injects calculated values back into preprocessed URDF before MuJoCo import

**Requirements**: Link must have `<mass>` defined in URDF and at least one mesh (visual or collision)

**Example output:**
```
Calculating inertia for visual mesh of link 'wrist_3_link': wrist_3_visual.stl (scale: 0.001)
-> Calculated inertia for link 'wrist_3_link' (mass: 0.35 kg):
<inertial>
  <origin xyz="0.0012 -0.0003 0.0245" rpy="0 0 0"/>
  <mass value="0.35"/>
  <inertia ixx="0.000123" ixy="0.000002" ixz="0.000001"
           iyy="0.000145" iyz="0.000003" izz="0.000089"/>
</inertial>
```

### 🔄 Inertial Tensor Transformation (Auto-enabled)

Automatically transforms inertial properties to zero RPY orientation using proper rotation matrix transformation:

**Problem**: MuJoCo 3.2+ doesn't support non-zero RPY in inertial frames  
**Solution**: Transform inertia tensor: **I'** = **R** · **I** · **R**^T

```xml
<!-- Before (URDF) -->
<inertial>
  <origin xyz="0 0 0.02" rpy="1.5708 0 0"/>  <!-- 90° rotation -->
  <inertia ixx="0.001" iyy="0.002" izz="0.003" .../>
</inertial>

<!-- After (MJCF) -->
<inertial pos="0 0 0.02" mass="..." 
          diaginertia="0.002 0.001 0.003"/>  <!-- Transformed -->
```

Disable with: `urdf2mjcf robot.urdf -nzi`

### 🎨 DAE Multi-Mesh Extraction with Materials

Extract separate STL files per mesh/material from DAE files, preserving colors:

```bash
urdf2mjcf robot.urdf.xacro -sdm
```

**Output**: `base_Grey.stl`, `base_Black.stl`, `arm_Metal.stl` (with RGBA colors in URDF)

Combine with mesh type suffixes: `urdf2mjcf robot.urdf -sdm -nmt` → `base_Grey_visual.stl`

### ✅ Automatic Mesh Validation & Fixing

MuJoCo has a 200,000 face limit. This tool auto-detects and fixes oversized meshes:

```
Validating mesh files before MuJoCo import...
Found 2 mesh(es) exceeding MuJoCo's 200,000 face limit:
  ✗ base.stl: 450,320 faces → reducing to ~100,000
Attempting to automatically fix oversized meshes...
✓ Successfully reduced base.stl to 99,847 faces
```

**Features**: Automatic backups (`.bak`), smart target reduction (50% of limit), supports STL/OBJ/DAE

Disable with: `urdf2mjcf robot.urdf -nvmf`

### 🔧 Custom MJCF Element Injection

Inject custom MuJoCo elements directly from your URDF using `<mujoco>` tags:

```xml
<robot>
  <mujoco>
    <compiler meshdir="assets/" autolimits="true"/>
    <worldbody>
      <!-- Inject site into gripper body -->
      <body inject_children="name='gripper_link'">
        <site name="grasp_site" pos="0 0 0.05" type="sphere" size="0.01"/>
      </body>
      
      <!-- Add visual attributes to all geoms -->
      <geom inject_attrs="class='visual';group='2'"/>
    </worldbody>
  </mujoco>
</robot>
```

**Supported operations:**
- `inject_attr` / `inject_attrs` - Add/overwrite attributes
- `replace_attrs` - Replace only existing attributes  
- `inject_children` - Inject child elements into matching parents

## Output Structure

```
output_directory/
├── robot.xml               # Main MJCF file
├── assets/                 # Meshes and textures
│   ├── base_visual.stl
│   ├── arm_collision.stl
│   └── ...
└── config.json            # Configuration used
```

## Common Workflows

### Standard ROS2 Robot
```bash
urdf2mjcf robot.urdf.xacro \
  -fb -af -haf 0.3 \
  -arp -arc \
  -djs 1000.0 10.0 \
  -ci \
  -xa "hardware_type:=sim_mujoco"
```

### High-Fidelity Simulation
```bash
urdf2mjcf robot.urdf.xacro \
  -fb -af \
  -gc \
  -dm 2.0 \
  -s Newton \
  -int RK4 \
  -ci \
  -sdm
```

### Debugging & Inspection
```bash
urdf2mjcf robot.urdf.xacro \
  -sp \
  -ll DEBUG \
  -tb \
  -o debug_output
```

## Troubleshooting

**Inertia calculation fails**: Ensure links have `<mass>` values in URDF and meshes have proper `scale` attribute  
**Mesh errors**: Check mesh files exist and are accessible. Use `-ll DEBUG` for details  
**Xacro errors**: Verify xacro arguments with `-xa` flag match your xacro file parameters  
**MuJoCo import fails**: Enable mesh validation with default settings (auto-enabled) or check face counts

**Debug mode**: `urdf2mjcf robot.urdf -ll DEBUG -tb -sp`

## Dependencies

- Python ≥ 3.8
- MuJoCo ≥ 3.0
- trimesh (for inertia calculation): `pip install trimesh[easy]`
- pymeshlab (for mesh simplification): `pip install pymeshlab`
- open3d (for collision generation): `pip install open3d`
- ament-index-python (for ROS2): `pip install ament-index-python`

## License

[LICENSE](./LICENSE)

## Contributing

See [TODO](doc/todo.md) for planned features and known issues.