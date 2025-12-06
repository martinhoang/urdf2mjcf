# Release Notes - urdf2mjcf

Convert URDF/Xacro files to MuJoCo MJCF format with advanced mesh processing and ROS2 integration.

---

## [v0.8.1] - 2025-10-XX - Bug Fixes

### Fixed
- Bug with not-injecting-no-match nodes from URDF to MJCF

---

## [v0.8.0] - 2025-10-XX - Inertial Transform & Mesh Validation

### Added
- **Inertial Tensor Transformation**: Automatically transforms inertial frames to zero RPY for MuJoCo 3.2+ compatibility
  - Uses proper rotation matrix transformation: I' = R·I·R^T
  - Handles expressions like `${pi/2}` in RPY values
  - Disable with `--no-zero-inertial-rpy`
- **Automatic Mesh Validation**: Auto-detect and fix meshes exceeding MuJoCo's 200,000 face limit
  - Creates automatic backups before modification
  - Smart reduction to 50% of limit (100,000 faces)
  - Supports STL, OBJ, DAE formats
- **DAE Multi-Mesh Extraction**: Extract separate STL files per mesh/material from DAE files
  - Preserves RGBA colors from materials
  - Optional mesh type suffixes (`_visual`, `_collision`)
  - Use `--separate-dae-meshes` flag

### Fixed
- Bug with `inject_attrs` multiple predications not working

---

## [v0.7.0] - 2025-10-18 - Advanced Collision Mesh Generation

### Added
- CoACD integration for high-quality collision mesh decomposition
- Multi-format support: STL, OBJ, DAE with case-insensitive extensions
- Real-time progress tracking with tqdm
- Parallel processing with configurable worker count
- Interactive visualization mode with Open3D

### Changed
- Enhanced `generate_collision_mesh.py` with comprehensive CLI arguments
- Improved error reporting and CoACD output handling

---

## [v0.6.0] - 2025-10-16 - CLI Utilities Integration

### Added
- Integrated mesh processing tools as CLI utilities:
  - `urdf2mjcf` - Main converter
  - `urdf2mjcf-generate-collision` - Collision mesh generator
  - `urdf2mjcf-simplify-mesh` - Mesh simplification
  - `urdf2mjcf-calculate-inertia` - Inertia calculator

### Changed
- Enhanced main converter interface for better tool integration
- Improved command-line argument parsing

---

## [v0.5.0] - 2025-10-05 - ROS2 Integration

### Added
- Actuator re-naming support for ROS2 plugin compatibility
- Improved ROS2 control interface integration

### Fixed
- Erroneous removal of direct-inserting attributes in special operations
- Post-processing attribute handling issues

---

## [v0.4.0] - 2025-10-02 - Mesh Processing Tools

### Added
- Mesh simplification tool (reduce complexity while preserving shape)
- Basic convex hull collision mesh generator
- Inertia calculator from mesh geometry
- Comprehensive tool documentation

---

## [v0.3.0] - 2025-10-02 - Bug Fixes

### Fixed
- `inject_attrs` operation bugs under parent tag scope
- Attribute scope resolution in post-processing

### Added
- Error handling for failed mesh conversions
- Better error messages for troubleshooting

---

## [v0.2.0] - 2025-10-01 - Enhanced Configuration

### Added
- Improved string parsing in `inject_attrs` and `replace_attrs` operations
- Support for complex attribute manipulation patterns

---

## [v0.1.0] - 2025-09-30 - Initial Release

### Added
- Core URDF to MJCF conversion engine
- JSON configuration file support
- Mesh operations: simplification, inertia calculation, basic processing
- URDF preprocessing and MJCF post-processing
- CLI interface with special operations: `inject_attrs`, `replace_attrs`

---

## Installation

```bash
# Install as UV tool (recommended)
cd urdf2mjcf
uv tool install -e .

# Or via pip
pip install urdf2mjcf
```

---

## Quick Examples

```bash
# Basic conversion with automatic inertia calculation
urdf2mjcf robot.urdf.xacro -ci

# Full ROS2 setup with floating base
urdf2mjcf robot.urdf.xacro -fb -af -arp -arc -ci

# With JSON config (auto-detected)
urdf2mjcf config.json

# Generate collision meshes using CoACD
urdf2mjcf-generate-collision /path/to/meshes -m coacd
```

---

## Dependencies

- Python ≥ 3.8
- MuJoCo ≥ 3.0
- trimesh (for inertia calculation)
- pymeshlab (for mesh simplification)
- open3d (for collision generation)
- ament-index-python (for ROS2)

---

## License

[MIT License](LICENSE)

---

## Documentation

- [README.md](README.md) - Complete usage guide
- [doc/todo.md](doc/todo.md) - Planned features
- [urdf2mjcf/tools/README.md](urdf2mjcf/tools/README.md) - Mesh processing tools

This project follows [Semantic Versioning](https://semver.org/).
