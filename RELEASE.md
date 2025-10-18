# Release Notes - urdf2mjcf

A tool for converting URDF (Unified Robot Description Format) files to MJCF (MuJoCo XML) format with advanced mesh processing capabilities.

---

## [v0.7.0] - 2025-10-18 - Advanced Collision Mesh Generation

### Added
- **CoACD Integration**: Collision-Aware Convex Decomposition for high-resolution collision hulls
- **Multi-format Support**: Process STL, OBJ, and DAE (COLLADA) files with case-insensitive extensions
- **Progress Tracking**: Real-time progress bar using tqdm for better user experience
- **Flexible Processing**: Support for both single files and entire directories
- **Parallel Processing**: Multi-threaded mesh processing with configurable worker count
- **Visualization Mode**: Interactive mesh visualization with Open3D
- **Verbose Mode**: Detailed output option for debugging and monitoring

### Changed
- Enhanced `generate_collision_mesh.py` with comprehensive CLI arguments
- Improved error reporting with always-visible error messages
- Better handling of CoACD output formats (Trimesh objects and tuple formats)

### Technical Details
```bash
# Generate collision meshes with CoACD
urdf2mjcf-generate-collision /path/to/meshes -m coacd

# Single file processing with custom output
urdf2mjcf-generate-collision robot.stl -o collision/ -m coacd -t 0.03

# With verbose output and progress bar
urdf2mjcf-generate-collision meshes/ -m coacd --verbose
```

---

## [v0.6.0] - 2025-10-16 - CLI Utilities Integration

### Added
- Integrated mesh processing tools as CLI utilities
- Command-line access to all mesh tools via `urdf2mjcf-*` commands
- Better tool accessibility through entry points

### Changed
- Enhanced main converter interface for better tool integration
- Improved command-line argument parsing

### Available CLI Tools
- `urdf2mjcf` - Main URDF to MJCF converter
- `urdf2mjcf-generate-collision` - Collision mesh generator
- `urdf2mjcf-simplify-mesh` - Mesh simplification tool
- `urdf2mjcf-calculate-inertia` - Inertia calculation tool

---

## [v0.5.0] - 2025-10-05 - ROS2 Integration

### Added
- Actuator re-naming support for ROS2 plugins to find actuators
- Improved compatibility with ROS2 control interfaces

### Fixed
- Erroneous removal of direct-inserting and non-wildcard attributes in special operations
- Post-processing attribute handling improvements

### Changed
- Enhanced post-processing procedures for better reliability

---

## [v0.4.0] - 2025-10-02 - Mesh Processing Tools

### Added
- **Mesh Simplification Tool**: Reduce mesh complexity while preserving shape
- **Collision Mesh Generator**: Basic convex hull collision mesh generation
- **Inertia Calculator**: Calculate accurate inertia properties from meshes
- **Documentation**: Added `doc/todo.md` for tracking development progress
- **Tool Documentation**: Comprehensive README for mesh processing tools

### Technical Details
- Integrated pymeshlab for advanced mesh operations
- Support for automatic mesh complexity reduction (1-200,000 faces)
- Tools can be used independently or as part of the conversion pipeline

---

## [v0.3.0] - 2025-10-02 - Bug Fixes and Error Handling

### Fixed
- Fixed bugs with `inject_attrs` operations under parent tag scope
- Corrected attribute scope resolution in post-processing

### Added
- Error handling for failed mesh conversions
- Exceptions raised on mesh processing failures for better debugging

### Changed
- Improved stability and reliability of the conversion process
- Better error messages for troubleshooting

---

## [v0.2.0] - 2025-10-01 - Enhanced Configuration

### Added
- Improved string parsing in `inject_attrs` and `replace_attrs` operations
- Support for more complex attribute manipulation patterns

### Changed
- Enhanced attribute manipulation in post-processing
- Better handling of special characters in attribute values

---

## [v0.1.0] - 2025-09-30 - Initial Release

### Added
- **Core Conversion**: URDF to MJCF conversion engine
- **JSON Configuration**: Configuration file support for conversion parameters
- **Mesh Operations**: 
  - Mesh simplification
  - Inertia calculation from meshes
  - Basic mesh processing utilities
- **XML Processing**:
  - URDF preprocessing
  - MJCF post-processing with special operations
- **CLI Interface**: Command-line tool for easy conversion
- **Special Operations**:
  - `inject_attrs`: Insert attributes into XML elements
  - `replace_attrs`: Replace attribute values
  - Tag scope awareness for operations

### Dependencies
- Python 3.8+
- lxml for XML processing
- numpy for numerical operations
- pymeshlab for mesh processing
- trimesh for mesh operations

---

## Installation

### Basic Installation
```bash
pip install urdf2mjcf
```

### With Advanced Features
```bash
# For CoACD collision mesh generation
pip install urdf2mjcf[coacd]

# Or install dependencies manually
pip install coacd trimesh tqdm
```

---

## Usage Examples

### Basic URDF to MJCF Conversion
```bash
urdf2mjcf input.urdf -o output.xml
```

### With JSON Configuration
```bash
urdf2mjcf input.urdf -c config.json -o output.xml
```

### Generate Collision Meshes

#### Simple Convex Hull (Fast)
```bash
urdf2mjcf-generate-collision /path/to/meshes
```

#### Advanced CoACD Method (Better Accuracy)
```bash
urdf2mjcf-generate-collision /path/to/meshes -m coacd -t 0.05 -c 32
```

#### Single File Processing
```bash
urdf2mjcf-generate-collision robot.stl -o collision/ -m coacd --verbose
```

### Simplify Mesh
```bash
urdf2mjcf-simplify-mesh input.stl -o output.stl --target-faces 10000
```

### Calculate Inertia
```bash
urdf2mjcf-calculate-inertia mesh.stl --density 1000
```

---

## Roadmap

### Planned Features
- [ ] Unit tests for the entire package
- [ ] Processing multiple meshes with colors inside DAE files
- [ ] Re-arranged post-processing procedures for maximum functionality
- [ ] Additional argument passing alongside JSON config files
- [ ] Support for more URDF features

### Under Consideration
- Integration with physics simulators
- Real-time conversion preview
- GUI interface for conversion configuration
- Batch processing improvements

---

## Contributing

Contributions are welcome! Please check the [TODO list](doc/todo.md) for planned features and open issues.

### Development Setup
```bash
git clone https://github.com/martinhoang/urdf2mjcf.git
cd urdf2mjcf
pip install -e .
```

---

## License

[MIT License](LICENSE)

---

## Changelog Format

This project follows [Semantic Versioning](https://semver.org/):
- **MAJOR** version for incompatible API changes
- **MINOR** version for new functionality in a backward compatible manner
- **PATCH** version for backward compatible bug fixes

---

## Acknowledgments

- MuJoCo team for the physics engine
- CoACD authors for collision decomposition algorithm
- Open3D and trimesh communities for mesh processing tools

---

For detailed technical documentation, see [README.md](README.md) and [urdf2mjcf/tools/README.md](urdf2mjcf/tools/README.md).
