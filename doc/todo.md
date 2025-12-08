# Development Roadmap - urdf2mjcf

This document tracks the development progress and future plans for the urdf2mjcf project.

---

## Current Release & Next Steps

### v0.9.0 - Planned (Next Release) 🎯

- [ ] Re-arrange post-processing procedures for max functionalities and avoid potential bugs
- [x] Show nice progress bar during mesh processing steps
- [x] Passing additional arguments alongside JSON config file
- [x] Add logging system with different verbosity levels

---

## Future Vision (After v0.9.0)

### Potential Features for Later Releases

- Unit tests for the whole package
- GUI interface for conversion configuration
- Real-time conversion progress display
- Automatic URDF validation before conversion

---

## Completed Releases

### v0.8.1 - Bug Fixes (2025-10-XX) ✨ Latest

- [x] Fixed bug with not-injecting-no-match nodes from URDF to MJCF
- [x] Fixed bug with `inject_attrs` multiple predications not working

### v0.8.0 - Inertial Transform & Mesh Validation (2025-10-XX)

- [x] Processing multiple meshes (with their colors) inside a DAE file
- [x] Auto detect and validate mesh face count before conversion
- [x] Automatic inertial tensor transformation to zero RPY orientation
- [x] Automatic mesh validation and face limit handling (200,000 faces)
- [x] DAE multi-mesh extraction with material preservation

### v0.7.0 - Advanced Collision Mesh Generation (2025-10-18)

- [x] Use CoACD for convex decomposition to achieve high-resolution collision hulls
- [x] Multi-format support (STL, OBJ, DAE) with case-insensitive matching
- [x] Progress bar with tqdm
- [x] Verbose and visualization modes
- [x] Single file and directory processing
- [x] Parallel processing support

### v0.6.0 - CLI Utilities Integration (2025-10-16)

- [x] Activate tool usage in cli arguments
- [x] Install tools as extension of the "urdf2mjcf" tool with command prefixes
- [x] Integrated mesh tools as CLI utilities

### v0.5.0 - ROS2 Integration (2025-10-05)

- [x] Actuator re-naming for ros2 plugins
- [x] Fixed special operations attribute handling
- [x] Improved post-processing procedures

### v0.4.0 - Mesh Processing Tools (2025-10-02)

- [x] Added mesh simplification tool
- [x] Added collision mesh generation tool (basic convex hull)
- [x] Added inertia calculation tool
- [x] Automatically use tool to reduce mesh complexity (1-200,000 faces)
- [x] Added todo.md documentation

### v0.3.0 - Bug Fixes and Error Handling (2025-10-02)

- [x] Fixed inject_attrs bugs under parent tag
- [x] Raise error on failed mesh conversion
- [x] Apply special operations with respect to parent tag scope

### v0.2.0 - Enhanced Configuration (2025-10-01)

- [x] Improved string parsing in inject_attrs and replace_attrs
- [x] Better attribute manipulation in post-processing

### v0.1.0 - Initial Release (2025-09-30)

- [x] Basic URDF to MJCF conversion
- [x] JSON configuration support
- [x] Mesh operations (simplification, inertia calculation)
- [x] XML pre/post-processing
- [x] CLI interface
- [x] Special operations (inject_attrs, replace_attrs)

---

## Recent Achievements (Since v0.8.1)

- [x] **Automatic Inertia Calculation**: Calculate and inject inertia tensors from mesh geometry using URDF link masses
- [x] **Dynamic Actuator Gains**: Support for per-actuator gain configurations via dict
- [x] **Dampratio Support**: Added support for `dampratio` attribute in velocity actuators (alternative to `kv`)
- [x] **Solver Options**: Added solver configuration to compiler options
- [x] **Scale Extraction from URDF**: Parse mesh scale from URDF `<mesh scale="">` attributes instead of hardcoding
- [x] **Unified Link Properties**: Clean data structure combining mass and mesh scales
- [x] **Updated Documentation**: Concise README and RELEASE notes with all new features

---

## Known Issues & Limitations

### Current Limitations

- No GUI interface yet

### Performance Considerations

- Large meshes (>1M faces) may be slow to process
- Parallel processing limited by CPU cores
- Memory usage scales with mesh complexity

### Compatibility Notes

- Requires Python 3.8+
- CoACD requires additional dependencies
- Some features require pymeshlab

---

## Contributing

Interested in contributing? Check the items in v0.9.0 or pick any feature you're interested in!

### How to Contribute

1. Check this todo list for items to work on
2. Create an issue to discuss your proposed changes
3. Fork the repository and create a feature branch
4. Submit a pull request with your changes
5. Ensure tests pass and documentation is updated

### Development Guidelines

- Follow PEP 8 style guide
- Add tests for new features
- Update documentation as needed
- Keep commits atomic and well-described

---

## Version History

For detailed release notes, see [RELEASE.md](../RELEASE.md)

**Latest Release:** v0.7.0 (2025-10-18)  
**Next Planned:** v0.8.0 (Configuration & UX improvements)

---

Last Updated: 2025-10-18
