# urdf2mjcf Development Roadmap

Last updated: 2026-06-12

This roadmap tracks concrete improvements for the Python API, CLI, conversion
pipeline, mesh processing, and release workflow. The detailed visual plan is
available in [plan.html](plan.html).

## Current Baseline

- Package version: `0.9.0`
- Python requirement: 3.10+
- Interfaces: CLI, JSON configuration, and Python `convert(...)` API
- Tests: API argument resolution, URDF preprocessing, DAE extraction, utilities
- Main risk: conversion failures are often logged and returned as `None`
  instead of raising a structured error

## P0 - Correctness and API Contract

- [ ] Add `ConversionError` with stage, input path, and original exception
- [ ] Make the Python API raise on every failed conversion
- [ ] Give the CLI consistent non-zero exit codes without duplicate tracebacks
- [ ] Return a `ConversionResult` containing MJCF path, output directory,
      generated assets, warnings, and saved config path
- [ ] Validate inline and JSON values with the same rules
- [ ] Reject unknown config keys instead of silently adding attributes
- [ ] Validate ranges and combinations before processing:
  - simplification ratios must be between 0 and 1
  - face limits and target counts must be positive
  - solver and integrator values must be supported
  - ROS plugin options must have their required companion settings
- [ ] Resolve config-relative paths relative to the config file directory
- [ ] Add regression tests for missing input, xacro failure, MuJoCo compile
      failure, output write failure, and invalid configuration

### P0 acceptance criteria

- No converter failure path returns an ambiguous `None`
- CLI and Python API use the same validated options
- Temporary files and Rich console state are cleaned up on all failures
- Tests assert error type, stage, and exit code

## P1 - Maintainability and Test Coverage

- [ ] Move option definitions and defaults out of `cli.py` into a typed config
      model shared by CLI, JSON, and Python callers
- [ ] Split the monolithic converter into explicit stages:
  - resolve inputs and output layout
  - expand xacro
  - preprocess URDF
  - process meshes
  - compile with MuJoCo
  - post-process MJCF
  - save artifacts and metadata
- [ ] Keep `URDFToMJCFConverter` as the stable orchestration facade
- [ ] Add progress callbacks so API users are not tied to Rich terminal output
- [ ] Add `quiet` and progress-disable modes for libraries and CI
- [ ] Replace broad exception handling where failures can be classified
- [ ] Add small fixture robots for primitive, mesh, mimic, ROS2 control, and
      custom `<mujoco>` cases
- [ ] Add golden MJCF tests with normalization for deterministic comparisons
- [ ] Add CLI/API parity tests using the same configuration matrix
- [ ] Add linting, formatting, type checking, and Python 3.10-3.13 CI

## P2 - Mesh Pipeline Reliability and Performance

- [ ] Cache converted meshes using source hash plus processing options
- [ ] Avoid modifying source meshes during validation or simplification
- [ ] Write all generated assets into a staging directory and publish
      atomically after successful conversion
- [ ] Add deterministic collision and visual mesh naming with a manifest
- [ ] Report mesh operations and before/after face counts in conversion results
- [ ] Add bounded worker controls and memory-aware defaults
- [ ] Test duplicate basenames, mixed-case extensions, nested directories,
      non-uniform scale, DAE transforms, and material bindings
- [ ] Add OBJ/DAE/STL capability checks with actionable dependency errors
- [ ] Benchmark representative small, medium, and large robot models

## P3 - User Experience and New Capabilities

- [ ] Add `urdf2mjcf validate` for configuration, dependency, URDF, mesh, and
      output checks without writing conversion artifacts
- [ ] Add `urdf2mjcf inspect` to summarize links, joints, meshes, actuators,
      mimic joints, and ROS2 control interfaces
- [ ] Add `dry_run=True` to the Python API
- [ ] Generate a machine-readable conversion report (`report.json`)
- [ ] Add configuration schema export and editor completion
- [ ] Add package URI resolver hooks for non-ROS environments
- [ ] Add optional conversion profiles such as `fixed-base`, `floating-base`,
      `ros2-control`, and `lightweight-meshes`
- [ ] Consider a GUI only after the config schema and validation API stabilize

## Documentation and Release Work

- [ ] Correct stale README option names and examples against `--help`
- [ ] Document every Python API option and return/error contract
- [ ] Add migration notes for configuration changes
- [ ] Keep `config_template.json` generated from canonical defaults
- [ ] Add changelog/release automation and version consistency checks
- [ ] Define semantic-versioning policy for CLI, config, and Python API changes
- [ ] Add installation guidance for minimal and full mesh-processing extras

## Recently Completed

- [x] Added public Python `convert(...)` and `convert_urdf(...)` API
- [x] Added inline options, mapping options, and JSON config overrides
- [x] Added `build_conversion_args(...)`
- [x] Returned the generated MJCF path on successful conversion
- [x] Added Python API tests and README examples
- [x] Fixed DAE scene transforms and mesh path handling
- [x] Added recursive STL conversion and duplicate mesh handling

## Deferred

- Full desktop GUI
- Live 3D preview embedded in the converter
- Automatic physical parameter tuning
- Cloud conversion service

These are intentionally deferred until conversion behavior, error contracts,
configuration schemas, and deterministic output are stable.
