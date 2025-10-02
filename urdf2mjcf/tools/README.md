# Utility Scripts for Mesh Processing

This directory contains Python scripts for processing mesh files, such as calculating inertia, generating collision meshes, and simplifying mesh geometry.

## `calculate_inertia.py`

This script calculates the moment of inertia for a given STL file and mass. It can output the inertia tensor and center of mass, formatted as a URDF `<inertial>` tag.

### Dependencies

The script requires the `trimesh` library. You can install it using pip:

```bash
pip install trimesh[easy]
```

### Usage

To use the script, you must provide the path to the mesh file and the object's mass. You can also optionally specify a new origin translation and orientation.

**Arguments:**

-   `mesh`: Path to the input STL mesh file.
-   `mass`: Mass of the object in kilograms.
-   `--translation`: (Optional) The translation `[x y z]` of the new origin.
-   `--orientation`: (Optional) The orientation `[roll pitch yaw]` in degrees of the new origin.

**Example:**

```bash
./calculate_inertia.py ../meshes/visual/base_link.stl 1.5 --translation 0.0 0.0 0.01
```

## `generate_collision_mesh.py`

This script generates simplified convex hull collision meshes from visual meshes. It reads all `.stl` files from the `../meshes/` directory and creates corresponding collision meshes in `../meshes/collision/`.

### Dependencies

This script requires the `open3d` library.

```bash
pip install open3d
```

### Usage

The script processes all STL files in the `../meshes` directory in parallel.

```bash
./generate_collision_mesh.py
```

## `simplify_mesh.py`

This script reduces the number of faces in an STL mesh. It can process a single file or a directory of files. It can also apply scaling and translation transformations before simplification.

### Dependencies

This script requires the `pymeshlab` library.

```bash
pip install pymeshlab
```

### Usage

You need to provide an input path (file or directory) and a reduction factor.

**Arguments:**

-   `input_path`: Path to the input STL file or folder.
-   `output_path`: (Optional) Path for the output file or folder. Defaults to appending `_simplified` to the name.
-   `--reduction`: Target reduction ratio (e.g., 0.5 for a 50% reduction).
-   `--translation`: (Optional) A translation `[x y z]` to apply.
-   `--scale`: (Optional) A uniform scaling factor to apply.

**Example (single file):**

```bash
./simplify_mesh.py ../meshes/visual/l_tip.stl ../meshes/collision/simplified/l_tip.stl --reduction 0.8
```

**Example (directory):**

```bash
./simplify_mesh.py ../meshes/visual/ ../meshes/collision/simplified/ --reduction 0.8
```
