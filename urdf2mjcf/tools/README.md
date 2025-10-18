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

## `extract_dae_meshes.py`

This script extracts individual meshes from a DAE (COLLADA) file and saves them as separate mesh files. This is useful when a single DAE file contains multiple meshes with different materials or colors, and you need to process them individually.

### Dependencies

This script requires the `pycollada` and `trimesh` libraries.

```bash
pip install pycollada trimesh
```

### Usage

You provide the path to a DAE file and an output directory. The script will extract all meshes and save them as individual files.

**Arguments:**

-   `input`: Path to the input DAE file.
-   `-o, --output`: Output directory for extracted meshes (default: `extracted_meshes`).
-   `-f, --format`: Output mesh format - `stl`, `obj`, `ply`, or `off` (default: `stl`).
-   `-v, --verbose`: Enable verbose output with detailed information.
-   `-j, --json`: Save mesh information to a custom JSON file path (default: `<output_dir>/mesh_info.json`).
-   `-s, --scale`: Scale factor to apply to meshes. If not specified, auto-detects from DAE units and converts to meters.
-   `-c, --combine`: Combine all meshes into a single file (legacy/backward compatible mode).

**Example (extract separate meshes with colors - default):**

```bash
urdf2mjcf-extract-dae robot_model.dae -o meshes/extracted/
```

**Example (combine all meshes into single file):**

```bash
urdf2mjcf-extract-dae robot_model.dae -o meshes/extracted/ --combine
```

**Example (extract to OBJ with verbose output):**

```bash
urdf2mjcf-extract-dae robot_model.dae -o meshes/extracted/ -f obj --verbose
```

**Example (extract with manual scale override):**

```bash
urdf2mjcf-extract-dae model.dae -o output/ --scale 0.001
```

The script will:
- Create the output directory if it doesn't exist
- Extract each mesh primitive from the DAE file
- Name files based on geometry IDs and material names
- Extract color and material information (diffuse, ambient, specular, shininess)
- Save mesh metadata to a JSON file including colors, materials, and mesh statistics
- Show a progress bar during extraction (unless `--verbose` is enabled)
- Report the number of successfully extracted meshes

**JSON Output Format:**
The generated JSON file contains:
- `source`: Original DAE filename
- `total_meshes`: Total number of meshes found
- `extracted_meshes`: Number of successfully extracted meshes
- `meshes`: Dictionary with details for each mesh:
  - `file`: Output filename
  - `geometry`: Geometry ID from DAE
  - `vertices`: Vertex count
  - `faces`: Face count
  - `material`: Material name (if available)
  - `rgba`: RGBA color array [R, G, B, A] in range [0-1] (URDF-compatible format, derived from diffuse color)
  - `color`: Detailed color information (if available):
    - `rgba`: Diffuse color [R, G, B, A]
    - `ambient`: Ambient color [R, G, B, A]
    - `specular`: Specular color [R, G, B, A]
    - `emission`: Emission color [R, G, B, A]
    - `shininess`: Shininess value

