# DAE Multi-Mesh Extraction with Material Preservation

**Implementation Date:** October 18, 2025  
**Feature:** Extract multiple meshes from DAE files and preserve materials in URDF before MuJoCo import

---

## 🎯 Problem Solved

**Before:** DAE files with multiple colored meshes were converted to a single STL file, losing all material/color information.

**After:** DAE files are extracted into multiple STL meshes during URDF preprocessing, with materials added directly to the URDF. MuJoCo then imports the URDF and preserves all materials automatically.

---

## 🏗️ Architecture

### Key Insight
Instead of trying to add materials to MJCF after MuJoCo import, we:
1. **Extract DAE meshes BEFORE preprocessing** the URDF
2. **Add materials to the URDF itself** using URDF `<material>` tags
3. **Let MuJoCo handle the rest** - it automatically preserves materials during import

### Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. Original XACRO/URDF                                          │
├─────────────────────────────────────────────────────────────────┤
│   <visual>                                                      │
│     <geometry>                                                  │
│       <mesh filename="package://.../base.dae"/>                 │
│     </geometry>                                                 │
│   </visual>                                                     │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. URDF Preprocessing (urdf_preprocess.py)                     │
├─────────────────────────────────────────────────────────────────┤
│  • Detect DAE file                                              │
│  • Extract to temp directory using extract_meshes_from_dae()   │
│  • Get mesh info with materials                                │
│                                                                 │
│  base.dae → [base_JointGrey.stl, base_Black.stl, base_Blue.stl]│
│              RGBA: [0.27, 0.27, 0.27, 1.0]                      │
│              RGBA: [0.03, 0.03, 0.03, 1.0]                      │
│              RGBA: [0.49, 0.67, 0.80, 1.0]                      │
│                                                                 │
│  • Replace single <visual> with multiple <visual> elements     │
│  • Add <material> tags with RGBA to each                       │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. Preprocessed URDF (saved to disk)                           │
├─────────────────────────────────────────────────────────────────┤
│   <visual>                                                      │
│     <origin rpy="0 0 3.14159" xyz="0 0 0"/>                     │
│     <geometry>                                                  │
│       <mesh filename="base_JointGrey.stl"/>                     │
│     </geometry>                                                 │
│     <material name="base_link_base_JointGrey_mat">              │
│       <color rgba="0.27 0.27 0.27 1.0"/>                        │
│     </material>                                                 │
│   </visual>                                                     │
│   <visual>                                                      │
│     <origin rpy="0 0 3.14159" xyz="0 0 0"/>                     │
│     <geometry>                                                  │
│       <mesh filename="base_Black.stl"/>                         │
│     </geometry>                                                 │
│     <material name="base_link_base_Black_mat">                  │
│       <color rgba="0.03 0.03 0.03 1.0"/>                        │
│     </material>                                                 │
│   </visual>                                                     │
│   <visual>                                                      │
│     <origin rpy="0 0 3.14159" xyz="0 0 0"/>                     │
│     <geometry>                                                  │
│       <mesh filename="base_Blue.stl"/>                          │
│     </geometry>                                                 │
│     <material name="base_link_base_Blue_mat">                   │
│       <color rgba="0.49 0.67 0.80 1.0"/>                        │
│     </material>                                                 │
│   </visual>                                                     │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. Mesh Copying (mesh_ops.py)                                  │
├─────────────────────────────────────────────────────────────────┤
│  • Copy extracted STL files from temp to output/assets/        │
│  • No material handling needed (already in URDF)               │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. MuJoCo Import                                                │
├─────────────────────────────────────────────────────────────────┤
│  model = mujoco.MjModel.from_xml_path(preprocessed_urdf_path)  │
│  • MuJoCo reads URDF with materials                            │
│  • Converts to internal format                                 │
│  • Preserves all material information                          │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│ 6. MJCF Output (via mj_saveLastXML)                            │
├─────────────────────────────────────────────────────────────────┤
│  <asset>                                                        │
│    <material name="base_link_base_JointGrey_mat"               │
│              rgba="0.27 0.27 0.27 1"/>                          │
│    <material name="base_link_base_Black_mat"                   │
│              rgba="0.03 0.03 0.03 1"/>                          │
│    <material name="base_link_base_Blue_mat"                    │
│              rgba="0.49 0.67 0.80 1"/>                          │
│    <mesh name="base_JointGrey" file="assets/base_JointGrey.stl"/>│
│    <mesh name="base_Black" file="assets/base_Black.stl"/>      │
│    <mesh name="base_Blue" file="assets/base_Blue.stl"/>        │
│  </asset>                                                       │
│                                                                 │
│  <worldbody>                                                    │
│    <body name="base_link">                                      │
│      <geom mesh="base_JointGrey"                                │
│            material="base_link_base_JointGrey_mat"              │
│            type="mesh"/>                                        │
│      <geom mesh="base_Black"                                    │
│            material="base_link_base_Black_mat"                  │
│            type="mesh"/>                                        │
│      <geom mesh="base_Blue"                                     │
│            material="base_link_base_Blue_mat"                   │
│            type="mesh"/>                                        │
│    </body>                                                      │
│  </worldbody>                                                   │
│                                                                 │
│  ✅ Materials automatically preserved by MuJoCo! ✅            │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📝 Implementation Details

### Modified Files

#### 1. `urdf_preprocess.py` - **Core Changes**

**New Imports:**
```python
import tempfile
from .tools.extract_dae_meshes import extract_meshes_from_dae
```

**Key Functionality:**
- Creates temporary directory for DAE extraction
- Detects DAE files in visual elements
- Extracts meshes using `extract_meshes_from_dae()`
- **Replaces single `<visual>` with multiple `<visual>` elements**
- **Adds `<material>` tags with RGBA colors to each visual**
- Tracks extracted mesh paths for copying

**Code Flow:**
```python
# For each visual element with DAE mesh:
1. Extract DAE → multiple STL + material info
2. Remove original <visual> element
3. For each extracted mesh:
   - Create new <visual> element
   - Add <geometry><mesh filename="..."/></geometry>
   - Add <material><color rgba="..."/></material>
   - Insert into link
4. Track mesh paths for copying
```

#### 2. `mesh_ops.py` - **Simplified**

**Changes:**
- Removed DAE extraction logic (now done in preprocessing)
- Simplified to just copy already-extracted STL files
- No material tracking needed (materials in URDF)

**Behavior:**
- STL/OBJ files → direct copy
- DAE files → should already be extracted (fallback to single-mesh if not)

#### 3. `converter.py` - **Minor Update**

**Changes:**
- Removed material post-processing call
- Added comment explaining materials are in URDF

**Flow unchanged:**
```python
1. Preprocess URDF (DAE extraction happens here)
2. Copy meshes
3. Import to MuJoCo (materials preserved automatically)
4. Post-process MJCF
5. Save
```

---

## 🎨 Material Handling

### URDF Material Format
```xml
<material name="link_meshname_mat">
  <color rgba="R G B A"/>
</material>
```

### Material Name Convention
- Format: `{link_name}_{mesh_name}_mat`
- Example: `base_link_JointGrey_mat`
- Unique per visual element

### RGBA Values
- Extracted directly from DAE file
- Preserved as-is (no conversion)
- Format: `"R G B A"` (space-separated floats 0.0-1.0)

---

## 📏 Unit Handling

DAE files often use different units (millimeters, centimeters, inches, etc.). The extraction tool automatically handles unit conversion:

### Auto-Detection
The tool reads the `<asset><unit>` element in the DAE file:
```xml
<asset>
  <unit name="millimeter" meter="0.001"/>
</asset>
```

### Conversion to SI Meters
| Unit | Scale Factor | Example |
|------|--------------|---------|
| Millimeter (mm) | 0.001 | 1000mm → 1m |
| Centimeter (cm) | 0.01 | 100cm → 1m |
| Inch (in) | 0.0254 | 39.37in → 1m |
| Meter (m) | 1.0 | 1m → 1m |

### Fallback Behavior
- If no unit info found in DAE → **defaults to 0.001 (assumes millimeters)**
- This is the most common case for CAD-exported DAE files

### Manual Override
You can explicitly specify a scale factor:
```python
# In urdf_preprocess.py or custom code
extract_meshes_from_dae(
    dae_file, 
    output_dir, 
    scale_factor=0.001  # Force mm to m conversion
)
```

### CLI Usage

#### Standalone Tool
```bash
# Extract multiple meshes with colors (default, recommended for URDF/MJCF)
python -m urdf2mjcf.tools.extract_dae_meshes model.dae -o output/

# Legacy mode: combine all meshes into single file (backward compatible)
python -m urdf2mjcf.tools.extract_dae_meshes model.dae -o output/ --combine

# Manual scale override
python -m urdf2mjcf.tools.extract_dae_meshes model.dae -o output/ --scale 0.001

# Combined with scale override
python -m urdf2mjcf.tools.extract_dae_meshes model.dae -o output/ --combine --scale 0.001
```

#### Main urdf2mjcf Tool
```bash
# Default: combine DAE meshes into single STL (backward compatible)
urdf2mjcf robot.urdf.xacro -o output/

# Extract separate meshes with materials (recommended for colored models)
urdf2mjcf robot.urdf.xacro -o output/ --separate-dae-meshes

# Append mesh type tags to filenames for easy identification
urdf2mjcf robot.urdf.xacro -o output/ --append-mesh-type
# Result: base_visual.stl, wrist_collision.stl, etc.

# Combined example: separate DAE meshes + append type tags
urdf2mjcf robot.urdf.xacro -o output/ --separate-dae-meshes --append-mesh-type
# Result: base_JointGrey_visual.stl, wrist2_collision.stl, etc.
```

### Extraction Modes

| Mode | Flag | Behavior | Use Case |
|------|------|----------|----------|
| **Combined (default)** | None | All DAE meshes → single STL | Backward compatibility, simple models |
| **Separate** | `--separate-dae-meshes` | Each mesh/material → separate STL + color info | URDF/MJCF conversion with materials |

### Filename Options

| Option | Flag | Behavior | Example Output |
|--------|------|----------|----------------|
| **Default** | None | Original filename preserved | `base.stl`, `wrist2.stl` |
| **Append Type** | `--append-mesh-type` | Add `_visual` or `_collision` suffix | `base_visual.stl`, `wrist2_collision.stl` |

---

## ✅ Benefits

1. **No Duplicate Materials** - MuJoCo automatically handles material deduplication
2. **Proper Geom Assignment** - MuJoCo assigns materials during import
3. **Clean Architecture** - Materials handled at the right stage (URDF, not MJCF post-processing)
4. **Better Compatibility** - Works with MuJoCo's standard URDF import
5. **Debuggable** - Can inspect preprocessed URDF to see materials
6. **Backward Compatible** - `--combine` flag preserves old single-mesh behavior
6. **Correct Scale** - Automatic unit conversion ensures meshes have proper size

---

## 🔧 Dependencies

**Required:**
- `pycollada` - Parse DAE/COLLADA files
- `trimesh` - Mesh manipulation and export

```bash
pip install pycollada trimesh
```

**Fallback Behavior:**
- If dependencies missing → single-mesh DAE conversion (legacy behavior)
- No materials preserved in fallback mode

---

## 🧪 Testing

### Expected Behavior

**Input:** URDF with DAE containing 3 colored meshes

**Preprocessed URDF:**
- 3 `<visual>` elements
- Each with unique `<material>` tag
- RGBA values from DAE

**Final MJCF:**
- 3 materials in `<asset>`
- 3 meshes in `<asset>`
- 3 geoms with materials assigned

### Validation Commands

```bash
# 1. Check preprocessed URDF (if --save-preprocessed used)
grep -A5 "<material" output/robot.preprocessed.urdf

# 2. Check final MJCF materials
grep "<material" output/robot.xml

# 3. Check geom material assignments
grep 'material=' output/robot.xml

# 4. Visualize
simulate output/robot.xml
```

---

## 🐛 Known Issues & Limitations

### Current Limitations

1. **Collision meshes not expanded** - Only visual meshes get DAE expansion
2. **Temporary directory not cleaned** - Extract creates temp dir that persists
3. **Material name collisions possible** - If link names + mesh names collide
4. **No texture support** - Only RGBA colors, not image textures

### Edge Cases

1. **DAE with no materials** - Meshes extracted, no material tags added
2. **DAE extraction fails** - Falls back to single-mesh conversion
3. **Multiple DAE files per link** - Each expanded independently

---

## 🚀 Future Enhancements

- [ ] Clean up temporary extraction directory
- [ ] Support collision mesh expansion
- [ ] Texture map preservation
- [ ] Material deduplication at URDF level
- [ ] Better material naming strategy
- [ ] Batch DAE extraction for performance

---

## 📚 Example

### Input XACRO
```xml
<link name="base_link">
  <visual>
    <origin rpy="0 0 3.14159" xyz="0 0 0"/>
    <geometry>
      <mesh filename="package://ur_description/meshes/ur3e/visual/base.dae"/>
    </geometry>
  </visual>
</link>
```

### Preprocessed URDF (Generated)
```xml
<link name="base_link">
  <visual>
    <origin rpy="0 0 3.14159" xyz="0 0 0"/>
    <geometry>
      <mesh filename="base_JointGrey.stl"/>
    </geometry>
    <material name="base_link_base_JointGrey_mat">
      <color rgba="0.2784314 0.2784314 0.2784314 1.0"/>
    </material>
  </visual>
  <visual>
    <origin rpy="0 0 3.14159" xyz="0 0 0"/>
    <geometry>
      <mesh filename="base_Black.stl"/>
    </geometry>
    <material name="base_link_base_Black_mat">
      <color rgba="0.03310248 0.03310248 0.03310248 1.0"/>
    </material>
  </visual>
  <visual>
    <origin rpy="0 0 3.14159" xyz="0 0 0"/>
    <geometry>
      <mesh filename="base_URBlue.stl"/>
    </geometry>
    <material name="base_link_base_URBlue_mat">
      <color rgba="0.4901961 0.6784314 0.8 1.0"/>
    </material>
  </visual>
</link>
```

### Final MJCF (Auto-generated by MuJoCo)
```xml
<mujoco>
  <asset>
    <material name="base_link_base_JointGrey_mat" rgba="0.2784314 0.2784314 0.2784314 1"/>
    <material name="base_link_base_Black_mat" rgba="0.03310248 0.03310248 0.03310248 1"/>
    <material name="base_link_base_URBlue_mat" rgba="0.4901961 0.6784314 0.8 1"/>
    <mesh name="base_JointGrey" file="assets/base_JointGrey.stl"/>
    <mesh name="base_Black" file="assets/base_Black.stl"/>
    <mesh name="base_URBlue" file="assets/base_URBlue.stl"/>
  </asset>
  
  <worldbody>
    <body name="base_link">
      <geom mesh="base_JointGrey" material="base_link_base_JointGrey_mat" type="mesh"/>
      <geom mesh="base_Black" material="base_link_base_Black_mat" type="mesh"/>
      <geom mesh="base_URBlue" material="base_link_base_URBlue_mat" type="mesh"/>
    </body>
  </worldbody>
</mujoco>
```

---

**Status:** ✅ **Implemented and Ready for Testing**

**Key Advantage:** By adding materials to URDF before MuJoCo import, we let MuJoCo handle all the complexity of material preservation, deduplication, and geom assignment. Much cleaner than post-processing!
