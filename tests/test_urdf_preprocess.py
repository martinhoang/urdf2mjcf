from urdf2mjcf.urdf_preprocess import preprocess_urdf


def _write_test_urdf(tmp_path):
    mesh_path = tmp_path / "mesh.stl"
    mesh_path.write_bytes(b"solid mesh\nendsolid mesh\n")

    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(
        f"""\
<robot name="test">
  <mujoco>
    <compiler meshdir="../../source/meshes" angle="radian"/>
  </mujoco>
  <link name="base">
    <visual>
      <geometry>
        <mesh filename="{mesh_path}"/>
      </geometry>
    </visual>
  </link>
</robot>
"""
    )
    return urdf_path


def _compiler_from_preprocessed(tmp_path, compiler_options=None):
    urdf_path = _write_test_urdf(tmp_path)
    tree, *_ = preprocess_urdf(
        str(urdf_path),
        compiler_options=compiler_options,
        default_mesh_dir="assets/",
    )
    return tree.getroot().find("./mujoco/compiler")


def test_preprocess_replaces_source_meshdir_with_output_meshdir(tmp_path):
    compiler = _compiler_from_preprocessed(tmp_path)

    assert compiler is not None
    assert compiler.get("meshdir") == "assets/"
    assert compiler.get("angle") == "radian"


def test_explicit_compiler_option_can_override_output_meshdir(tmp_path):
    compiler = _compiler_from_preprocessed(
        tmp_path, compiler_options=["meshdir=custom-assets"]
    )

    assert compiler is not None
    assert compiler.get("meshdir") == "custom-assets"
