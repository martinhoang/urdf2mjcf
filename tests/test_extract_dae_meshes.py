import numpy as np
import pytest
import trimesh

from urdf2mjcf.tools.extract_dae_meshes import (
    extract_meshes_from_dae,
    _resolve_dae_up_axis,
    _up_axis_rotation_matrix,
)


def _transform_direction(matrix, direction):
    homogeneous = np.array([*direction, 0.0])
    return (matrix @ homogeneous)[:3]


def test_auto_uses_declared_y_up_axis():
    assert _resolve_dae_up_axis("Y_UP", "auto") == "Y_UP"


def test_auto_uses_collada_y_up_default_when_metadata_is_missing():
    assert _resolve_dae_up_axis(None, "auto") == "Y_UP"


def test_auto_does_not_double_rotate_scene_bound_geometry():
    assert (
        _resolve_dae_up_axis(
            "Y_UP", "auto", scene_geometry_available=True
        )
        == "Z_UP"
    )


def test_forced_axis_overrides_dae_metadata():
    assert _resolve_dae_up_axis("Z_UP", "y") == "Y_UP"
    assert _resolve_dae_up_axis("Y_UP", "z") == "Z_UP"


def test_y_up_rotation_maps_positive_y_to_positive_z():
    matrix = _up_axis_rotation_matrix("Y_UP")
    np.testing.assert_allclose(
        _transform_direction(matrix, [0.0, 1.0, 0.0]),
        [0.0, 0.0, 1.0],
        atol=1e-12,
    )


def test_x_up_rotation_maps_positive_x_to_positive_z():
    matrix = _up_axis_rotation_matrix("X_UP")
    np.testing.assert_allclose(
        _transform_direction(matrix, [1.0, 0.0, 0.0]),
        [0.0, 0.0, 1.0],
        atol=1e-12,
    )


def test_z_up_needs_no_rotation():
    assert _up_axis_rotation_matrix("Z_UP") is None


def test_invalid_axis_is_rejected():
    with pytest.raises(ValueError, match="Invalid DAE up-axis"):
        _resolve_dae_up_axis("Y_UP", "sideways")


def test_auto_applies_collada_scene_node_transform(tmp_path):
    dae_path = tmp_path / "scene_transform.dae"
    dae_path.write_text(
        """\
<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <unit name="meter" meter="1"/>
    <up_axis>Y_UP</up_axis>
  </asset>
  <library_geometries>
    <geometry id="mesh" name="mesh">
      <mesh>
        <source id="positions">
          <float_array id="positions-array" count="9">0 0 0 1 0 0 0 2 0</float_array>
          <technique_common>
            <accessor source="#positions-array" count="3" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="vertices">
          <input semantic="POSITION" source="#positions"/>
        </vertices>
        <triangles count="1">
          <input semantic="VERTEX" source="#vertices" offset="0"/>
          <p>0 1 2</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="scene">
      <node id="node">
        <rotate>1 0 0 90</rotate>
        <instance_geometry url="#mesh"/>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#scene"/>
  </scene>
</COLLADA>
"""
    )

    output_dir = tmp_path / "output"
    success, total, mesh_info = extract_meshes_from_dae(
        str(dae_path),
        str(output_dir),
        separate_meshes=True,
        dae_up_axis="auto",
    )

    assert (success, total) == (1, 1)
    mesh_data = next(iter(mesh_info.values()))
    mesh = trimesh.load(output_dir / mesh_data["file"], process=False)
    np.testing.assert_allclose(mesh.extents, [1.0, 0.0, 2.0], atol=1e-6)
    assert mesh_data["scene_transforms_applied"] is True
    assert mesh_data["axis_rotation_applied"] is False
