import xml.etree.ElementTree as ET

from urdf2mjcf.mjcf_postprocess import (
    post_process_add_actuators,
    post_process_add_mimic_equalities,
    post_process_make_base_floating,
    post_process_set_base_height,
    post_process_set_default_joint_properties,
)


def test_set_default_joint_properties_creates_main_default():
    root = ET.fromstring("<mujoco><worldbody /></mujoco>")

    post_process_set_default_joint_properties(
        root,
        stiffness=0.1,
        damping=1.0,
        friction=0.01,
    )

    joint = root.find("./default/joint")
    assert joint is not None
    assert joint.attrib == {
        "stiffness": "0.1",
        "damping": "1.0",
        "frictionloss": "0.01",
    }


def test_set_default_joint_properties_preserves_existing_attributes():
    root = ET.fromstring(
        '<mujoco><default><joint armature="0.2" damping="0.5" /></default></mujoco>'
    )

    post_process_set_default_joint_properties(root, damping=1.0)

    joint = root.find("./default/joint")
    assert joint is not None
    assert joint.attrib == {"armature": "0.2", "damping": "1.0"}


def test_add_mimic_equalities_uses_urdf_multiplier_and_offset():
    root = ET.fromstring(
        """
        <mujoco>
          <worldbody>
            <body>
              <joint name="source" type="hinge" />
              <body>
                <joint name="follower" type="hinge" />
              </body>
            </body>
          </worldbody>
          <actuator />
        </mujoco>
        """
    )

    post_process_add_mimic_equalities(
        root,
        {
            "follower": {
                "joint": "source",
                "multiplier": "-2.0",
                "offset": "0.25",
            }
        },
    )

    equality_joint = root.find("./equality/joint")
    assert equality_joint is not None
    assert equality_joint.attrib == {
        "joint1": "follower",
        "joint2": "source",
        "polycoef": "0.25 -2.0 0 0 0",
    }
    assert list(root).index(root.find("equality")) < list(root).index(
        root.find("actuator")
    )


def test_equality_constrained_mimic_joint_does_not_get_an_actuator():
    root = ET.fromstring(
        """
        <mujoco>
          <worldbody>
            <body>
              <joint name="source" type="hinge" />
              <body>
                <joint name="follower" type="hinge" />
              </body>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    mimic_joints = {
        "follower": {
            "joint": "source",
            "multiplier": "1.0",
            "offset": "0.0",
        }
    }

    post_process_add_mimic_equalities(root, mimic_joints)
    post_process_add_actuators(
        root,
        default_ros2_control_instance="ros2_control",
        mimic_joints=mimic_joints,
        default_actuator_gains={"kp": 100.0},
        ros2c_joint_map={
            "source": {"position"},
            "follower": {"position"},
        },
        skip_mimic_actuators=True,
    )

    actuator_joints = {
        actuator.get("joint") for actuator in root.findall("./actuator/*")
    }
    assert actuator_joints == {"source"}
    assert list(root).index(root.find("equality")) < list(root).index(
        root.find("actuator")
    )


def test_set_base_height_mounts_fixed_base_above_floor():
    root = ET.fromstring(
        '<mujoco><worldbody><body name="base" pos="1 2 0" /></worldbody></mujoco>'
    )

    post_process_set_base_height(root, 0.4)

    assert root.find("./worldbody/body").get("pos") == "1 2 0.4"
    assert root.find("./worldbody/body/freejoint") is None


def test_make_base_floating_applies_height_to_existing_position():
    root = ET.fromstring(
        '<mujoco><worldbody><body name="base" pos="1 2 3" /></worldbody></mujoco>'
    )

    post_process_make_base_floating(root, 0.5)

    base = root.find("./worldbody/body")
    assert base.get("pos") == "1 2 0.5"
    assert base.find("freejoint") is not None
