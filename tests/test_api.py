import json
from argparse import Namespace

import pytest

from urdf2mjcf import build_conversion_args, convert


def test_build_conversion_args_uses_defaults_and_inline_options():
    args = build_conversion_args(
        "robot.urdf",
        floating_base=True,
        default_actuator_gains="kp=800,kv=5",
    )

    assert args.input == "robot.urdf"
    assert args.floating_base is True
    assert args.add_floor is False
    assert args.default_actuator_gains == {"kp": 800.0, "kv": 5.0}


def test_build_conversion_args_normalizes_path_options(tmp_path):
    args = build_conversion_args(tmp_path / "robot.urdf", output=tmp_path)

    assert args.input == str(tmp_path / "robot.urdf")
    assert args.output == str(tmp_path)


def test_inline_options_override_config(tmp_path):
    config_path = tmp_path / "config.json"
    config_path.write_text(
        json.dumps(
            {
                "input": "from-config.xacro",
                "floating_base": False,
                "add_floor": True,
            }
        )
    )

    args = build_conversion_args(
        "inline.urdf",
        config_file=config_path,
        floating_base=True,
    )

    assert args.input == "inline.urdf"
    assert args.floating_base is True
    assert args.add_floor is True


def test_json_input_is_auto_detected_as_config(tmp_path):
    config_path = tmp_path / "config.json"
    config_path.write_text(json.dumps({"input": "robot.xacro"}))

    args = build_conversion_args(config_path)

    assert args.input == "robot.xacro"
    assert args.config_file == str(config_path)


def test_unknown_inline_option_is_rejected():
    with pytest.raises(TypeError, match="Unknown conversion option"):
        build_conversion_args("robot.urdf", does_not_exist=True)


def test_convert_calls_converter_with_resolved_args(monkeypatch):
    received = {}

    class FakeConverter:
        def __init__(self, args: Namespace):
            received["args"] = args

        def convert(self):
            return "/tmp/robot/robot.xml"

    monkeypatch.setattr(
        "urdf2mjcf.converter.URDFToMJCFConverter", FakeConverter
    )

    result = convert("robot.urdf", add_floor=True)

    assert result == "/tmp/robot/robot.xml"
    assert received["args"].add_floor is True
