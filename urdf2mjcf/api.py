"""Public Python API for URDF/Xacro to MJCF conversion."""

from collections.abc import Mapping
from os import PathLike
from typing import Any

from _utils import parse_actuator_gains, set_log_level

from .cli import ConfigLoader, build_argument_parser


def build_conversion_args(
    input_path: str | PathLike[str] | None = None,
    *,
    config_file: str | PathLike[str] | None = None,
    options: Mapping[str, Any] | None = None,
    **overrides: Any,
):
    """Build converter arguments from defaults, JSON config, and inline options.

    Precedence is: keyword overrides, ``options`` mapping, JSON config, defaults.
    Inline option names use the argparse destination names, such as
    ``floating_base``, ``default_actuator_gains``, or
    ``default_joint_damping``.
    """
    parser = build_argument_parser()
    args = parser.parse_args([])

    inline_options = dict(options or {})
    inline_options.update(overrides)
    inline_options = {
        key.replace("-", "_"): value for key, value in inline_options.items()
    }

    if input_path is not None:
        input_path = str(input_path)
    if config_file is not None:
        config_file = str(config_file)

    if input_path and input_path.endswith(".json"):
        if config_file is not None:
            raise ValueError(
                "Specify a JSON file as input_path or config_file, not both."
            )
        config_file = input_path
        input_path = None

    if config_file:
        loaded_args = ConfigLoader(parser).load_config(
            config_file, args, strict=True
        )
        if loaded_args is not None:
            args = loaded_args
    args.config_file = config_file

    valid_options = {
        action.dest
        for action in parser._actions
        if action.dest not in {"help", "version", "input", "config_file"}
    }
    unknown_options = sorted(set(inline_options) - valid_options)
    if unknown_options:
        names = ", ".join(unknown_options)
        raise TypeError(f"Unknown conversion option(s): {names}")

    for key, value in inline_options.items():
        if isinstance(value, PathLike):
            value = str(value)
        if key == "default_actuator_gains":
            value = parse_actuator_gains(value)
        setattr(args, key, value)

    if input_path is not None:
        args.input = input_path

    if args.input is None:
        raise ValueError(
            "Input URDF or xacro file must be specified via input_path "
            "or the config file."
        )

    args.input = str(args.input)
    input_ext = args.input.rsplit(".", 1)[-1].lower()
    if input_ext not in {"urdf", "xacro"}:
        raise ValueError(
            "Input file must be a URDF (.urdf) or xacro (.xacro) file, "
            f"not .{input_ext}"
        )

    if args.debug:
        args.log_level = "DEBUG"
    set_log_level(args.log_level, args.traceback)
    return args


def convert(
    input_path: str | PathLike[str] | None = None,
    *,
    config_file: str | PathLike[str] | None = None,
    options: Mapping[str, Any] | None = None,
    **overrides: Any,
):
    """Convert a URDF/Xacro file and return the generated MJCF path.

    Args:
        input_path: URDF/Xacro path, or a JSON config path for auto-detection.
        config_file: Optional JSON configuration path.
        options: Optional mapping of inline conversion options.
        **overrides: Inline conversion options. These have highest precedence.
            Joint defaults can be set with ``default_joint_stiffness``,
            ``default_joint_damping``, and ``default_joint_friction``.
    """
    args = build_conversion_args(
        input_path,
        config_file=config_file,
        options=options,
        **overrides,
    )

    # Keep the converter import lazy so callers can inspect/build options without
    # loading MuJoCo and the mesh-processing stack.
    from .converter import URDFToMJCFConverter

    return URDFToMJCFConverter(args).convert()


convert_urdf = convert
