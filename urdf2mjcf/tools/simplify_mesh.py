#! /usr/bin/env python3

import os
import pymeshlab as ml
import argparse
from _utils import print_debug, print_error, print_warning, print_confirm


def simplify_mesh(
    input_file,
    output_file,
    target_reduction=None,
    target_faces=None,
    translate=None,
    scale_factor=None,
):
    """Simplifies a single mesh file.

    Args:
        input_file: Path to input mesh
        output_file: Path to output mesh
        target_reduction: Reduction ratio (0.0-1.0), percentage of faces to remove
        target_faces: Target number of faces (alternative to target_reduction)
        translate: Translation vector [x, y, z]
        scale_factor: Uniform scale factor
    """
    print_debug(f"Processing: {input_file}")
    try:
        ms = ml.MeshSet()
        ms.load_new_mesh(input_file)

        if translate:
            print_debug(f"Translating mesh by: {translate}")
            ms.apply_filter(
                "compute_matrix_from_translation",
                axisx=translate[0],
                axisy=translate[1],
                axisz=translate[2],
                freeze=True,
            )

        if scale_factor:
            print_debug(f"Scaling mesh by factor: {scale_factor}")
            ms.apply_filter(
                "compute_matrix_from_scaling_or_normalization",
                axisx=scale_factor,
                axisy=scale_factor,
                axisz=scale_factor,
                uniformflag=True,
                freeze=True,
            )

        # Determine simplification parameters
        current_faces = ms.current_mesh().face_number()
        print_debug(f"Current mesh '{input_file}' has {current_faces} faces")

        if target_faces is not None:
            # User specified target number of faces
            if target_faces >= current_faces:
                print_debug(
                    f"Target faces ({target_faces}) >= current faces ({current_faces}), skipping simplification"
                )
                target_percentage = 1.0
            else:
                target_percentage = target_faces / current_faces
                print_debug(
                    f"Simplifying mesh to {target_faces} faces (keeping {target_percentage * 100:.1f}% of faces)"
                )
        elif target_reduction is not None:
            # User specified reduction ratio (percentage to remove)
            target_percentage = 1.0 - target_reduction
            target_face_count = int(current_faces * target_percentage)
            print_debug(
                f"Simplifying mesh with reduction: {target_reduction} (keeping {target_percentage * 100:.1f}%, target ~{target_face_count} faces)"
            )
        else:
            # Default: no reduction
            print_warning("No reduction specified, keeping original mesh")
            target_percentage = 1.0

        if target_percentage < 1.0:
            ms.apply_filter(
                "meshing_decimation_quadric_edge_collapse",
                targetperc=target_percentage,
                preservenormal=True,
            )
            final_faces = ms.current_mesh().face_number()
            print_debug(f"Simplified mesh has {final_faces} faces")
        # Ensure output directory exists
        output_dir = os.path.dirname(output_file)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        ms.save_current_mesh(output_file)
        print_debug(f"Saved processed mesh to: {output_file}")
    except ml.PyMeshLabException as e:
        print_error(
            f"PyMeshLab error processing {input_file}: {e}:\n{ml.print_filter_list()}"
        )
        raise
    except Exception as e:
        print_error(f"Failed to process {input_file}: {e}")
        raise


def main():
    parser = argparse.ArgumentParser(
        description="Simplify and scale STL files or a directory of STL files using pymeshlab."
    )
    parser.add_argument("input_path", help="Path to the input STL file or folder.")
    parser.add_argument(
        "output_path",
        nargs="?",
        default=None,
        help="Path to the output file or folder. If not provided, a new file/folder with '_simplified' suffix will be created.",
    )
    parser.add_argument(
        "-r",
        "--reduction",
        type=float,
        default=None,
        help="Target reduction ratio (0 to 1), e.g. 0.5 = 50%% fewer faces. Mutually exclusive with --target-faces.",
    )
    parser.add_argument(
        "-tf",
        "--target-faces",
        type=int,
        default=None,
        help="Target number of faces to simplify to, e.g. 200000. Mutually exclusive with --reduction.",
    )
    parser.add_argument(
        "-t",
        "--translation",
        type=float,
        nargs=3,
        default=None,
        help="The translation [x y z] to apply to the mesh before simplification.",
    )
    parser.add_argument(
        "-s",
        "--scale",
        type=float,
        default=None,
        help="Scale factor to apply to the mesh before simplification.",
    )
    args = parser.parse_args()

    input_path = args.input_path
    output_path = args.output_path
    reduction = args.reduction
    target_faces = args.target_faces
    translate = args.translation
    scale_factor = args.scale

    # Validate that only one simplification method is specified
    if reduction is not None and target_faces is not None:
        print_error(
            "Error: Cannot specify both --reduction and --target-faces. Choose one."
        )
        return

    # Default to 50% reduction if neither is specified
    if reduction is None and target_faces is None:
        reduction = 0.5
        print_warning(
            f"No simplification method specified, using default reduction of {reduction} (50%)"
        )

    if not os.path.exists(input_path):
        print_error(f"Error: Input path '{input_path}' does not exist.")
        return

    if os.path.isdir(input_path):
        # Input is a directory, output should be a directory
        if output_path is None:
            output_path = input_path.rstrip("/\\") + "_simplified"
        os.makedirs(output_path, exist_ok=True)
        for filename in os.listdir(input_path):
            if filename.lower().endswith(".stl"):
                input_file = os.path.join(input_path, filename)
                output_file = os.path.join(output_path, filename)
                simplify_mesh(
                    input_file,
                    output_file,
                    reduction,
                    target_faces,
                    translate,
                    scale_factor,
                )
        print_debug("All STL files in directory processed.")
    elif os.path.isfile(input_path):
        # Input is a file
        if not input_path.lower().endswith(".stl"):
            print_error(f"Error: Input file '{input_path}' is not an STL file.")
            return

        final_output_path = output_path
        if final_output_path is None:
            base, ext = os.path.splitext(input_path)
            final_output_path = f"{base}_simplified{ext}"
        elif os.path.isdir(output_path):
            # If output is a directory, save with same filename inside it
            final_output_path = os.path.join(output_path, os.path.basename(input_path))

        simplify_mesh(
            input_path,
            final_output_path,
            reduction,
            target_faces,
            translate,
            scale_factor,
        )
        print_confirm("STL file processed.")
    else:
        print_error(
            f"Error: Input path '{input_path}' is not a valid file or directory."
        )


if __name__ == "__main__":
    main()
