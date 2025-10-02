#! /usr/bin/env python3

import os
import pymeshlab as ml
import argparse

def simplify_mesh(input_file, output_file, target_reduction, translate=None, scale_factor=None):
    """Simplifies a single mesh file."""
    print(f"Processing: {input_file}")
    try:
        ms = ml.MeshSet()
        ms.load_new_mesh(input_file)
        
        if translate:
            print(f"Translating mesh by: {translate}")
            ms.apply_filter(
                "compute_matrix_from_translation",
                axisx=translate[0],
                axisy=translate[1],
                axisz=translate[2],
                freeze=True,
            )

        if scale_factor:
            print(f"Scaling mesh by factor: {scale_factor}")
            ms.apply_filter(
                "compute_matrix_from_scaling_or_normalization",
                axisx=scale_factor,
                axisy=scale_factor,
                axisz=scale_factor,
                uniformflag=True,
                freeze=True,
            )

        print(f"Simplifying mesh with reduction: {target_reduction}")
        # The target reduction is the percentage of faces to remove.
        # Pymeshlab's filter uses the target percentage of faces to *keep*.
        target_percentage = 1.0 - target_reduction

        ms.apply_filter(
            "meshing_decimation_quadric_edge_collapse",
            targetperc=target_percentage,
            preservenormal=True,
        )
        # Ensure output directory exists
        output_dir = os.path.dirname(output_file)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        ms.save_current_mesh(output_file)
        print(f"Saved processed mesh to: {output_file}")
    except ml.PyMeshLabException as e:
        print(f"PyMeshLab error processing {input_file}: {e}:\n{ml.print_filter_list()}")
        raise 
    except Exception as e:
        print(f"Failed to process {input_file}: {e}")
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
        "--reduction",
        type=float,
        default=0.5,
        help="Target reduction ratio (0 to 1), e.g. 0.5 = 50%% fewer faces.",
    )
    parser.add_argument(
        "--translation",
        type=float,
        nargs=3,
        default=None,
        help="The translation [x y z] to apply to the mesh before simplification.",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=None,
        help="Scale factor to apply to the mesh before simplification.",
    )
    args = parser.parse_args()

    input_path = args.input_path
    output_path = args.output_path
    reduction = args.reduction
    translate = args.translation
    scale_factor = args.scale

    if not os.path.exists(input_path):
        print(f"Error: Input path '{input_path}' does not exist.")
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
                simplify_mesh(input_file, output_file, reduction, translate, scale_factor)
        print("All STL files in directory processed.")
    elif os.path.isfile(input_path):
        # Input is a file
        if not input_path.lower().endswith(".stl"):
            print(f"Error: Input file '{input_path}' is not an STL file.")
            return

        final_output_path = output_path
        if final_output_path is None:
            base, ext = os.path.splitext(input_path)
            final_output_path = f"{base}_simplified{ext}"
        elif os.path.isdir(output_path):
            # If output is a directory, save with same filename inside it
            final_output_path = os.path.join(output_path, os.path.basename(input_path))

        simplify_mesh(input_path, final_output_path, reduction, translate, scale_factor)
        print("STL file processed.")
    else:
        print(f"Error: Input path '{input_path}' is not a valid file or directory.")


if __name__ == "__main__":
    main()
