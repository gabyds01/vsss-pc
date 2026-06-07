import sys
import argparse
from pathlib import Path

# Add root folder to sys.path to enable absolute package imports
root_dir = str(Path(__file__).parent.parent)
if root_dir not in sys.path:
    sys.path.insert(0, root_dir)

from vsss.trajectory.shapes import (
    generate_line,
    generate_square,
    generate_circle,
    generate_s_curve,
    generate_spline,
)
from vsss.analysis.plot_trajectory import plot_reference_trajectory


def main():
    parser = argparse.ArgumentParser(
        description="Generate and plot standard VSSS trajectories."
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Interactively display the plots using the matplotlib window.",
    )
    args = parser.parse_args()

    print("Generating trajectories and saving plots to 'graphs/' directory...")

    # 1. Straight Line Path
    print("Generating Line Path...")
    line_path = generate_line(start=(-0.6, -0.4), end=(0.6, 0.4))
    plot_reference_trajectory(
        line_path,
        title="VSSS Straight Line Trajectory Test",
        save_path="graphs/line_trajectory.png",
        show=args.show,
    )

    # 2. Square Path
    print("Generating Square Path...")
    square_path = generate_square(side=0.6, center=(0.0, 0.0))
    plot_reference_trajectory(
        square_path,
        title="VSSS Square Trajectory Test",
        save_path="graphs/square_trajectory.png",
        show=args.show,
    )

    # 3. Circular Path
    print("Generating Circle Path...")
    circle_path = generate_circle(radius=0.4, center=(0.0, 0.0))
    plot_reference_trajectory(
        circle_path,
        title="VSSS Circular Trajectory Test",
        save_path="graphs/circle_trajectory.png",
        show=args.show,
    )

    # 4. S-Curve Path
    print("Generating S-Curve Path...")
    s_curve_path = generate_s_curve(start_x=-0.6, end_x=0.6, amplitude=0.3)
    plot_reference_trajectory(
        s_curve_path,
        title="VSSS S-Curve Trajectory Test",
        save_path="graphs/s_curve_trajectory.png",
        show=args.show,
    )

    # 5. Spline Path
    print("Generating Spline Path...")
    control_points = [
        (-0.6, 0.0),
        (-0.3, 0.15),
        (0.0, -0.15),
        (0.3, 0.15),
        (0.6, 0.0),
    ]
    spline_path = generate_spline(control_points)
    plot_reference_trajectory(
        spline_path,
        title="VSSS Spline Trajectory Test",
        save_path="graphs/spline_trajectory.png",
        show=args.show,
    )

    print("\nAll plots generated successfully inside 'graphs/' folder!")


if __name__ == "__main__":
    main()
