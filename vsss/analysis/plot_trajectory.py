import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from vsss.trajectory.path import Path


def draw_vsss_field(ax):
    """Draw the VSSS field lines and features on the provided matplotlib axes."""
    # Set background color to green (sleek dark pitch green)
    ax.set_facecolor("#1e4620")

    # Play area boundary lines (150cm x 130cm, centered at 0,0)
    boundary = patches.Rectangle(
        (-0.75, -0.65),
        1.50,
        1.30,
        fill=False,
        edgecolor="white",
        linewidth=2,
        zorder=2,
    )
    ax.add_patch(boundary)

    # Center line
    ax.plot([0, 0], [-0.65, 0.65], color="white", linewidth=2, zorder=2)

    # Center circle (radius 20cm = 0.2m)
    center_circle = patches.Circle(
        (0, 0), 0.20, fill=False, edgecolor="white", linewidth=2, zorder=2
    )
    ax.add_patch(center_circle)

    # Center point (0.5cm radius reference point)
    center_point = patches.Circle((0, 0), 0.005, color="white", zorder=3)
    ax.add_patch(center_point)

    # Goalkeeper areas (70cm width x 15cm depth)
    # Left area (x = -0.75 to -0.60, y = -0.35 to 0.35)
    left_gk = patches.Rectangle(
        (-0.75, -0.35),
        0.15,
        0.70,
        fill=False,
        edgecolor="white",
        linewidth=2,
        zorder=2,
    )
    ax.add_patch(left_gk)

    # Right area (x = 0.60 to 0.75, y = -0.35 to 0.35)
    right_gk = patches.Rectangle(
        (0.60, -0.35),
        0.15,
        0.70,
        fill=False,
        edgecolor="white",
        linewidth=2,
        zorder=2,
    )
    ax.add_patch(right_gk)

    # Goals (10cm depth x 40cm width, centered at y = 0, extending outwards)
    # Left goal (x = -0.85 to -0.75, y = -0.20 to 0.20)
    left_goal = patches.Rectangle(
        (-0.85, -0.20),
        0.10,
        0.40,
        fill=False,
        edgecolor="white",
        linewidth=2,
        zorder=2,
    )
    ax.add_patch(left_goal)

    # Right goal (x = 0.75 to 0.85, y = -0.20 to 0.20)
    right_goal = patches.Rectangle(
        (0.75, -0.20),
        0.10,
        0.40,
        fill=False,
        edgecolor="white",
        linewidth=2,
        zorder=2,
    )
    ax.add_patch(right_goal)

    # Corner triangles (7cm x 7cm)
    # Bottom-left
    ax.plot([-0.75, -0.68], [-0.58, -0.65], color="white", linewidth=2, zorder=2)
    # Top-left
    ax.plot([-0.75, -0.68], [0.58, 0.65], color="white", linewidth=2, zorder=2)
    # Bottom-right
    ax.plot([0.75, 0.68], [-0.58, -0.65], color="white", linewidth=2, zorder=2)
    # Top-right
    ax.plot([0.75, 0.68], [0.58, 0.65], color="white", linewidth=2, zorder=2)

    # Set limits with some padding
    ax.set_xlim(-0.95, 0.95)
    ax.set_ylim(-0.80, 0.80)
    ax.set_aspect("equal")
    ax.set_xlabel("X (meters)", color="white")
    ax.set_ylabel("Y (meters)", color="white")
    ax.tick_params(colors="white")
    ax.grid(True, linestyle="--", alpha=0.3)


def plot_reference_trajectory(
    path: Path, title: str = "Trajectory", save_path: str = None, show: bool = True
):
    """Plot the reference trajectory on the VSSS field and optionally save it."""
    fig, ax = plt.subplots(figsize=(10, 8), facecolor="#0e1f11")
    draw_vsss_field(ax)

    # Plot raw waypoints in orange-red
    ax.scatter(
        path.raw_waypoints[:, 0],
        path.raw_waypoints[:, 1],
        color="#ff4500",
        s=40,
        zorder=5,
        label="Waypoints",
    )

    # Plot interpolated path in vibrant cyan
    ax.plot(
        path.x,
        path.y,
        color="#00ffff",
        linewidth=2.5,
        linestyle="--",
        zorder=4,
        label="Interpolated Path",
    )

    # Add orientation arrows at intervals (e.g., every 15-20 points along the path)
    step = max(1, len(path.s) // 25)
    ax.quiver(
        path.x[::step],
        path.y[::step],
        np.cos(path.theta[::step]),
        np.sin(path.theta[::step]),
        color="#ffff00",
        scale=30,
        width=0.004,
        zorder=6,
        label="Heading (direction)",
    )

    legend = ax.legend(loc="upper right", facecolor="#0e1f11", edgecolor="white")
    for text in legend.get_texts():
        text.set_color("white")

    ax.set_title(title, color="white", fontsize=14, pad=15)

    # Make sure output directory exists if saving
    if save_path:
        os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
        plt.savefig(
            save_path, dpi=300, bbox_inches="tight", facecolor=fig.get_facecolor()
        )
        print(f"Saved trajectory plot to: {save_path}")

    if show:
        plt.show()
    else:
        plt.close()
