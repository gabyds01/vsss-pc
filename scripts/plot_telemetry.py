import os
import sys
import glob
import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Add root folder to sys.path
root_dir = str(Path(__file__).parent.parent)
if root_dir not in sys.path:
    sys.path.insert(0, root_dir)

from vsss.analysis.plot_trajectory import draw_vsss_field


def get_latest_log_file(log_dir="telemetry_logs"):
    """Find the most recently modified CSV file in the telemetry log directory."""
    files = glob.glob(os.path.join(log_dir, "*.csv"))
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def main():
    parser = argparse.ArgumentParser(description="Plot telemetry logs from CSV.")
    parser.add_argument(
        "--file",
        type=str,
        default=None,
        help="Path to telemetry CSV file. If not specified, the newest file in 'telemetry_logs/' is used.",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save plots as images in the same directory as the CSV file.",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not display the plots interactively (useful for headlessly generating images).",
    )
    args = parser.parse_args()

    file_path = args.file
    if not file_path:
        file_path = get_latest_log_file()
        if not file_path:
            print("Error: No CSV files found in 'telemetry_logs/' directory.")
            sys.exit(1)
        print(f"Using latest telemetry file: {file_path}")

    if not os.path.exists(file_path):
        print(f"Error: File not found: {file_path}")
        sys.exit(1)

    # Read the CSV data using python's built-in csv module
    data = {
        "timestamp": [],
        "state": [],
        "robot_id": [],
        "vision_x": [],
        "vision_y": [],
        "vision_theta": [],
        "ref_x": [],
        "ref_y": [],
        "ref_theta": [],
        "target_v_left": [],
        "target_v_right": [],
        "actual_v_left": [],
        "actual_v_right": [],
        "enc_delta_left": [],
        "enc_delta_right": [],
    }

    with open(file_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            data["timestamp"].append(float(row["timestamp"]))
            data["state"].append(row["state"])
            data["robot_id"].append(int(row["robot_id"]))
            data["vision_x"].append(float(row["vision_x"]))
            data["vision_y"].append(float(row["vision_y"]))
            data["vision_theta"].append(float(row["vision_theta"]))
            data["ref_x"].append(float(row["ref_x"]))
            data["ref_y"].append(float(row["ref_y"]))
            data["ref_theta"].append(float(row["ref_theta"]))
            data["target_v_left"].append(float(row["target_v_left"]))
            data["target_v_right"].append(float(row["target_v_right"]))
            data["actual_v_left"].append(float(row["actual_v_left"]))
            data["actual_v_right"].append(float(row["actual_v_right"]))
            data["enc_delta_left"].append(int(row["enc_delta_left"]))
            data["enc_delta_right"].append(int(row["enc_delta_right"]))

    # Convert to numpy arrays for easier plotting
    for key in data:
        if key != "state":
            data[key] = np.array(data[key])

    # Extract meta info from filename
    filename = os.path.basename(file_path)
    parts = filename.split("_")
    controller = "Unknown"
    shape = "Unknown"
    if len(parts) >= 4:
        controller = parts[1].upper()
        shape = parts[2].capitalize()

    # Create directory for saving if requested
    save_dir = os.path.dirname(os.path.abspath(file_path))

    # --- Plot 1: 2D Trajectory (Field View) ---
    fig1, ax1 = plt.subplots(figsize=(10, 8), facecolor="#0e1f11")
    draw_vsss_field(ax1)

    # Reference Path (cyan dashed)
    ax1.plot(
        data["ref_x"],
        data["ref_y"],
        color="#00ffff",
        linewidth=2.0,
        linestyle="--",
        zorder=4,
        label="Reference Path",
    )

    # Actual Vision Path (orange solid, filter where tracking/aligning)
    state_arr = np.array(data["state"])
    tracking_indices = np.where(state_arr == "TRACKING")[0]
    if len(tracking_indices) == 0:
        tracking_indices = np.arange(len(state_arr))

    ax1.plot(
        data["vision_x"][tracking_indices],
        data["vision_y"][tracking_indices],
        color="#ffa500",
        linewidth=2.5,
        zorder=5,
        label=f"Actual Path ({controller})",
    )

    # Start/End points
    if len(tracking_indices) > 0:
        ax1.scatter(
            data["vision_x"][tracking_indices[0]],
            data["vision_y"][tracking_indices[0]],
            color="green",
            s=100,
            zorder=6,
            label="Start Point",
        )
        ax1.scatter(
            data["vision_x"][tracking_indices[-1]],
            data["vision_y"][tracking_indices[-1]],
            color="red",
            s=100,
            zorder=6,
            label="End Point",
        )

    legend = ax1.legend(loc="upper right", facecolor="#0e1f11", edgecolor="white")
    for text in legend.get_texts():
        text.set_color("white")

    ax1.set_title(
        f"Path Tracking: {shape} with {controller}",
        color="white",
        fontsize=14,
        pad=15,
    )

    if args.save:
        save_path = os.path.join(save_dir, filename.replace(".csv", "_path.png"))
        plt.savefig(
            save_path, dpi=300, bbox_inches="tight", facecolor=fig1.get_facecolor()
        )
        print(f"Saved path plot to: {save_path}")

    # --- Plot 2: Velocity & Encoders Profiles ---
    fig2, (ax_l, ax_r, ax_enc) = plt.subplots(
        3, 1, figsize=(12, 10), sharex=True, facecolor="#121212"
    )
    for ax in (ax_l, ax_r, ax_enc):
        ax.set_facecolor("#1e1e1e")
        ax.tick_params(colors="white")
        ax.grid(True, linestyle="--", alpha=0.3, color="gray")

    # Left Wheel Velocity
    ax_l.plot(
        data["timestamp"],
        data["target_v_left"],
        color="#00ffcc",
        label="Target Left",
        alpha=0.9,
        linewidth=1.5,
    )
    # Check if we have hardware actual velocity telemetry
    has_actual_vel = np.abs(data["actual_v_left"]).sum() > 0.01
    if has_actual_vel:
        ax_l.plot(
            data["timestamp"],
            data["actual_v_left"],
            color="#ff007f",
            label="Actual Left",
            alpha=0.8,
            linewidth=1.5,
        )
    ax_l.set_ylabel("Velocity (m/s)", color="white")
    ax_l.set_title("Left Wheel Velocity", color="white")
    legend_l = ax_l.legend(loc="upper right", facecolor="#1e1e1e", edgecolor="white")
    for text in legend_l.get_texts():
        text.set_color("white")

    # Right Wheel Velocity
    ax_r.plot(
        data["timestamp"],
        data["target_v_right"],
        color="#33ccff",
        label="Target Right",
        alpha=0.9,
        linewidth=1.5,
    )
    if has_actual_vel:
        ax_r.plot(
            data["timestamp"],
            data["actual_v_right"],
            color="#ff9900",
            label="Actual Right",
            alpha=0.8,
            linewidth=1.5,
        )
    ax_r.set_ylabel("Velocity (m/s)", color="white")
    ax_r.set_title("Right Wheel Velocity", color="white")
    legend_r = ax_r.legend(loc="upper right", facecolor="#1e1e1e", edgecolor="white")
    for text in legend_r.get_texts():
        text.set_color("white")

    # Encoder Delta ticks
    ax_enc.plot(
        data["timestamp"],
        data["enc_delta_left"],
        color="#a2ff00",
        label="Encoder Left Ticks",
        alpha=0.8,
        linewidth=1.2,
    )
    ax_enc.plot(
        data["timestamp"],
        data["enc_delta_right"],
        color="#ffea00",
        label="Encoder Right Ticks",
        alpha=0.8,
        linewidth=1.2,
    )
    ax_enc.set_ylabel("Ticks Delta", color="white")
    ax_enc.set_xlabel("Time (seconds)", color="white")
    ax_enc.set_title("Encoder Tick Deltas", color="white")
    legend_enc = ax_enc.legend(
        loc="upper right", facecolor="#1e1e1e", edgecolor="white"
    )
    for text in legend_enc.get_texts():
        text.set_color("white")

    fig2.suptitle(
        f"Velocity & Encoder Profiles: {shape} with {controller}",
        color="white",
        fontsize=14,
    )
    plt.tight_layout()

    if args.save:
        save_path2 = os.path.join(save_dir, filename.replace(".csv", "_profiles.png"))
        plt.savefig(
            save_path2, dpi=300, bbox_inches="tight", facecolor=fig2.get_facecolor()
        )
        print(f"Saved velocity/encoder profiles plot to: {save_path2}")

    if not args.no_show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()
