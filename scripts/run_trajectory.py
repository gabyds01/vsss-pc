import sys
import asyncio
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt
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
from vsss.control.lqr_tracker import LQRTracker
from vsss.vision.reader import UDPReceiver
from vsss.comms import RobotCommand, SimSender
from vsss.analysis.plot_trajectory import draw_vsss_field
from vsss.kinematics.differential import unicycle_to_differential
from vsss.config import SETTINGS

TRACK_WIDTH = SETTINGS["robot_dimensions"]["track_width"]


async def main():
    parser = argparse.ArgumentParser(
        description="Run LQR path tracking control in FIRASim."
    )
    parser.add_argument(
        "--shape",
        type=str,
        default="square",
        choices=["line", "square", "circle", "s_curve", "spline"],
        help="Trajectory shape to track.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.3,
        help="Nominal tracking speed in m/s (default: 0.3).",
    )
    parser.add_argument(
        "--yellow",
        action="store_true",
        help="Control the yellow team (default is blue).",
    )
    parser.add_argument(
        "--id",
        type=int,
        default=0,
        help="Robot ID to control (default: 0).",
    )
    args = parser.parse_args()

    # 1. Initialize Comms, Vision and LQR Controller
    receiver = UDPReceiver()
    sender = SimSender()
    tracker = LQRTracker()

    # 2. Generate Reference Path
    print(f"Generating '{args.shape}' path...")
    if args.shape == "line":
        path = generate_line(start=(-0.5, -0.3), end=(0.5, 0.3))
    elif args.shape == "square":
        path = generate_square(side=0.6, center=(0.0, 0.0))
    elif args.shape == "circle":
        path = generate_circle(radius=0.45, center=(0.0, 0.0))
    elif args.shape == "s_curve":
        path = generate_s_curve(start_x=-0.6, end_x=0.6, amplitude=0.25)
    else:  # spline
        control_points = [
            (-0.6, 0.0),
            (-0.3, 0.15),
            (0.0, -0.15),
            (0.3, 0.15),
            (0.6, 0.0),
        ]
        path = generate_spline(control_points)

    # Curvature is now computed inside Path (analytically for splines,
    # or via the planar curvature formula for other shapes)

    # Tracked coordinates history for plotting at the end
    actual_x = []
    actual_y = []

    print("\nWaiting for first vision packet from FIRASim...")

    # State machine setup
    # ALIGNING: Move to start point of path
    # TRACKING: Move along path
    # FINISHED: Stop at the end
    state = "ALIGNING"
    current_s = 0.0

    loop_period = 1.0 / 60.0  # 60 Hz control loop
    last_print_time = 0.0

    try:
        while True:
            start_loop_time = time.time()

            # Read latest vision data (awaits next packet)
            data = await receiver.receive()
            frame, field, _, _ = receiver.deserialize(data)

            # Find target robot based on team and ID
            robots = frame.robots_yellow if args.yellow else frame.robots_blue
            robot = None
            for r in robots:
                if r.robot_id == args.id:
                    robot = r
                    break

            if robot is None:
                # Robot not found in frame (maybe simulator is paused or robot removed)
                continue

            robot_pos = (robot.x, robot.y, robot.orientation)

            # --- State Machine ---
            if state == "ALIGNING":
                # Reference targets are the starting point of the path
                x_r = path.x[0]
                y_r = path.y[0]
                theta_r = path.theta[0]

                # Distance to target start point
                dx = x_r - robot.x
                dy = y_r - robot.y
                dist_to_start = np.hypot(dx, dy)

                if dist_to_start > 0.05:
                    # 1. Drive to target coordinates (P-control on distance and heading to target)
                    target_angle = np.arctan2(dy, dx)
                    angle_error = (target_angle - robot.orientation + np.pi) % (
                        2.0 * np.pi
                    ) - np.pi

                    kp_v = 2.0
                    kp_w = 5.0

                    # cos(angle_error) ensures it slows down/reverses if facing the wrong way
                    v = np.clip(kp_v * dist_to_start * np.cos(angle_error), -0.25, 0.25)
                    omega = kp_w * angle_error
                else:
                    # 2. Close to start coordinates, now align orientation to the path starting angle
                    angle_error = (theta_r - robot.orientation + np.pi) % (
                        2.0 * np.pi
                    ) - np.pi
                    kp_w = 5.0
                    v = 0.0
                    omega = kp_w * angle_error

                # Convert to differential velocities (in m/s) using kinematics
                v_left, v_right = unicycle_to_differential(v, omega, TRACK_WIDTH)

                # Transition to tracking if close enough (within 5 cm and 0.15 rad / 8 degrees)
                align_angle_error = abs(
                    (theta_r - robot.orientation + np.pi) % (2.0 * np.pi) - np.pi
                )
                if dist_to_start < 0.05 and align_angle_error < 0.15:
                    state = "TRACKING"
                    print("\nAligned! Starting trajectory tracking...")

            elif state == "TRACKING":
                actual_x.append(robot.x)
                actual_y.append(robot.y)

                # Local window search for progress s to prevent jumps (critical for circles/closed loops)
                search_margin_forward = 0.50
                search_margin_backward = 0.05
                # Allow a small backward margin so the robot can recover when it
                # drifts behind the reference, while still preventing large jumps.
                min_s = max(0.0, current_s - search_margin_backward)
                max_s = min(path.total_length, current_s + search_margin_forward)

                indices = np.where((path.s >= min_s) & (path.s <= max_s))[0]
                if len(indices) == 0:
                    indices = np.arange(len(path.s))

                dx = path.x[indices] - robot.x
                dy = path.y[indices] - robot.y
                sq_distances = dx**2 + dy**2
                min_idx = indices[np.argmin(sq_distances)]
                current_s = path.s[min_idx]

                # Look ahead slightly (e.g. 5 cm) to avoid cutting corners
                lookahead = 0.05
                target_s = min(current_s + lookahead, path.total_length)

                # Get reference target state at target progress
                x_r, y_r, theta_r = path.get_point_at_distance(target_s)

                # Interpolate curvature at target progress (uses analytic curvature for splines)
                k = np.interp(target_s, path.s, path.curvature)

                # Adaptive speed: reduce velocity at high-curvature sections
                # to prevent feedforward omega_r from exceeding physical limits
                curvature_scale = 3.0  # higher = more aggressive slowdown
                v_r = args.speed / (1.0 + abs(k) * TRACK_WIDTH * curvature_scale)

                # Decelerate near the end of the path to avoid overshooting
                remaining = path.total_length - current_s
                decel_zone = 0.15  # start slowing down 15 cm before the end
                if remaining < decel_zone:
                    v_r *= max(0.15, remaining / decel_zone)

                omega_r = v_r * k

                # Compute optimal velocities using LQR
                v_left, v_right = tracker.compute_wheel_velocities(
                    robot_pos, (x_r, y_r, theta_r), (v_r, omega_r)
                )

                # Check if robot has reached the end of the trajectory
                dist_to_end = np.hypot(robot.x - path.x[-1], robot.y - path.y[-1])
                has_reached_end = (
                    (path.total_length - current_s < 0.05) or (dist_to_end < 0.05)
                )
                if has_reached_end:
                    state = "FINISHED"
                    print("\nTrajectory completed!")
                    break

            else:  # FINISHED
                v_left, v_right = 0.0, 0.0

            # 3. Transmit command packet to simulator
            cmd = RobotCommand(
                robot_id=args.id,
                yellow_team=args.yellow,
                wheel_left=v_left,
                wheel_right=v_right,
            )
            sender.send([cmd])

            # Print status at 5 Hz
            current_time = time.time()
            if current_time - last_print_time >= 0.2:
                team_str = "Yellow" if args.yellow else "Blue"
                print(
                    f"State: {state:<9} | {team_str} Robot {args.id} at ({robot.x:.2f}, {robot.y:.2f}, {robot.orientation:.2f}) | Cmd: L={v_left:.2f} m/s, R={v_right:.2f} m/s",
                    end="\r",
                )
                last_print_time = current_time

            # Ensure loop runs at 60 Hz
            elapsed = time.time() - start_loop_time
            sleep_time = max(0.001, loop_period - elapsed)
            await asyncio.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nTrajectory control interrupted by user.")
    finally:
        # Stop the robot on exit
        print("Stopping robot...")
        stop_cmd = RobotCommand(
            robot_id=args.id,
            yellow_team=args.yellow,
            wheel_left=0.0,
            wheel_right=0.0,
        )
        sender.send([stop_cmd])
        sender.close()

        # Plot actual vs reference trajectory
        if actual_x:
            print("Plotting actual vs reference trajectory...")
            fig, ax = plt.subplots(figsize=(10, 8), facecolor="#0e1f11")
            draw_vsss_field(ax)

            # Reference Path (cyan dashed)
            ax.plot(
                path.x,
                path.y,
                color="#00ffff",
                linewidth=2.0,
                linestyle="--",
                zorder=4,
                label="Reference Path",
            )
            # Actual Robot Path (orange solid)
            ax.plot(
                actual_x,
                actual_y,
                color="#ffa500",
                linewidth=2.5,
                zorder=5,
                label="Actual Path (LQR)",
            )
            # Start/End points
            ax.scatter(
                path.x[0],
                path.y[0],
                color="green",
                s=100,
                zorder=6,
                label="Start Point",
            )
            ax.scatter(
                path.x[-1], path.y[-1], color="red", s=100, zorder=6, label="End Point"
            )

            legend = ax.legend(
                loc="upper right", facecolor="#0e1f11", edgecolor="white"
            )
            for text in legend.get_texts():
                text.set_color("white")

            ax.set_title(
                f"LQR Path Tracking Results: {args.shape.upper()}",
                color="white",
                fontsize=14,
                pad=15,
            )

            # Save results graph
            shape_suffix = args.shape.replace("_", "")
            results_path = f"graphs/lqr_tracking_results_{shape_suffix}.png"
            plt.savefig(
                results_path,
                dpi=300,
                bbox_inches="tight",
                facecolor=fig.get_facecolor(),
            )
            print(f"Tracking results plot saved to: {results_path}")
            plt.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
