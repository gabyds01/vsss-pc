import sys
import asyncio
import time
from pathlib import Path
import numpy as np

# Add root folder to sys.path to enable absolute package imports
root_dir = str(Path(__file__).parent.parent)
if root_dir not in sys.path:
    sys.path.insert(0, root_dir)

from vsss.vision.reader import UDPReceiver
from vsss.comms import RobotCommand, SimSender


async def main():
    print("Initializing velocity ramp-up test for FIRASim...")
    receiver = UDPReceiver()
    sender = SimSender()

    print("\nThis test will command Blue Robot 0 to move forward, starting at 0.0")
    print("and increasing the commanded velocity value by 0.5 every 1.5 seconds.")
    print(
        "It will read the actual velocity from FIRASim vision packets to determine the mapping."
    )
    print("Press Ctrl+C to stop the test and hold the robot.")
    print("---------------------------------------------------------------------------")

    # Command value to send
    cmd_value = 0.0
    step_duration = 1.5
    last_step_time = time.time()

    try:
        while True:
            # 1. Read actual robot velocity from vision reader
            data = await receiver.receive()
            frame, field, _, _ = receiver.deserialize(data)

            # Find blue robot 0
            robot = None
            for r in frame.robots_blue:
                if r.robot_id == 0:
                    robot = r
                    break

            # 2. Increment command value periodically
            current_time = time.time()
            if current_time - last_step_time >= step_duration:
                cmd_value += 0.5
                last_step_time = current_time
                if cmd_value > 50.0:
                    print("\nMaximum ramp value reached. Ending test...")
                    break

            # 3. Transmit commands
            # Send the same velocity command to both wheels to move straight forward
            cmd = RobotCommand(
                robot_id=0,
                yellow_team=False,
                wheel_left=cmd_value,
                wheel_right=cmd_value,
            )
            sender.send([cmd])

            # 4. Display results
            if robot is not None:
                # Calculate actual linear speed from vx and vy reported by FIRASim
                actual_speed = np.hypot(robot.vx, robot.vy)
                print(
                    f"Commanded: {cmd_value:3.1f} | Actual: Vx={robot.vx:5.2f} m/s, Vy={robot.vy:5.2f} m/s, Speed={actual_speed:5.2f} m/s | Orientation={robot.orientation:5.2f} rad",
                    end="\r",
                )
            else:
                print(
                    f"Commanded: {cmd_value:3.1f} | Actual: Robot 0 not visible in frame",
                    end="\r",
                )

            # Small delay to keep the loop rate reasonable
            await asyncio.sleep(0.01)

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        # Send stop command on exit
        print("\nStopping robot...")
        stop_cmd = RobotCommand(
            robot_id=0,
            yellow_team=False,
            wheel_left=0.0,
            wheel_right=0.0,
        )
        sender.send([stop_cmd])
        sender.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
