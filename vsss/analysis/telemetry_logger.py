import os
import csv
import time

class TelemetryLogger:
    """Logs robot tracking states, reference path positions, targets, and telemetry to CSV."""

    def __init__(self, controller: str, shape: str, log_dir: str = "telemetry_logs"):
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(
            self.log_dir, f"telemetry_{controller}_{shape}_{timestamp_str}.csv"
        )
        
        self.start_time = time.time()
        
        self.headers = [
            "timestamp",
            "state",
            "robot_id",
            "vision_x",
            "vision_y",
            "vision_theta",
            "ref_x",
            "ref_y",
            "ref_theta",
            "target_v_left",
            "target_v_right",
            "actual_v_left",
            "actual_v_right",
            "enc_delta_left",
            "enc_delta_right",
        ]
        
        self.file = open(self.filename, mode="w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.headers)
        self.file.flush()
        
    def log(
        self,
        state: str,
        robot_id: int,
        vision_pos: tuple[float, float, float],
        ref_pos: tuple[float, float, float],
        target_vel: tuple[float, float],
        actual_vel_and_encoders: dict | None = None,
    ):
        """Log a new row of telemetry to the CSV file."""
        elapsed_time = time.time() - self.start_time
        
        vx, vy, vtheta = vision_pos
        rx, ry, rtheta = ref_pos
        tv_l, tv_r = target_vel
        
        if actual_vel_and_encoders:
            av_l = actual_vel_and_encoders.get("wheel_left_vel", 0.0)
            av_r = actual_vel_and_encoders.get("wheel_right_vel", 0.0)
            ed_l = actual_vel_and_encoders.get("enc_delta_left", 0)
            ed_r = actual_vel_and_encoders.get("enc_delta_right", 0)
        else:
            av_l, av_r, ed_l, ed_r = 0.0, 0.0, 0, 0
            
        row = [
            f"{elapsed_time:.4f}",
            state,
            robot_id,
            f"{vx:.4f}",
            f"{vy:.4f}",
            f"{vtheta:.4f}",
            f"{rx:.4f}",
            f"{ry:.4f}",
            f"{rtheta:.4f}",
            f"{tv_l:.4f}",
            f"{tv_r:.4f}",
            f"{av_l:.4f}",
            f"{av_r:.4f}",
            ed_l,
            ed_r,
        ]
        
        self.writer.writerow(row)
        self.file.flush()
        
    def close(self):
        """Close the CSV file."""
        if self.file:
            self.file.flush()
            self.file.close()
            self.file = None
            print(f"\nTelemetry log saved to: {self.filename}")
