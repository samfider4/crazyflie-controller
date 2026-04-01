from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt


class FlightLogger:
    """Logs flight data, shows optional live plots, and saves files after the run."""

    def __init__(self, base_output_dir: str = "flight_logs", run_timestamp: str | None = None):
        timestamp = run_timestamp or datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.base_output_dir = Path(base_output_dir)
        self.output_dir = self.base_output_dir / timestamp
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.rows: list[dict] = []

    def log_sample(self, runtime, drone_pose, goal, command, ground_pose: Optional[object] = None) -> None:
        row = {
            "runtime": float(runtime),
            "drone_x": float(drone_pose.x),
            "drone_y": float(drone_pose.y),
            "drone_z": float(drone_pose.z),
            "drone_yaw": float(drone_pose.yaw),
            "goal_x": float(goal.x),
            "goal_y": float(goal.y),
            "goal_z": float(goal.z),
            "roll_cmd": float(command.roll),
            "pitch_cmd": float(command.pitch),
            "thrust_cmd": int(command.thrust),
            "ground_x": float(ground_pose.x) if ground_pose is not None else "",
            "ground_y": float(ground_pose.y) if ground_pose is not None else "",
            "ground_z": float(ground_pose.z) if ground_pose is not None else "",
        }
        self.rows.append(row)

    def save_all(self) -> None:
        if not self.rows:
            print(f"FlightLogger: no samples to save in {self.output_dir.resolve()}")
            return

        self._save_csv()
        self._save_position_plot()
        self._save_command_plot()
        print(f"FlightLogger: saved {len(self.rows)} samples to {self.output_dir.resolve()}")

    def _save_csv(self) -> None:
        csv_path = self.output_dir / "flight_log.csv"
        fieldnames = list(self.rows[0].keys())

        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

    def _save_position_plot(self) -> None:
        t = [row["runtime"] for row in self.rows]

        plt.figure(figsize=(10, 6))
        plt.plot(t, [row["drone_x"] for row in self.rows], label="x")
        plt.plot(t, [row["goal_x"] for row in self.rows], "--", label="x goal")
        plt.plot(t, [row["drone_y"] for row in self.rows], label="y")
        plt.plot(t, [row["goal_y"] for row in self.rows], "--", label="y goal")
        plt.plot(t, [row["drone_z"] for row in self.rows], label="z")
        plt.plot(t, [row["goal_z"] for row in self.rows], "--", label="z goal")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.title("Position vs Goal")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(self.output_dir / "position_vs_goal.png", dpi=150)
        plt.close()

    def _save_command_plot(self) -> None:
        t = [row["runtime"] for row in self.rows]

        plt.figure(figsize=(10, 6))
        plt.plot(t, [row["roll_cmd"] for row in self.rows], label="roll cmd (deg)")
        plt.plot(t, [row["pitch_cmd"] for row in self.rows], label="pitch cmd (deg)")
        plt.plot(t, [row["thrust_cmd"] for row in self.rows], label="thrust cmd")
        plt.xlabel("Time (s)")
        plt.ylabel("Command")
        plt.title("Controller Commands")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(self.output_dir / "commands.png", dpi=150)
        plt.close()
