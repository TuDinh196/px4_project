import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Create plots directory if it doesn't exist
script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)
plots_dir = os.path.join(project_dir, "plots")
os.makedirs(plots_dir, exist_ok=True)

# Load telemetry from logs/ or root
log_path = os.path.join(project_dir, "logs", "flight_telemetry.csv")
if not os.path.exists(log_path):
    log_path = os.path.join(project_dir, "flight_telemetry.csv")

if not os.path.exists(log_path):
    print(
        f"⚠️ No telemetry log found at {log_path}. "
        "Run a flight mission first (e.g. ./manage.sh geometric)."
    )
    exit(0)

df = pd.read_csv(log_path)

# Calculate errors
df["err_x"] = df["pos_x"] - df["sp_x"]
df["err_y"] = df["pos_y"] - df["sp_y"]
df["err_z"] = df["pos_z"] - df["sp_z"]
df["pos_error_norm"] = np.sqrt(df["err_x"] ** 2 + df["err_y"] ** 2 + df["err_z"] ** 2)

# Set up the figure
fig = plt.figure(figsize=(16, 10))

# 1. 3D Trajectory
ax1 = fig.add_subplot(2, 2, 1, projection="3d")
ax1.plot(df["sp_x"], df["sp_y"], df["sp_z"], "b--", label="Target Setpoint")
ax1.plot(df["pos_x"], df["pos_y"], df["pos_z"], "r-", label="Actual Drone")
ax1.set_xlabel("X (m)")
ax1.set_ylabel("Y (m)")
ax1.set_zlabel("Z (m)")
ax1.set_title("3D Flight Trajectory")
ax1.legend()
ax1.view_init(elev=20.0, azim=-35)

# 2. Z-Axis Tracking (Altitude)
ax2 = fig.add_subplot(2, 2, 2)
ax2.plot(df["time_s"], df["sp_z"], "b--", label="Target Altitude (Z)")
ax2.plot(df["time_s"], df["pos_z"], "r-", label="Actual Altitude (Z)")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Z Position (m)")
ax2.set_title("Z-Axis Tracking (Testing Z-Integrator)")
ax2.grid(True)
ax2.legend()

# 3. Attitude Tracking (Roll, Pitch & Yaw lag)
ax3 = fig.add_subplot(2, 2, 3)
ax3.plot(df["time_s"], df["cmd_roll_deg"], "b--", alpha=0.7, label="Commanded Roll")
ax3.plot(df["time_s"], df["roll_deg"], "b-", label="Actual Roll")
ax3.plot(df["time_s"], df["cmd_pitch_deg"], "r--", alpha=0.7, label="Commanded Pitch")
ax3.plot(df["time_s"], df["pitch_deg"], "r-", label="Actual Pitch")
ax3.plot(df["time_s"], df["cmd_yaw_deg"], "g--", alpha=0.7, label="Commanded Yaw")
ax3.plot(df["time_s"], df["yaw_deg"], "g-", label="Actual Yaw")
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Angle (deg)")
ax3.set_title("Attitude Tracking (Showing Yaw Alignment)")
ax3.grid(True)
ax3.legend(loc="upper right", fontsize=8)

# 4. Total Position Error
ax4 = fig.add_subplot(2, 2, 4)
ax4.plot(df["time_s"], df["pos_error_norm"], "k-", label="Total Position RMSE")
ax4.set_xlabel("Time (s)")
ax4.set_ylabel("Error Distance (m)")
ax4.set_title("Overall Tracking Error over time")
ax4.grid(True)
ax4.legend()

plt.tight_layout()
output_path = "plots/comprehensive_flight_analysis.png"
plt.savefig(output_path, dpi=150)
print(f"Saved plot to {output_path}")
