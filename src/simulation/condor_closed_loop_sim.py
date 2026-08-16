"""
Closed-Loop Simulation
======================
Offline simulation connecting QuadrotorDynamics with multiple controllers.
Runs flight scenarios without PX4, outputs state logs, comparison plots,
and performance metrics.

Supports: CascadeController (PID), LQRController, MPCController
"""

import random
import sys
import time

import matplotlib
import numpy as np

matplotlib.use("Agg")
from pathlib import Path  # noqa: E402

import matplotlib.pyplot as plt  # noqa: E402

# Add project root to path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.controllers.cascade_controller import CascadeController  # noqa: E402
from src.controllers.controller_base import ControllerBase  # noqa: E402
from src.controllers.geometric_controller import GeometricController  # noqa: E402
from src.controllers.lqr_controller import LQRController  # noqa: E402
from src.controllers.mpc_controller import MPCController  # noqa: E402
from src.uav_model.condor_dynamics import (  # noqa: E402
    QuadplaneDynamics as QuadrotorDynamics,
    QuadplaneParams as QuadrotorParams,
)

# ============================================================
#  Scenario generators
# ============================================================


def generate_hover_setpoints(t: float) -> np.ndarray:
    """Takeoff -> Hover at 5m -> Land. NED convention."""
    if t < 3.0:
        target_alt = -5.0 * min(t / 3.0, 1.0)
        return np.array([0.0, 0.0, target_alt])
    elif t < 13.0:
        return np.array([0.0, 0.0, -5.0])
    elif t < 18.0:
        progress = (t - 13.0) / 5.0
        return np.array([0.0, 0.0, -5.0 * (1.0 - progress)])
    else:
        return np.array([0.0, 0.0, 0.0])


def generate_square_setpoints(
    t: float, drone_pos: np.ndarray, _state: dict
) -> np.ndarray:
    """
    Square trajectory at 5m altitude with smooth minimum jerk interpolation.
    """
    waypoints = [
        np.array([0.0, 0.0, -5.0]),
        np.array([5.0, 0.0, -5.0]),
        np.array([5.0, 5.0, -5.0]),
        np.array([0.0, 5.0, -5.0]),
        np.array([0.0, 0.0, -5.0]),
        np.array([0.0, 0.0, 0.0]),
    ]

    idx = _state.get("wp_idx", 0)
    start_t = _state.get("start_t", 0.0)

    if idx >= len(waypoints) - 1:
        return np.concatenate([waypoints[-1], np.zeros(6)])

    start_wp = waypoints[idx]
    target_wp = waypoints[idx + 1]

    interval = 5.0  # 5 seconds to travel between corners
    tau = np.clip((t - start_t) / interval, 0.0, 1.0)

    if tau >= 1.0:
        idx += 1
        start_t = t
        _state["wp_idx"] = idx
        _state["start_t"] = start_t
        if idx >= len(waypoints) - 1:
            return np.concatenate([waypoints[-1], np.zeros(6)])
        start_wp = waypoints[idx]
        target_wp = waypoints[idx + 1]
        tau = 0.0

    _state["wp_idx"] = idx
    _state["start_t"] = start_t

    # Minimum jerk polynomial
    scale = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    scale_dot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / interval
    scale_ddot = (60 * tau - 180 * tau**2 + 120 * tau**3) / (interval**2)

    pos = start_wp + (target_wp - start_wp) * scale
    vel = (target_wp - start_wp) * scale_dot
    acc = (target_wp - start_wp) * scale_ddot

    return np.concatenate([pos, vel, acc])


def generate_lemniscate_setpoints(t, A=10.0, omega=np.pi / 10.0):
    x = A * np.sin(omega * t)
    y = A * np.sin(omega * t) * np.cos(omega * t)
    z = -5.0

    vx = A * omega * np.cos(omega * t)
    vy = A * omega * (np.cos(omega * t) ** 2 - np.sin(omega * t) ** 2)
    vz = 0.0

    ax = -A * omega**2 * np.sin(omega * t)
    ay = -A * omega**2 * 4 * np.sin(omega * t) * np.cos(omega * t)
    az = 0.0

    return np.array([x, y, z, vx, vy, vz, ax, ay, az])


def generate_giant_lemniscate_setpoints(t, A=40.0, omega=np.pi / 20.0):
    # Thời gian 10s đầu tiên dành cho việc cất cánh thẳng đứng lên -15m
    if t < 10.0:
        progress = t / 10.0
        z = -15.0 * (10 * progress**3 - 15 * progress**4 + 6 * progress**5)
        vz = -15.0 * (30 * progress**2 - 60 * progress**3 + 30 * progress**4) / 10.0
        az = -15.0 * (60 * progress - 180 * progress**2 + 120 * progress**3) / 100.0
        return np.array([0.0, 0.0, z, 0.0, 0.0, vz, 0.0, 0.0, az])

    # Kích thước x4 (A=40), bay trên cao -15m, chu kỳ chậm hơn (omega=pi/20) ~ 40s/vòng
    t_act = t - 10.0
    x = A * np.sin(omega * t_act)
    y = A * np.sin(omega * t_act) * np.cos(omega * t_act)
    z = -15.0

    vx = A * omega * np.cos(omega * t_act)
    vy = A * omega * (np.cos(omega * t_act) ** 2 - np.sin(omega * t_act) ** 2)
    vz = 0.0

    ax = -A * omega**2 * np.sin(omega * t_act)
    ay = -A * omega**2 * 4 * np.sin(omega * t_act) * np.cos(omega * t_act)
    az = 0.0

    # Lọc phong bì mềm (Envelope Filter) triệt tiêu gia tốc vô cực lúc bắt đầu chu kỳ
    blend = 1.0 - np.exp(-t_act)

    return np.array([x, y, z, vx * blend, vy * blend, vz, ax * blend, ay * blend, az])


class RandomWaypointGenerator:
    def __init__(
        self,
        interval=6.0,
        x_bounds=(-15.0, 15.0),
        y_bounds=(-15.0, 15.0),
        z_bounds=(-15.0, -3.0),
    ):
        self.interval = interval
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.z_bounds = z_bounds
        self.last_switch_time = -self.interval
        self.start_wp = np.array([0.0, 0.0, -5.0])
        self.target_wp = np.array([0.0, 0.0, -5.0])
        self.current_wp = np.array([0.0, 0.0, -5.0])  # Back compat

    def reset(self):
        self.last_switch_time = -self.interval
        self.start_wp = np.array([0.0, 0.0, -5.0])
        self.target_wp = np.array([0.0, 0.0, -5.0])
        self.current_wp = np.array([0.0, 0.0, -5.0])

    def get_setpoint(self, t: float) -> np.ndarray:
        if t - self.last_switch_time >= self.interval:
            self.last_switch_time = t
            self.start_wp = np.copy(self.target_wp)
            x = random.uniform(*self.x_bounds)
            y = random.uniform(*self.y_bounds)
            z = random.uniform(*self.z_bounds)
            self.target_wp = np.array([x, y, z])
            self.current_wp = self.target_wp  # Bơm tạm biến cũ để AI PPO tương thích
            print(
                f"[RandomWaypoint] New waypoint at t={t:.1f}: [{x:.2f}, {y:.2f}, {z:.2f}]"
            )

        # Áp dụng đường cong Minimum Jerk để nối Điểm xuất phát và Đích đến
        tau = np.clip((t - self.last_switch_time) / self.interval, 0.0, 1.0)

        scale = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        scale_dot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / self.interval
        scale_ddot = (60 * tau - 180 * tau**2 + 120 * tau**3) / (self.interval**2)

        pos = self.start_wp + (self.target_wp - self.start_wp) * scale
        vel = (self.target_wp - self.start_wp) * scale_dot
        acc = (self.target_wp - self.start_wp) * scale_ddot

        return np.concatenate([pos, vel, acc])


class Figure8TrajectoryGenerator:
    def __init__(self, X_amp=20.0, Y_amp=40.0, omega=np.pi / 20.0, z=-10.0):
        self.X_amp = X_amp
        self.Y_amp = Y_amp
        self.omega = omega
        self.z = z

    def reset(self):
        pass

    def get_setpoint(self, t: float) -> np.ndarray:
        # Gazebo map equation: x = 20*sin(2*omega*t), y = 40*sin(omega*t)
        X = self.X_amp
        Y = self.Y_amp
        w = self.omega

        x = X * np.sin(2 * w * t)
        y = Y * np.sin(w * t)
        z = self.z

        vx = 2 * X * w * np.cos(2 * w * t)
        vy = Y * w * np.cos(w * t)
        vz = 0.0

        ax = -4 * X * w**2 * np.sin(2 * w * t)
        ay = -Y * w**2 * np.sin(w * t)
        az = 0.0

        # Envelope filter to prevent infinite initial acceleration spike
        blend = 1.0 - np.exp(-t)
        return np.array(
            [x, y, z, vx * blend, vy * blend, vz, ax * blend, ay * blend, az]
        )


# ============================================================
#  Single-controller simulation runner
# ============================================================


def run_simulation(
    controller: ControllerBase,
    scenario: str = "hover",
    duration: float = 20.0,
    dt: float = 0.005,
) -> dict:
    """
    Run a closed-loop simulation with the given controller.

    Args:
        controller: any ControllerBase implementation
        scenario: 'hover' or 'square'
        duration: total simulation time (s)
        dt: integration timestep (s)

    Returns:
        dict with keys: t, pos, vel, euler, thrust, setpoint, name, elapsed
    """
    params = QuadrotorParams()
    drone = QuadrotorDynamics(params)
    drone.reset()
    controller.reset()

    n_steps = int(duration / dt)
    t_log = np.zeros(n_steps)
    pos_log = np.zeros((n_steps, 3))
    vel_log = np.zeros((n_steps, 3))
    euler_log = np.zeros((n_steps, 3))
    thrust_log = np.zeros(n_steps)
    setpoint_log = np.zeros((n_steps, 3))

    wp_state = {"wp_idx": 0, "wp_hold": 0.0, "dt": dt}

    wall_start = time.perf_counter()

    for i in range(n_steps):
        t = i * dt

        # Get setpoint
        if scenario == "hover":
            sp = generate_hover_setpoints(t)
        elif scenario == "square":
            sp = generate_square_setpoints(t, drone.position, wp_state)
        elif scenario == "lemniscate":
            sp = generate_lemniscate_setpoints(t)
        else:
            sp = np.zeros(3)

        # Compute control
        ctrl_sp = sp if isinstance(controller, GeometricController) else sp[0:3]
        thrust, torque = controller.compute(
            position_setpoint=ctrl_sp,
            yaw_setpoint=0.0,
            current_state=drone.state,
            dt=dt,
        )

        # Step dynamics
        drone.step(thrust, torque, dt)

        # Prevent going underground
        if drone.state[2] > 0:
            drone.state[2] = 0.0
            drone.state[5] = min(drone.state[5], 0.0)

        # Log
        t_log[i] = t
        pos_log[i] = drone.position
        vel_log[i] = drone.velocity
        euler_log[i] = drone.euler_angles
        thrust_log[i] = thrust
        setpoint_log[i] = sp[0:3]

    wall_elapsed = time.perf_counter() - wall_start

    return {
        "t": t_log,
        "pos": pos_log,
        "vel": vel_log,
        "euler": euler_log,
        "thrust": thrust_log,
        "setpoint": setpoint_log,
        "name": controller.name,
        "elapsed": wall_elapsed,
    }


# ============================================================
#  Performance metrics
# ============================================================


def compute_metrics(
    result: dict, hover_start: float = 3.0, hover_end: float = 13.0, dt: float = 0.005
) -> dict:
    """
    Compute performance metrics from simulation result.

    Metrics (computed during hover phase t=[hover_start, hover_end]):
        - RMSE: root mean square position error
        - Settling time: time to stay within 5% of setpoint
        - Overshoot: max altitude overshoot (%)
        - Control effort: integral of |thrust - hover_thrust|^2
    """
    t = result["t"]
    pos = result["pos"]
    sp = result["setpoint"]
    thrust = result["thrust"]

    # Focus on hover phase
    mask = (t >= hover_start) & (t <= hover_end)
    if not np.any(mask):
        return {
            "rmse": np.nan,
            "settling_time": np.nan,
            "overshoot": np.nan,
            "effort": np.nan,
        }

    t_h = t[mask]
    pos_h = pos[mask]
    sp_h = sp[mask]
    thrust_h = thrust[mask]

    # RMSE (3D position error)
    errors = np.linalg.norm(pos_h - sp_h, axis=1)
    rmse = np.sqrt(np.mean(errors**2))

    # Settling time (5% band of target altitude = 5m -> band = 0.25m)
    alt_actual = -pos_h[:, 2]
    alt_target = -sp_h[:, 2]
    band = (
        0.05 * np.mean(np.abs(alt_target[alt_target != 0]))
        if np.any(alt_target != 0)
        else 0.25
    )

    settled_mask = np.abs(alt_actual - alt_target) <= band
    settling_time = np.nan
    if np.any(settled_mask):
        # Find first time it settles AND stays settled
        for idx in range(len(settled_mask)):
            if np.all(settled_mask[idx:]):
                settling_time = t_h[idx] - hover_start
                break

    # Overshoot (altitude)
    # During takeoff phase (0 to hover_start)
    takeoff_mask = (t >= 0) & (t <= hover_start + 2.0)
    alt_takeoff = -pos[takeoff_mask, 2]
    target_alt = 5.0
    max_alt = np.max(alt_takeoff) if len(alt_takeoff) > 0 else target_alt
    overshoot = max(0.0, (max_alt - target_alt) / target_alt * 100.0)

    # Control effort (integrated squared thrust deviation)
    hover_thrust = 1.535 * 9.81
    effort = np.sum((thrust_h - hover_thrust) ** 2) * dt

    return {
        "rmse": rmse,
        "settling_time": settling_time,
        "overshoot": overshoot,
        "effort": effort,
    }


# ============================================================
#  Plotting functions
# ============================================================


def plot_comparison(results: list[dict], save_path: str):
    """Plot comparison of multiple controllers on the same scenario."""
    colors = {"Cascade PID": "#3b82f6", "LQR": "#10b981", "MPC": "#f59e0b"}
    fig, axes = plt.subplots(2, 2, figsize=(16, 11))
    fig.suptitle(
        "Controller Comparison: Takeoff → Hover → Land",
        fontsize=15,
        fontweight="bold",
    )

    for r in results:
        c = colors.get(r["name"], "#888888")
        t = r["t"]
        lbl = r["name"]

        # Altitude
        ax = axes[0, 0]
        ax.plot(t, -r["pos"][:, 2], color=c, linewidth=1.5, label=lbl)

        # Velocity magnitude
        ax = axes[0, 1]
        v_mag = np.linalg.norm(r["vel"], axis=1)
        ax.plot(t, v_mag, color=c, linewidth=1.2, label=lbl)

        # Roll/Pitch
        ax = axes[1, 0]
        ax.plot(
            t, np.degrees(r["euler"][:, 0]), color=c, linewidth=1, label=f"{lbl} roll"
        )
        ax.plot(
            t,
            np.degrees(r["euler"][:, 1]),
            color=c,
            linewidth=1,
            linestyle="--",
            alpha=0.6,
        )

        # Thrust
        ax = axes[1, 1]
        ax.plot(t, r["thrust"], color=c, linewidth=1.2, label=lbl)

    # Setpoint reference (from first result)
    axes[0, 0].plot(
        results[0]["t"],
        -results[0]["setpoint"][:, 2],
        "r--",
        linewidth=1,
        alpha=0.5,
        label="Setpoint",
    )

    # Formatting
    axes[0, 0].set_ylabel("Altitude (m)")
    axes[0, 0].set_title("Altitude vs Time")
    axes[0, 0].legend(fontsize=9)
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].set_ylabel("Speed (m/s)")
    axes[0, 1].set_title("Velocity Magnitude")
    axes[0, 1].legend(fontsize=9)
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].set_ylabel("Angle (deg)")
    axes[1, 0].set_title("Roll / Pitch Angles")
    axes[1, 0].legend(fontsize=9, ncol=2)
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].set_ylabel("Thrust (N)")
    axes[1, 1].set_title("Total Thrust")
    hover_T = 1.535 * 9.81
    axes[1, 1].axhline(y=hover_T, color="r", linestyle="--", alpha=0.4, label="Hover")
    axes[1, 1].legend(fontsize=9)
    axes[1, 1].grid(True, alpha=0.3)

    for ax in axes.ravel():
        ax.set_xlabel("Time (s)")

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Comparison plot saved: {save_path}")


def plot_trajectory_comparison(
    results: list[dict], save_path: str, title: str = "Trajectory Comparison"
):
    """Plot 2D trajectory comparison (top-down X-Y view)."""
    colors = {
        "Cascade PID": "#3b82f6",
        "LQR": "#10b981",
        "MPC": "#f59e0b",
        "Geometric SE(3)": "#ef4444",
    }

    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    fig.suptitle(title, fontsize=15, fontweight="bold")

    # Top-down
    ax = axes[0]
    for r in results:
        c = colors.get(r["name"], "#888888")
        ax.plot(r["pos"][:, 0], r["pos"][:, 1], color=c, linewidth=1.5, label=r["name"])

    # Waypoints/Reference
    if "Square" in title:
        wps = [(0, 0), (5, 0), (5, 5), (0, 5), (0, 0)]
        wp_x, wp_y = zip(*wps)
        ax.plot(wp_x, wp_y, "rs--", markersize=8, alpha=0.5, label="Waypoints")
    else:
        # Reference path from first result's setpoint
        ax.plot(
            results[0]["setpoint"][:, 0],
            results[0]["setpoint"][:, 1],
            "r--",
            alpha=0.5,
            label="Reference",
        )
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Top-Down View (X-Y)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")

    # Altitude
    ax = axes[1]
    for r in results:
        c = colors.get(r["name"], "#888888")
        ax.plot(r["t"], -r["pos"][:, 2], color=c, linewidth=1.5, label=r["name"])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Altitude Profile")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  Trajectory comparison saved: {save_path}")


def print_metrics_table(all_metrics: list[tuple[str, dict]]):
    """Print a formatted comparison table of controller metrics."""
    header = f"{
        'Controller':<15} {
        'RMSE (m)':>10} {
            'Settling (s)':>13} {
                'Overshoot (%)':>14} {
                    'Effort':>12} {
                        'Wall Time (s)':>14}"
    sep = "-" * len(header)

    print(f"\n{sep}")
    print("  PERFORMANCE METRICS COMPARISON")
    print(sep)
    print(header)
    print(sep)

    for name, m in all_metrics:
        st = f"{m['settling_time']:.3f}" if not np.isnan(m["settling_time"]) else "N/A"
        print(
            f"{name:<15} {m['rmse']:>10.4f} {st:>13} "
            f"{m['overshoot']:>14.2f} {m['effort']:>12.2f} {m['elapsed']:>14.3f}"
        )
    print(sep)


# ============================================================
#  Legacy single-controller functions (backward compatible)
# ============================================================


def run_takeoff_hover_land(duration: float = 20.0, dt: float = 0.005):
    """Legacy: run hover with CascadeController only."""
    controller = CascadeController()
    result = run_simulation(controller, "hover", duration, dt)
    return (
        result["t"],
        result["pos"],
        result["vel"],
        result["euler"],
        result["thrust"],
        result["setpoint"],
    )


def run_square_trajectory(duration: float = 30.0, dt: float = 0.005):
    """Legacy: run square trajectory with CascadeController only."""
    controller = CascadeController()
    result = run_simulation(controller, "square", duration, dt)
    waypoints = [
        np.array([0.0, 0.0, -5.0]),
        np.array([5.0, 0.0, -5.0]),
        np.array([5.0, 5.0, -5.0]),
        np.array([0.0, 5.0, -5.0]),
        np.array([0.0, 0.0, -5.0]),
        np.array([0.0, 0.0, 0.0]),
    ]
    return result["t"], result["pos"], result["setpoint"], waypoints


def plot_results(t, pos, vel, euler, thrust, setpoint, save_path: str):
    """Generate comprehensive result plots."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(
        "Closed-Loop Simulation: Takeoff -> Hover -> Land",
        fontsize=14,
        fontweight="bold",
    )

    # Position (convert NED z to altitude)
    ax = axes[0, 0]
    ax.plot(t, -pos[:, 2], "b-", linewidth=1.5, label="Altitude (actual)")
    ax.plot(t, -setpoint[:, 2], "r--", linewidth=1, label="Altitude (setpoint)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Altitude vs Time")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Velocity
    ax = axes[0, 1]
    ax.plot(t, vel[:, 0], label="vx")
    ax.plot(t, vel[:, 1], label="vy")
    ax.plot(t, vel[:, 2], label="vz")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("Velocity Components")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Euler angles
    ax = axes[1, 0]
    ax.plot(t, np.degrees(euler[:, 0]), label="Roll (phi)")
    ax.plot(t, np.degrees(euler[:, 1]), label="Pitch (theta)")
    ax.plot(t, np.degrees(euler[:, 2]), label="Yaw (psi)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Euler Angles")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Thrust
    ax = axes[1, 1]
    ax.plot(t, thrust, "g-", linewidth=1)
    ax.axhline(
        y=1.535 * 9.81, color="r", linestyle="--", alpha=0.5, label="Hover thrust"
    )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Thrust (N)")
    ax.set_title("Total Thrust")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Plot saved to: {save_path}")


def plot_trajectory_2d(t, pos, setpoint, waypoints, save_path: str):
    """Plot 2D trajectory (top-down view)."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Square Trajectory Tracking", fontsize=14, fontweight="bold")

    # Top-down (X-Y)
    ax = axes[0]
    ax.plot(pos[:, 0], pos[:, 1], "b-", linewidth=1.5, label="Actual path")
    wp_x = [wp[0] for wp in waypoints[:-1]]
    wp_y = [wp[1] for wp in waypoints[:-1]]
    ax.plot(wp_x, wp_y, "rs--", markersize=8, label="Waypoints")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Top-Down View (X-Y)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")

    # Altitude vs time
    ax = axes[1]
    ax.plot(t, -pos[:, 2], "b-", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Altitude Profile")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Trajectory plot saved to: {save_path}")


# ============================================================
#  Controller comparison runner
# ============================================================


def run_controller_comparison():
    """
    Run hover and square trajectory with all 3 controllers,
    compute metrics, and generate comparison plots.
    """
    params = QuadrotorParams()

    controllers = [
        CascadeController(mass=params.mass, gravity=params.gravity),
        LQRController(
            mass=params.mass,
            gravity=params.gravity,
            Ixx=params.Ixx,
            Iyy=params.Iyy,
            Izz=params.Izz,
            drag_coeff=params.drag_coeff,
        ),
        MPCController(
            mass=params.mass,
            gravity=params.gravity,
            Ixx=params.Ixx,
            Iyy=params.Iyy,
            Izz=params.Izz,
            drag_coeff=params.drag_coeff,
            horizon=15,
            control_dt=params.dt,
        ),
        GeometricController(
            mass=params.mass,
            gravity=params.gravity,
            Ixx=params.Ixx,
            Iyy=params.Iyy,
            Izz=params.Izz,
        ),
    ]

    output_dir = Path(__file__).resolve().parents[2] / "output"
    output_dir.mkdir(exist_ok=True)

    # ---- Hover comparison ----
    print("\n[1/2] Hover Comparison (Takeoff -> Hover -> Land)...")
    hover_results = []
    hover_metrics = []

    for ctrl in controllers:
        print(f"  Running {ctrl.name}...")
        r = run_simulation(ctrl, "hover", duration=20.0, dt=params.dt)
        hover_results.append(r)

        m = compute_metrics(r, hover_start=3.0, hover_end=13.0, dt=params.dt)
        m["elapsed"] = r["elapsed"]
        hover_metrics.append((ctrl.name, m))

    plot_comparison(hover_results, str(output_dir / "hover_comparison.png"))
    print_metrics_table(hover_metrics)

    # ---- Square trajectory comparison ----
    print("\n[2/2] Square Trajectory Comparison...")
    square_results = []

    for ctrl in controllers:
        print(f"  Running {ctrl.name}...")
        r = run_simulation(ctrl, "square", duration=30.0, dt=params.dt)
        square_results.append(r)

    plot_trajectory_comparison(
        square_results,
        str(output_dir / "trajectory_comparison.png"),
        "Square Trajectory Comparison",
    )

    # ---- Lemniscate trajectory comparison ----
    print("\n[3/3] Lemniscate Trajectory Comparison (Differential Flatness)...")
    lemn_results = []

    for ctrl in controllers:
        print(f"  Running {ctrl.name}...")
        r = run_simulation(ctrl, "lemniscate", duration=25.0, dt=params.dt)
        lemn_results.append(r)

    plot_trajectory_comparison(
        lemn_results,
        str(output_dir / "lemniscate_comparison.png"),
        "Lemniscate Trajectory Tracking",
    )

    return hover_results, hover_metrics, square_results, lemn_results


# ============================================================
#  Main
# ============================================================

if __name__ == "__main__":
    output_dir = Path(__file__).resolve().parents[2] / "output"
    output_dir.mkdir(exist_ok=True)

    print("=" * 60)
    print("  CLOSED-LOOP SIMULATION: Controller Comparison")
    print("  PID  vs  LQR  vs  MPC  vs  Geometric SE(3)")
    print("=" * 60)

    run_controller_comparison()

    print(f"\n{'=' * 60}")
    print("  All simulations completed successfully!")
    print(f"  Output directory: {output_dir}")
    print(f"{'=' * 60}")
