"""
Cascade Controller
==================
Three-layer cascade PID controller for quadrotor:
    Outer loop  (Position PID)  -> desired attitude + thrust
    Middle loop (Attitude PID)  -> desired body rates
    Inner loop  (Rate PID)      -> control torques
"""

import numpy as np

from src.controllers.controller_base import ControllerBase
from src.controllers.pid_controller import PIDController


class CascadeController(ControllerBase):
    """
    Cascade PID controller for position -> attitude -> rate control.

    NED convention: z-down, so altitude control uses negative z.
    """

    def __init__(self, mass: float = 7.8, gravity: float = 9.81):
        self.mass = mass
        self.gravity = gravity

        # ---- Outer loop: Position PID (x, y, z) ----
        # Output: desired roll, pitch angles + thrust
        self.pid_x = PIDController(
            kp=2.2,
            ki=0.04,
            kd=1.4,
            output_min=-0.45,
            output_max=0.45,
            integral_min=-1.5,
            integral_max=1.5,
            derivative_filter_coeff=0.3,
        )
        self.pid_y = PIDController(
            kp=2.2,
            ki=0.04,
            kd=1.4,
            output_min=-0.45,
            output_max=0.45,
            integral_min=-1.5,
            integral_max=1.5,
            derivative_filter_coeff=0.3,
        )
        self.pid_z = PIDController(
            kp=4.2,
            ki=0.25,
            kd=3.2,
            output_min=-45.0,
            output_max=45.0,
            integral_min=-10.0,
            integral_max=10.0,
            derivative_filter_coeff=0.3,
        )

        # ---- Middle loop: Attitude PID (roll, pitch, yaw) ----
        # Output: desired body rates [p, q, r]
        self.pid_roll = PIDController(
            kp=8.5,
            ki=0.0,
            kd=0.7,
            output_min=-3.0,
            output_max=3.0,
            derivative_filter_coeff=0.2,
        )
        self.pid_pitch = PIDController(
            kp=8.5,
            ki=0.0,
            kd=0.7,
            output_min=-3.0,
            output_max=3.0,
            derivative_filter_coeff=0.2,
        )
        self.pid_yaw = PIDController(
            kp=3.5,
            ki=0.05,
            kd=0.2,
            output_min=-2.0,
            output_max=2.0,
            integral_min=-1.0,
            integral_max=1.0,
            derivative_filter_coeff=0.3,
        )

        # ---- Inner loop: Rate PID (p, q, r) ----
        # Output: control torques [tau_x, tau_y, tau_z] scaled for high inertia
        self.pid_p = PIDController(
            kp=4.5,
            ki=0.2,
            kd=0.05,
            output_min=-15.0,
            output_max=15.0,
            integral_min=-5.0,
            integral_max=5.0,
            derivative_filter_coeff=0.5,
        )
        self.pid_q = PIDController(
            kp=3.5,
            ki=0.15,
            kd=0.04,
            output_min=-15.0,
            output_max=15.0,
            integral_min=-5.0,
            integral_max=5.0,
            derivative_filter_coeff=0.5,
        )
        self.pid_r = PIDController(
            kp=6.0,
            ki=0.2,
            kd=0.08,
            output_min=-10.0,
            output_max=10.0,
            integral_min=-4.0,
            integral_max=4.0,
            derivative_filter_coeff=0.5,
        )

    @property
    def name(self) -> str:
        return "Cascade PID"

    def reset(self):
        """Reset all PID controllers."""
        for pid in [
            self.pid_x,
            self.pid_y,
            self.pid_z,
            self.pid_roll,
            self.pid_pitch,
            self.pid_yaw,
            self.pid_p,
            self.pid_q,
            self.pid_r,
        ]:
            pid.reset()

    def compute(
        self,
        position_setpoint: np.ndarray,
        yaw_setpoint: float,
        current_state: np.ndarray,
        dt: float,
    ) -> tuple[float, np.ndarray]:
        """
        Run the full cascade controller.

        Args:
            position_setpoint: desired [x, y, z] in NED (m)
            yaw_setpoint: desired yaw angle (rad)
            current_state: full 12-element state vector
            dt: timestep (s)

        Returns:
            (thrust, torque):
                thrust - total thrust in Newtons (positive up in body)
                torque - [tau_x, tau_y, tau_z] in body frame (N*m)
        """
        # Unpack current state
        pos = current_state[0:3]
        phi, theta, psi = current_state[6:9]
        omega = current_state[9:12]  # body rates [p, q, r]

        # ===== OUTER LOOP: Position -> desired attitude =====
        # Position errors
        ex = position_setpoint[0] - pos[0]
        ey = position_setpoint[1] - pos[1]
        ez = position_setpoint[2] - pos[2]

        # Position PID outputs -> desired accelerations in NED
        ax_des = self.pid_x.update(ex, dt)
        ay_des = self.pid_y.update(ey, dt)
        az_cmd = self.pid_z.update(ez, dt)

        # Thrust = mass * (gravity - az_cmd) for NED (z-down)
        thrust = self.mass * (self.gravity - az_cmd)
        thrust = np.clip(thrust, 0.0, 2.0 * self.mass * self.gravity)

        # Convert desired NED accelerations to desired roll/pitch
        # Using small-angle approximation:
        #   phi_des   =  (ax_des * sin(psi) - ay_des * cos(psi)) / g
        #   theta_des = -(ax_des * cos(psi) + ay_des * sin(psi)) / g
        cpsi, spsi = np.cos(psi), np.sin(psi)
        phi_des = (ax_des * spsi - ay_des * cpsi) / self.gravity
        theta_des = -(ax_des * cpsi + ay_des * spsi) / self.gravity

        # Clamp desired angles
        max_tilt = 0.5  # ~28 degrees
        phi_des = np.clip(phi_des, -max_tilt, max_tilt)
        theta_des = np.clip(theta_des, -max_tilt, max_tilt)

        # ===== MIDDLE LOOP: Attitude -> desired body rates =====
        e_roll = phi_des - phi
        e_pitch = theta_des - theta

        # Yaw error wrapped to [-pi, pi]
        e_yaw = yaw_setpoint - psi
        e_yaw = (e_yaw + np.pi) % (2 * np.pi) - np.pi

        p_des = self.pid_roll.update(e_roll, dt)
        q_des = self.pid_pitch.update(e_pitch, dt)
        r_des = self.pid_yaw.update(e_yaw, dt)

        # ===== INNER LOOP: Rate -> torques =====
        e_p = p_des - omega[0]
        e_q = q_des - omega[1]
        e_r = r_des - omega[2]

        tau_x = self.pid_p.update(e_p, dt)
        tau_y = self.pid_q.update(e_q, dt)
        tau_z = self.pid_r.update(e_r, dt)

        torque = np.array([tau_x, tau_y, tau_z])

        return float(thrust), torque
