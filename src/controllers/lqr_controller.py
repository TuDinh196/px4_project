"""
LQR Controller
==============
Linear Quadratic Regulator for quadrotor position control.

Linearizes the 6-DOF dynamics around the hover equilibrium,
solves the Continuous Algebraic Riccati Equation (CARE) for
the optimal gain matrix K, then applies:

    u = u_hover + K @ (x_ref - x)

This provides globally optimal linear state feedback for the
linearized system with respect to the chosen Q and R weights.
"""

import numpy as np
from scipy.linalg import solve_continuous_are

from src.controllers.controller_base import ControllerBase


class LQRController(ControllerBase):
    """
    LQR controller for quadrotor hover and position tracking.

    State vector (12):  [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    Control vector (4): [thrust_delta, tau_x, tau_y, tau_z]

    The system is linearized around hover:
        x_eq = 0, u_eq = [m*g, 0, 0, 0]
    """

    def __init__(
        self,
        mass: float = 7.8,
        gravity: float = 9.81,
        Ixx: float = 1.46,
        Iyy: float = 1.06,
        Izz: float = 2.50,
        drag_coeff: float = 0.05,
        q_pos_xy: float = 40.0,
        q_pos_z: float = 80.0,
        q_vel: float = 15.0,
        q_angle: float = 60.0,
        q_rate: float = 5.0,
        r_thrust: float = 0.02,
        r_torque: float = 0.3,
    ):
        """
        Args:
            mass, gravity, Ixx/Iyy/Izz, drag_coeff: physical parameters
            q_pos:    Q weight for position states [x, y, z]
            q_vel:    Q weight for velocity states [vx, vy, vz]
            q_angle:  Q weight for attitude states [phi, theta, psi]
            q_rate:   Q weight for body rate states [p, q, r]
            r_thrust: R weight for thrust deviation
            r_torque: R weight for torque commands
        """
        self.mass = mass
        self.gravity = gravity
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz
        self.drag_coeff = drag_coeff

        # Build linearized system matrices
        self.A, self.B = self._build_linearized_system()

        # Build weight matrices
        self.Q = np.diag(
            [
                q_pos_xy,
                q_pos_xy,
                q_pos_z,  # position
                q_vel,
                q_vel,
                q_vel,  # velocity
                q_angle,
                q_angle,
                q_angle,  # attitude
                q_rate,
                q_rate,
                q_rate,  # body rates
            ]
        )
        self.R = np.diag(
            [r_thrust, r_torque, r_torque, r_torque]  # thrust deviation  # torques
        )

        # Solve CARE for optimal gain
        self.K = self._solve_lqr()

        # Max tilt angle for safety (prevents divergence from linear regime)
        self._max_tilt = 0.4  # ~23 degrees

    @property
    def name(self) -> str:
        return "LQR"

    def _build_linearized_system(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Build linearized state-space matrices A, B around hover.

        At hover equilibrium:
            - All angles = 0, all rates = 0
            - Thrust = m*g (exactly cancels gravity)
            - Linearized thrust acts along -z_NED (upward)

        Derivation (Newton-Euler, small angle):
            dx/dt = vx
            dvx/dt = g * theta - (drag/m) * vx   (pitch -> forward accel)
            dy/dt = vy
            dvy/dt = -g * phi - (drag/m) * vy    (roll -> lateral accel)
            dz/dt = vz
            dvz/dt = -dT/m - (drag/m) * vz       (thrust deviation)
            dphi/dt = p
            dp/dt = tau_x / Ixx
            dtheta/dt = q
            dq/dt = tau_y / Iyy
            dpsi/dt = r
            dr/dt = tau_z / Izz
        """
        m = self.mass
        g = self.gravity
        d = self.drag_coeff

        # State: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
        # Index:  0  1  2   3   4   5    6     7     8   9  10 11

        A = np.zeros((12, 12))

        # Position derivatives = velocity
        A[0, 3] = 1.0  # dx/dt = vx
        A[1, 4] = 1.0  # dy/dt = vy
        A[2, 5] = 1.0  # dz/dt = vz

        # Velocity derivatives (linearized around hover, psi=0)
        A[3, 7] = -g  # dvx/dt ~ -g * theta
        A[3, 3] = -d / m  # drag on vx
        A[4, 6] = g  # dvy/dt ~ g * phi
        A[4, 4] = -d / m  # drag on vy
        A[5, 5] = -d / m  # drag on vz

        # Euler rate derivatives = body rates (small angle: W ≈ I)
        A[6, 9] = 1.0  # dphi/dt = p
        A[7, 10] = 1.0  # dtheta/dt = q
        A[8, 11] = 1.0  # dpsi/dt = r

        # Body rate derivatives are from torques (handled by B)

        # Control: [delta_thrust, tau_x, tau_y, tau_z]
        # Index:      0            1      2      3
        B = np.zeros((12, 4))

        # Thrust deviation affects z-acceleration
        # dvz/dt = -(T_hover + dT) / m + g ≈ -dT/m (after subtracting eq.)
        B[5, 0] = -1.0 / m

        # Torques affect body rate derivatives
        B[9, 1] = 1.0 / self.Ixx  # dp/dt = tau_x / Ixx
        B[10, 2] = 1.0 / self.Iyy  # dq/dt = tau_y / Iyy
        B[11, 3] = 1.0 / self.Izz  # dr/dt = tau_z / Izz

        return A, B

    def _solve_lqr(self) -> np.ndarray:
        """Solve Continuous Algebraic Riccati Equation for gain K."""
        P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K

    def reset(self):
        """LQR is memoryless (no internal state to reset)."""
        pass

    def compute(
        self,
        position_setpoint: np.ndarray,
        yaw_setpoint: float,
        current_state: np.ndarray,
        dt: float,
    ) -> tuple[float, np.ndarray]:
        """
        Compute LQR control output.

        Args:
            position_setpoint: desired [x, y, z] in NED (m)
            yaw_setpoint: desired yaw angle (rad)
            current_state: full 12-element state vector
            dt: timestep (s) — not used by LQR but kept for interface

        Returns:
            (thrust, torque): thrust in N, torque as [tau_x, tau_y, tau_z]
        """
        # Build reference state (hover at desired position)
        x_ref = np.zeros(12)
        x_ref[0:3] = position_setpoint
        x_ref[8] = yaw_setpoint

        # State error
        x_err = current_state - x_ref

        # Wrap yaw error to [-pi, pi]
        x_err[8] = (x_err[8] + np.pi) % (2 * np.pi) - np.pi

        # (Removed attitude clamping as it causes under-correction of large angles)

        # Optimal control: u = -K @ x_err
        # u = [delta_thrust, tau_x, tau_y, tau_z]
        u = -self.K @ x_err

        # Add hover thrust feedforward
        hover_thrust = self.mass * self.gravity
        thrust = hover_thrust + u[0]

        # Clamp thrust to physical limits
        thrust = np.clip(thrust, 0.0, 2.0 * hover_thrust)

        # Torques with limit
        torque = np.clip(u[1:4], -1.0, 1.0)

        return float(thrust), torque
