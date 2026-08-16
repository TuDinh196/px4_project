"""
MPC Controller
==============
Model Predictive Controller for quadrotor position control.

Uses a linearized model (same as LQR) with a receding horizon
strategy: at each timestep, solve a constrained optimization
problem over N future steps, apply only the first control input.

Features:
    - Quadratic cost with terminal penalty (from LQR Riccati P)
    - Input constraints (thrust, torque limits)
    - Receding horizon for robustness to model mismatch
"""

import numpy as np
from scipy.linalg import expm, solve_continuous_are
from scipy.optimize import minimize

from src.controllers.controller_base import ControllerBase


class MPCController(ControllerBase):
    """
    MPC controller for quadrotor hover and position tracking.

    State vector (12):  [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    Control vector (4): [thrust_delta, tau_x, tau_y, tau_z]

    Uses discrete-time linearized model with ZOH discretization.
    """

    def __init__(
        self,
        mass: float = 7.8,
        gravity: float = 9.81,
        Ixx: float = 1.46,
        Iyy: float = 1.06,
        Izz: float = 2.50,
        drag_coeff: float = 0.05,
        horizon: int = 15,
        control_dt: float = 0.02,
        q_pos_xy: float = 20.0,
        q_pos_z: float = 50.0,
        q_vel: float = 10.0,
        q_angle: float = 200.0,
        q_rate: float = 5.0,
        r_thrust: float = 0.1,
        r_torque: float = 0.5,
        thrust_max_factor: float = 2.0,
        torque_max: float = 15.0,
    ):
        """
        Args:
            mass, gravity, Ixx/Iyy/Izz, drag_coeff: physical parameters
            horizon: prediction horizon N (number of steps)
            control_dt: discretization timestep for the prediction model
            q_*/r_*: cost weights (same meaning as LQR)
            thrust_max_factor: max thrust as factor of hover thrust
            torque_max: max torque per axis (N*m)
        """
        self.mass = mass
        self.gravity = gravity
        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz
        self.drag_coeff = drag_coeff
        self.N = horizon
        self.control_dt = control_dt
        self.thrust_max_factor = thrust_max_factor
        self.torque_max = torque_max

        self.nx = 12  # state dimension
        self.nu = 4  # control dimension

        # Max tilt for error clamping
        self._max_tilt = 0.4

        # Build continuous-time linearized system
        Ac, Bc = self._build_continuous_system()

        # Discretize with ZOH
        self.Ad, self.Bd = self._discretize(Ac, Bc, control_dt)

        # Cost matrices
        self.Q = np.diag(
            [
                q_pos_xy,
                q_pos_xy,
                q_pos_z,
                q_vel,
                q_vel,
                q_vel,
                q_angle,
                q_angle,
                q_angle,
                q_rate,
                q_rate,
                q_rate,
            ]
        )
        self.R = np.diag([r_thrust, r_torque, r_torque, r_torque])

        # Terminal cost from CARE (same as LQR, provides stability guarantee)
        self.P = solve_continuous_are(Ac, Bc, self.Q, self.R)

        # LQR gain for initial guess generation
        self._K_init = np.linalg.inv(self.R) @ Bc.T @ self.P

        # Warm start: store previous solution for faster convergence
        self._prev_solution = None

    @property
    def name(self) -> str:
        return "MPC"

    def _build_continuous_system(self) -> tuple[np.ndarray, np.ndarray]:
        """Build linearized continuous-time A, B matrices around hover."""
        m = self.mass
        g = self.gravity
        d = self.drag_coeff

        A = np.zeros((12, 12))

        A[0, 3] = 1.0
        A[1, 4] = 1.0
        A[2, 5] = 1.0

        A[3, 7] = -g
        A[3, 3] = -d / m
        A[4, 6] = g
        A[4, 4] = -d / m
        A[5, 5] = -d / m

        A[6, 9] = 1.0
        A[7, 10] = 1.0
        A[8, 11] = 1.0

        B = np.zeros((12, 4))
        B[5, 0] = -1.0 / m
        B[9, 1] = 1.0 / self.Ixx
        B[10, 2] = 1.0 / self.Iyy
        B[11, 3] = 1.0 / self.Izz

        return A, B

    @staticmethod
    def _discretize(
        A: np.ndarray, B: np.ndarray, dt: float
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Exact ZOH discretization using matrix exponential.

        [Ad, Bd] = expm([A B; 0 0] * dt) partitioned.
        """
        nx = A.shape[0]
        nu = B.shape[1]

        # Build augmented matrix [A B; 0 0]
        M = np.zeros((nx + nu, nx + nu))
        M[:nx, :nx] = A
        M[:nx, nx:] = B
        # bottom rows are zero

        Md = expm(M * dt)
        Ad = Md[:nx, :nx]
        Bd = Md[:nx, nx:]

        return Ad, Bd

    def _cost_function(self, u_flat: np.ndarray, x0: np.ndarray) -> float:
        """
        Evaluate the MPC cost for a sequence of controls.

        J = sum_{k=0}^{N-1} [x_k' Q x_k + u_k' R u_k] + x_N' P x_N
        """
        U = u_flat.reshape(self.N, self.nu)
        x = x0.copy()
        cost = 0.0

        for k in range(self.N):
            # Stage cost
            cost += x @ self.Q @ x + U[k] @ self.R @ U[k]
            # Propagate state
            x = self.Ad @ x + self.Bd @ U[k]

        # Terminal cost
        cost += x @ self.P @ x

        return cost

    def _cost_gradient(self, u_flat: np.ndarray, x0: np.ndarray) -> np.ndarray:
        """
        Analytical gradient of cost w.r.t. the control sequence.
        Computed via backward pass (adjoint method) for efficiency.
        """
        U = u_flat.reshape(self.N, self.nu)

        # Forward pass: store states
        states = np.zeros((self.N + 1, self.nx))
        states[0] = x0.copy()
        for k in range(self.N):
            states[k + 1] = self.Ad @ states[k] + self.Bd @ U[k]

        # Backward pass: compute adjoint (costate)
        grad = np.zeros((self.N, self.nu))
        lam = 2.0 * self.P @ states[self.N]  # terminal costate

        for k in range(self.N - 1, -1, -1):
            # Gradient w.r.t. u_k
            grad[k] = 2.0 * self.R @ U[k] + self.Bd.T @ lam
            # Propagate adjoint backward
            lam = 2.0 * self.Q @ states[k] + self.Ad.T @ lam

        return grad.ravel()

    def reset(self):
        """Clear the warm start solution."""
        self._prev_solution = None

    def compute(
        self,
        position_setpoint: np.ndarray,
        yaw_setpoint: float,
        current_state: np.ndarray,
        dt: float,
    ) -> tuple[float, np.ndarray]:
        """
        Compute MPC control output via receding horizon optimization.

        Args:
            position_setpoint: desired [x, y, z] in NED (m)
            yaw_setpoint: desired yaw angle (rad)
            current_state: full 12-element state vector
            dt: timestep (s) — the prediction model uses its own control_dt

        Returns:
            (thrust, torque): thrust in N, torque as [tau_x, tau_y, tau_z]
        """
        # Build reference state
        x_ref = np.zeros(12)
        x_ref[0:3] = position_setpoint
        x_ref[8] = yaw_setpoint

        # Error state (MPC optimizes around deviation from reference)
        x_err = current_state - x_ref
        x_err[8] = (x_err[8] + np.pi) % (2 * np.pi) - np.pi

        # Dual Mode MPC: if we are very close to the setpoint, the constraints
        # are inactive. We bypass the numerical optimizer and use the LQR gain directly.
        # This prevents numerical solver noise from causing exponential drift near hover.
        if np.linalg.norm(x_err) < 0.05:
            u_lqr = -self._K_init @ x_err
            thrust = self.mass * self.gravity + u_lqr[0]
            return float(thrust), u_lqr[1:4]

        # Control bounds (on deviation from hover)
        hover_thrust = self.mass * self.gravity
        max_dT = (self.thrust_max_factor - 1.0) * hover_thrust
        tau_max = self.torque_max

        bounds = []
        for _ in range(self.N):
            bounds.append((-hover_thrust, max_dT))  # thrust deviation
            bounds.append((-tau_max, tau_max))  # tau_x
            bounds.append((-tau_max, tau_max))  # tau_y
            bounds.append((-tau_max, tau_max))  # tau_z

        # Initial guess: LQR-based rollout
        # This prevents numerical noise from persisting across steps when the optimizer
        # exits early (which caused attitude drift near the setpoint).
        u0 = np.zeros(self.N * self.nu)
        x_pred = x_err.copy()
        for k in range(self.N):
            u_lqr = -self._K_init @ x_pred
            # Clip to bounds
            u_lqr[0] = np.clip(u_lqr[0], -hover_thrust, max_dT)
            u_lqr[1:4] = np.clip(u_lqr[1:4], -tau_max, tau_max)
            u0[k * self.nu: (k + 1) * self.nu] = u_lqr
            x_pred = self.Ad @ x_pred + self.Bd @ u_lqr

        # Solve QP via L-BFGS-B (handles box constraints)
        result = minimize(
            self._cost_function,
            u0,
            args=(x_err,),
            method="L-BFGS-B",
            jac=self._cost_gradient,
            bounds=bounds,
            options={
                "maxiter": 100,
                "ftol": 1e-8,
            },
        )

        # Store for warm start
        self._prev_solution = result.x.copy()

        # Extract first control action (receding horizon)
        u_opt = result.x[: self.nu]

        # Convert from deviation to absolute
        thrust = hover_thrust + u_opt[0]
        thrust = np.clip(thrust, 0.0, self.thrust_max_factor * hover_thrust)

        torque = np.clip(u_opt[1:4], -tau_max, tau_max)

        return float(thrust), torque
