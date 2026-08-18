"""
Quadplane Condor 6-DOF Aerodynamic Dynamics Model
=================================================
Simulates the complete physics of a Hybrid VTOL UAV (4+1 tractor layout):
  - 4 VTOL lift motors providing vertical thrust & hover attitude moments
  - 1 Nose tractor motor providing forward thrust along body +X axis
  - High-aspect-ratio 2.4m wing with lift, induced drag, and stall dynamics
  - V-tail aerodynamic control surfaces (ruddervators for pitch & yaw control)
  - 6-DOF rigid body dynamics using Newton-Euler equations in NED frame.

State vector (12):
  [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
   Position(NED)  Velocity(NED)   Euler angles    Body rates
"""

from dataclasses import dataclass
from pathlib import Path
from enum import Enum
import numpy as np
import yaml


class FlightPhase(Enum):
    VTOL_HOVER = "VTOL_HOVER"
    FORWARD_TRANSITION = "FORWARD_TRANSITION"
    FIXED_WING_CRUISE = "FIXED_WING_CRUISE"
    BACK_TRANSITION = "BACK_TRANSITION"
    VTOL_LAND = "VTOL_LAND"


@dataclass
class QuadplaneParams:
    """Complete physical and aerodynamic parameters for Quadplane Condor."""

    mass: float = 7.8
    wingspan: float = 2.40
    wing_area: float = 0.42
    wing_chord: float = 0.175
    aspect_ratio: float = 13.71
    Ixx: float = 1.46
    Iyy: float = 1.06
    Izz: float = 2.50
    air_density: float = 1.225
    CL0: float = 0.28
    CL_alpha: float = 4.86
    CD0: float = 0.024
    induced_drag_k: float = 0.028
    alpha_stall: float = 0.297
    v_stall: float = 12.0
    v_trans: float = 15.0
    v_cruise: float = 18.0
    v_max: float = 25.0
    vtail_area: float = 0.12
    vtail_arm: float = 0.8655
    max_tractor_thrust: float = 34.0
    gravity: float = 9.81
    dt: float = 0.005

    @classmethod
    def from_yaml(cls, path: str | Path | None = None) -> "QuadplaneParams":
        """Load parameters from YAML file."""
        if path is None:
            path = Path(__file__).resolve().parents[0] / "parameters.yaml"
        with open(path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        return cls(
            mass=cfg["mass"],
            wingspan=cfg["wingspan"],
            wing_area=cfg["wing_area"],
            wing_chord=cfg["wing_chord"],
            aspect_ratio=cfg["aspect_ratio"],
            Ixx=cfg["inertia"]["Ixx"],
            Iyy=cfg["inertia"]["Iyy"],
            Izz=cfg["inertia"]["Izz"],
            air_density=cfg["aerodynamics"]["air_density"],
            CL0=cfg["aerodynamics"]["CL0"],
            CL_alpha=cfg["aerodynamics"]["CL_alpha"],
            CD0=cfg["aerodynamics"]["CD0"],
            induced_drag_k=cfg["aerodynamics"]["induced_drag_factor"],
            alpha_stall=cfg["aerodynamics"]["alpha_stall"],
            v_stall=cfg["vtol_transition"]["v_stall"],
            v_trans=cfg["vtol_transition"]["v_trans"],
            v_cruise=cfg["vtol_transition"]["v_cruise"],
            v_max=cfg["vtol_transition"]["v_max"],
            vtail_area=cfg["vtail"]["area"],
            vtail_arm=cfg["vtail"]["lever_arm"],
            max_tractor_thrust=cfg["tractor_motor"]["max_thrust"],
            gravity=cfg["gravity"],
            dt=cfg["simulation"]["dt"],
        )

    @property
    def inertia_matrix(self) -> np.ndarray:
        return np.diag([self.Ixx, self.Iyy, self.Izz])

    @property
    def inertia_inv(self) -> np.ndarray:
        return np.diag([1.0 / self.Ixx, 1.0 / self.Iyy, 1.0 / self.Izz])

    @property
    def hover_thrust(self) -> float:
        """Total vertical thrust required to hover (N)."""
        return self.mass * self.gravity

    @property
    def drag_coeff(self) -> float:
        """Parasitic zero-lift drag coefficient (alias for CD0)."""
        return self.CD0

    @property
    def rho(self) -> float:
        """Air density (alias for air_density)."""
        return self.air_density


# Alias QuadrotorParams to QuadplaneParams for backwards compatibility in existing tests
QuadrotorParams = QuadplaneParams


class QuadplaneDynamics:
    """
    6-DOF Hybrid Quadplane Flight Dynamics Simulation.

    Integrates:
      1. VTOL multicopter vertical thrust & torque commands
      2. Forward tractor propeller thrust
      3. Aerodynamic wing lift, parasitic drag, induced drag, and post-stall blending
      4. V-Tail ruddervator control moments (pitch and yaw)
    """

    NUM_STATES = 12

    def __init__(self, params: QuadplaneParams | None = None):
        if params is None:
            params = QuadplaneParams()
        self.params = params
        self.state = np.zeros(self.NUM_STATES)
        self.time = 0.0

    def reset(self, initial_state: np.ndarray | None = None):
        """Reset the simulation state."""
        if initial_state is not None:
            assert len(initial_state) == self.NUM_STATES
            self.state = np.array(initial_state, dtype=float)
        else:
            self.state = np.zeros(self.NUM_STATES)
        self.time = 0.0

    @staticmethod
    def rotation_matrix(phi: float, theta: float, psi: float) -> np.ndarray:
        """ZYX Euler rotation matrix: body frame -> NED frame."""
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        cpsi, spsi = np.cos(psi), np.sin(psi)

        return np.array(
            [
                [cpsi * cth, cpsi * sth * sphi - spsi * cphi, cpsi * sth * cphi + spsi * sphi],
                [spsi * cth, spsi * sth * sphi + cpsi * cphi, spsi * sth * cphi - cpsi * sphi],
                [-sth, cth * sphi, cth * cphi],
            ]
        )

    @staticmethod
    def euler_rate_matrix(phi: float, theta: float) -> np.ndarray:
        """Matrix converting body rates [p, q, r] to Euler angle rates."""
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth = np.cos(theta)
        if abs(cth) < 1e-4:
            cth = 1e-4 * (np.sign(cth) if cth != 0 else 1.0)
        tth = np.sin(theta) / cth

        return np.array(
            [
                [1.0, sphi * tth, cphi * tth],
                [0.0, cphi, -sphi],
                [0.0, sphi / cth, cphi / cth],
            ]
        )

    def compute_aerodynamics(
        self,
        u_body: float,
        w_body: float,
        v_body: float,
        delta_e: float = 0.0,
        delta_r: float = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Calculates total aerodynamic forces and moments in body frame.

        Args:
            u_body: Forward velocity in body frame (m/s)
            w_body: Downward velocity in body frame (m/s)
            v_body: Lateral velocity in body frame (m/s)
            delta_e: Elevator deflection (rad, positive trailing edge down)
            delta_r: Rudder deflection (rad)

        Returns:
            F_aero: [Fx, Fy, Fz] aerodynamic forces in body frame (N)
            M_aero: [Mx, My, Mz] aerodynamic moments in body frame (N*m)
        """
        p = self.params
        airspeed = np.sqrt(u_body**2 + v_body**2 + w_body**2)

        if airspeed < 0.1:
            return np.zeros(3), np.zeros(3)

        # Angle of attack (alpha) and sideslip (beta)
        alpha = np.arctan2(w_body, max(u_body, 0.01))
        beta = np.arcsin(np.clip(v_body / airspeed, -1.0, 1.0))

        # Dynamic pressure: q = 0.5 * rho * V²
        q_dyn = 0.5 * p.air_density * (airspeed**2)

        # Lift coefficient CL with linear region and stall modeling
        if abs(alpha) < p.alpha_stall:
            CL = p.CL0 + p.CL_alpha * alpha
        else:
            # Post-stall flat-plate approximation
            sign_a = np.sign(alpha)
            CL = sign_a * (p.CL0 + p.CL_alpha * p.alpha_stall) * np.cos(alpha)

        # Drag coefficient CD = CD0 + k * CL²
        CD = p.CD0 + p.induced_drag_k * (CL**2) + 0.1 * (np.sin(beta) ** 2)

        # Aerodynamic Lift and Drag magnitudes
        Lift = q_dyn * p.wing_area * CL
        Drag = q_dyn * p.wing_area * CD

        # Transform from wind frame (Lift, Drag) to body frame:
        # Fx = -Drag*cos(alpha) + Lift*sin(alpha)
        # Fz = -Drag*sin(alpha) - Lift*cos(alpha)
        Fx_aero = -Drag * np.cos(alpha) + Lift * np.sin(alpha)
        Fy_aero = -q_dyn * p.wing_area * 0.2 * beta
        Fz_aero = -Drag * np.sin(alpha) - Lift * np.cos(alpha)

        F_aero = np.array([Fx_aero, Fy_aero, Fz_aero])

        # V-Tail Pitch & Yaw Moments:
        # Pitching moment from V-tail elevator: My = q_dyn * S_tail * arm * Cm_delta * delta_e
        My_aero = -q_dyn * p.vtail_area * p.vtail_arm * (0.8 * delta_e + 0.15 * alpha)
        # Yawing moment from V-tail rudder: Mz = q_dyn * S_tail * arm * Cn_delta * delta_r
        Mz_aero = q_dyn * p.vtail_area * p.vtail_arm * (0.6 * delta_r - 0.1 * beta)
        # Roll damping moment
        Mx_aero = -q_dyn * p.wing_area * p.wingspan * 0.05 * beta

        M_aero = np.array([Mx_aero, My_aero, Mz_aero])

        return F_aero, M_aero

    def _derivatives(
        self,
        state: np.ndarray,
        vtol_thrust: float,
        vtol_torque: np.ndarray,
        tractor_thrust: float = 0.0,
        delta_e: float = 0.0,
        delta_r: float = 0.0,
    ) -> np.ndarray:
        """Compute state derivatives (xdot) for 6-DOF Hybrid Quadplane."""
        p = self.params

        vel = state[3:6]
        phi, theta, psi = state[6:9]
        omega = state[9:12]

        R = self.rotation_matrix(phi, theta, psi)
        R_inv = R.T  # NED -> body

        # Velocity in body frame
        vel_body = R_inv @ vel
        u_b, v_b, w_b = vel_body[0], vel_body[1], vel_body[2]

        # 1. Aerodynamic forces & moments
        F_aero_b, M_aero_b = self.compute_aerodynamics(u_b, w_b, v_b, delta_e, delta_r)

        # 2. Propulsion forces in body frame
        # VTOL motors produce upward thrust (negative body Z in NED)
        F_vtol_b = np.array([0.0, 0.0, -vtol_thrust])
        # Tractor motor produces forward thrust (positive body X in NED)
        F_tractor_b = np.array([tractor_thrust, 0.0, 0.0])

        F_prop_b = F_vtol_b + F_tractor_b
        F_total_b = F_prop_b + F_aero_b

        # Transform total body forces to NED frame
        F_total_ned = R @ F_total_b
        gravity_ned = np.array([0.0, 0.0, p.mass * p.gravity])

        # Linear acceleration in NED frame
        accel_ned = (F_total_ned + gravity_ned) / p.mass

        # 3. Rotational dynamics in body frame
        M_total_b = vtol_torque + M_aero_b
        J = p.inertia_matrix
        J_inv = p.inertia_inv
        omega_dot = J_inv @ (M_total_b - np.cross(omega, J @ omega))

        # 4. Euler angle rates
        W = self.euler_rate_matrix(phi, theta)
        euler_dot = W @ omega

        xdot = np.zeros(self.NUM_STATES)
        xdot[0:3] = vel
        xdot[3:6] = accel_ned
        xdot[6:9] = euler_dot
        xdot[9:12] = omega_dot

        return xdot

    def step(
        self,
        vtol_thrust: float,
        vtol_torque: np.ndarray,
        dt: float | None = None,
        tractor_thrust: float = 0.0,
        delta_e: float = 0.0,
        delta_r: float = 0.0,
    ) -> np.ndarray:
        """
        Advance simulation by one timestep using 4th-order Runge-Kutta (RK4).

        Args:
            vtol_thrust: Total vertical thrust from 4 VTOL motors (N, positive upward)
            vtol_torque: [tau_x, tau_y, tau_z] attitude control moments (N*m)
            tractor_thrust: Forward thrust from nose tractor motor (N, positive forward)
            delta_e: V-tail elevator command (rad)
            delta_r: V-tail rudder command (rad)
            dt: Timestep (default: params.dt)

        Returns:
            Updated state vector (12,)
        """
        if dt is None:
            dt = self.params.dt
        vtol_torque = np.asarray(vtol_torque, dtype=float)

        s = self.state
        k1 = self._derivatives(s, vtol_thrust, vtol_torque, tractor_thrust, delta_e, delta_r)
        k2 = self._derivatives(
            s + 0.5 * dt * k1, vtol_thrust, vtol_torque, tractor_thrust, delta_e, delta_r
        )
        k3 = self._derivatives(
            s + 0.5 * dt * k2, vtol_thrust, vtol_torque, tractor_thrust, delta_e, delta_r
        )
        k4 = self._derivatives(
            s + dt * k3, vtol_thrust, vtol_torque, tractor_thrust, delta_e, delta_r
        )

        self.state = s + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        # Normalize Euler angles to [-pi, pi]
        self.state[6] = (self.state[6] + np.pi) % (2 * np.pi) - np.pi
        self.state[7] = (self.state[7] + np.pi) % (2 * np.pi) - np.pi
        self.state[8] = (self.state[8] + np.pi) % (2 * np.pi) - np.pi

        self.time += dt
        return self.state.copy()

    # ---- Convenience Properties ----
    @property
    def position(self) -> np.ndarray:
        return self.state[0:3]

    @property
    def velocity(self) -> np.ndarray:
        return self.state[3:6]

    @property
    def airspeed(self) -> float:
        """Total true airspeed magnitude (m/s)."""
        return float(np.linalg.norm(self.state[3:6]))

    @property
    def euler_angles(self) -> np.ndarray:
        return self.state[6:9]

    @property
    def body_rates(self) -> np.ndarray:
        return self.state[9:12]

    @property
    def altitude(self) -> float:
        """Altitude above ground (m, positive up)."""
        return -self.state[2]


# Backward compatibility alias
QuadrotorDynamics = QuadplaneDynamics
