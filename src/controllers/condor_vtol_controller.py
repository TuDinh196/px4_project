"""
Hybrid VTOL Controller for Quadplane Condor (4+1 Layout)
========================================================
Implements full 5-phase flight control:
  1. VTOL Takeoff & Hover (4 vertical motors, SE(3) tracking)
  2. Forward Transition (Tractor motor ramps up, VTOL motors blend down with dynamic wing lift)
  3. Fixed-Wing Cruise (TECS Total Energy Control + Coordinated Bank Turns, VTOL motors OFF)
  4. Back Transition (Airspeed deceleration + Quad motor spin-up)
  5. Precision VTOL Landing

Authors: Autonomous UAV Navigation Team
"""

import numpy as np

from src.controllers.controller_base import ControllerBase
from src.controllers.geometric_controller import GeometricController
from src.controllers.pid_controller import PIDController
from src.uav_model.condor_dynamics import FlightPhase, QuadplaneParams


class VTOLHybridController(ControllerBase):
    """
    Unified Hybrid VTOL Flight Controller for Quadplane Condor.
    """

    def __init__(
        self,
        params: QuadplaneParams | None = None,
        target_cruise_speed: float = 18.0,
        transition_altitude: float = 15.0,
    ):
        if params is None:
            params = QuadplaneParams()
        self.params = params
        self.target_cruise_speed = target_cruise_speed
        self.transition_altitude = transition_altitude
        self.phase = FlightPhase.VTOL_HOVER

        # 1. Multicopter SE(3) Controller for VTOL phases
        self.mc_controller = GeometricController(
            mass=params.mass,
            gravity=params.gravity,
            Ixx=params.Ixx,
            Iyy=params.Iyy,
            Izz=params.Izz,
            kp=5.0,
            kv=3.5,
            kR=40.0,
            kw=8.0,
        )

        # 2. Fixed-Wing Airspeed & Altitude Controllers (TECS)
        # Airspeed PID -> Tractor Thrust
        self.pid_airspeed = PIDController(
            kp=3.5,
            ki=0.4,
            kd=0.5,
            output_min=0.0,
            output_max=params.max_tractor_thrust,
            integral_min=-5.0,
            integral_max=15.0,
        )

        # Altitude PID -> Desired Pitch Angle (rad)
        self.pid_altitude = PIDController(
            kp=0.08,
            ki=0.01,
            kd=0.04,
            output_min=-0.26,  # -15 deg max dive
            output_max=0.35,  # +20 deg max climb
            integral_min=-0.1,
            integral_max=0.1,
        )

        # Pitch Angle PID -> Elevator Deflection (rad)
        self.pid_pitch_fw = PIDController(
            kp=1.8,
            ki=0.1,
            kd=0.15,
            output_min=-0.52,  # -30 deg
            output_max=0.52,  # +30 deg
        )

        # Lateral Cross-Track / Heading PID -> Desired Roll Angle (rad)
        self.pid_heading_fw = PIDController(
            kp=1.5,
            ki=0.05,
            kd=0.4,
            output_min=-0.60,  # -35 deg bank angle
            output_max=0.60,  # +35 deg bank angle
            integral_min=-0.2,
            integral_max=0.2,
        )

        # Roll Angle PID -> Aileron / V-Tail Roll Moment
        self.pid_roll_fw = PIDController(
            kp=2.5,
            ki=0.1,
            kd=0.2,
            output_min=-0.52,
            output_max=0.52,
        )

        # Internal timers
        self.phase_start_time = 0.0
        self.current_time = 0.0

    @property
    def name(self) -> str:
        return "VTOL Hybrid Controller (Condor 4+1)"

    def reset(self):
        self.phase = FlightPhase.VTOL_HOVER
        self.mc_controller.reset()
        self.pid_airspeed.reset()
        self.pid_altitude.reset()
        self.pid_pitch_fw.reset()
        self.pid_heading_fw.reset()
        self.pid_roll_fw.reset()
        self.phase_start_time = 0.0
        self.current_time = 0.0

    def compute_hybrid(
        self,
        target_pos_ned: np.ndarray,
        target_airspeed: float,
        current_state: np.ndarray,
        dt: float,
    ) -> dict[str, float | np.ndarray | FlightPhase]:
        """
        Computes all control signals across 4+1 hybrid layout.

        Args:
            target_pos_ned: [North, East, Down] target position (m)
            target_airspeed: Target forward airspeed in fixed-wing mode (m/s)
            current_state: 12-state vector
            dt: Timestep (s)

        Returns:
            Dictionary containing:
              - 'vtol_thrust': Total vertical thrust from 4 lift motors (N)
              - 'vtol_torque': [tau_x, tau_y, tau_z] from lift motors (N*m)
              - 'tractor_thrust': Forward thrust from nose motor (N)
              - 'delta_e': Elevator deflection (rad)
              - 'delta_r': Rudder deflection (rad)
              - 'phase': Current FlightPhase
              - 'blend_factor': 0.0 (Pure FW) to 1.0 (Pure MC)
        """
        self.current_time += dt
        p = self.params

        pos = current_state[0:3]
        vel = current_state[3:6]
        phi, theta, psi = current_state[6:9]
        airspeed = float(np.linalg.norm(vel))
        altitude = -pos[2]
        target_alt = -target_pos_ned[2]

        vtol_thrust = 0.0
        vtol_torque = np.zeros(3)
        tractor_thrust = 0.0
        delta_e = 0.0
        delta_r = 0.0
        blend_factor = 1.0

        # =====================================================================
        # STATE MACHINE TRANSITIONS
        # =====================================================================
        if self.phase == FlightPhase.VTOL_HOVER:
            # When near transition altitude (e.g. 15m), initiate forward transition
            if altitude >= self.transition_altitude - 1.0:
                self.phase = FlightPhase.FORWARD_TRANSITION
                self.phase_start_time = self.current_time

        elif self.phase == FlightPhase.FORWARD_TRANSITION:
            # Transition complete when airspeed reaches transition threshold (15 m/s)
            if airspeed >= p.v_trans:
                self.phase = FlightPhase.FIXED_WING_CRUISE
                self.phase_start_time = self.current_time

        elif self.phase == FlightPhase.FIXED_WING_CRUISE:
            # Check distance to target landing location
            dist_to_wp = np.linalg.norm(pos[0:2] - target_pos_ned[0:2])
            if dist_to_wp < 30.0:  # Within 30m of landing point
                self.phase = FlightPhase.BACK_TRANSITION
                self.phase_start_time = self.current_time

        elif self.phase == FlightPhase.BACK_TRANSITION:
            if airspeed < 5.0:
                self.phase = FlightPhase.VTOL_LAND
                self.phase_start_time = self.current_time

        # =====================================================================
        # CONTROL COMPUTATIONS PER PHASE
        # =====================================================================
        if self.phase == FlightPhase.VTOL_HOVER or self.phase == FlightPhase.VTOL_LAND:
            blend_factor = 1.0
            tractor_thrust = 0.0
            sp = np.array([
                target_pos_ned[0], target_pos_ned[1], target_pos_ned[2], 0, 0, 0, 0, 0, 0
            ])
            vtol_thrust, vtol_torque = self.mc_controller.compute(
                position_setpoint=sp, yaw_setpoint=0.0, current_state=current_state, dt=dt
            )

        elif self.phase == FlightPhase.FORWARD_TRANSITION:
            # 1. Tractor motor pushes at maximum transition thrust
            tractor_thrust = p.max_tractor_thrust * 0.95

            # 2. Dynamic lift blending factor:
            # As airspeed approaches v_trans, wing lift supports weight -> reduce VTOL thrust
            speed_ratio = min(airspeed / p.v_trans, 1.0)
            blend_factor = float(np.clip(1.0 - (speed_ratio**2), 0.0, 1.0))

            # Altitude hold via blended MC thrust + small positive pitch
            sp = np.array([
                pos[0] + 50.0, target_pos_ned[1], target_pos_ned[2], 0, 0, 0, 0, 0, 0
            ])
            raw_vtol_thrust, vtol_torque = self.mc_controller.compute(
                position_setpoint=sp, yaw_setpoint=0.0, current_state=current_state, dt=dt
            )
            vtol_thrust = raw_vtol_thrust * blend_factor

        elif self.phase == FlightPhase.FIXED_WING_CRUISE:
            blend_factor = 0.0
            vtol_thrust = 0.0
            vtol_torque = np.zeros(3)

            # 1. Forward Speed Control -> Tractor Motor
            speed_err = target_airspeed - airspeed
            tractor_thrust = self.pid_airspeed.update(speed_err, dt)

            # 2. Altitude Control -> Desired Pitch Angle
            alt_err = target_alt - altitude
            desired_pitch = self.pid_altitude.update(alt_err, dt)

            # Pitch Control -> Elevator Deflection
            pitch_err = desired_pitch - theta
            delta_e = self.pid_pitch_fw.update(pitch_err, dt)

            # 3. Horizontal Navigation -> Desired Roll Angle
            # Compute cross-track and heading error to target
            dx = target_pos_ned[0] - pos[0]
            dy = target_pos_ned[1] - pos[1]
            desired_heading = np.arctan2(dy, dx)
            heading_err = (desired_heading - psi + np.pi) % (2 * np.pi) - np.pi

            desired_roll = self.pid_heading_fw.update(heading_err, dt)
            roll_err = desired_roll - phi
            delta_r = -0.5 * self.pid_roll_fw.update(roll_err, dt)

        elif self.phase == FlightPhase.BACK_TRANSITION:
            # Tractor motor cuts off, quad motors spin up as speed drops
            tractor_thrust = 0.0
            speed_ratio = min(airspeed / p.v_trans, 1.0)
            blend_factor = float(np.clip(1.0 - (speed_ratio**2), 0.0, 1.0))

            sp = np.array([
                target_pos_ned[0], target_pos_ned[1], target_pos_ned[2], 0, 0, 0, 0, 0, 0
            ])
            raw_vtol_thrust, vtol_torque = self.mc_controller.compute(
                position_setpoint=sp, yaw_setpoint=0.0, current_state=current_state, dt=dt
            )
            vtol_thrust = raw_vtol_thrust * max(blend_factor, 0.4)

        return {
            "vtol_thrust": float(vtol_thrust),
            "vtol_torque": vtol_torque,
            "tractor_thrust": float(tractor_thrust),
            "delta_e": float(delta_e),
            "delta_r": float(delta_r),
            "phase": self.phase,
            "blend_factor": float(blend_factor),
        }

    def compute(
        self,
        position_setpoint: np.ndarray,
        yaw_setpoint: float,
        current_state: np.ndarray,
        dt: float,
    ) -> tuple[float, np.ndarray]:
        """ControllerBase standard interface compatibility."""
        res = self.compute_hybrid(position_setpoint, self.target_cruise_speed, current_state, dt)
        return res["vtol_thrust"], res["vtol_torque"]
