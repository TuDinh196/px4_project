"""
Unit Tests for Quadplane Condor Hybrid VTOL Physics & Control
============================================================
Validates:
  1. Wing Aerodynamics (Lift, Induced Drag, Stall dynamics)
  2. V-Tail Moments (Elevator pitch, Rudder yaw)
  3. 6-DOF Hybrid Quadplane Flight Dynamics (4 VTOL + 1 Tractor)
  4. Hybrid VTOL Flight State Machine (5 phases: VTOL_HOVER -> FWD_TRANS ->
     FW_CRUISE -> BACK_TRANS -> LAND)
  5. L1/NPFG Fixed-Wing Path Follower (Cross-track error, coordinated bank turn angle)
"""

import sys
import unittest
from pathlib import Path

import numpy as np

# Add project root
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.controllers.condor_path_follower import FixedWingPathFollower  # noqa: E402
from src.controllers.condor_vtol_controller import VTOLHybridController  # noqa: E402
from src.uav_model.condor_dynamics import (  # noqa: E402
    FlightPhase,
    QuadplaneDynamics,
    QuadplaneParams,
)


class TestQuadplaneAerodynamics(unittest.TestCase):
    """Test aerodynamic lift, drag, and V-tail moments for Quadplane Condor."""

    def setUp(self):
        self.params = QuadplaneParams()
        self.drone = QuadplaneDynamics(self.params)

    def test_wing_lift_at_cruise_speed(self):
        """At cruise speed (18 m/s) and moderate AoA, wing lift supports weight (76.5 N)."""
        u_b = 18.0  # 18 m/s forward speed
        w_b = 1.0   # small downward body velocity -> positive AoA ~3.18 deg

        F_aero, M_aero = self.drone.compute_aerodynamics(u_b, w_b, 0.0)

        # Fz in body frame is negative for upward aerodynamic lift
        lift_force = -F_aero[2]
        weight_n = self.params.mass * 9.81
        print(f"Calculated Wing Lift at 18 m/s: {lift_force:.2f} N (Weight = {weight_n:.2f} N)")

        self.assertGreater(lift_force, 40.0, "Wing should produce substantial lift at cruise speed")
        self.assertLess(lift_force, 150.0, "Lift should be within physical wing envelope")

    def test_drag_increases_quadratically_with_speed(self):
        """Aerodynamic drag should scale with dynamic pressure (V²)."""
        F1, _ = self.drone.compute_aerodynamics(10.0, 0.0, 0.0)
        F2, _ = self.drone.compute_aerodynamics(20.0, 0.0, 0.0)

        drag_10 = -F1[0]  # Fx is negative drag
        drag_20 = -F2[0]

        ratio = drag_20 / max(drag_10, 1e-4)
        msg = "Drag should roughly quadruple when speed doubles"
        self.assertAlmostEqual(ratio, 4.0, delta=0.5, msg=msg)

    def test_vtail_pitch_control(self):
        """Elevator deflection should produce restoring pitching moment."""
        _, M_pos = self.drone.compute_aerodynamics(18.0, 0.0, 0.0, delta_e=+0.2)
        _, M_neg = self.drone.compute_aerodynamics(18.0, 0.0, 0.0, delta_e=-0.2)

        # Positive delta_e (trailing edge down) produces negative pitch moment (pitch down)
        self.assertLess(M_pos[1], 0.0)
        self.assertGreater(M_neg[1], 0.0)


class TestQuadplaneDynamics(unittest.TestCase):
    """Test 6-DOF Hybrid Quadplane equations of motion."""

    def setUp(self):
        self.params = QuadplaneParams()
        self.drone = QuadplaneDynamics(self.params)

    def test_vtol_hover_equilibrium(self):
        """4 VTOL motors at hover thrust should maintain zero vertical acceleration."""
        hover_thrust = self.params.hover_thrust
        for _ in range(200):  # 1 second
            self.drone.step(vtol_thrust=hover_thrust, vtol_torque=np.zeros(3))

        self.assertLess(abs(self.drone.altitude), 0.5, "Hover thrust should hold altitude")

    def test_tractor_motor_acceleration(self):
        """Nose tractor motor should accelerate the aircraft forward."""
        self.drone.reset()
        for _ in range(200):  # 1 second with 30N forward puller thrust
            self.drone.step(
                vtol_thrust=self.params.hover_thrust,
                vtol_torque=np.zeros(3),
                tractor_thrust=30.0,
            )

        forward_vel = self.drone.state[3]  # vx in NED
        self.assertGreater(forward_vel, 2.0, "Tractor motor should accelerate aircraft forward")


class TestVTOLHybridController(unittest.TestCase):
    """Test Hybrid VTOL flight controller and state machine transitions."""

    def setUp(self):
        self.params = QuadplaneParams()
        self.controller = VTOLHybridController(self.params)

    def test_vtol_hover_mode_outputs(self):
        """In VTOL_HOVER mode, tractor thrust should be 0 and VTOL thrust should be active."""
        state = np.zeros(12)
        state[2] = -5.0  # 5m altitude (below transition altitude 15m)
        target = np.array([0.0, 0.0, -15.0])

        res = self.controller.compute_hybrid(target, 18.0, state, 0.005)

        self.assertEqual(res["phase"], FlightPhase.VTOL_HOVER)
        self.assertGreater(res["vtol_thrust"], 0.0)
        self.assertEqual(res["tractor_thrust"], 0.0)
        self.assertEqual(res["blend_factor"], 1.0)

    def test_forward_transition_triggers_at_transition_altitude(self):
        """When reaching transition altitude, switch to FORWARD_TRANSITION and fire tractor."""
        state = np.zeros(12)
        state[2] = -15.0  # 15m altitude
        target = np.array([100.0, 0.0, -25.0])

        res = self.controller.compute_hybrid(target, 18.0, state, 0.005)

        self.assertEqual(res["phase"], FlightPhase.FORWARD_TRANSITION)
        msg = "Tractor thrust must be active during forward transition"
        self.assertGreater(res["tractor_thrust"], 20.0, msg)


class TestFixedWingPathFollower(unittest.TestCase):
    """Test L1/NPFG nonlinear path following."""

    def setUp(self):
        self.follower = FixedWingPathFollower(lookahead_distance=30.0)

    def test_on_track_zero_cross_track(self):
        """When flying along track, cross-track error should be zero."""
        pos = np.array([50.0, 0.0])
        start = np.array([0.0, 0.0])
        end = np.array([100.0, 0.0])

        cte, lookahead_pt = self.follower.compute_cross_track_error(pos, start, end)
        self.assertAlmostEqual(cte, 0.0, delta=1e-3)
        self.assertAlmostEqual(lookahead_pt[0], 80.0, delta=1e-3)

    def test_coordinated_turn_bank_angle(self):
        """Lateral error should command coordinated bank angle."""
        pos = np.array([50.0, 20.0])  # 20m right of track
        vel = np.array([18.0, 0.0])   # 18 m/s forward speed
        target_pt = np.array([80.0, 0.0])

        a_lat, phi_cmd = self.follower.compute_lateral_acceleration(pos, vel, target_pt)
        self.assertNotEqual(phi_cmd, 0.0)
        self.assertLess(abs(phi_cmd), 0.70, "Bank angle should be within safe flight limits")


if __name__ == "__main__":
    unittest.main()
