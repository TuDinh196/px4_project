"""Unit tests for PID controller, Quadrotor dynamics, LQR, MPC, and Controller interface."""

import sys
import numpy as np
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.controllers.pid_controller import PIDController  # noqa: E402
from src.controllers.controller_base import ControllerBase  # noqa: E402
from src.controllers.cascade_controller import CascadeController  # noqa: E402
from src.controllers.lqr_controller import LQRController  # noqa: E402
from src.controllers.mpc_controller import MPCController  # noqa: E402
from src.controllers.geometric_controller import GeometricController  # noqa: E402
from src.uav_model.condor_dynamics import (  # noqa: E402
    QuadplaneDynamics as QuadrotorDynamics,
    QuadplaneParams as QuadrotorParams,
)


class TestPIDController:
    """Tests for PIDController."""

    def test_proportional_only(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
        output = pid.update(error=5.0, dt=0.01)
        assert abs(output - 10.0) < 1e-6, f"P-only output should be 10.0, got {output}"

    def test_zero_error(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
        output = pid.update(error=0.0, dt=0.01)
        assert abs(output) < 1e-6, f"Zero error should give zero output, got {output}"

    def test_integral_accumulates(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
        for _ in range(100):
            pid.update(error=1.0, dt=0.01)
        output = pid.update(error=1.0, dt=0.01)
        # After 101 steps: integral ~= 101*0.01 = 1.01, output = 1.0 * 1.01
        assert output > 0.9, f"Integral should accumulate, got {output}"

    def test_anti_windup(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, integral_min=-0.5, integral_max=0.5)
        for _ in range(1000):
            pid.update(error=10.0, dt=0.01)
        output = pid.update(error=10.0, dt=0.01)
        assert (
            abs(output - 0.5) < 1e-6
        ), f"Anti-windup should clamp at 0.5, got {output}"

    def test_output_saturation(self):
        pid = PIDController(kp=100.0, output_min=-5.0, output_max=5.0)
        output = pid.update(error=10.0, dt=0.01)
        assert abs(output - 5.0) < 1e-6, f"Output should saturate at 5.0, got {output}"

    def test_reset(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
        pid.update(error=5.0, dt=0.01)
        pid.reset()
        assert pid._integral == 0.0
        assert pid._prev_error == 0.0

    def test_negative_error(self):
        pid = PIDController(kp=2.0)
        output = pid.update(error=-3.0, dt=0.01)
        assert abs(output - (-6.0)) < 1e-6


class TestQuadrotorDynamics:
    """Tests for QuadrotorDynamics."""

    def test_initial_state_zero(self):
        drone = QuadrotorDynamics()
        assert np.allclose(drone.state, 0.0), "Initial state should be all zeros"

    def test_hover_thrust_counteracts_gravity(self):
        params = QuadrotorParams()
        drone = QuadrotorDynamics(params)

        # Apply hover thrust for 1 second
        hover_thrust = params.hover_thrust
        for _ in range(200):  # 200 * 0.005 = 1s
            drone.step(hover_thrust, np.zeros(3))

        # Altitude should remain near zero (drone hovering on ground)
        assert (
            abs(drone.altitude) < 0.5
        ), f"Hover thrust should keep drone near ground, got alt={drone.altitude}"

    def test_free_fall(self):
        drone = QuadrotorDynamics()
        # Start at 10m altitude (z = -10 in NED)
        initial = np.zeros(12)
        initial[2] = -10.0
        drone.reset(initial)

        # No thrust for 0.5s
        for _ in range(100):
            drone.step(0.0, np.zeros(3))

        # Should have fallen (z increased toward 0)
        assert drone.state[2] > -10.0, "Drone should fall under gravity"
        assert drone.state[5] > 0.0, "Downward velocity should be positive in NED"

    def test_rotation_matrix_identity_at_zero(self):
        R = QuadrotorDynamics.rotation_matrix(0, 0, 0)
        assert np.allclose(R, np.eye(3), atol=1e-10)

    def test_reset(self):
        drone = QuadrotorDynamics()
        drone.step(15.0, np.array([0.1, 0.0, 0.0]))
        drone.reset()
        assert np.allclose(drone.state, 0.0)
        assert drone.time == 0.0

    def test_params_from_yaml(self):
        yaml_path = (
            Path(__file__).resolve().parents[1]
            / "src"
            / "uav_model"
            / "parameters.yaml"
        )
        if yaml_path.exists():
            params = QuadrotorParams.from_yaml(str(yaml_path))
            assert params.mass > 0
            assert params.gravity > 0


class TestControllerInterface:
    """Tests for ControllerBase interface compliance."""

    def test_cascade_is_controller_base(self):
        ctrl = CascadeController()
        assert isinstance(ctrl, ControllerBase), "CascadeController must inherit ControllerBase"

    def test_lqr_is_controller_base(self):
        ctrl = LQRController()
        assert isinstance(ctrl, ControllerBase), "LQRController must inherit ControllerBase"

    def test_mpc_is_controller_base(self):
        ctrl = MPCController()
        assert isinstance(ctrl, ControllerBase), "MPCController must inherit ControllerBase"

    def test_all_have_name(self):
        controllers = [CascadeController(), LQRController(), MPCController()]
        names = [c.name for c in controllers]
        assert all(isinstance(n, str) and len(n) > 0 for n in names), \
            f"All controllers must have a non-empty name, got {names}"

    def test_all_have_compute_and_reset(self):
        ctrls = [CascadeController(), LQRController(), MPCController(), GeometricController()]
        for c in ctrls:
            assert isinstance(c, ControllerBase), f"{c.name} must inherit ControllerBase"
            assert hasattr(c, "reset"), f"{c.name} missing reset()"

    def test_compute_returns_correct_types(self):
        """All controllers must return (float, ndarray[3]) from compute()."""
        controllers = [CascadeController(), LQRController(), MPCController()]
        sp = np.array([0.0, 0.0, -5.0])
        state = np.zeros(12)

        for c in controllers:
            c.reset()
            thrust, torque = c.compute(sp, 0.0, state, 0.005)
            assert isinstance(thrust, float), \
                f"{c.name}.compute() thrust should be float, got {type(thrust)}"
            assert isinstance(torque, np.ndarray), \
                f"{c.name}.compute() torque should be ndarray, got {type(torque)}"
            assert torque.shape == (3,), \
                f"{c.name}.compute() torque shape should be (3,), got {torque.shape}"


class TestCascadeController:
    """Tests for CascadeController."""

    def test_hover_converges(self):
        """Cascade controller should stabilize a quadrotor at hover setpoint."""
        params = QuadrotorParams()
        drone = QuadrotorDynamics(params)
        ctrl = CascadeController(
            mass=params.mass, gravity=params.gravity
        )

        # Start at origin, target: hover at z=-5 (5m altitude)
        drone.reset()
        ctrl.reset()
        sp = np.array([0.0, 0.0, -5.0])

        for _ in range(4000):  # 20 seconds
            thrust, torque = ctrl.compute(sp, 0.0, drone.state, 0.005)
            drone.step(thrust, torque, 0.005)
            if drone.state[2] > 0:
                drone.state[2] = 0.0
                drone.state[5] = min(drone.state[5], 0.0)

        alt_error = abs(drone.altitude - 5.0)
        assert alt_error < 0.5, f"Cascade hover altitude error {alt_error:.3f}m (expected < 0.5m)"


class TestLQRController:
    """Tests for LQRController."""

    def test_gain_matrix_shape(self):
        ctrl = LQRController()
        assert ctrl.K.shape == (4, 12), f"K should be 4x12, got {ctrl.K.shape}"

    def test_closed_loop_stability(self):
        """Closed-loop eigenvalues (A - B*K) must all have negative real parts."""
        ctrl = LQRController()
        A_cl = ctrl.A - ctrl.B @ ctrl.K
        eigvals = np.linalg.eigvals(A_cl)
        assert all(np.real(eigvals) < 0), \
            f"Closed-loop system is unstable! Eigenvalues: {eigvals}"

    def test_hover_converges(self):
        """LQR should stabilize a quadrotor at hover setpoint."""
        params = QuadrotorParams()
        drone = QuadrotorDynamics(params)
        ctrl = LQRController(mass=params.mass, gravity=params.gravity,
                             Ixx=params.Ixx, Iyy=params.Iyy, Izz=params.Izz)

        # Start at origin, target: hover at z=-5 (5m altitude)
        drone.reset()
        ctrl.reset()
        sp = np.array([0.0, 0.0, -5.0])

        for _ in range(4000):  # 20 seconds
            thrust, torque = ctrl.compute(sp, 0.0, drone.state, 0.005)
            drone.step(thrust, torque, 0.005)
            if drone.state[2] > 0:
                drone.state[2] = 0.0
                drone.state[5] = min(drone.state[5], 0.0)

        alt_error = abs(drone.altitude - 5.0)
        assert alt_error < 0.5, f"LQR hover altitude error {alt_error:.3f}m (expected < 0.5m)"

    def test_reset_is_safe(self):
        """LQR reset should not crash (it's a no-op)."""
        ctrl = LQRController()
        ctrl.reset()  # Should not raise

    def test_thrust_within_bounds(self):
        ctrl = LQRController()
        sp = np.array([0.0, 0.0, -5.0])
        state = np.zeros(12)
        thrust, _ = ctrl.compute(sp, 0.0, state, 0.005)
        hover_T = ctrl.mass * ctrl.gravity
        assert 0.0 <= thrust <= 2.0 * hover_T, \
            f"Thrust {thrust} out of bounds [0, {2 * hover_T}]"


class TestMPCController:
    """Tests for MPCController."""

    def test_output_within_constraints(self):
        """MPC output must respect thrust and torque constraints."""
        ctrl = MPCController(horizon=10)
        sp = np.array([0.0, 0.0, -5.0])
        state = np.zeros(12)

        thrust, torque = ctrl.compute(sp, 0.0, state, 0.005)
        hover_T = ctrl.mass * ctrl.gravity

        assert 0.0 <= thrust <= ctrl.thrust_max_factor * hover_T, \
            f"Thrust {thrust} out of bounds"
        assert np.all(np.abs(torque) <= ctrl.torque_max + 1e-6), \
            f"Torque {torque} exceeds max {ctrl.torque_max}"

    def test_hover_converges(self):
        """MPC should stabilize a quadrotor at hover setpoint."""
        params = QuadrotorParams()
        drone = QuadrotorDynamics(params)
        ctrl = MPCController(
            mass=params.mass,
            gravity=params.gravity,
            Ixx=params.Ixx,
            Iyy=params.Iyy,
            Izz=params.Izz,
            control_dt=0.02,
        )

        drone.reset()
        ctrl.reset()
        sp = np.array([0.0, 0.0, -5.0])

        for _ in range(1000):  # 20 seconds at 0.02s
            thrust, torque = ctrl.compute(sp, 0.0, drone.state, 0.02)
            drone.step(thrust, torque, dt=0.02)
            if drone.state[2] > 0:
                drone.state[2] = 0.0
                drone.state[5] = min(drone.state[5], 0.0)

        alt_error = abs(drone.altitude - 5.0)
        assert alt_error < 1.0, f"MPC hover altitude error {alt_error:.3f}m (expected < 1.0m)"

    def test_warm_start(self):
        """After one call, warm start should be populated."""
        ctrl = MPCController(horizon=10)
        sp = np.array([0.0, 0.0, -5.0])
        state = np.zeros(12)
        ctrl.compute(sp, 0.0, state, 0.005)
        assert ctrl._prev_solution is not None, "Warm start should be set after first call"

    def test_reset_clears_warm_start(self):
        ctrl = MPCController(horizon=10)
        sp = np.array([0.0, 0.0, -5.0])
        state = np.zeros(12)
        ctrl.compute(sp, 0.0, state, 0.005)
        ctrl.reset()
        assert ctrl._prev_solution is None, "Reset should clear warm start"


class TestGeometricController(unittest.TestCase):
    def test_hover_converges(self):
        drone = QuadrotorDynamics(QuadrotorParams())
        ctrl = GeometricController()
        sp = np.array([0.0, 0.0, -5.0])

        for _ in range(2000):
            thrust, torque = ctrl.compute(sp, 0.0, drone.state, 0.005)
            drone.step(thrust, torque, 0.005)
            if drone.state[2] > 0:
                drone.state[2] = 0.0
                drone.state[5] = min(drone.state[5], 0.0)

        # Should be extremely close to 5m altitude, zero x/y
        self.assertLess(np.linalg.norm(drone.state[0:3] - sp), 0.1)

    def test_inverted_recovery(self):
        """
        Euler angle controllers fail or lock up at pitch = pi.
        Geometric SE(3) controller should effortlessly recover from an upside-down drop.
        """
        drone = QuadrotorDynamics(QuadrotorParams())
        ctrl = GeometricController()

        # Start at 10m high (z=-10), completely upside down (pitch=pi)
        drone.reset(np.array([0, 0, -10, 0, 0, 0, 0, np.pi, 0, 0, 0, 0]))

        sp = np.array([0.0, 0.0, -10.0])  # Try to hover at 10m

        recovered = False
        for i in range(1000):
            thrust, torque = ctrl.compute(sp, 0.0, drone.state, 0.005)
            drone.step(thrust, torque, 0.005)

            # If pitch gets close to 0, it has recovered its attitude
            if abs(drone.state[7]) < 0.1 and abs(drone.state[6]) < 0.1:
                recovered = True
                break

        self.assertTrue(recovered, "Geometric controller failed to recover from inverted state")


def run_tests():
    """Simple test runner."""
    passed = 0
    failed = 0
    errors = []

    test_classes = [
        TestPIDController,
        TestQuadrotorDynamics,
        TestControllerInterface,
        TestCascadeController,
        TestLQRController,
        TestMPCController,
        TestGeometricController,
    ]

    for cls in test_classes:
        instance = cls()
        methods = [m for m in dir(instance) if m.startswith("test_")]

        for method_name in sorted(methods):
            try:
                getattr(instance, method_name)()
                passed += 1
                print(f"  PASS  {cls.__name__}.{method_name}")
            except Exception as e:
                failed += 1
                errors.append((f"{cls.__name__}.{method_name}", str(e)))
                print(f"  FAIL  {cls.__name__}.{method_name}: {e}")

    print(f"\n{'=' * 50}")
    print(f"Results: {passed} passed, {failed} failed")
    if errors:
        print("\nFailures:")
        for name, err in errors:
            print(f"  - {name}: {err}")
    print(f"{'=' * 50}")

    return failed == 0


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
