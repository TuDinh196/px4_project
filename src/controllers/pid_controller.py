"""
PID Controller
==============
General-purpose PID controller with anti-windup, derivative filtering,
and output saturation.
"""

import numpy as np


class PIDController:
    """
    Discrete PID controller.

    Features:
        - Proportional, Integral, Derivative terms
        - Anti-windup via integral clamping
        - First-order derivative filter (reduces noise)
        - Output saturation
        - Integral reset on demand
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -np.inf,
        output_max: float = np.inf,
        integral_min: float = -np.inf,
        integral_max: float = np.inf,
        derivative_filter_coeff: float = 0.0,
    ):
        """
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_min/max: Output saturation limits
            integral_min/max: Anti-windup integral limits
            derivative_filter_coeff: Low-pass filter for derivative (0=no filter, 0-1)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.alpha = derivative_filter_coeff

        # Internal state
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._initialized = False

    def reset(self):
        """Reset all internal state."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._initialized = False

    def update(self, error: float, dt: float) -> float:
        """
        Compute PID output for the given error.

        Args:
            error: setpoint - measurement
            dt: time step (seconds), must be > 0

        Returns:
            Saturated control output
        """
        if dt <= 0:
            return 0.0

        # -- Proportional --
        p_term = self.kp * error

        # -- Integral with anti-windup --
        self._integral += error * dt
        self._integral = np.clip(self._integral, self.integral_min, self.integral_max)
        i_term = self.ki * self._integral

        # -- Derivative with filtering --
        if not self._initialized:
            raw_derivative = 0.0
            self._initialized = True
        else:
            raw_derivative = (error - self._prev_error) / dt

        # First-order low-pass filter on derivative
        filtered_derivative = (
            self.alpha * self._prev_derivative + (1.0 - self.alpha) * raw_derivative
        )
        d_term = self.kd * filtered_derivative

        self._prev_error = error
        self._prev_derivative = filtered_derivative

        # -- Total output with saturation --
        output = p_term + i_term + d_term
        output = np.clip(output, self.output_min, self.output_max)

        return float(output)

    def set_gains(self, kp: float, ki: float, kd: float):
        """Update PID gains at runtime."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
