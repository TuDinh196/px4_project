"""
Controller Base Interface
=========================
Abstract base class for all quadrotor controllers.
Ensures a unified interface so controllers can be swapped seamlessly
in simulation and PX4 integration code.
"""

from abc import ABC, abstractmethod

import numpy as np


class ControllerBase(ABC):
    """
    Abstract base class for quadrotor controllers.

    All controllers must implement:
        - compute(): run one control step
        - reset(): clear internal state
        - name: human-readable identifier
    """

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable name for plots and logs."""
        ...

    @abstractmethod
    def reset(self):
        """Reset all internal controller state."""
        ...

    @abstractmethod
    def compute(
        self,
        position_setpoint: np.ndarray,
        yaw_setpoint: float,
        current_state: np.ndarray,
        dt: float,
    ) -> tuple[float, np.ndarray]:
        """
        Run one control step.

        Args:
            position_setpoint: desired [x, y, z] in NED (m)
            yaw_setpoint: desired yaw angle (rad)
            current_state: full 12-element state vector
                [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
            dt: timestep (s)

        Returns:
            (thrust, torque):
                thrust - total thrust in Newtons (positive up in body)
                torque - [tau_x, tau_y, tau_z] in body frame (N*m)
        """
        ...
