"""Controllers Package for Quadplane Condor."""

from .cascade_controller import CascadeController
from .condor_path_follower import FixedWingPathFollower
from .condor_vtol_controller import VTOLHybridController
from .controller_base import ControllerBase
from .geometric_controller import GeometricController
from .lqr_controller import LQRController
from .mpc_controller import MPCController
from .pid_controller import PIDController

# Backward compatibility alias
VTOLController = VTOLHybridController

__all__ = [
    "CascadeController",
    "ControllerBase",
    "FixedWingPathFollower",
    "GeometricController",
    "LQRController",
    "MPCController",
    "PIDController",
    "VTOLController",
    "VTOLHybridController",
]
