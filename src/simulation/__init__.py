"""Simulation Package for Quadplane Condor Hybrid VTOL."""

from .condor_closed_loop_sim import run_controller_comparison
from .condor_mission_sim import run_vtol_mission_simulation

# Aliases
run_closed_loop_simulation = run_controller_comparison
run_condor_mission_simulation = run_vtol_mission_simulation
run_condor_closed_loop_simulation = run_controller_comparison

__all__ = [
    "run_closed_loop_simulation",
    "run_condor_closed_loop_simulation",
    "run_condor_mission_simulation",
    "run_controller_comparison",
    "run_vtol_mission_simulation",
]
