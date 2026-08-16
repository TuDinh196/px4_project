"""
Nonlinear Path Following Guidance (NPFG / L1) for Quadplane Fixed-Wing Cruise
=============================================================================
Computes lateral acceleration and coordinated roll commands to follow
straight line segments and curved loiter paths at high speeds (18-25 m/s).
"""

import numpy as np


class FixedWingPathFollower:
    """
    L1 / NPFG Path Following Guidance for Fixed-Wing UAVs.
    """

    def __init__(
        self,
        lookahead_distance: float = 35.0,
        max_bank_angle: float = 0.65,  # ~37 degrees
        gravity: float = 9.81,
    ):
        self.L1 = lookahead_distance
        self.max_bank_angle = max_bank_angle
        self.g = gravity

    def compute_cross_track_error(
        self,
        current_pos: np.ndarray,
        seg_start: np.ndarray,
        seg_end: np.ndarray,
    ) -> tuple[float, np.ndarray]:
        """
        Calculates cross-track error to a 2D line segment.

        Args:
            current_pos: [x, y] in NED
            seg_start: [x, y] start waypoint
            seg_end: [x, y] end waypoint

        Returns:
            cross_track_err: signed distance to track (m, positive = right of track)
            target_point: [x, y] lookahead reference point on path
        """
        seg_vec = seg_end[0:2] - seg_start[0:2]
        seg_len = np.linalg.norm(seg_vec)

        if seg_len < 1e-3:
            return 0.0, seg_end[0:2]

        seg_unit = seg_vec / seg_len
        rel_pos = current_pos[0:2] - seg_start[0:2]

        # Projection along segment
        proj_dist = np.dot(rel_pos, seg_unit)
        proj_point = seg_start[0:2] + proj_dist * seg_unit

        # Cross track vector: 2D cross product for sign
        cross_vec = current_pos[0:2] - proj_point
        cross_track_err = cross_vec[0] * seg_unit[1] - cross_vec[1] * seg_unit[0]

        # Lookahead point
        target_proj = min(proj_dist + self.L1, seg_len)
        target_point = seg_start[0:2] + target_proj * seg_unit

        return cross_track_err, target_point

    def compute_lateral_acceleration(
        self,
        current_pos: np.ndarray,
        current_vel: np.ndarray,
        target_point: np.ndarray,
    ) -> tuple[float, float]:
        """
        Computes lateral acceleration command and desired coordinated bank angle.

        Args:
            current_pos: [x, y] NED
            current_vel: [vx, vy] NED
            target_point: [x, y] lookahead target point

        Returns:
            a_lat: lateral acceleration command (m/s²)
            phi_cmd: desired roll / bank angle (rad)
        """
        ground_speed = max(float(np.linalg.norm(current_vel[0:2])), 1.0)
        los_vec = target_point[0:2] - current_pos[0:2]
        los_dist = max(float(np.linalg.norm(los_vec)), 1.0)

        # Angle between ground velocity and line-of-sight vector (eta)
        vel_heading = np.arctan2(current_vel[1], current_vel[0])
        los_heading = np.arctan2(los_vec[1], los_vec[0])
        eta = (los_heading - vel_heading + np.pi) % (2 * np.pi) - np.pi

        # L1 guidance lateral acceleration: a_lat = 2 * (V² / L1) * sin(eta)
        a_lat = 2.0 * (ground_speed**2 / max(self.L1, los_dist)) * np.sin(eta)

        # Coordinated turn bank angle: tan(phi) = a_lat / g
        phi_cmd = float(np.arctan2(a_lat, self.g))
        phi_cmd = float(np.clip(phi_cmd, -self.max_bank_angle, self.max_bank_angle))

        return a_lat, phi_cmd
