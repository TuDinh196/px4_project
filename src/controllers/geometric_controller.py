import numpy as np

from .controller_base import ControllerBase


class GeometricController(ControllerBase):
    """
    Geometric Tracking Controller on SE(3).
    Provides global asymptotic stability and complete avoidance of Euler angle singularities.
    Supports Differential Flatness feedforward by accepting [p, v, a] setpoints.
    """

    def __init__(
        self,
        mass: float = 7.8,
        gravity: float = 9.81,
        Ixx: float = 1.46,
        Iyy: float = 1.06,
        Izz: float = 2.50,
        kp: float = 5.0,
        kv: float = 3.5,
        kR: float = 40.0,
        kw: float = 8.0,
    ):
        self.mass = mass
        self.gravity = gravity
        self.J = np.diag([Ixx, Iyy, Izz])
        self.mg_vec = np.array([0.0, 0.0, self.mass * self.gravity])

        # Gains
        self.Kp = np.diag([kp, kp, kp * 1.5])  # Stronger Z tracking
        self.Kv = np.diag([kv, kv, kv * 1.2])
        self.KR = np.diag([kR, kR, kR])
        self.Kw = np.diag([kw, kw, kw])
        self.integral_z = 0.0  # Tích phân sai số trục Z
        self.ki_z = 2.0  # Hệ số tích phân trục Z

    @property
    def name(self) -> str:
        return "Geometric SE(3)"

    def reset(self):
        self.integral_z = 0.0

    def vee_map(self, R: np.ndarray) -> np.ndarray:
        """Maps a 3x3 skew-symmetric matrix to a 3-vector."""
        return np.array([R[2, 1], R[0, 2], R[1, 0]])

    def compute(
        self,
        position_setpoint: np.ndarray,
        yaw_setpoint: float,
        current_state: np.ndarray,
        dt: float,
    ) -> tuple[float, np.ndarray]:
        # Parse setpoint
        p_ref = position_setpoint[0:3]
        v_ref = np.zeros(3)
        a_ref = np.zeros(3)

        if len(position_setpoint) >= 6:
            v_ref = position_setpoint[3:6]
        if len(position_setpoint) >= 9:
            a_ref = position_setpoint[6:9]

        # Parse current state
        p = current_state[0:3]
        v = current_state[3:6]
        phi, theta, psi = current_state[6:9]
        w = current_state[9:12]

        # Current rotation matrix (NED convention ZYX)
        cp = np.cos(psi)
        sp = np.sin(psi)
        ct = np.cos(theta)
        st = np.sin(theta)
        cr = np.cos(phi)
        sr = np.sin(phi)
        R = np.array(
            [
                [cp * ct, cp * st * sr - sp * cr, cp * st * cr + sp * sr],
                [sp * ct, sp * st * sr + cp * cr, sp * st * cr - cp * sr],
                [-st, ct * sr, ct * cr],
            ]
        )

        # 1. Position Control
        e_p = p - p_ref
        e_v = v - v_ref

        # Desired acceleration
        a_des = a_ref - self.Kp @ e_p - self.Kv @ e_v

        # Desired force vector in world frame
        # In NED, gravity pulls in +Z direction. To counteract gravity, we need negative force in Z.
        # F_des = m * a_des - m * g * e3
        F_des = self.mass * a_des - self.mg_vec

        # Total thrust is the projection of F_des onto the body -Z axis
        z_B = R[:, 2]
        thrust = -np.dot(F_des, z_B)
        thrust = float(np.clip(thrust, 0.0, 2.5 * self.mass * self.gravity))

        # 2. Attitude Control
        # Desired Z-axis points opposite to desired force
        norm_F = np.linalg.norm(F_des)
        if norm_F > 1e-6:
            z_d = -F_des / norm_F
        else:
            z_d = np.array([0.0, 0.0, -1.0])

        # Desired heading
        x_c = np.array([np.cos(yaw_setpoint), np.sin(yaw_setpoint), 0.0])

        # Desired Y-axis
        z_cross_x = np.cross(z_d, x_c)
        norm_zx = np.linalg.norm(z_cross_x)
        if norm_zx > 1e-6:
            y_d = z_cross_x / norm_zx
        else:
            y_d = np.array([0.0, 1.0, 0.0])

        # Desired X-axis
        x_d = np.cross(y_d, z_d)

        R_d = np.column_stack((x_d, y_d, z_d))

        # Attitude error on SO(3) via matrix logarithm (singularity-free across full sphere)
        R_err = R_d.T @ R
        cos_angle = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
        angle = np.arccos(cos_angle)

        if angle < 1e-4:
            e_R = 0.5 * self.vee_map(R_err - R_err.T)
        elif angle > np.pi - 1e-3:
            # Near 180 degrees inverted
            e_R = np.array([0.1, angle, 0.0])
        else:
            e_R = (angle / (2.0 * np.sin(angle))) * self.vee_map(R_err - R_err.T)

        # Angular velocity error (assuming w_d = 0 for simplicity)
        e_w = w

        # Torque command with torque saturation
        gyro_term = np.cross(w, self.J @ w)
        torque = -self.KR @ e_R - self.Kw @ e_w + gyro_term
        torque = np.clip(torque, -15.0, 15.0)

        return thrust, torque

    def compute_attitude_thrust(
        self,
        position_setpoint: np.ndarray,
        yaw_setpoint: float,
        current_state: np.ndarray,
    ):
        """
        Calculates the desired attitude (Roll, Pitch, Yaw) and Thrust norm.
        Used for Cascaded Control on real PX4 hardware via MAVSDK `set_attitude`.
        """
        from scipy.spatial.transform import Rotation as R_scipy

        sp = position_setpoint
        if len(sp) >= 9:
            pos_d = sp[0:3]
            vel_d = sp[3:6]
            acc_d = sp[6:9]
        else:
            pos_d = sp[0:3]
            vel_d = np.zeros(3)
            acc_d = np.zeros(3)

        pos = current_state[0:3]
        vel = current_state[3:6]

        e_p = pos - pos_d
        e_v = vel - vel_d

        # Tích phân sai số Z (Chống chìm độ cao)
        # dt = 0.02s (50Hz control loop, matches PX4 SITL offboard rate)
        control_dt = 0.02
        self.integral_z += e_p[2] * control_dt
        # Anti-windup (Giới hạn tích phân ở mức +-3N lực đẩy)
        self.integral_z = np.clip(self.integral_z, -3.0 / self.ki_z, 3.0 / self.ki_z)

        F_des = -self.Kp @ e_p - self.Kv @ e_v - self.mg_vec + self.mass * acc_d

        # Bù thêm lực đẩy từ thành phần Tích phân để diệt tận gốc sai số tĩnh
        F_des[2] -= self.ki_z * self.integral_z

        # Tilt limiting (Cap maximum tilt to 35 degrees to prevent Gazebo/PX4
        # flip-over and Failsafe)
        max_tilt_rad = np.radians(35.0)

        # Ensure F_des[2] is negative (thrust pushing UP to fight gravity)
        # If it's positive, the drone wants to flip upside down to thrust down! We prevent this.
        if F_des[2] > -0.1:
            F_des[2] = -0.1

        max_F_xy = abs(F_des[2]) * np.tan(max_tilt_rad)
        F_xy_norm = np.linalg.norm(F_des[0:2])

        if F_xy_norm > max_F_xy:
            F_des[0:2] = F_des[0:2] * (max_F_xy / F_xy_norm)

        thrust_norm = np.linalg.norm(F_des)

        # Desired Z body axis
        if thrust_norm > 1e-6:
            z_b_des = -F_des / thrust_norm
        else:
            z_b_des = np.array([0, 0, -1])

        # Desired X body axis based on yaw
        x_c = np.array([np.cos(yaw_setpoint), np.sin(yaw_setpoint), 0.0])
        y_b_des = np.cross(z_b_des, x_c)

        if np.linalg.norm(y_b_des) < 1e-6:
            # Singularity (pitch = +-90)
            y_b_des = np.array([0, 1, 0])
        else:
            y_b_des = y_b_des / np.linalg.norm(y_b_des)

        x_b_des = np.cross(y_b_des, z_b_des)

        R_d = np.column_stack((x_b_des, y_b_des, z_b_des))

        # Convert R_d to Euler angles (ZYX order -> Yaw, Pitch, Roll)
        euler = R_scipy.from_matrix(R_d).as_euler("ZYX", degrees=True)
        yaw_deg, pitch_deg, roll_deg = euler[0], euler[1], euler[2]

        return roll_deg, pitch_deg, yaw_deg, thrust_norm
