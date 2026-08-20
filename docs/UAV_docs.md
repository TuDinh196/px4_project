# TÀI LIỆU KỸ THUẬT — QUADPLANE CONDOR HYBRID VTOL

---

## CHƯƠNG 1. GIỚI THIỆU PHƯƠNG PHÁP ĐIỀU KHIỂN & MÔ PHỎNG UAV

### 1.1. Tổng Quan Hệ Thống Điều Khiển

Hệ thống mô phỏng UAV của dự án được xây dựng hoàn toàn bằng Python, triển khai nhiều thuật toán điều khiển từ cơ bản đến nâng cao, cho phép **Quadplane Condor** — máy bay lai VTOL (4+1 layout) — bay bám theo các quỹ đạo 3D phức tạp.

Toàn bộ quy trình từ mô hình toán học, bộ điều khiển, đến giả lập vật lý đều được lập trình và kiểm thử offline trước khi tích hợp với PX4 Autopilot và Gazebo Harmonic.

Các bộ điều khiển được triển khai:

| Bộ Điều Khiển | File | Mô Tả |
|---|---|---|
| **PID Cơ Bản** | `pid_controller.py` | PID với anti-windup, derivative filter |
| **Cascade PID** | `cascade_controller.py` | 3 vòng lồng nhau: Position → Attitude → Rate |
| **LQR** | `lqr_controller.py` | Linear Quadratic Regulator (CARE solver) |
| **MPC** | `mpc_controller.py` | Model Predictive Control, horizon N=20 |
| **Geometric SE(3)** | `geometric_controller.py` | Điều khiển trên nhóm Lie SE(3) — ổn định toàn cục |
| **VTOL Hybrid** | `condor_vtol_controller.py` | State machine 5 giai đoạn chuyển tiếp VTOL |
| **Path Follower** | `condor_path_follower.py` | L1/NPFG bám đường bay cánh cứng |

### 1.2. Bộ Điều Khiển PID — Nền Tảng Cốt Lõi

#### 1.2.1. Lý Thuyết PID

PID (Proportional-Integral-Derivative) là thuật toán điều khiển phản hồi kinh điển. Tín hiệu điều khiển u(t) được tính từ sai số e(t) = setpoint − measurement:

```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·ė(t)
```

Trong đó:
- **Kp** (Proportional): Phản ứng tỉ lệ với sai số hiện tại. Tăng Kp → phản ứng nhanh hơn nhưng dễ dao động.
- **Ki** (Integral): Tích lũy sai số theo thời gian để triệt tiêu sai số tĩnh (steady-state error). Giúp drone bám chính xác điểm đích khi có nhiễu ngoại cảnh (gió).
- **Kd** (Derivative): Dự đoán xu hướng sai số, giảm dao động và tăng tốc ổn định. Giúp drone dừng lại dứt khoát tại điểm đích.

#### 1.2.2. Triển Khai PID Trong Code

File `src/controllers/pid_controller.py` triển khai class `PIDController` với các tính năng nâng cao:

```python
class PIDController:
    def update(self, error: float, dt: float) -> float:
        # Proportional
        p_out = self.kp * error

        # Integral với anti-windup (giới hạn tích phân)
        self.integral += error * dt
        self.integral = np.clip(self.integral, self.integral_min, self.integral_max)
        i_out = self.ki * self.integral

        # Derivative với low-pass filter (lọc nhiễu)
        derivative = (error - self.prev_error) / dt
        self.filtered_deriv = (self.alpha * self.filtered_deriv
                               + (1 - self.alpha) * derivative)
        d_out = self.kd * self.filtered_deriv

        output = np.clip(p_out + i_out + d_out, self.output_min, self.output_max)
        self.prev_error = error
        return output
```

Các tính năng quan trọng:
- **Anti-windup**: Giới hạn phần tích phân (`integral_min/max`) để tránh bão hòa khi drone ở xa setpoint lâu.
- **Derivative filter**: Lọc thông thấp bậc 1 (hệ số `alpha`) để lọc nhiễu tín hiệu đạo hàm.
- **Output clamp**: Giới hạn đầu ra theo giới hạn vật lý.

#### 1.2.3. Bảng Tham Số PID Hiện Tại

**Cascade Controller — Vòng Ngoài (Position):**

| PID | Kp | Ki | Kd | Output |
|---|---|---|---|---|
| pid_x | 2.2 | 0.04 | 1.4 | Gia tốc X (m/s²) |
| pid_y | 2.2 | 0.04 | 1.4 | Gia tốc Y (m/s²) |
| pid_z | 4.2 | 0.25 | 3.2 | Thrust lệnh (N) |

**VTOL Hybrid Controller — Fixed-Wing:**

| PID | Kp | Ki | Kd | Output |
|---|---|---|---|---|
| pid_airspeed | 3.5 | 0.4 | 0.5 | Tractor thrust (N) |
| pid_altitude | 0.08 | 0.01 | 0.04 | Góc pitch mong muốn (rad) |
| pid_pitch_fw | 1.8 | 0.1 | 0.15 | Elevator δe (rad) |
| pid_heading_fw | 1.5 | 0.05 | 0.4 | Roll mong muốn (rad) |

### 1.3. Kiến Trúc Điều Khiển Cascade (3 Tầng)

#### 1.3.1. Tại Sao Cần Cascade?

Quadplane Condor là hệ MIMO (Multi-Input Multi-Output) phi tuyến với 12 trạng thái và 6 đầu vào điều khiển. Kiến trúc Cascade phân rã bài toán phức tạp thành 3 vòng có tốc độ khác nhau, mỗi vòng dễ điều chỉnh độc lập:

```
Vòng Ngoài (Position)   ~10 Hz  →  Acceleration commands
Vòng Giữa (Attitude)    ~50 Hz  →  Body rate commands
Vòng Trong (Rate)       ~200 Hz →  Torque commands
```

#### 1.3.2. Vòng Ngoài — Điều Khiển Vị Trí (Position Loop)

**Đầu vào:** Vị trí mong muốn [x_d, y_d, z_d] (NED, mét)
**Đầu ra:** Gia tốc mong muốn + Thrust

```python
# Sai số vị trí
ex = x_des - x;  ey = y_des - y;  ez = z_des - z

# PID → gia tốc mong muốn
ax_des = pid_x.update(ex, dt)
ay_des = pid_y.update(ey, dt)

# Thrust từ sai số độ cao (NED: z dương = xuống)
thrust = mass * (gravity - pid_z.update(ez, dt))

# Chuyển gia tốc NED → góc Roll/Pitch mong muốn (small angle)
phi_des   = ( ax_des*sin(psi) - ay_des*cos(psi)) / g
theta_des = -(ax_des*cos(psi) + ay_des*sin(psi)) / g
```

#### 1.3.3. Vòng Giữa — Điều Khiển Góc Nghiêng (Attitude Loop)

**Đầu vào:** [φ_des, θ_des, ψ_des]
**Đầu ra:** Tốc độ góc mong muốn [p_des, q_des, r_des]

```python
p_des = pid_roll.update(phi_des - phi, dt)
q_des = pid_pitch.update(theta_des - theta, dt)
r_des = pid_yaw.update(wrap(psi_des - psi), dt)
```

#### 1.3.4. Vòng Trong — Điều Khiển Tốc Độ Góc (Rate Loop)

**Đầu vào:** [p_des, q_des, r_des]
**Đầu ra:** Momen lực [τx, τy, τz] (N·m)

```python
tau_x = pid_p.update(p_des - p, dt)
tau_y = pid_q.update(q_des - q, dt)
tau_z = pid_r.update(r_des - r, dt)
```

### 1.4. Các Bộ Điều Khiển Nâng Cao

#### 1.4.1. LQR (Linear Quadratic Regulator)

Tuyến tính hóa mô hình Quadplane quanh trạng thái hover, giải phương trình đại số Riccati liên tục (CARE) để tìm gain tối ưu K:

```
u = u_hover - K × x_err
K = R⁻¹ × Bᵀ × P    (P giải từ: AᵀP + PA - PBR⁻¹BᵀP + Q = 0)
```

Ma trận trọng số Q (phạt sai số trạng thái) và R (phạt điều khiển) quyết định trade-off giữa hiệu năng bám và tiết kiệm năng lượng. Giải CARE 1 lần lúc khởi tạo → rất nhanh mỗi bước điều khiển.

#### 1.4.2. MPC (Model Predictive Control)

Tại mỗi bước, giải bài toán tối ưu hóa cho N=20 bước tương lai:

```
min  J = Σ(k=0..N-1) [xₖ'Qxₖ + uₖ'Ruₖ] + x_N'Px_N
u₀..N-1

subject to: uₘᵢₙ ≤ uₖ ≤ uₘₐₓ
```

Chỉ áp dụng u₀ (bước đầu tiên), rồi giải lại ở bước tiếp theo — **Receding Horizon**. Warm start từ nghiệm trước → hội tụ nhanh. Dual-Mode: gần setpoint → chuyển sang LQR gain.

#### 1.4.3. Geometric Controller trên SE(3)

Điều khiển trực tiếp trên nhóm Lie SE(3) bằng ma trận quay R ∈ SO(3), hoàn toàn tránh góc Euler và gimbal lock. Đảm bảo **global asymptotic stability** — ổn định từ mọi trạng thái ban đầu.

Hỗ trợ **Differential Flatness feedforward**: nhận setpoint 9 phần tử [pos, vel, acc] để bám quỹ đạo chính xác hơn ở tốc độ cao.

### 1.5. Thuật Toán Bám Quỹ Đạo (Trajectory Tracking)

Hệ thống triển khai nhiều thuật toán phát sinh quỹ đạo trong `src/simulation/condor_closed_loop_sim.py`:

#### 1.5.1. Minimum Jerk (Đa thức bậc 5)

Tối ưu hóa chuyển động giữa các waypoint bằng cách giảm thiểu Jerk (đạo hàm bậc 3 của vị trí):

```
p(τ) = a₀ + a₁τ + a₂τ² + a₃τ³ + a₄τ⁴ + a₅τ⁵    (τ ∈ [0,1])
```

Trong đó τ là thời gian chuẩn hóa. Áp dụng cho tất cả chuyển tiếp waypoint trong kịch bản Square và VTOL Mission.

#### 1.5.2. Quỹ Đạo Lemniscate (Hình số 8)

Phương trình tham số tạo quỹ đạo hình số 8 của Bernoulli:

```
x(t) = A × sin(ω·t)
y(t) = A × sin(ω·t) × cos(ω·t)
```

Kèm Envelope Filter để làm mượt gia tốc ban đầu: `blend = 1 - exp(-t/3.0)`

#### 1.5.3. Quỹ Đạo Figure-8 (Lissajous)

Quỹ đạo Lissajous mở rộng cho phép kích thước X và Y khác nhau:

```
vx(t) = 2·X_amp·ω·cos(2ω·t) × blend(t)
vy(t) =   Y_amp·ω·cos(ω·t)  × blend(t)
```

Với X_amp=20m, Y_amp=40m, ω=π/20 rad/s (trong `flight_scenarios.py`).

#### 1.5.4. Giant Lemniscate (SITL — 80m × 40m)

Dùng trong `sitl_condor_geometric.py` với biên độ A=40m cho SITL Gazebo:

```
N(t) = 40 × sin(0.12t)
E(t) = 40 × sin(0.12t) × cos(0.12t)
```

Vận tốc tối đa ≈ 4.8 m/s, bay ở -5m altitude, thời gian 60 giây.

#### 1.5.5. Random Waypoint

`RandomWaypointGenerator` phát sinh ngẫu nhiên các điểm đích trong không gian 3D, nối bằng Minimum Jerk, phục vụ kiểm thử khả năng bám quỹ đạo trong điều kiện không xác định.

### 1.6. Vai Trò Mô Phỏng: SIL, HIL & Mô Phỏng 3D

#### 1.6.1. Mô Phỏng Offline (Closed-Loop Simulation)

Chạy hoàn toàn bằng Python, không cần PX4 hay Gazebo:

```bash
python3 src/simulation/condor_closed_loop_sim.py   # So sánh controllers
python3 src/simulation/condor_mission_sim.py        # Nhiệm vụ VTOL đầy đủ
```

Ưu điểm: Chạy nhanh (vài giây cho 60s mô phỏng), dễ debug, xuất đồ thị so sánh trực tiếp.

#### 1.6.2. SIL — Software-In-The-Loop (SITL + Gazebo)

Chạy firmware PX4 thật (biên dịch cho x86) kết hợp Gazebo mô phỏng vật lý 3D:

```bash
./manage.sh sim          # PX4 SITL + Gazebo
./manage.sh mission      # Nhiệm vụ VTOL qua MAVSDK Offboard
./manage.sh geometric    # Bay Lemniscate với Geometric SE(3)
```

#### 1.6.3. HIL — Hardware-In-The-Loop

Thay PX4 SITL bằng board Pixhawk thực. Gazebo gửi HIL_SENSOR, Pixhawk trả HIL_ACTUATOR_CONTROLS. Chỉ cần thay chuỗi kết nối MAVSDK:

```python
# SITL:
await drone.connect(system_address="udp://:14540")
# HIL (Pixhawk USB):
await drone.connect(system_address="serial:///dev/ttyUSB0:921600")
```

100% logic điều khiển, kịch bản bay, và dashboard giữ nguyên.

#### 1.6.4. Cơ Chế Lockstep (SITL)

Trong SITL, Gazebo và PX4 chạy đồng bộ từng bước thời gian dt:

1. Gazebo mô phỏng 1 bước dt → gửi cảm biến → đóng băng
2. PX4 nhận cảm biến → tính toán điều khiển → gửi lệnh motor
3. Gazebo nhận lệnh → bước tiếp theo

→ Đảm bảo kết quả mô phỏng chính xác toán học bất kể tốc độ CPU.

### 1.7. Quy Trình Hoàn Chỉnh: Từ Thuật Toán Đến Bay 3D

```
[Thiết kế thuật toán]
        ↓
[Kiểm thử Offline Sim]  ←  Nhanh, không cần phần cứng
        ↓
[Tinh chỉnh gains]
        ↓
[Kiểm thử SITL + Gazebo]  ←  Firmware PX4 thật + vật lý 3D
        ↓
[Nâng cấp HIL]  ←  Pixhawk thật + Gazebo
        ↓
[Bay thực tế]
```

Toàn bộ quy trình này đều được kiểm soát thông qua file Python, cho phép kỹ sư điều khiển tự động hóa hoàn toàn quá trình phát triển và kiểm thử thuật toán bay.

### 1.8. Minh Họa Giao Diện Hệ Thống

#### 1.8.1. Gazebo 3D Simulation — Mô phỏng Quadplane Condor

Giao diện Gazebo Harmonic hiển thị mô hình 3D Quadplane Condor (sải cánh 2.4m) trong môi trường sân bay mô phỏng. Bên trái là cây thực thể (Entity Tree) liệt kê các model. Phần dưới hiển thị thời gian mô phỏng và tỉ lệ thời gian thực (Real-Time Factor).

#### 1.8.2. Terminal Layout — Bố Trí 4 Cửa Sổ Terminal

Bố trí 4 terminal trong cấu hình lưới 2×2 khi khởi chạy hệ thống SITL:

- **Trên-trái**: PX4 SITL console (hiển thị `Ready for takeoff` và prompt `pxh>`)
- **Trên-phải**: Gazebo server log
- **Dưới-trái**: Python MAVSDK script (kết nối drone, gửi setpoints, log trạng thái)
- **Dưới-phải**: Web Dashboard server (WebSocket listening port 8765)


---

## CHƯƠNG 2. YÊU CẦU CHỨC NĂNG — HƯỚNG DẪN VẬN HÀNH HỆ THỐNG

### 2.1. Tổ Chức Thư Mục Dự Án

#### 2.1.1. Cây Thư Mục Chính

```
px4_project/
├── README.md                          # Giới thiệu & hướng dẫn chạy nhanh
├── manage.sh                          # Script quản lý tổng thể (10 lệnh)
├── requirements.txt                   # Python dependencies
├── .flake8                            # Cấu hình kiểm tra code PEP 8
│
├── src/                               # Mã nguồn Python chính
│   ├── uav_model/
│   │   ├── condor_dynamics.py         # Mô hình 6-DOF Quadplane Condor
│   │   └── parameters.yaml            # Tham số vật lý (tự động load)
│   ├── controllers/
│   │   ├── controller_base.py         # Giao diện trừu tượng ControllerBase
│   │   ├── pid_controller.py          # PID cơ bản với anti-windup & filter
│   │   ├── cascade_controller.py      # Cascade PID 3 tầng (9 PID)
│   │   ├── lqr_controller.py          # LQR (CARE solver)
│   │   ├── mpc_controller.py          # MPC (QP, horizon-20, ZOH discrete)
│   │   ├── geometric_controller.py    # Geometric SE(3) + feedforward
│   │   ├── condor_vtol_controller.py  # VTOL Hybrid 5 giai đoạn
│   │   └── condor_path_follower.py    # L1/NPFG path following
│   ├── simulation/
│   │   ├── condor_closed_loop_sim.py  # So sánh controllers offline
│   │   └── condor_mission_sim.py      # Nhiệm vụ VTOL đầy đủ offline
│   ├── scenarios/
│   │   ├── flight_scenarios.py        # 5 kịch bản bay (hover/square/circle/figure8/manual)
│   │   └── scenario_config.yaml       # Tham số kịch bản bay
│   ├── px4_integration/
│   │   ├── mavsdk_bridge.py           # Kết nối PX4 qua MAVSDK (UDP 14540)
│   │   ├── offboard_controller.py     # Gửi Offboard setpoints đến PX4
│   │   ├── sitl_condor_geometric.py   # Bay Lemniscate trên SITL
│   │   └── sitl_condor_mission.py     # Nhiệm vụ VTOL tự động trên SITL
│   └── dashboard/web_dashboard/
│       ├── server.py                  # WebSocket Server (port 8765)
│       ├── index.html                 # Giao diện trình duyệt
│       ├── app.js                     # Logic frontend (Plotly, WebSocket, Leaflet)
│       └── style.css                  # Dark theme stylesheet
│
├── models/                            # Tài nguyên mô hình Gazebo & PX4
│   ├── README.md                      # Hướng dẫn model & packaging
│   ├── airframes/
│   │   └── 4030_gz_quadplane_condor   # Airframe PX4 (motor geometry)
│   ├── quadplane_condor/              # Model Gazebo Harmonic SDF
│   │   ├── model.config               # Metadata Gazebo
│   │   ├── model.sdf                  # Vật lý, khí động học, cảm biến, plugin
│   │   └── meshes/                    # Lưới 3D (STL, DAE)
│   └── worlds/
│       └── condor_figure8.sdf         # World sân bay + cổng Figure-8 + helipad
│
├── scripts/                           # Tiện ích hỗ trợ
│   ├── generate_figure8_world.py      # Tạo world SDF Figure-8
│   ├── plot_telemetry.py              # Vẽ đồ thị telemetry CSV
│   ├── read_telemetry.py              # Đọc dữ liệu telemetry
│   └── md_to_docx.py                  # Chuyển đổi Markdown → Word
│
├── tests/
│   ├── test_condor_controllers.py     # Unit test 4+ bộ điều khiển
│   └── test_condor_quadplane.py       # Unit test mô hình Quadplane
│
├── logs/                              # Log runtime (PX4, Gazebo, Dashboard)
├── output/                            # CSV telemetry sau bay
└── plots/                             # Đồ thị xuất từ mô phỏng
```

### 2.2. Giải Thích Vai Trò Từng Lớp

#### 2.2.1. Nền Tảng Hệ Thống — PX4 & QGroundControl

**PX4 Autopilot** là firmware điều khiển bay mã nguồn mở, được sử dụng rộng rãi trong công nghiệp UAV. Trong dự án này, PX4 chạy ở chế độ SITL (Software-In-The-Loop): firmware được biên dịch cho máy tính (x86) thay vì board Pixhawk.

Vị trí cài đặt trên WSL: `~/PX4-Autopilot/`

Các module PX4 quan trọng (chạy tự động bên trong firmware):
- `mc_pos_control`: Điều khiển vị trí multicopter
- `mc_att_control`: Điều khiển thái độ (attitude)
- `mc_rate_control`: Điều khiển tốc độ góc (rate PID nội bộ)
- `vtol_att_control`: Điều khiển chuyển tiếp VTOL
- `ekf2`: Ước lượng trạng thái (Kalman Filter)
- `mixer`: Phân phối lực đẩy cho từng motor

> **Quan trọng:** Khi dùng chế độ Offboard, Python script của chúng ta gửi setpoint trực tiếp vào `mc_pos_control`. PX4 vẫn tự chạy Rate PID và Motor Mixing nội bộ.

#### 2.2.2. Gazebo Harmonic — Môi Trường Mô Phỏng 3D

Gazebo là phần mềm mô phỏng vật lý 3D. Khi chạy SITL, Gazebo đóng vai trò:
- **Engine vật lý**: Tính toán va chạm, trọng lực, lực khí động
- **Cảm biến ảo**: Cung cấp dữ liệu IMU, GPS, Barometer cho PX4
- **Hiển thị 3D**: Render drone Quadplane Condor và môi trường xung quanh

Giao tiếp với PX4: qua giao thức `gz-transport` (không dùng MAVLink).

#### 2.2.3. QGroundControl (QGC) — Trạm Điều Khiển Mặt Đất

QGroundControl là phần mềm trạm điều khiển mặt đất chạy trên Windows. Vai trò:
- Hiển thị bản đồ + vị trí drone
- Cấu hình tham số PX4
- Giao diện Arm/Disarm, chọn chế độ bay
- Cảnh báo sức khỏe hệ thống (battery, GPS fix, EKF status)

Kết nối với PX4: Qua TCP port 5760 (thông qua `networking/tcp_bridge.py` nếu chạy WSL2).

#### 2.2.4. MAVSDK-Python — API Điều Khiển Bằng Python

MAVSDK là thư viện dịch các lệnh Python thành giao thức MAVLink để giao tiếp với PX4:

```python
from mavsdk import System
drone = System()
await drone.connect(system_address="udp://:14540")  # Kết nối UDP

# Các lệnh cơ bản:
await drone.action.arm()                  # Arm động cơ
await drone.action.takeoff()              # Cất cánh tự động
await drone.offboard.start()             # Bắt đầu chế độ Offboard
await drone.offboard.set_position_ned()  # Gửi setpoint vị trí NED
```

#### 2.2.5. Các Cửa Sổ Lệnh (Terminal) Cần Thiết

Khi vận hành hệ thống SITL đầy đủ, cần 3-4 cửa sổ terminal trên WSL2:

| Terminal | Vai Trò | Lệnh Chạy |
|---|---|---|
| **Terminal 1** | PX4 SITL + Gazebo | `./manage.sh sim` |
| **Terminal 2** | QGC TCP Bridge (tùy chọn) | `python3 scripts/networking/tcp_bridge.py` |
| **Terminal 3** | Kịch bản bay Python | `./manage.sh mission` hoặc `./manage.sh geometric` |
| **Terminal 4** | Web Dashboard | `./manage.sh dashboard` |

#### 2.2.6. Vai Trò Của Từng Terminal

- **Terminal 1 (PX4)**: Hiển thị log PX4 liên tục và có prompt `pxh>`. Có thể gõ lệnh PX4 trực tiếp ở đây (ví dụ: `commander status`, `param show VT_TYPE`).
- **Terminal 2 (TCP Bridge)**: Chuyển đổi UDP 18570 từ PX4 sang TCP 5760 cho QGroundControl trên Windows. Cần thiết khi chạy WSL2.
- **Terminal 3 (Script bay)**: Kết nối drone qua MAVSDK, gửi setpoints, log dữ liệu ra CSV.
- **Terminal 4 (Dashboard)**: WebSocket server cung cấp telemetry cho trình duyệt.

### 2.3. Lệnh Chạy Mô Phỏng — Từ A Đến Z
Trước tiên hãy di chuyển vào thư mục dự án
Tùy vào môi trường bạn đang mở terminal mà lệnh sẽ là:

1. Nếu bạn đang ở PowerShell/CMD trên Windows:

```bash
cd D:\px4_project
```
2. Nếu bạn đang ở trong Terminal của WSL / Ubuntu (Linux): Trường hợp thư mục nằm trên ổ D của Windows và bạn muốn truy cập từ WSL:
```bash
cd /mnt/d/px4_project
```
(Nếu bạn đã clone dự án vào thư mục home của Linux, lệnh sẽ là cd ~/px4_project)

Sau khi vào đúng thư mục dự án, bạn mới có thể chạy được các lệnh như ./manage.sh hay python3 src/....


#### 2.3.1. Chế Độ 1: Mô Phỏng Offline (Không Cần PX4 / Gazebo)

Đây là cách nhanh nhất để kiểm tra thuật toán điều khiển. Chỉ cần 1 terminal:

```bash
# Cài đặt môi trường (1 lần duy nhất)
cd px4_project
python3 -m venv venv_linux
source venv_linux/bin/activate
pip install -r requirements.txt

# Chạy so sánh 4 bộ điều khiển (Cascade, LQR, MPC, Geometric)
python3 src/simulation/condor_closed_loop_sim.py

# Chạy mô phỏng nhiệm vụ VTOL đầy đủ 5 giai đoạn
python3 src/simulation/condor_mission_sim.py
```

Kết quả script sẽ:
1. Chạy mô phỏng vòng kín cho các controller trên kịch bản (hover, square, lemniscate)
2. So sánh trên nhiều kịch bản bay khác nhau
3. Xuất đồ thị Matplotlib so sánh hiệu năng vào thư mục `plots/`
4. In bảng metrics (RMSE, Settling time, Overshoot) ra terminal

#### 2.3.2. Chế Độ 2: Mô Phỏng SITL (PX4 + Gazebo 3D)

Cần 3 terminal trên WSL2:

**Terminal 1 — Khởi Động PX4 + Gazebo:**
```bash
# Cài model vào PX4 (1 lần)
./manage.sh setup

# Khởi động SITL với world mặc định
./manage.sh sim

# Hoặc với world Figure-8 (cổng bay 3D + helipad)
./manage.sh sim figure8
```

Chờ đến khi thấy dòng:
```
[Ready for takeoff]
```
→ Hệ thống đã sẵn sàng. Cửa sổ Gazebo 3D sẽ hiện ra (nếu có X11/Wayland forwarding).

Lưu ý: Dấu nhắc `pxh>` là PX4 Console. Có thể gõ lệnh PX4 trực tiếp ở đây:
```
pxh> commander status
pxh> param show VT_TYPE
pxh> param show CA_AIRFRAME
```

**Terminal 2 — TCP Bridge Cho QGroundControl (Tùy Chọn):**
```bash
python3 scripts/networking/tcp_bridge.py
```
Sau đó trên Windows: Mở QGroundControl → Comm Links → Thêm kết nối TCP:
- Host: `localhost` | Port: `5760` → Bấm **Connect**
- QGC sẽ hiện drone trên bản đồ.

**Terminal 3 — Chạy Kịch Bản Bay:**
```bash
# Nhiệm vụ VTOL tự động (Arm → Cất cánh → Bay → Hạ cánh)
./manage.sh mission

# Bay theo quỹ đạo Lemniscate với Geometric SE(3)
./manage.sh geometric
```

Kết quả: Drone sẽ arm → cất cánh → bay theo quỹ đạo → log dữ liệu ra CSV trong `output/`.

#### 2.3.3. Chế Độ 3: Toàn Bộ Hệ Thống (Khuyến Nghị)

Lệnh duy nhất khởi động tất cả (PX4 SITL + Gazebo + Web Dashboard):

```bash
./manage.sh all
```

Sau khi chạy, mở trình duyệt tại: **http://127.0.0.1:8080**

#### 2.3.4. Chế Độ 4: Chạy Unit Test

```bash
./manage.sh test
```

Kết quả mong đợi: Tất cả tests passed + xuất ảnh đồ thị kiểm thử.

#### 2.3.5. Các Lệnh Phụ Trợ
Các lệnh phụ 
```bash
./manage.sh dashboard  # Chỉ khởi Web Dashboard (không cần SITL)
./manage.sh stop       # Dừng tất cả tiến trình nền (PX4, Gazebo, Dashboard)
./manage.sh clean      # Xóa logs, __pycache__, .pytest_cache
./manage.sh package    # Đóng gói dự án thành .tar.gz phân phối
./manage.sh help       # Hiển thị tất cả lệnh có sẵn
```
Sửa lỗi (nếu có)

1. Sửa lỗi xuống dòng (Lỗi CRLF / bad interpreter)
Khi bạn copy script Bash (như manage.sh) từ Windows sang chạy trên Linux/WSL, mã xuống dòng \r\n của Windows sẽ gây ra lỗi \r: command not found hoặc bad interpreter. Câu lệnh sửa lỗi:
```bash
# Lệnh xóa ký tự \r thừa ra khỏi file
sed -i 's/\r$//' manage.sh
# (Hoặc) Sử dụng dos2unix nếu máy bạn đã cài:
dos2unix manage.sh
```
2. Xóa Cache (Build cache & Python cache)
Khi mô phỏng chạy sai kết quả (do Python gọi nhầm cache cũ) hoặc PX4 SITL biên dịch bị kẹt lỗi, bạn sử dụng các lệnh sau để dọn dẹp:
```bash
# Xóa cache của hệ thống điều khiển Python (xóa __pycache__, log, .pytest_cache):
./manage.sh clean
# Xóa cache biên dịch của PX4 (bắt buộc chạy trong thư mục chứa PX4):
cd ~/PX4-Autopilot
make clean
# Nếu PX4 vẫn báo lỗi biên dịch, dùng lệnh Hard Clean để xóa tận gốc:
rm -rf build/
```
### 2.4. Giao Diện Hiển Thị — QGroundControl & PX4 Console

#### 2.4.1. Cửa Sổ QGroundControl

Ở giai đoạn SITL, QGroundControl là giao diện giám sát chính:
- **Fly View**: Bản đồ + vị trí drone thời gian thực + quỹ đạo đã bay
- **Thanh trên**: Trạng thái kết nối, Armed/Disarmed, chế độ bay (Offboard/Manual)
- **Bảng phải**: Telemetry (độ cao, vận tốc, heading, climb rate)
- **La bàn**: Hướng đầu drone hiện tại
- **Cảnh báo**: Battery level, GPS fix, EKF status

#### 2.4.2. PX4 Console (pxh>)

Terminal 1 chạy PX4 sẽ hiện thị log liên tục và có prompt `pxh>`. Các lệnh hữu ích:

```bash
commander status          # Trạng thái hệ thống (armed, mode, health)
param show VT_TYPE        # Loại airframe (2 = QuadPlane)
param show CA_AIRFRAME    # Cấu hình motor (7 = Quad + Pusher)
listener vehicle_status   # Lắng nghe topic trạng thái
listener vehicle_local_position  # Vị trí NED thời gian thực
ekf2 status               # Trạng thái bộ ước lượng EKF2
```

#### 2.4.3. Mô Phỏng Offline — Đồ Thị Matplotlib

Khi chạy `condor_closed_loop_sim.py`, hệ thống tự động xuất:
- **Đồ thị quỹ đạo XY**: So sánh đường bay thực tế vs setpoint của các controller
- **Đồ thị độ cao vs thời gian**: Settling time, overshoot rõ ràng
- **Đồ thị lực đẩy**: Thrust command theo thời gian
- **Bảng metrics**: RMSE, Settling time, Overshoot, Control effort

Khi chạy `condor_mission_sim.py`, xuất thêm:
- **Đồ thị 3D quỹ đạo bay**: Toàn bộ 5 giai đoạn VTOL
- **Airspeed & Altitude vs Flight Phase**: Thể hiện rõ các phase chuyển tiếp
- **Motor Thrusts**: VTOL 4-rotor vs Tractor theo thời gian
- **Năng lượng tiêu thụ**: So sánh với thuần multicopter

### 2.5. Cách Xây Dựng Kịch Bản Bay & Waypoints

#### 2.5.1. Cấu Hình Kịch Bản (scenario_config.yaml)

```yaml
hover:
  target_altitude: -10.0    # [m NED] Độ cao hover (âm = lên cao)
  hover_time: 15.0          # [s] Thời gian hover

square:
  altitude: -10.0           # [m NED]
  side_length: 20.0         # [m] Cạnh hình vuông
  speed: 3.0                # [m/s]

circle:
  altitude: -10.0
  radius: 15.0              # [m]
  angular_speed: 0.2        # [rad/s]
  duration: 60.0            # [s]

figure8:
  altitude: -10.0
  x_amplitude: 20.0         # [m]
  y_amplitude: 40.0         # [m]
  angular_speed: 0.157      # [rad/s] = π/20
  duration: 80.0            # [s]
```

#### 2.5.2. Thêm Kịch Bản Bay Mới

Chỉ cần thêm phương thức mới vào `FlightScenarios` trong `flight_scenarios.py`:

```python
async def run_survey_grid(self):
    """Bay lưới khảo sát."""
    cfg = self.config.get("survey_grid", {})
    rows = cfg.get("rows", 5)
    spacing = cfg.get("spacing", 20.0)   # m
    altitude = cfg.get("altitude", -15.0)

    for i in range(rows):
        x = i * spacing
        y_end = 50.0 if i % 2 == 0 else 0.0
        await self.controller.set_position(x, y_end, altitude, 0)
        await asyncio.sleep(spacing / cfg.get("speed", 5.0))
```

Thêm vào `scenario_config.yaml`:
```yaml
survey_grid:
  rows: 5
  spacing: 20.0
  speed: 5.0
  altitude: -15.0
```

Không cần sửa core system.

### 2.6. Tổng Kết Quy Trình Vận Hành

**Nhanh nhất (kiểm tra thuật toán):**
```bash
source venv_linux/bin/activate
python3 src/simulation/condor_closed_loop_sim.py
```

**Đầy đủ nhất (SITL 3D):**
```bash
# Terminal 1:
./manage.sh sim figure8

# Terminal 2 (sau khi thấy "Ready for takeoff"):
./manage.sh mission
# hoặc: ./manage.sh geometric

# Terminal 3 (tùy chọn — Dashboard):
./manage.sh dashboard
# Mở: http://127.0.0.1:8080
```

**Một lệnh duy nhất:**
```bash
./manage.sh all     # Khởi động mọi thứ tự động
```


---

## CHƯƠNG 3. YÊU CẦU PHI CHỨC NĂNG

### 3.1. Giao Diện Lập Trình (API — Application Programming Interface)

#### 3.1.1. Lớp Trừu Tượng ControllerBase

Tất cả các thuật toán điều khiển trong hệ thống đều kế thừa từ lớp trừu tượng `ControllerBase` được định nghĩa tại `src/controllers/controller_base.py`. Đây là **giao ước thiết kế** (Design Contract) đảm bảo mọi bộ điều khiển có thể hoán đổi cho nhau (swap) trong cả mô phỏng offline lẫn tích hợp PX4 SITL.

```python
from abc import ABC, abstractmethod
import numpy as np

class ControllerBase(ABC):
    @property
    @abstractmethod
    def name(self) -> str:
        """Tên bộ điều khiển."""
        ...

    @abstractmethod
    def reset(self) -> None:
        """Reset trạng thái nội bộ (tích phân, warm-start...)."""
        ...

    @abstractmethod
    def compute(self,
                position_setpoint: np.ndarray,  # [x,y,z] hoặc [x,y,z,vx,vy,vz,ax,ay,az]
                yaw_setpoint: float,             # [rad]
                current_state: np.ndarray,       # 12 phần tử
                dt: float                        # timestep [s]
                ) -> tuple[float, np.ndarray]:   # (thrust [N], torque [N·m, 3])
        ...
```

#### 3.1.2. Đặc Tả Mảng Trạng Thái current_state

Tham số `current_state` là mảng NumPy 1D kích thước 12, chứa toàn bộ trạng thái chuyển động của UAV:

```
Index:   0    1    2    3     4     5     6    7      8    9    10   11
State: [ x,   y,   z,  vx,   vy,   vz,  phi, theta, psi,  p,   q,   r  ]
Unit:  [ m,   m,   m, m/s, m/s, m/s, rad,  rad,  rad, r/s, r/s, r/s ]
Frame: [←────── NED ──────→] [←── NED ───→] [← Euler ZYX →] [← Body →]
```

#### 3.1.3. Đặc Tả Đầu Ra

| Đầu Ra | Kiểu | Đơn Vị | Mô Tả |
|---|---|---|---|
| `thrust` | float | N | Tổng lực đẩy thẳng đứng từ 4 VTOL rotors |
| `torque[0]` | float | N·m | Momen lực trục Roll (τx) |
| `torque[1]` | float | N·m | Momen lực trục Pitch (τy) |
| `torque[2]` | float | N·m | Momen lực trục Yaw (τz) |

#### 3.1.4. Geometric Controller — Mở Rộng Giao Diện

Bộ điều khiển Geometric SE(3) hỗ trợ Differential Flatness feedforward bằng cách chấp nhận setpoint mở rộng (9 phần tử thay vì 3):

```python
# Setpoint cơ bản (3 phần tử):
position_setpoint = np.array([x_d, y_d, z_d])

# Setpoint mở rộng (9 phần tử) — feedforward velocity + acceleration:
position_setpoint = np.array([x_d, y_d, z_d,
                               vx_d, vy_d, vz_d,
                               ax_d, ay_d, az_d])
```

Điều này cho phép Geometric controller bám quỹ đạo chính xác hơn ở tốc độ cao, nhờ biết trước vận tốc và gia tốc cần đạt tại mỗi thời điểm.

Ngoài ra, `GeometricController` còn cung cấp method `compute_attitude_thrust()` dùng riêng cho SITL:
```python
roll_deg, pitch_deg, yaw_deg, thrust_norm = controller.compute_attitude_thrust(
    position_setpoint, yaw_setpoint, current_state
)
# → Gửi trực tiếp đến PX4 qua: drone.offboard.set_attitude(...)
```

### 3.2. Quy Trình Tích Hợp Thuật Toán Điều Khiển Mới

Kiến trúc phần mềm được thiết kế theo nguyên tắc **Open/Closed Principle**: mở cho việc mở rộng (thêm controller mới), đóng cho việc sửa đổi (không cần thay đổi core simulation).

**Bước 1: Tạo File Thuật Toán**

Tạo file `src/controllers/my_controller.py` kế thừa `ControllerBase`:

```python
from src.controllers.controller_base import ControllerBase
import numpy as np

class MyController(ControllerBase):
    def __init__(self, mass: float = 7.8, gravity: float = 9.81):
        self.mass = mass
        self.gravity = gravity
        # Khởi tạo tham số thuật toán...

    @property
    def name(self) -> str:
        return "My Custom Controller"

    def reset(self) -> None:
        # Reset trạng thái nội bộ (tích phân, bộ nhớ...)
        pass

    def compute(self, position_setpoint, yaw_setpoint,
                current_state, dt) -> tuple[float, np.ndarray]:
        # Triển khai thuật toán tính thrust và torque
        pos = current_state[0:3]
        e_p = pos - position_setpoint[0:3]
        # ... tính toán ...
        thrust = self.mass * self.gravity  # ví dụ
        torque = np.zeros(3)
        return float(thrust), torque
```

**Bước 2: Đăng Ký Vào Hệ Thống Mô Phỏng**

Mở `src/simulation/condor_closed_loop_sim.py`, thêm controller vào danh sách so sánh:

```python
from src.controllers.my_controller import MyController

controllers = [
    CascadeController(),
    LQRController(),
    MPCController(),
    GeometricController(),
    MyController(),           # ← Thêm vào đây
]
```

**Bước 3: Thêm Màu Đồ Thị**

Trong các hàm `plot_comparison` và `plot_trajectory_comparison`:

```python
COLORS = {
    "Cascade PID": "blue",
    "LQR": "green",
    "MPC": "red",
    "Geometric SE(3)": "purple",
    "My Custom Controller": "orange",   # ← Thêm màu
}
```

Kết quả: Controller mới sẽ tự động được chạy song song với các controller hiện tại và hiển thị trên cùng đồ thị so sánh.

### 3.3. Độ Ổn Định Kết Nối Giữa Các Công Cụ

#### 3.3.1. Kiến Trúc Mạng Hybrid (WSL2 ↔ Windows)

Hệ thống chạy trên mô hình lai giữa WSL2 Linux (lõi tính toán) và Windows (giao diện):

```
┌──────────────────────────────────────────────────────┐
│  WSL2 Linux                                          │
│                                                      │
│  PX4 SITL ←── gz-transport ──→ Gazebo Harmonic      │
│      │                                               │
│      │ UDP 14540                                     │
│      ↓                                               │
│  mavsdk_server ←── gRPC 50051 ──→ Python Scripts    │
│      │                                               │
│      │ UDP 18570 ──→ tcp_bridge.py ──→ TCP 5760     │
└──────────────────────────────┬───────────────────────┘
                               │ TCP 5760
                        Windows Host
                        QGroundControl
```

#### 3.3.2. Giải Pháp Ổn Định Kết Nối

**Vấn đề 1: IP Động của WSL2**

WSL2 chạy trên Hyper-V NAT, IP thay đổi mỗi lần khởi động. QGroundControl trên Windows mất kết nối do IP cũ.

Giải pháp — `tcp_bridge.py` chuyển đổi giao thức:
- Phía WSL: Lắng nghe UDP 18570 từ PX4
- Phía Windows: Cung cấp TCP server tại `0.0.0.0:5760`
- QGC kết nối cố định tới `localhost:5760` → Ổn định 100%

**Vấn đề 2: Xung Đột Cổng MAVSDK**

PX4 chỉ cấp 1 cổng UDP 14540. Nếu 2 script Python cùng kết nối, bị lỗi "Port in use".

Giải pháp — Mô hình 1 gRPC Server → N Clients:
```bash
# Chạy 1 tiến trình mavsdk_server (C++) kết nối vào UDP 14540
mavsdk_server -p 50051 udp://:14540 &

# Tất cả script Python kết nối vào gRPC port 50051:
await drone.connect(system_address="grpc://localhost:50051")
```
→ Nhiều script (Dashboard + Flight Scenario + Monitoring) chạy song song mà không xung đột.

**Vấn đề 3: Quá Tải WebSocket**

Vòng lặp điều khiển chạy 200-250Hz. Truyền tải toàn bộ qua WebSocket sẽ khiến trình duyệt lag/đơ.

Giải pháp — Throttling ở tầng server:
```python
# server.py — chỉ gửi mỗi 100ms (10Hz)
DASHBOARD_UPDATE_RATE = 0.1  # giây

async def broadcast_telemetry():
    while True:
        data = get_latest_telemetry()
        await websocket.send(json.dumps(data))
        await asyncio.sleep(DASHBOARD_UPDATE_RATE)  # 10Hz
```
→ Tần số 10Hz đủ để mắt người quan sát đồ thị mà CPU trình duyệt chỉ chiếm dưới 5%.

#### 3.3.3. Bảng Tóm Tắt Giao Thức Kết Nối

| Kết Nối | Giao Thức | Cổng | Hướng |
|---|---|---|---|
| PX4 ↔ Gazebo | gz-transport | N/A | Hai chiều |
| PX4 ↔ MAVSDK server | UDP | 14540 | Hai chiều |
| MAVSDK server ↔ Python | gRPC | 50051 | Hai chiều |
| PX4 ↔ TCP Bridge | UDP | 18570 | PX4 → Bridge |
| TCP Bridge ↔ QGC | TCP | 5760 | Bridge → QGC |
| WebSocket server ↔ Browser | WebSocket | 8765 | Server → Browser |
| HTTP server ↔ Browser | HTTP | 8080 | Server → Browser |

### 3.4. Khả Năng Mở Rộng & Tích Hợp

#### 3.4.1. Thêm Kịch Bản Bay Mới

Chỉ cần tạo phương thức mới trong class `FlightScenarios` (file `src/scenarios/flight_scenarios.py`) và thêm cấu hình vào `scenario_config.yaml`. Không cần sửa core system.

#### 3.4.2. Thêm Module AI / Computer Vision

Kiến trúc gRPC cho phép tích hợp song song:

```python
# Module AI riêng biệt kết nối vào cùng mavsdk_server
drone_ai = System()
await drone_ai.connect(system_address="grpc://localhost:50051")

# Đọc camera từ Gazebo topic
async for image in drone_ai.camera.capture_info():
    # Xử lý ảnh, phát hiện vật thể...
    target_pos = detect_object(image)
    # Gửi setpoint mới
    await drone_ai.offboard.set_position_ned(target_pos)
```

#### 3.4.3. Chuyển Đổi Sang Phần Cứng (HIL)

Chỉ cần thay 1 dòng code — chuỗi kết nối MAVSDK:

```python
# Từ SITL:
await drone.connect(system_address="udp://:14540")

# Sang HIL (Pixhawk qua USB):
await drone.connect(system_address="serial:///dev/ttyUSB0:921600")

# Sang HIL (Pixhawk qua Ethernet):
await drone.connect(system_address="udp://192.168.1.10:14550")
```

100% code điều khiển không cần thay đổi.

### 3.5. Hiệu Năng & Tần Số Hoạt Động

#### 3.5.1. Yêu Cầu Tần Số Tối Thiểu

| Thành Phần | Tần Số | Timestep | Ghi Chú |
|---|---|---|---|
| Mô phỏng offline (RK4) | 200 Hz | dt=0.005s | Đảm bảo chính xác số trị |
| SITL Offboard setpoints | 50 Hz | dt=0.02s | Giới hạn PX4 Offboard mode |
| PX4 Rate control (nội bộ) | 1000 Hz | dt=0.001s | PX4 firmware tự xử lý |
| Web Dashboard | 10 Hz | dt=0.1s | Throttled từ 200 Hz |

#### 3.5.2. Yêu Cầu Độ Trễ (SITL)

| Luồng Dữ Liệu | Latency Target |
|---|---|
| Python → MAVSDK → PX4 setpoint | < 20 ms |
| PX4 → Gazebo (vật lý 1 bước) | < 5 ms |
| Telemetry → WebSocket → Browser | < 100 ms |

#### 3.5.3. Yêu Cầu Độ Ổn Định

- PX4 SITL phải duy trì Real-Time Factor ≥ 0.9 (không bị trễ quá 10%)
- Python Offboard loop không được bỏ lỡ quá 2 bước liên tiếp (PX4 sẽ timeout Offboard mode sau 500ms không có setpoint)
- WebSocket connection tự động reconnect khi mất kết nối

#### 3.5.4. Tham Số Hóa — Không Hardcode

Toàn bộ tham số vật lý lưu trong `parameters.yaml`, tham số kịch bản trong `scenario_config.yaml`. Thay đổi UAV mới chỉ cần cập nhật file YAML — không sửa code Python:

```yaml
# parameters.yaml — nguồn dữ liệu duy nhất cho toàn hệ thống
mass: 7.8               # Thay bằng khối lượng UAV mới
wingspan: 2.40          # Thay bằng sải cánh thực tế
wing_area: 0.42         # Diện tích cánh thực tế
...
```

### 3.6. Tổng Kết Yêu Cầu Phi Chức Năng

| NFR | Yêu Cầu | Giải Pháp Kỹ Thuật |
|---|---|---|
| NFR-01: Tần số | Controller ≥ 50 Hz SITL | asyncio non-blocking loop |
| NFR-02: Ổn định | Auto-reconnect WebSocket | try/except + asyncio.sleep retry |
| NFR-03: Khả năng mở rộng | Thêm controller không sửa core | ControllerBase interface |
| NFR-04: HIL upgrade | Chỉ thay 1 dòng kết nối | MAVSDK abstraction layer |
| NFR-05: Giao diện | Dark theme, responsive, Plotly | Tailwind CSS + Plotly.js |
| NFR-06: Tham số hóa | Không hardcode hằng số | YAML config files |
| NFR-07: Kiểm thử | Unit test tự động | pytest + flake8 |


---

## CHƯƠNG 4. THIẾT KẾ LUỒNG DỮ LIỆU & NÂNG CẤP SANG HIL

### 4.1. Bối Cảnh & Mục Tiêu

Để chạy mô phỏng với mô hình UAV thực tế (thay vì mặc định X500), hệ thống cần loading 3 nhóm file cấu hình từ các tầng khác nhau. Chương này mô tả chính xác:
- Cần cung cấp **những file định dạng nào**
- **Đặt chúng ở đâu** trong cây thư mục
- **Chương trình load như thế nào** để đọc thông số thiết bị vào simulation

### 4.2. Tổng Quan: 3 Nhóm File Cần Cung Cấp

```
┌─────────────────────────────────────────────────────────────────┐
│  File Nhóm 1: model.sdf + meshes/    → Gazebo loading (vật lý)  │
│  File Nhóm 2: airframe PX4           → PX4 loading (motor geo)  │
│  File Nhóm 3: parameters.yaml        → Python loading (control) │
└─────────────────────────────────────────────────────────────────┘
```

### 4.3. File Nhóm 1: Mô Hình 3D Gazebo (SDF + Mesh)

#### 4.3.1. Cấu Trúc Thư Mục Bắt Buộc

```
models/quadplane_condor/
├── model.config                # Metadata Gazebo (tên, author, version)
├── model.sdf                   # Định nghĩa vật lý đầy đủ
└── meshes/
    ├── quadplane_condor.dae    # Mesh visual Collada 3D (hiển thị)
    ├── condor_cad.stl          # Mesh CAD độ trung thực cao (tham chiếu)
    ├── iris_prop_cw.dae        # Cánh quạt chiều CW (4 VTOL rotors)
    └── iris_prop_ccw.dae       # Cánh quạt chiều CCW (4 VTOL rotors)
```

#### 4.3.2. Định Dạng File Mesh (Lưới 3D)

| Định dạng | Mục đích | Ghi chú |
|---|---|---|
| `.dae` (Collada) | Visual mesh — hiển thị 3D trong Gazebo | Hỗ trợ texture, màu sắc |
| `.stl` | CAD mesh — tham chiếu hình học | Không có texture |
| `.obj` | Tùy chọn thay thế Collada | Cần đi kèm file `.mtl` |

#### 4.3.3. File model.config — Metadata

```xml
<?xml version="1.0"?>
<model>
  <name>quadplane_condor</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Autonomous UAV Navigation Team</name>
  </author>
  <description>
    Quadplane Condor Hybrid VTOL — 4+1 layout, 2.4m wingspan, 7.8kg MTOW
  </description>
</model>
```

#### 4.3.4. File model.sdf — Đặc Tả Chi Tiết

File SDF (Simulation Description Format) mô tả toàn bộ cấu trúc vật lý drone. Các phần quan trọng cần đồng bộ với `parameters.yaml`:

**Phần Inertial (khối lượng & quán tính — phải khớp với parameters.yaml):**
```xml
<inertial>
  <mass>7.8</mass>        <!-- [kg] MTOW — khớp: parameters.yaml/mass -->
  <inertia>
    <ixx>1.46</ixx>       <!-- [kg·m²] Roll inertia — khớp: Ixx -->
    <iyy>1.06</iyy>       <!-- [kg·m²] Pitch inertia — khớp: Iyy -->
    <izz>2.50</izz>       <!-- [kg·m²] Yaw inertia — khớp: Izz -->
    <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
  </inertia>
</inertial>
```

**Phần Visual (mesh hiển thị):**
```xml
<visual name="base_visual">
  <geometry>
    <mesh>
      <uri>model://quadplane_condor/meshes/quadplane_condor.dae</uri>
      <scale>1 1 1</scale>
    </mesh>
  </geometry>
</visual>
```

**Plugin 4 VTOL Motor (k_thrust phải khớp parameters.yaml):**
```xml
<!-- Motor 0: Front-Right, CCW -->
<plugin filename="gz-sim-multicopter-motor-model-system"
        name="gz::sim::systems::MulticopterMotorModel">
  <robotNamespace>quadplane_condor</robotNamespace>
  <jointName>rotor_0_joint</jointName>
  <motorNumber>0</motorNumber>
  <turningDirection>ccw</turningDirection>
  <maxRotVelocity>1500.0</maxRotVelocity>      <!-- ~14300 RPM — khớp: max_rpm -->
  <motorConstant>2.5e-5</motorConstant>         <!-- k_thrust — khớp: vtol_motor/k_thrust -->
  <momentConstant>0.0424</momentConstant>        <!-- k_drag/k_thrust ratio -->
  <motorSpeedPubTopic>motor_speed/0</motorSpeedPubTopic>
  <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
  <rollingMomentCoefficient>1e-6</rollingMomentCoefficient>
  <timeConstantUp>0.0125</timeConstantUp>
  <timeConstantDown>0.025</timeConstantDown>
</plugin>

<!-- Motor 4: Tractor Nose, +X forward -->
<plugin filename="gz-sim-multicopter-motor-model-system"
        name="gz::sim::systems::MulticopterMotorModel">
  <motorNumber>4</motorNumber>
  <motorConstant>8.55e-6</motorConstant>         <!-- khớp: tractor_motor/k_thrust -->
  <maxRotVelocity>3500.0</maxRotVelocity>         <!-- ~33400 RPM -->
</plugin>
```

**Plugin Khí Động Học (AerodynamicsPlugin) — đặc thù Quadplane:**
```xml
<plugin filename="gz-sim-aerodynamics-system"
        name="gz::sim::systems::Aerodynamics">
  <wind_velocity_topic>/world/quadplane_condor/wind</wind_velocity_topic>
  <link_name>base_link</link_name>
  <!-- Thông số khí động học cánh chính -->
  <lift_coefficient>4.86</lift_coefficient>      <!-- CL_alpha — khớp YAML -->
  <drag_coefficient>0.024</drag_coefficient>      <!-- CD0 — khớp YAML -->
  <wing_area>0.42</wing_area>                    <!-- S — khớp YAML -->
  <stall_angle>0.297</stall_angle>               <!-- alpha_stall — khớp YAML -->
</plugin>
```

#### 4.3.5. Cài Đặt Model Vào PX4

```bash
# Tự động qua manage.sh
./manage.sh setup

# Hoặc thủ công:
cp -r models/quadplane_condor/ ~/PX4-Autopilot/Tools/simulation/gz/models/
cp models/worlds/condor_figure8.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

### 4.4. File Nhóm 2: Airframe PX4 (Cấu Hình Motor Geometry)

#### 4.4.1. Airframe File Là Gì?

Airframe file là script shell định nghĩa **hình học motor** cho PX4 Control Allocator. Nó mô tả vị trí, hướng và đặc tính của từng motor để PX4 biết cách phân phối thrust/torque command ra từng actuator.

**Vị trí trong PX4:**
```
~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4030_gz_quadplane_condor
```

**ID airframe:** `4030` (đăng ký trong CMakeLists.txt)

#### 4.4.2. Nội Dung Airframe File Quadplane Condor

```bash
#!/bin/sh
# Quadplane Condor — 4+1 VTOL Hybrid
# Airframe ID: 4030
# Layout: QuadPlane (4 VTOL lifters + 1 forward tractor)

. ${R}etc/init.d/rc.vtol_defaults

# Loại airframe VTOL
param set-default VT_TYPE 2          # 2 = QuadPlane

# Cấu hình Control Allocator
param set-default CA_AIRFRAME 7      # 7 = Quad + Pusher

# --- 4 VTOL Lift Motors ---
# Motor 0: Front-Right (CCW)
param set-default CA_ROTOR0_PX  0.4416   # x từ CG [m]
param set-default CA_ROTOR0_PY  0.4236   # y từ CG [m]
param set-default CA_ROTOR0_PZ  0.0
param set-default CA_ROTOR0_KM -0.05     # CCW: âm

# Motor 1: Rear-Left (CCW)
param set-default CA_ROTOR1_PX -0.4428
param set-default CA_ROTOR1_PY -0.4192
param set-default CA_ROTOR1_KM -0.05    # CCW: âm

# Motor 2: Front-Left (CW)
param set-default CA_ROTOR2_PX  0.4416
param set-default CA_ROTOR2_PY -0.4236
param set-default CA_ROTOR2_KM  0.05    # CW: dương

# Motor 3: Rear-Right (CW)
param set-default CA_ROTOR3_PX -0.4428
param set-default CA_ROTOR3_PY  0.4192
param set-default CA_ROTOR3_KM  0.05    # CW: dương

# Motor 4: Nose Tractor (+X forward thrust)
param set-default CA_ROTOR4_AX  1.0     # Hướng +X (forward)
param set-default CA_ROTOR4_AY  0.0
param set-default CA_ROTOR4_AZ  0.0

# --- V-Tail Control Surfaces ---
param set-default CA_SV0_FUNC  202      # CS0: Left ruddervator
param set-default CA_SV1_FUNC  203      # CS1: Right ruddervator

# Tốc độ chuyển tiếp VTOL
param set-default VT_ARSP_TRANS 15.0   # [m/s] — khớp: parameters.yaml/v_trans
param set-default VT_ARSP_BLEND 12.0   # [m/s] bắt đầu blend

# Giới hạn góc nghiêng
param set-default MC_ROLL_P    6.5
param set-default MC_PITCH_P   6.5
```

#### 4.4.3. Bảng Tham Số CA_ROTOR Cần Cung Cấp

| Tham Số | Đơn Vị | Mô Tả | Nguồn |
|---|---|---|---|
| `CA_ROTOR{n}_PX` | m | Vị trí X (North) từ CG | `parameters.yaml/arm_length_x` |
| `CA_ROTOR{n}_PY` | m | Vị trí Y (East) từ CG | `parameters.yaml/arm_length_y` |
| `CA_ROTOR{n}_PZ` | m | Vị trí Z (Down) từ CG | Thường = 0 |
| `CA_ROTOR{n}_KM` | — | Hệ số momen yaw (+ CW, − CCW) | Từ thiết kế cơ khí |
| `CA_ROTOR4_AX/Y/Z` | — | Hướng thrust motor tractor | `[1,0,0]` = +X forward |

### 4.5. File Nhóm 3: Tham Số Python (parameters.yaml)

#### 4.5.1. Tại Sao Cần File YAML Riêng?

- Tách biệt hoàn toàn tham số khỏi code logic → thay đổi UAV không cần sửa Python
- Đồng bộ tham số giữa Python simulation và Gazebo/PX4
- Dễ version control và chia sẻ với nhóm

#### 4.5.2. Định Dạng File YAML Đầy Đủ

```yaml
# ========================================================
# Quadplane Condor — Physical, Aerodynamic, VTOL Parameters
# QUAN TRỌNG: Phải đồng bộ với model.sdf và airframe file!
# ========================================================

# -- Khối lượng & Kích thước -- (đồng bộ với model.sdf/inertial)
mass: 7.8                 # [kg] MTOW
wingspan: 2.40            # [m] Sải cánh
wing_area: 0.42           # [m²] Diện tích cánh (KHÔNG phải 0.72!)
wing_chord: 0.175         # [m] Mean Aerodynamic Chord (KHÔNG phải 0.30!)
aspect_ratio: 13.71       # = wingspan² / wing_area

# -- Mô men quán tính [kg·m²] -- (đồng bộ với model.sdf/ixx,iyy,izz)
inertia:
  Ixx: 1.46               # Roll
  Iyy: 1.06               # Pitch
  Izz: 2.50               # Yaw

# -- Động cơ VTOL (đồng bộ với model.sdf/motorConstant) --
vtol_motor:
  k_thrust: 2.5e-5        # [N/(rad/s)²] F = k_thrust × ω²
  k_drag: 1.06e-6         # [N·m/(rad/s)²]
  max_rpm: 14300
  min_rpm: 100
  arm_length_x: 0.4416    # [m] (đồng bộ CA_ROTOR{n}_PX)
  arm_length_y: 0.4236    # [m] (đồng bộ CA_ROTOR{n}_PY)

# -- Động cơ Tractor --
tractor_motor:
  k_thrust: 8.55e-6
  max_thrust: 34.0        # [N]
  max_rpm: 33400

# -- Khí động học cánh -- (đồng bộ với SDF AerodynamicsPlugin)
aerodynamics:
  air_density: 1.225
  CL0: 0.28
  CL_alpha: 4.86          # [1/rad]
  CD0: 0.024
  induced_drag_factor: 0.028
  alpha_stall: 0.297      # [rad] ~17°

# -- V-Tail --
vtail:
  area: 0.12              # [m²]
  lever_arm: 0.8655       # [m]
  dihedral_angle: 0.7854  # [rad] 45°

# -- Chuyển tiếp VTOL -- (đồng bộ VT_ARSP_TRANS trong airframe)
vtol_transition:
  v_stall: 12.0           # [m/s]
  v_trans: 15.0           # [m/s] (đồng bộ VT_ARSP_TRANS)
  v_cruise: 18.0          # [m/s]
  v_max: 25.0             # [m/s]
  transition_timeout: 15.0
  hover_altitude: 15.0    # [m]

gravity: 9.81             # [m/s²]
simulation:
  dt: 0.005               # [s] 200 Hz
  duration: 60.0          # [s]
```

#### 4.5.3. Cách Python Load File YAML

```python
# src/uav_model/condor_dynamics.py
@classmethod
def from_yaml(cls, path: str | Path | None = None) -> "QuadplaneParams":
    """Load tham số vật lý từ file YAML."""
    if path is None:
        # Tự động tìm parameters.yaml cùng thư mục
        path = Path(__file__).resolve().parents[0] / "parameters.yaml"
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    return cls(
        mass          = cfg["mass"],
        wingspan      = cfg["wingspan"],
        wing_area     = cfg["wing_area"],       # 0.42 m²
        wing_chord    = cfg["wing_chord"],      # 0.175 m
        Ixx           = cfg["inertia"]["Ixx"],
        CL_alpha      = cfg["aerodynamics"]["CL_alpha"],
        v_trans       = cfg["vtol_transition"]["v_trans"],
        # ... tất cả tham số ...
    )

# Sử dụng:
params = QuadplaneParams.from_yaml()              # Dùng file mặc định
params = QuadplaneParams.from_yaml("custom.yaml") # Dùng file tùy chỉnh
drone  = QuadplaneDynamics(params)
```

### 4.6. Bản Đồ Đồng Bộ Tham Số (Parameter Coupling Map)

```
parameters.yaml          model.sdf                    airframe PX4
─────────────────        ────────────────────         ─────────────────
mass: 7.8           ↔   <mass>7.8</mass>
Ixx: 1.46           ↔   <ixx>1.46</ixx>
wing_area: 0.42     ↔   AerodynamicsPlugin/wing_area
v_trans: 15.0       ↔                            ↔   VT_ARSP_TRANS=15.0
arm_length_x: 0.4416↔                            ↔   CA_ROTOR0_PX=0.4416
k_thrust: 2.5e-5    ↔   <motorConstant>2.5e-5
```

> ⚠️ **Quan trọng:** 3 nhóm file phải đồng bộ với nhau. Nếu thay đổi một file mà không cập nhật file kia, mô phỏng sẽ cho kết quả không nhất quán.

#### 4.6.1. Quy Trình Tích Hợp Mô Hình Mới — Step by Step

```
Bước 1: Có CAD model mới
    → Xuất mesh: .dae (visual) + .stl (collision)
    → Đặt vào: models/<tên_model>/meshes/

Bước 2: Tạo model.config
    → Khai báo tên, version, đường dẫn SDF

Bước 3: Tạo model.sdf
    → Điền mass, inertia từ thông số kỹ thuật thực tế
    → Thêm visual/collision mesh
    → Cấu hình motor plugins (k_thrust, số motor, vị trí)
    → Cấu hình AerodynamicsPlugin nếu là fixed-wing

Bước 4: Cập nhật parameters.yaml
    → Đồng bộ mass, Ixx/Iyy/Izz với model.sdf
    → Điền thông số khí động học (CL_alpha, CD0, wing_area...)
    → Cập nhật arm_length từ vị trí motor thực tế

Bước 5: Tạo airframe file PX4
    → Đặt đúng CA_ROTOR{n}_PX/PY từ arm_length trong YAML
    → Đặt VT_ARSP_TRANS = v_trans trong YAML

Bước 6: Chạy ./manage.sh setup
    → Tự động copy model, airframe, world vào PX4-Autopilot

Bước 7: Kiểm thử offline trước
    → python3 src/simulation/condor_closed_loop_sim.py
    → Xác nhận model bay ổn định với thông số mới

Bước 8: Chạy SITL
    → ./manage.sh sim
```

### 4.7. Bảng Tóm Tắt: Nhóm Cơ Khí Cần Cung Cấp Gì?

#### 4.7.1. Checklist Giao Nhận File

| File | Bắt buộc | Vị Trí Đặt |
|---|---|---|
| `model.sdf` | ✅ | `models/<tên>/model.sdf` |
| `model.config` | ✅ | `models/<tên>/model.config` |
| `*.dae` (visual mesh) | ✅ | `models/<tên>/meshes/` |
| `*.stl` (CAD mesh) | Tùy chọn | `models/<tên>/meshes/` |
| `4030_gz_*` (airframe) | ✅ | `models/airframes/` |
| `parameters.yaml` | ✅ | `src/uav_model/` |

#### 4.7.2. Bảng Số Liệu Kỹ Thuật Cần Cung Cấp

| Thông Số | Ký Hiệu | Cần Đo/Tính | Dùng Ở |
|---|---|---|---|
| Khối lượng | m | Cân drone đầy tải | YAML + SDF |
| Sải cánh | b | Đo trực tiếp | YAML |
| Diện tích cánh | S | Tính từ b × chord | YAML + SDF |
| Mô men quán tính | Ixx, Iyy, Izz | CAD hoặc đo thực | YAML + SDF |
| Vị trí motor | (x, y) từ CG | Đo từ trọng tâm | YAML + airframe |
| Hệ số lực đẩy | k_thrust | Bench test motor | YAML + SDF |
| Hệ số nâng | CL_alpha | Wind tunnel/CFD | YAML + SDF |
| Tốc độ stall | v_stall | Tính toán khí động | YAML + airframe |

### 4.8. Luồng Dữ Liệu Sau Khi Tích Hợp

```
Khởi động ./manage.sh sim
    │
    ├── [1] Gazebo load: models/quadplane_condor/model.sdf
    │       → Vật lý 3D, motor plugins, aerodynamics
    │       → Cảm biến ảo (IMU/GPS) → PX4 EKF2
    │
    ├── [2] PX4 load: airframes/4030_gz_quadplane_condor
    │       → Motor geometry (CA_ROTOR positions)
    │       → VTOL params (VT_ARSP_TRANS, VT_TYPE)
    │       → Rate PID defaults
    │
    └── [3] Python load: src/uav_model/parameters.yaml
            → QuadplaneParams.from_yaml()
            → QuadplaneDynamics(params)
            → VTOLHybridController(params)
```

```
Vòng lặp SITL (50 Hz):
    │
    ├── Gazebo → sensor data → PX4 EKF2
    ├── PX4 EKF2 → estimated state → MAVSDK telemetry
    ├── Python đọc telemetry → current_state[12]
    ├── Controller.compute(setpoint, state) → thrust, torque
    ├── Python gửi Offboard setpoint → PX4
    ├── PX4 Rate PID → actuator commands → Gazebo
    └── Gazebo cập nhật vật lý → lặp lại
```

### 4.9. Định Hướng Nâng Cấp Sang HIL

Kiến trúc phần mềm tách biệt hoàn toàn logic điều khiển khỏi lớp truyền thông. Chuyển SITL → HIL chỉ cần đổi 1 dòng:

```python
# SITL:
await drone.connect(system_address="udp://:14540")

# HIL — Pixhawk via USB:
await drone.connect(system_address="serial:///dev/ttyUSB0:921600")

# HIL — Pixhawk via Ethernet:
await drone.connect(system_address="udp://192.168.1.10:14550")
```

Toàn bộ code điều khiển, kịch bản bay, Web Dashboard không thay đổi.

Thêm vào đó, Gazebo vẫn chạy để hiển thị trạng thái 3D, nhưng vật lý thực được cung cấp bởi cảm biến Pixhawk thật thay vì mô phỏng.


---

## CHƯƠNG 5. MÔ HÌNH UAV & BỘ ĐIỀU KHIỂN

### 5.1. Tổng Quan Mô Hình Động Lực Học

File: `src/uav_model/condor_dynamics.py`

Mô hình toán học Quadplane Condor sử dụng phương trình Newton-Euler **6 bậc tự do (6-DOF)** để mô tả đầy đủ chuyển động tịnh tiến và quay của vật rắn trong không gian 3D. Đặc điểm triển khai:

- **12 biến trạng thái** biểu diễn toàn bộ trạng thái chuyển động
- **Tích phân số RK4** (Runge-Kutta bậc 4) — sai số O(dt⁴), tần số 200 Hz
- **Quy ước NED** (North-East-Down): trục Z hướng xuống, bay lên = z âm
- **Tham số hóa YAML**: Mọi hằng số vật lý load từ `parameters.yaml`
- **Mô hình khí động học** đầy đủ: lực nâng, lực cản, stall, V-tail moments

### 5.2. Hệ Tọa Độ (Coordinate Frames)

#### 5.2.1. Hệ Tọa Độ Trái Đất — NED Frame (F_E)

Gốc tọa độ cố định trên mặt đất (điểm xuất phát của drone):
- **X (North)**: Hướng Bắc, dương về phía Bắc
- **Y (East)**: Hướng Đông, dương về phía Đông
- **Z (Down)**: Hướng xuống tâm Trái Đất, **dương khi đi xuống**

> Quy ước quan trọng: Drone bay lên 15m → `state[2] = z = −15.0 m`

#### 5.2.2. Hệ Tọa Độ Gắn Với Thân — Body Frame (F_B)

Gốc tọa độ tại trọng tâm (CoM) của UAV:
- **X (Front)**: Hướng mũi drone (hướng tractor motor)
- **Y (Right)**: Hướng cánh phải drone
- **Z (Down)**: Vuông góc mặt phẳng cánh quạt, hướng xuống

### 5.3. Biến Trạng Thái (12 Biến)

Trạng thái UAV tại mỗi thời điểm được biểu diễn bằng vector 12 phần tử:

```python
state = np.zeros(12)
# state[0:3]  = [x, y, z]          Vị trí NED [m]
# state[3:6]  = [vx, vy, vz]       Vận tốc NED [m/s]
# state[6:9]  = [phi, theta, psi]  Góc Euler ZYX [rad]
# state[9:12] = [p, q, r]          Tốc độ góc body frame [rad/s]
```

Triển khai trong class `QuadplaneDynamics`:
```python
class QuadplaneDynamics:
    NUM_STATES = 12

    def __init__(self, params: QuadplaneParams | None = None):
        self.params = params or QuadplaneParams()
        self.state = np.zeros(self.NUM_STATES)   # Khởi tạo tại gốc
        self.time = 0.0
```

### 5.4. Tham Số Vật Lý (QuadplaneParams)

#### 5.4.1. Cấu Trúc Dữ Liệu

```python
@dataclass
class QuadplaneParams:
    """Toàn bộ tham số vật lý và khí động học Quadplane Condor."""
    # Khối lượng & kích thước
    mass: float = 7.8           # [kg]
    wingspan: float = 2.40      # [m]
    wing_area: float = 0.42     # [m²]
    wing_chord: float = 0.175   # [m] Mean Aerodynamic Chord
    aspect_ratio: float = 13.71 # b²/S

    # Mô men quán tính [kg·m²]
    Ixx: float = 1.46           # Roll
    Iyy: float = 1.06           # Pitch
    Izz: float = 2.50           # Yaw

    # Khí động học cánh
    CL0: float = 0.28           # Hệ số nâng tại AoA=0
    CL_alpha: float = 4.86      # Độ dốc [1/rad]
    CD0: float = 0.024          # Cản ký sinh
    induced_drag_k: float = 0.028  # k trong CD = CD0 + k·CL²
    alpha_stall: float = 0.297  # [rad] ~17°

    # Giới hạn tốc độ
    v_stall: float = 12.0       # [m/s]
    v_trans: float = 15.0       # [m/s]
    v_cruise: float = 18.0      # [m/s]

    # V-Tail
    vtail_area: float = 0.12    # [m²]
    vtail_arm: float = 0.8655   # [m] từ CG đến V-tail

    # Tractor motor
    max_tractor_thrust: float = 34.0  # [N]

    # Môi trường & mô phỏng
    air_density: float = 1.225  # [kg/m³]
    gravity: float = 9.81       # [m/s²]
    dt: float = 0.005           # [s] 200 Hz
```

#### 5.4.2. Các Thuộc Tính Tính Toán

```python
@property
def inertia_matrix(self) -> np.ndarray:
    return np.diag([self.Ixx, self.Iyy, self.Izz])   # J [3×3]

@property
def hover_thrust(self) -> float:
    return self.mass * self.gravity                    # = 76.5 N
```

#### 5.4.3. Ý Nghĩa Vật Lý Các Tham Số

| Tham Số | Giá Trị | Ý Nghĩa Vật Lý |
|---|---|---|
| `mass = 7.8 kg` | 7.8 | Tổng khối lượng → trọng lực = 76.5 N |
| `Ixx = 1.46 kg·m²` | 1.46 | Quán tính Roll → cánh 2.4m gây lớn |
| `Iyy = 1.06 kg·m²` | 1.06 | Quán tính Pitch → thân dài 1.64m |
| `CL_alpha = 4.86/rad` | 4.86 | Cánh hiệu quả cao (AR=13.71) |
| `CD0 = 0.024` | 0.024 | Cản thấp (airfoil mỏng, cánh sạch) |
| `alpha_stall = 17°` | 0.297 rad | Cánh AR cao → stall ở AoA nhỏ |
| `v_trans = 15 m/s` | 15 | Đủ airspeed để cánh chịu toàn tải |

### 5.5. Ma Trận Quay Euler ZYX (Rotation Matrix)

#### 5.5.1. Công Thức Toán Học

Ma trận quay từ Body Frame sang NED Frame theo thứ tự Z-Y-X (Yaw → Pitch → Roll):

```
R(φ,θ,ψ) = Rz(ψ) × Ry(θ) × Rx(φ)

     ⎡ cψ·cθ   cψ·sθ·sφ − sψ·cφ   cψ·sθ·cφ + sψ·sφ ⎤
R =  ⎢ sψ·cθ   sψ·sθ·sφ + cψ·cφ   sψ·sθ·cφ − cψ·sφ ⎥
     ⎣  −sθ         cθ·sφ                 cθ·cφ       ⎦
```

#### 5.5.2. Triển Khai Code

```python
@staticmethod
def rotation_matrix(phi: float, theta: float, psi: float) -> np.ndarray:
    """ZYX Euler rotation matrix: body frame → NED frame."""
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth,  sth  = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)
    return np.array([
        [cpsi*cth, cpsi*sth*sphi - spsi*cphi, cpsi*sth*cphi + spsi*sphi],
        [spsi*cth, spsi*sth*sphi + cpsi*cphi, spsi*sth*cphi - cpsi*sphi],
        [  -sth,           cth*sphi,                   cth*cphi         ],
    ])
```

Ứng dụng: Chuyển đổi vector lực đẩy từ body frame sang NED frame:
```python
F_total_ned = R @ F_total_body
```

### 5.6. Ma Trận Chuyển Đổi Tốc Độ Góc (Euler Rate Matrix)

#### 5.6.1. Công Thức

Mối quan hệ giữa tốc độ thay đổi góc Euler `[φ̇, θ̇, ψ̇]` và tốc độ góc body frame `[p, q, r]`:

```
⎡ φ̇ ⎤   ⎡ 1  sφ·tθ  cφ·tθ ⎤   ⎡ p ⎤
⎢ θ̇ ⎥ = ⎢ 0    cφ    −sφ  ⎥ × ⎢ q ⎥
⎣ ψ̇ ⎦   ⎣ 0  sφ/cθ  cφ/cθ ⎦   ⎣ r ⎦
```

Lưu ý: Ma trận W có đặc dị (singularity) khi θ = ±90° (gimbal lock). Code xử lý bằng cách clamp `cos(θ) ≥ 1e-4`.

#### 5.6.2. Triển Khai Code

```python
@staticmethod
def euler_rate_matrix(phi: float, theta: float) -> np.ndarray:
    """Matrix converting body rates [p,q,r] to Euler angle rates."""
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth = np.cos(theta)
    if abs(cth) < 1e-4:          # Tránh chia cho 0 (gimbal lock)
        cth = 1e-4 * np.sign(cth) if cth != 0 else 1e-4
    tth = np.sin(theta) / cth
    return np.array([
        [1.0,  sphi*tth,  cphi*tth],
        [0.0,  cphi,      -sphi   ],
        [0.0,  sphi/cth,  cphi/cth],
    ])
```

### 5.7. Phương Trình Newton-Euler (Đạo Hàm Trạng Thái)

#### 5.7.1. Động Lực Học Tịnh Tiến

Áp dụng Định luật 2 Newton cho chuyển động tịnh tiến trong hệ NED:

```
m × v̇_NED = R × (F_vtol + F_tractor + F_aero)_body + F_gravity_NED
```

Trong đó:
- `R × [0, 0, -T_vtol]`: Lực VTOL chiều từ body sang NED (T dương = đẩy lên trong body frame)
- `R × [T_tractor, 0, 0]`: Lực tractor motor (+X forward)
- `R × F_aero`: Lực khí động học (lift, drag, side force)
- `[0, 0, mg]`: Trọng lực trong NED (hướng xuống = Z dương)

#### 5.7.2. Động Lực Học Quay

Phương trình Euler cho vật rắn:

```
J × ω̇ = τ_total − ω × (J × ω)
```

Suy ra:
```
ω̇ = J⁻¹ × [τ_total − ω × (J × ω)]
```

Trong đó:
- `J = diag(Ixx, Iyy, Izz)`: Ma trận quán tính
- `τ_total = τ_vtol + τ_aero`: Tổng momen điều khiển
- `ω × (J × ω)`: Hiệu ứng con quay hồi chuyển (gyroscopic coupling)

#### 5.7.3. Mô Hình Khí Động Học — Hàm compute_aerodynamics()

```python
def compute_aerodynamics(self, u_body, w_body, v_body,
                          delta_e=0.0, delta_r=0.0):
    """Lực và momen khí động trong body frame."""
    p = self.params
    airspeed = np.sqrt(u_body**2 + v_body**2 + w_body**2)
    if airspeed < 0.1:
        return np.zeros(3), np.zeros(3)

    # Góc tấn (AoA) và góc trượt (sideslip)
    alpha = np.arctan2(w_body, max(u_body, 0.01))
    beta  = np.arcsin(np.clip(v_body/airspeed, -1.0, 1.0))

    # Áp suất động q = 0.5·ρ·V²
    q_dyn = 0.5 * p.air_density * airspeed**2

    # Hệ số nâng CL (tuyến tính + stall)
    if abs(alpha) < p.alpha_stall:
        CL = p.CL0 + p.CL_alpha * alpha
    else:
        CL = np.sign(alpha) * (p.CL0 + p.CL_alpha*p.alpha_stall) * np.cos(alpha)

    # Hệ số cản CD = CD0 + k·CL² + cản sideslip
    CD = p.CD0 + p.induced_drag_k * CL**2 + 0.1*np.sin(beta)**2

    # Lực nâng và cản
    Lift = q_dyn * p.wing_area * CL
    Drag = q_dyn * p.wing_area * CD

    # Chuyển Wind Frame → Body Frame
    Fx = -Drag*np.cos(alpha) + Lift*np.sin(alpha)  # Forward
    Fy = -q_dyn * p.wing_area * 0.2 * beta          # Side force
    Fz = -Drag*np.sin(alpha) - Lift*np.cos(alpha)  # Vertical

    # Momen V-Tail (pitch + yaw)
    My = -q_dyn*p.vtail_area*p.vtail_arm*(0.8*delta_e + 0.15*alpha)
    Mz =  q_dyn*p.vtail_area*p.vtail_arm*(0.6*delta_r - 0.1*beta)
    Mx = -q_dyn*p.wing_area*p.wingspan*0.05*beta    # Roll damping

    return np.array([Fx, Fy, Fz]), np.array([Mx, My, Mz])
```

#### 5.7.4. Triển Khai Hàm _derivatives()

```python
def _derivatives(self, state, vtol_thrust, vtol_torque,
                  tractor_thrust=0.0, delta_e=0.0, delta_r=0.0):
    """Tính đạo hàm trạng thái xdot = f(x, u) cho Hybrid Quadplane."""
    p = self.params
    vel   = state[3:6]
    phi, theta, psi = state[6:9]
    omega = state[9:12]

    R     = self.rotation_matrix(phi, theta, psi)
    vel_body = R.T @ vel                        # NED → body

    # Lực & momen khí động
    F_aero_b, M_aero_b = self.compute_aerodynamics(
        vel_body[0], vel_body[2], vel_body[1], delta_e, delta_r)

    # Lực đẩy trong body frame
    F_vtol_b    = np.array([0.0, 0.0, -vtol_thrust])   # VTOL: lên (-Z)
    F_tractor_b = np.array([tractor_thrust, 0.0, 0.0]) # Tractor: forward (+X)

    F_total_b   = F_vtol_b + F_tractor_b + F_aero_b

    # Chuyển sang NED + trọng lực
    F_total_ned = R @ F_total_b
    gravity_ned = np.array([0.0, 0.0, p.mass * p.gravity])
    accel_ned   = (F_total_ned + gravity_ned) / p.mass

    # Động lực học quay
    J     = p.inertia_matrix
    J_inv = p.inertia_inv
    M_total_b = vtol_torque + M_aero_b
    omega_dot = J_inv @ (M_total_b - np.cross(omega, J @ omega))

    # Tốc độ góc Euler
    W         = self.euler_rate_matrix(phi, theta)
    euler_dot = W @ omega

    xdot = np.zeros(self.NUM_STATES)
    xdot[0:3]  = vel          # ṙ = v
    xdot[3:6]  = accel_ned    # v̇ = F/m
    xdot[6:9]  = euler_dot    # Euler angles rate
    xdot[9:12] = omega_dot    # Angular acceleration
    return xdot
```

### 5.8. Tích Phân RK4 (Runge-Kutta Bậc 4)

#### 5.8.1. Lý Thuyết

Phương pháp RK4 xấp xỉ nghiệm phương trình vi phân `ẋ = f(x, u)` với độ chính xác bậc 4:

```
k1 = f(sₙ, u)
k2 = f(sₙ + 0.5·dt·k1, u)
k3 = f(sₙ + 0.5·dt·k2, u)
k4 = f(sₙ + dt·k3, u)
sₙ₊₁ = sₙ + (dt/6)·(k1 + 2k2 + 2k3 + k4)
```

Tại sao chọn RK4 thay vì Euler?
- **Euler bậc 1**: Sai số O(dt) → với dt=0.005s sai số tích lũy nhanh, drone "bay lệch"
- **RK4 bậc 4**: Sai số O(dt⁴) → chính xác hơn hàng nghìn lần với cùng dt

#### 5.8.2. Triển Khai Hàm step()

```python
def step(self, vtol_thrust, vtol_torque, dt=None,
         tractor_thrust=0.0, delta_e=0.0, delta_r=0.0):
    """Advance simulation one timestep — RK4 integration."""
    if dt is None:
        dt = self.params.dt     # Mặc định 0.005s (200 Hz)

    s = self.state
    # 4 lần đánh giá hàm đạo hàm
    k1 = self._derivatives(s,                  vtol_thrust, vtol_torque, tractor_thrust, delta_e, delta_r)
    k2 = self._derivatives(s + 0.5*dt*k1,     vtol_thrust, vtol_torque, tractor_thrust, delta_e, delta_r)
    k3 = self._derivatives(s + 0.5*dt*k2,     vtol_thrust, vtol_torque, tractor_thrust, delta_e, delta_r)
    k4 = self._derivatives(s + dt*k3,          vtol_thrust, vtol_torque, tractor_thrust, delta_e, delta_r)

    # Cập nhật trạng thái
    self.state = s + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)

    # Chuẩn hóa góc Euler về [-π, π]
    for i in range(6, 9):
        self.state[i] = (self.state[i] + np.pi) % (2*np.pi) - np.pi

    self.time += dt
    return self.state.copy()
```

#### 5.8.3. Sơ Đồ 1 Bước RK4

```
state(t)
   │
   ├─→ _derivatives(state(t))              → k1
   ├─→ _derivatives(state + 0.5·dt·k1)    → k2
   ├─→ _derivatives(state + 0.5·dt·k2)    → k3
   ├─→ _derivatives(state + dt·k3)         → k4
   │
   └─→ state(t+dt) = state(t) + dt/6 × (k1 + 2k2 + 2k3 + k4)
```

Tần số cập nhật: **200 Hz** (dt = 0.005s) → Mỗi giây mô phỏng thực hiện 200 lần tính toán RK4 (800 lần đánh giá `_derivatives`).

### 5.9. Các FlightPhase — Giai Đoạn Bay

```python
class FlightPhase(Enum):
    VTOL_HOVER          = "VTOL_HOVER"           # Cất cánh & hover VTOL
    FORWARD_TRANSITION  = "FORWARD_TRANSITION"   # Chuyển tiếp sang cánh cứng
    FIXED_WING_CRUISE   = "FIXED_WING_CRUISE"    # Bay cánh cứng hiệu quả
    BACK_TRANSITION     = "BACK_TRANSITION"      # Chuyển tiếp về VTOL
    VTOL_LAND           = "VTOL_LAND"            # Hạ cánh VTOL
```

### 5.10. Tổng Kết: Dòng Chảy Cập Nhật Trạng Thái

```
Bộ điều khiển → (thrust, torque, tractor_thrust, delta_e, delta_r)
                                    │
                                    ▼
                    QuadplaneDynamics.step()
                            │
                    ┌───────┴────────┐
                    │                │
              _derivatives()    (×4 lần cho RK4)
                    │
        ┌───────────┼───────────────┐
        │           │               │
  Khí động     Propulsion      Trọng lực
  (lift/drag   (VTOL + tractor  (NED)
   V-tail)      thrust)
        │           │               │
        └───────────┴───────────────┘
                    │
              Tổng F_body → R → F_NED
              Tổng τ_body → ω̇
                    │
              RK4 integration
                    │
              state(t+dt)  ← trạng thái mới
                    │
              [x,y,z, vx,vy,vz, φ,θ,ψ, p,q,r]
```


---

## CHƯƠNG 6. TÍCH HỢP MÔ HÌNH UAV VỚI ĐIỀU KHIỂN VÒNG KÍN

### 6.1. Nguyên Lý Điều Khiển Vòng Kín (Closed-Loop Control)

#### 6.1.1. Vòng Kín vs Vòng Hở

| Tiêu Chí | Vòng Hở | Vòng Kín |
|---|---|---|
| Phản hồi trạng thái | Không | Có (feedback liên tục) |
| Xử lý nhiễu | Không | Có (tự bù sai số) |
| Độ chính xác | Thấp | Cao |
| Ứng dụng UAV | Kiểm thử đơn giản | Bay thực tế, SITL |

#### 6.1.2. Kiến Trúc Vòng Kín Tổng Quát

```
r(t) ──→ [+] ──→ Controller ──→ u(t) ──→ Plant (UAV) ──→ y(t) ──→
            ↑  [−]                                              │
            └──────────────── Sensor ◄──────────────────────────┘
```

Các thành phần:
- **r(t)** — Reference/Setpoint: Quỹ đạo mong muốn tại thời điểm t
- **e(t) = r(t) − y(t)** — Sai số: Độ lệch giữa mong muốn và thực tế
- **Controller** — Bộ điều khiển: Tính tín hiệu điều khiển u từ sai số
- **Plant** — Đối tượng điều khiển: Mô hình động lực học Quadplane Condor
- **Sensor** — Cảm biến: Đo trạng thái thực tế (offline = đọc trực tiếp; SITL = telemetry MAVSDK)

### 6.2. Cấu Trúc Vòng Kín Trong Code

#### 6.2.1. Pseudocode 1 Bước Thời Gian

Mỗi bước dt = 0.005s (offline) hoặc 0.02s (SITL), hệ thống thực hiện:

```
1. Tạo setpoint r(t):
      sp = generate_setpoints(t, drone.state)

2. Đọc trạng thái hiện tại y(t):
      current_state = drone.state       [offline]
      current_state = read_telemetry()  [SITL]

3. Tính sai số e(t) = r(t) − y(t):
      (thực hiện bên trong controller.compute())

4. Bộ điều khiển tính u(t):
      thrust, torque = controller.compute(sp, yaw_sp, current_state, dt)

5. Plant cập nhật trạng thái:
      new_state = drone.step(thrust, torque, dt, tractor_thrust, delta_e, delta_r)

6. Ghi log:
      log(t, current_state, sp, thrust, torque)

7. t += dt → lặp lại
```

Điểm mấu chốt: Dòng `current_state = drone.state` chính là mắt xích vòng kín — trạng thái mới từ bước trước được phản hồi làm đầu vào cho bước hiện tại.

#### 6.2.2. Triển Khai Thực Tế: run_simulation()

Hàm chính trong `src/simulation/condor_closed_loop_sim.py`:

```python
def run_simulation(controller: ControllerBase,
                   scenario: str,
                   duration: float = 60.0) -> dict:
    """
    Chạy mô phỏng vòng kín offline.
    Returns: dictionary chứa toàn bộ log dữ liệu
    """
    params = QuadplaneParams()
    drone  = QuadplaneDynamics(params)
    controller.reset()
    drone.reset()

    dt = params.dt          # 0.005s (200 Hz)
    t  = 0.0
    state_dict = {}         # Trạng thái nội bộ của trajectory generator

    # Buffer log
    times, positions, setpoints = [], [], []
    thrusts, torques = [], []

    while t < duration:
        # 1. Tạo setpoint theo kịch bản
        sp = generate_setpoints(scenario, t, drone.state, state_dict)

        # 2. Bộ điều khiển tính lệnh (vòng kín: đọc drone.state)
        thrust, torque = controller.compute(
            position_setpoint = sp[:3],
            yaw_setpoint      = 0.0,
            current_state     = drone.state,   # ← feedback
            dt                = dt
        )

        # 3. Cập nhật mô hình vật lý
        drone.step(vtol_thrust  = thrust,
                   vtol_torque  = torque,
                   dt           = dt)

        # 4. Ghi log
        times.append(t)
        positions.append(drone.state[0:3].copy())
        setpoints.append(sp[:3].copy())
        thrusts.append(thrust)
        torques.append(torque.copy())

        t += dt

    return {
        "times":      np.array(times),
        "positions":  np.array(positions),
        "setpoints":  np.array(setpoints),
        "thrusts":    np.array(thrusts),
        "torques":    np.array(torques),
        "metrics":    compute_metrics(positions, setpoints, times),
    }
```

### 6.3. Phân Tích Chi Tiết Các Bộ Điều Khiển Trong Vòng Kín

#### 6.3.1. Cascade PID — 9 Bộ PID Lồng Nhau

Quy trình tính toán trong `cascade_controller.py` mỗi bước:

1. **Position → Acceleration**: PID_xyz tính gia tốc mong muốn
   ```python
   ax_des = pid_x.update(x_des - pos[0], dt)
   ay_des = pid_y.update(y_des - pos[1], dt)
   thrust = mass * (gravity - pid_z.update(z_des - pos[2], dt))
   ```

2. **Acceleration → Thrust + Angles**: Chuyển gia tốc NED sang lực đẩy và góc Roll/Pitch:
   ```python
   phi_des   = ( ax_des*sin(psi) - ay_des*cos(psi)) / g
   theta_des = -(ax_des*cos(psi) + ay_des*sin(psi)) / g
   ```

3. **Angle → Rate**: PID_attitude tính tốc độ góc mong muốn
   ```python
   p_des = pid_roll.update(phi_des   - phi,   dt)
   q_des = pid_pitch.update(theta_des - theta, dt)
   r_des = pid_yaw.update(wrap(psi_des - psi), dt)
   ```

4. **Rate → Torque**: PID_rate tính momen lực
   ```python
   tau_x = pid_p.update(p_des - p, dt)
   tau_y = pid_q.update(q_des - q, dt)
   tau_z = pid_r.update(r_des - r, dt)
   ```

Ưu điểm: Trực quan, dễ hiệu chỉnh từng vòng riêng biệt.
Nhược điểm: 27 tham số cần tinh chỉnh. Không tối ưu toán học.

#### 6.3.2. LQR — Tối Ưu Tuyến Tính

Đặc điểm trong code (`lqr_controller.py`):
- Ma trận A (12×12) và B (12×4) xây dựng từ tham số vật lý (mass, inertia, drag)
- Ma trận trọng số Q (trọng số trạng thái) và R (trọng số điều khiển) quyết định trade-off
- Giải Riccati 1 lần khi khởi tạo → sau đó chỉ nhân ma trận mỗi bước → **rất nhanh**

```python
def compute(self, position_setpoint, yaw_setpoint, current_state, dt):
    x_ref       = np.zeros(12)
    x_ref[0:3]  = position_setpoint[:3]
    x_ref[8]    = yaw_setpoint

    x_err       = current_state - x_ref
    x_err[8]    = wrap_to_pi(x_err[8])      # Yaw error wrap

    u           = -self.K @ x_err            # Optimal control law
    thrust      = self.mass*self.gravity + u[0]
    torque      = np.clip(u[1:4], -1.0, 1.0)
    return float(thrust), torque
```

Ưu điểm: Optimal cho hệ tuyến tính, không có biến nội bộ (memoryless).
Nhược điểm: Chỉ tối ưu gần hover (vùng tuyến tính hóa hợp lệ).

#### 6.3.3. MPC — Dự Đoán & Tối Ưu Tương Lai

Đặc điểm trong code (`mpc_controller.py`):
- Horizon N = 20 bước (lookahead 20 × dt giây)
- **Warm start**: Dùng nghiệm bước trước làm điểm khởi đầu → hội tụ nhanh hơn
- **Dual-Mode**: Khi `|x_error| < 0.05` (rất gần setpoint), chuyển sang LQR gain trực tiếp để tránh nhiễu solver gần điểm cân bằng
- Giải QP bằng L-BFGS-B với gradient giải tích (backward pass)

```python
# Dual-Mode: bypass optimizer gần setpoint
if np.linalg.norm(x_err) < 0.05:
    u_lqr  = -self._K_init @ x_err
    thrust = self.mass * self.gravity + u_lqr[0]
    return float(thrust), u_lqr[1:4]

# Receding horizon optimization
result = minimize(self._cost_function, u0,
                  args=(x_err,),
                  method="L-BFGS-B",
                  jac=self._cost_gradient,
                  bounds=bounds)
```

Ưu điểm: Xử lý ràng buộc (thrust/torque limits), dự đoán tương lai.
Nhược điểm: Tính toán nặng nhất (~50× chậm hơn Cascade PID).

#### 6.3.4. Geometric SE(3) — Ổn Định Toàn Cục

Quy trình tính toán chi tiết trong `geometric_controller.py`:

**Bước 1 — Tính lực mong muốn:**
```python
e_p    = p - p_ref                             # Position error (NED)
e_v    = v - v_ref                             # Velocity error
a_des  = a_ref - Kp @ e_p - Kv @ e_v         # Desired acceleration
F_des  = mass * a_des - mass * g * e3_ned      # Desired force (NED)
```

**Bước 2 — Suy ra thrust:**
```python
z_B    = R[:, 2]                               # Body Z-axis (NED)
thrust = -np.dot(F_des, z_B)                  # Project F_des onto -z_B
thrust = np.clip(thrust, 0, 2.5*mass*gravity)
```

**Bước 3 — Xây dựng ma trận quay mong muốn R_d:**
```python
z_d    = -F_des / np.linalg.norm(F_des)        # Desired body Z (up)
x_c    = [cos(psi_d), sin(psi_d), 0]           # Desired heading
y_d    = cross(z_d, x_c) / |cross(z_d, x_c)|  # Desired body Y
x_d    = cross(y_d, z_d)                        # Desired body X
R_d    = column_stack([x_d, y_d, z_d])
```

**Bước 4 — Sai số attitude trên SO(3):**
```python
R_err  = R_d.T @ R                             # Rotation error matrix
angle  = arccos(clip((trace(R_err)-1)/2, -1,1))
e_R    = (angle / (2*sin(angle))) * vee(R_err - R_err.T)
```

**Bước 5 — Tính torque:**
```python
gyro_term = cross(omega, J @ omega)             # Con quay hồi chuyển
torque    = -KR @ e_R - Kw @ omega + gyro_term
torque    = clip(torque, -15.0, 15.0)
```

Ưu điểm: Ổn định toàn cục (global asymptotic stability), hỗ trợ feedforward, không gimbal lock.
Nhược điểm: Cần quỹ đạo trơn (smooth trajectory) để feedforward hiệu quả.

#### 6.3.5. Bảng So Sánh Tổng Hợp

| Tiêu Chí | Cascade PID | LQR | MPC | Geometric SE(3) |
|---|---|---|---|---|
| RMSE hover | Thấp | Thấp | Thấp | **Thấp nhất** |
| Settling time | ~1.0s | ~0.8s | ~0.7s | **~0.5s** |
| Overshoot | ~8% | ~5% | ~4% | **~2%** |
| Tốc độ tính toán | **Nhanh nhất** | Nhanh | Chậm nhất | Nhanh |
| Số tham số | 27 | 7 | 10 | 4 |
| Ràng buộc u | Không | Không | **Có** | Không |
| Ổn định toàn cục | Không | Cục bộ | Cục bộ | **Có** |
| Feedforward | Không | Không | Không | **Có** |

### 6.4. Metrics Đánh Giá Hiệu Năng

#### 6.4.1. RMSE — Root Mean Square Error

Đo sai số trung bình giữa vị trí thực và setpoint trong giai đoạn hover ổn định:

```
RMSE = √(Σ‖pos(t) − ref(t)‖² / N)
```

Ý nghĩa: RMSE = 0.01m → drone chênh trung bình 1cm so với điểm đích.
Ngưỡng tốt: RMSE < 0.05m cho ứng dụng bay thực.

#### 6.4.2. Settling Time — Thời Gian Ổn Định

Thời gian từ khi bắt đầu hover đến khi altitude nằm vĩnh viễn trong dải 5% setpoint:

```
t_s = min{t : |h(t') − h_ref| ≤ 0.05 × h_ref, ∀t' ≥ t}
```

Ý nghĩa: Settling time = 0.5s → drone đạt 95% mục tiêu sau nửa giây.
Ngưỡng tốt: Settling time < 2s.

#### 6.4.3. Overshoot — Vượt Lố

Phần trăm altitude vượt quá setpoint so với giá trị mong muốn (đo trong pha cất cánh):

```
Overshoot = (peak − ref) / ref × 100%
```

Ý nghĩa: Overshoot = 5% → drone bay quá 25cm khi target 5m rồi mới ổn định.
Ngưỡng tốt: Overshoot < 10%.

#### 6.4.4. Control Effort — Năng Lượng Điều Khiển

Tổng bình phương độ lệch thrust so với hover thrust — đo "mức gắng sức" của controller:

```
Effort = Σ(T(t) − T_hover)² / N
```

Ý nghĩa: Effort thấp = drone bay "nhẹ nhàng" hơn, tiết kiệm pin.
Controller tốt có effort thấp nhưng vẫn bám quỹ đạo chính xác.

### 6.5. Hai Chế Độ Vòng Kín: Offline vs SITL

#### 6.5.1. So Sánh Chi Tiết

| Khía Cạnh | Offline Sim | SITL (PX4 + Gazebo) |
|---|---|---|
| Đọc trạng thái | `drone.state` (Python array) | MAVSDK telemetry (async) |
| Cập nhật trạng thái | `drone.step()` (Python RK4) | Gazebo physics + PX4 EKF2 |
| Timestep | 0.005s (200 Hz) | 0.02s (50 Hz Offboard) |
| Rate control | Python tự làm (torque → thrust) | PX4 firmware nội bộ |
| Nhiễu cảm biến | Không có | Có (EKF2 + Gazebo noise) |
| Tốc độ mô phỏng | Nhanh hơn thực tế nhiều | Realtime (RTF ≈ 1.0) |

#### 6.5.2. Sự Khác Biệt Quan Trọng: compute() vs compute_attitude_thrust()

Trong SITL, PX4 firmware đã có sẵn Rate PID nội bộ (module `mc_rate_control`). Script Python chỉ cần gửi **attitude setpoint** (góc Roll, Pitch, Yaw + thrust %), PX4 tự tính torque và motor mixing:

```python
# Offline: controller trả thrust + torque trực tiếp
thrust, torque = controller.compute(sp, yaw_sp, state, dt)
drone.step(thrust, torque)   # Python tự tích phân

# SITL: controller trả attitude + thrust_norm
roll, pitch, yaw, thrust_pct = controller.compute_attitude_thrust(sp, yaw_sp, state)
await drone.offboard.set_attitude(
    Attitude(roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw,
             thrust_value=thrust_pct)
)  # PX4 xử lý rate control + motor mixing
```

Tại sao SITL cần Integral Z mà Offline không cần?
- **Offline**: Mô hình toán lý tưởng, không có sai lệch → PD đủ để bám chính xác
- **SITL**: PX4 EKF2 + Gazebo có nhiễu cảm biến, sai lệch mô hình → Cần Integral để bù sai số tĩnh trên trục Z

```python
# geometric_controller.py — compute_attitude_thrust() cho SITL
self.integral_z += e_p[2] * control_dt         # Tích phân sai số Z
self.integral_z  = np.clip(self.integral_z, -3.0/ki_z, 3.0/ki_z)  # Anti-windup
F_des[2] -= self.ki_z * self.integral_z        # Bù lực đẩy từ tích phân
```

### 6.6. Các Quỹ Đạo Bay Trong Mô Phỏng

#### 6.6.1. Hover — Kiểm Tra Ổn Định Cơ Bản

```python
def generate_hover_setpoints(t: float) -> np.ndarray:
    if t < 3.0:                              # Cất cánh 3 giây
        return np.array([0, 0, -5.0 * min(t/3.0, 1.0)])
    elif t < 13.0:                           # Hover 10 giây
        return np.array([0, 0, -5.0])
    elif t < 18.0:                           # Hạ cánh 5 giây
        return np.array([0, 0, -5.0 * (1 - (t-13)/5.0)])
    else:
        return np.array([0, 0, 0.0])
```

Đo lường: Settling time, overshoot, RMSE tại vùng hover.

#### 6.6.2. Square — Kiểm Tra Chuyển Hướng

Bay hình vuông 5×5m ở 5m cao, dùng Minimum Jerk interpolation giữa 4 góc:

```python
waypoints = [
    [0.0, 0.0, -5.0], [5.0, 0.0, -5.0],
    [5.0, 5.0, -5.0], [0.0, 5.0, -5.0],
    [0.0, 0.0, -5.0], [0.0, 0.0,  0.0],  # Hạ cánh
]
```

Đo lường: Quỹ đạo XY, overshoot tại góc rẽ.

#### 6.6.3. Lemniscate — Kiểm Tra Bám Quỹ Đạo Liên Tục

```
x(t) = 10·sin(0.3t),  y(t) = 10·sin(0.3t)·cos(0.3t)
```

Kèm feedforward velocity + acceleration cho Geometric controller. Envelope filter `(1 − e⁻ᵗ/³)` để tránh sốc gia tốc ban đầu.

#### 6.6.4. Giant Lemniscate — SITL Gazebo (80m × 40m)

Sử dụng trong `sitl_condor_geometric.py` với biên độ A = 40m:

```
N(t) = 40·sin(0.12t),  E(t) = 40·sin(0.12t)·cos(0.12t)
```

Tốc độ tối đa ≈ 4.8 m/s, bay ở −5m altitude, thời gian 60 giây.

### 6.7. Ghi Log & Phân Tích Sau Bay

#### 6.7.1. Offline — NumPy Arrays + Matplotlib

Hàm `run_simulation()` trả về dictionary chứa toàn bộ log:

```python
results = {
    "times":      np.array([t0, t1, ...]),         # Thời gian [s]
    "positions":  np.array([[x,y,z], ...]),         # Vị trí NED [m]
    "setpoints":  np.array([[x,y,z], ...]),         # Điểm đặt [m]
    "velocities": np.array([[vx,vy,vz], ...]),      # Vận tốc [m/s]
    "euler":      np.array([[phi,th,psi], ...]),    # Góc Euler [rad]
    "thrusts":    np.array([T0, T1, ...]),           # Thrust [N]
    "torques":    np.array([[tx,ty,tz], ...]),       # Momen [N·m]
    "phases":     [FlightPhase.VTOL_HOVER, ...],    # Giai đoạn bay
    "metrics": {
        "rmse": 0.023,                              # [m]
        "settling_time": 0.48,                      # [s]
        "overshoot_pct": 3.2,                       # [%]
        "control_effort": 12.4,                     # [N²]
        "compute_time": 0.043,                      # [s] wall clock
    }
}
```

#### 6.7.2. SITL — CSV File

File `sitl_condor_geometric.py` ghi ra CSV với các cột:

```
time, x, y, z, vx, vy, vz, roll_deg, pitch_deg, yaw_deg,
p, q, r, thrust, tx, ty, tz, phase, sp_x, sp_y, sp_z
```

Sau bay, script tự động tính và in metrics:
```
=== Flight Metrics ===
Duration:     60.2 s
RMSE 3D:      0.031 m
RMSE Altitude:0.012 m
Max Error:    0.187 m
Settling:     0.52 s
```

Phân tích sau bay:
```bash
python3 scripts/plot_telemetry.py output/telemetry_*.csv
```

### 6.8. Tổng Kết: Quy Trình End-to-End

1. **Chọn quỹ đạo**: Thiết lập bài toán bằng cách chọn dạng quỹ đạo bay mong muốn (hover, square, lemniscate, figure-8 hoặc VTOL mission)

2. **Chạy offline sim**: Thực thi script mô phỏng vòng lặp kín `condor_closed_loop_sim.py` để kiểm tra thuật toán mà chưa cần phần cứng hay Gazebo

3. **So sánh controllers**: Đánh giá và đối chiếu các bộ điều khiển thông qua các chỉ số metrics và biểu đồ trực quan

4. **Chọn controller tốt nhất**: Khối quyết định để lựa chọn bộ điều khiển tối ưu nhất tiếp tục phát triển

5. **Tinh chỉnh gains**: Hiệu chỉnh các thông số bộ điều khiển (Kp, Kv, KR, Kw) dựa trên kết quả offline

6. **Chạy SITL**: Triển khai bay mô phỏng phần cứng trong vòng lặp thông qua `sitl_condor_geometric.py` hoặc `sitl_condor_mission.py` với Gazebo 3D

7. **Phân tích CSV**: Đọc và xử lý file dữ liệu telemetry (`plot_telemetry.py`) xuất ra từ quá trình bay

8. **Bám quỹ đạo**: Khởi điều kiện kiểm tra xem hiệu năng bay đã đạt yêu cầu thực tế hay chưa

9. **Chuyển HIL**: Nâng cấp và chạy thử nghiệm phần cứng trong vòng lặp với vi điều khiển Pixhawk thật

Toàn bộ quy trình dựa trên nguyên lý vòng kín: mỗi bước, controller đọc state → tính sai số → xuất lệnh → mô hình cập nhật → state mới phản hồi. Vòng lặp này chạy ở **200 Hz** (offline) hoặc **50 Hz** (SITL), liên tục cho đến khi nhiệm vụ hoàn thành.

---

*Tài liệu này được cập nhật theo codebase Quadplane Condor tại thư mục `px4_project/`. Mọi giá trị tham số phản ánh đúng `src/uav_model/parameters.yaml` và `models/quadplane_condor/model.sdf`.*
