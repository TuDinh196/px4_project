# Quadplane Condor — Hybrid VTOL Flight Control Platform

Nền tảng điều khiển bay cho UAV **Quadplane Condor** — máy bay lai VTOL (4+1 layout): 4 động cơ nâng thẳng đứng + 1 động cơ kéo mũi cánh cứng, sải cánh 2.4m, MTOW 7.8kg.

Dự án bao gồm mô hình động lực học 6-DOF, bộ điều khiển đa thuật toán (Cascade PID / LQR / MPC / Geometric SE(3) / VTOL Hybrid), mô phỏng vòng kín offline, tích hợp PX4 SITL + Gazebo và web dashboard giám sát thời gian thực.

📄 **Tài liệu kỹ thuật đầy đủ:** [`docs/UAV_docs.md`](docs/UAV_docs.md)

---

## Yêu Cầu Hệ Thống

| Thành phần | Phiên bản |
|---|---|
| OS | Ubuntu 22.04 / WSL2 (Windows) |
| Python | >= 3.10 |
| PX4-Autopilot | v1.14+ |
| Gazebo | Harmonic (gz-harmonic) |
| QGroundControl | v4.x (Windows/Linux) |

---

## Cài Đặt Nhanh

```bash
# 1. Clone & cài đặt dependencies Python
git clone <repo_url> px4_project
cd px4_project
python3 -m venv venv_linux
source venv_linux/bin/activate
pip install -r requirements.txt

# 2. Cài đặt model Condor vào PX4-Autopilot
./manage.sh setup
```

> **Lưu ý:** Biến môi trường `PX4_DIR` mặc định trỏ đến `~/PX4-Autopilot`.
> Nếu PX4 cài ở thư mục khác: `export PX4_DIR=/path/to/PX4-Autopilot`

---

## Hướng Dẫn Chạy
Di chuyển vào thư mục dự án, ví dụ 
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


### Chế Độ 1 — Mô Phỏng Offline (Không cần PX4 / Gazebo)

Chạy nhanh nhất để kiểm tra thuật toán điều khiển:

```bash
# So sánh 4 bộ điều khiển (Cascade PID, LQR, MPC, Geometric SE(3))
python3 src/simulation/condor_closed_loop_sim.py

# Mô phỏng nhiệm vụ bay đầy đủ 5 giai đoạn VTOL
python3 src/simulation/condor_mission_sim.py
```

Kết quả: Đồ thị so sánh hiệu năng tự động lưu vào `plots/`, metrics in ra terminal.

---

### Chế Độ 2 — SITL (PX4 + Gazebo 3D)

```bash
# Khởi động toàn bộ hệ thống (PX4 SITL + Gazebo + QGC + Web Dashboard)
./manage.sh all

# Hoặc chỉ SITL + Gazebo (world mặc định)
./manage.sh sim

# SITL với thế giới Figure-8 (cổng bay 3D + helipad)
./manage.sh sim figure8
```

Sau khi PX4 SITL khởi động xong (`Ready for takeoff`), mở terminal mới:

```bash
# Chạy nhiệm vụ tự động 5 giai đoạn VTOL (MAVSDK Offboard)
./manage.sh mission

# Chạy bộ điều khiển Geometric SE(3) theo quỹ đạo Lemniscate
./manage.sh geometric
```

---

### Chế Độ 3 — Web Dashboard

```bash
./manage.sh dashboard
```

Mở trình duyệt tại: **http://127.0.0.1:8080**

Dashboard cung cấp: bản đồ thời gian thực, đồ thị độ cao, quỹ đạo 3D, điều khiển kịch bản bay.

---

### Các Lệnh Phụ Trợ

```bash
./manage.sh test       # Chạy unit test (pytest) + kiểm tra code (flake8)
./manage.sh clean      # Xóa logs, __pycache__, test cache
./manage.sh stop       # Dừng tất cả tiến trình nền
./manage.sh package    # Đóng gói dự án thành .tar.gz
./manage.sh help       # Hiển thị tất cả lệnh
```

---

## Cấu Trúc Thư Mục

```
px4_project/
├── src/
│   ├── uav_model/          # Mô hình động lực học 6-DOF Quadplane Condor
│   ├── controllers/        # 6 bộ điều khiển (PID, Cascade, LQR, MPC, Geometric, VTOL)
│   ├── simulation/         # Mô phỏng vòng kín offline
│   ├── scenarios/          # Kịch bản bay (hover, square, circle, figure-8)
│   ├── px4_integration/    # MAVSDK bridge + SITL scripts
│   └── dashboard/          # Web dashboard (WebSocket + Plotly)
├── models/                 # Model Gazebo SDF, meshes, worlds, airframe PX4
├── scripts/                # Tiện ích (generate world, plot telemetry, convert docs)
├── tests/                  # Unit tests
├── docs/                   # Tài liệu kỹ thuật
├── manage.sh               # Script quản lý tổng thể
└── requirements.txt        # Python dependencies
```

---

## Giấy Phép

Dự án phục vụ nghiên cứu và phát triển UAV. Xem chi tiết kỹ thuật tại [`docs/UAV_docs.md`](docs/UAV_docs.md).

## Xử Lý Sự Cố Thường Gặp (Troubleshooting)

**1. Lỗi xuống dòng (CRLF) khi chạy script trên Linux/WSL**
Nếu bạn nhận được lỗi `bash: ./manage.sh: /bin/bash^M: bad interpreter` hoặc `\r: command not found`, nguyên nhân là do file bị lưu với định dạng xuống dòng của Windows (CRLF). Cách sửa:
```bash
sed -i 's/\r$//' manage.sh
# Hoặc dùng dos2unix nếu đã cài đặt:
dos2unix manage.sh
```

**2. Xóa Cache (Build cache & Python cache)**
Nếu PX4 SITL biên dịch lỗi hoặc Python chạy ra kết quả cũ, bạn cần xóa cache:
```bash
# Xóa cache Python, logs và dữ liệu tạm của dự án:
./manage.sh clean

# Xóa cache biên dịch của PX4 (thực hiện trong thư mục PX4):
cd ~/PX4-Autopilot
make clean
# Xóa triệt để thư mục build nếu lỗi nặng:
rm -rf build/
```
