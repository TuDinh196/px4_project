import os
import socket
import select

px4_port = 14550

win_ip = os.popen("grep nameserver /etc/resolv.conf | awk '{print $2}'").read().strip()
if not win_ip:
    win_ip = "172.18.128.1"  # Fallback

win_port = 14550
px4_target = ("127.0.0.1", 18570)

s_px4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_px4.bind(("0.0.0.0", px4_port))

s_win = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_win.bind(("0.0.0.0", 0))

print(f"UDP Router running. Forwarding PX4 -> {win_ip}:{win_port}")

while True:
    r, _, _ = select.select([s_px4, s_win], [], [])
    for s in r:
        if s is s_px4:
            data, _ = s_px4.recvfrom(4096)
            s_win.sendto(data, (win_ip, win_port))
        elif s is s_win:
            data, _ = s_win.recvfrom(4096)
            s_px4.sendto(data, px4_target)
