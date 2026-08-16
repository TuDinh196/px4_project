import socket
import select
import sys

TCP_PORT = 5760
UDP_IP = "127.0.0.1"
UDP_PORT_SEND = 18570
UDP_PORT_RECV = 14550

tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
tcp_server.bind(("0.0.0.0", TCP_PORT))
tcp_server.listen(1)

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# We must bind to the port PX4 is sending to!
try:
    udp_sock.bind(("0.0.0.0", UDP_PORT_RECV))
except Exception as e:
    print(f"Error binding UDP: {e}")
    sys.exit(1)

print(f"TCP Bridge Listening on port {TCP_PORT}...")

while True:
    conn, addr = tcp_server.accept()
    print(f"TCP connected from {addr}!")
    try:
        while True:
            r, _, _ = select.select([conn, udp_sock], [], [])
            for s in r:
                if s is conn:
                    data = conn.recv(4096)
                    if not data:
                        raise Exception("TCP disconnected")
                    udp_sock.sendto(data, (UDP_IP, UDP_PORT_SEND))
                elif s is udp_sock:
                    data, _ = udp_sock.recvfrom(4096)
                    conn.sendall(data)
    except Exception as e:
        print(f"Connection lost: {e}")
        conn.close()
