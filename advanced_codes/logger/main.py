import socket
from zeroconf import Zeroconf

UDP_IP = "0.0.0.0"
UDP_PORT = 4210

# Advertise our hostname py.local
from zeroconf import ServiceInfo

desc = {'service': 'Python Logger'}
info = ServiceInfo(
    "_udp.local.",
    "py._udp.local.",
    addresses=[socket.inet_aton(socket.gethostbyname(socket.gethostname()))],
    port=UDP_PORT,
    properties=desc,
    server="py.local."
)

zconf = Zeroconf()
zconf.register_service(info)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

with open("log.txt", "a") as f:
    print(f"Listening on py.local:{UDP_PORT}")
    while True:
        data, addr = sock.recvfrom(1024)
        line = data.decode(errors="ignore").strip()
        print(line)
        f.write(line + "\n")
        f.flush()
