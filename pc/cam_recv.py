import socket
import cv2
import numpy as np

UDP_IP = "0.0.0.0"
UDP_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(1)

frame_buf = bytearray()
syncing = False

while True:
    try:
        packet, _ = sock.recvfrom(1500)
        # Look for JPEG start marker
        if b'\xff\xd8' in packet:
            syncing = True
            frame_buf = bytearray()
            # Start marker may not be at start of packet
            start = packet.find(b'\xff\xd8')
            frame_buf.extend(packet[start:])
        elif syncing:
            frame_buf.extend(packet)

        if syncing and b'\xff\xd9' in packet:
            # End marker may not be at end of packet
            end = packet.find(b'\xff\xd9') + 2
            frame = cv2.imdecode(np.frombuffer(frame_buf[:end], np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow("Stream", frame)
            frame_buf = bytearray()
            syncing = False

        if cv2.waitKey(1) == 27:
            break

    except socket.timeout:
        frame_buf = bytearray()
        syncing = False