import socket
import struct
import time
import cv2
import numpy as np

ESP32_IP = '192.168.1.56'
ESP32_PORT = 3333
RETRY_DELAY = 3
TIMEOUT = 5

def connect_to_esp32():
    while True:
        try:
            print(f"[🔌] Connecting to {ESP32_IP}:{ESP32_PORT}...")
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(TIMEOUT)
            s.connect((ESP32_IP, ESP32_PORT))
            print(f"[+] Connected to ESP32 at {ESP32_IP}:{ESP32_PORT}")
            return s
        except Exception as e:
            print(f"[⚠️] Connection error: {e}. Retrying in {RETRY_DELAY} seconds...")
            time.sleep(RETRY_DELAY)

def receive_frame(sock):
    len_bytes = sock.recv(4)
    if len(len_bytes) < 4:
        raise ConnectionError("Incomplete length header")

    frame_len = struct.unpack('<I', len_bytes)[0]
    frame_data = b''
    while len(frame_data) < frame_len:
        chunk = sock.recv(frame_len - len(frame_data))
        if not chunk:
            raise ConnectionError("Connection closed by ESP32")
        frame_data += chunk

    return frame_data

def main():
    sock = connect_to_esp32()
    try:
        while True:
            frame_data = receive_frame(sock)

            # Decode JPEG frame
            np_arr = np.frombuffer(frame_data, dtype=np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is not None:
                cv2.imshow("ESP32 Camera", img)

            # Break on key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"[⚠️] Disconnected or error: {e}")
    finally:
        sock.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
