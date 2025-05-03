import socket
import struct
import threading
import time
import cv2
import numpy as np

ESP32_IP = '192.168.1.23'
CAMERA_PORT = 8080
CONTROL_PORT = 3333

def connect_socket(ip, port):
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((ip, port))
            print(f"[+] Connected to {ip}:{port}")
            return s
        except Exception as e:
            print(f"[!] Error connecting to {ip}:{port}: {e}. Retrying...")
            time.sleep(2)

def camera_loop(cam_sock):
    try:
        while True:
            # Receive 4-byte frame length
            len_bytes = cam_sock.recv(4)
            if len(len_bytes) < 4:
                raise ConnectionError("Incomplete frame length header")

            frame_len = struct.unpack('<I', len_bytes)[0]

            # Receive frame data
            frame_data = b''
            while len(frame_data) < frame_len:
                chunk = cam_sock.recv(frame_len - len(frame_data))
                if not chunk:
                    raise ConnectionError("Camera socket closed")
                frame_data += chunk

            # Decode and show
            np_arr = np.frombuffer(frame_data, dtype=np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                cv2.imshow("ESP32 Camera", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except Exception as e:
        print(f"[CAMERA] Error: {e}")
    finally:
        cam_sock.close()
        cv2.destroyAllWindows()

def control_recv_loop(ctrl_sock):
    try:
        while True:
            data = ctrl_sock.recv(128)
            if not data:
                break
            print(f"[SENSOR] {data.decode().strip()}")
    except Exception as e:
        print(f"[CTRL-RECV] Error: {e}")
    finally:
        ctrl_sock.close()

def control_send_loop(ctrl_sock):
    try:
        while True:
            cmd = input("Command to Arduino (e.g. SERVO 90): ")
            if cmd:
                ctrl_sock.sendall((cmd + "\n").encode())
    except Exception as e:
        print(f"[CTRL-SEND] Error: {e}")
    finally:
        ctrl_sock.close()

def main():
    time.sleep(8)
    cam_sock = connect_socket(ESP32_IP, CAMERA_PORT)
    ctrl_sock = connect_socket(ESP32_IP, CONTROL_PORT)

    # Start threads
    threading.Thread(target=camera_loop, args=(cam_sock,), daemon=True).start()
    threading.Thread(target=control_recv_loop, args=(ctrl_sock,), daemon=True).start()
    control_send_loop(ctrl_sock)  # Blocking input loop

if __name__ == "__main__":
    main()
