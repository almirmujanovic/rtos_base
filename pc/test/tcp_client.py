import socket
import struct
import threading
import time
import cv2
import numpy as np

ESP32_IP = '192.168.1.28'
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

def receive_exact(sock, n):
    """Fast receive of exactly n bytes"""
    buf = bytearray(n)
    view = memoryview(buf)
    i = 0
    while i < n:
        nbytes = sock.recv_into(view[i:], n - i)
        if nbytes == 0:
            raise ConnectionError("Socket closed")
        i += nbytes
    return buf

def camera_loop(cam_sock):
    try:
        while True:
            t0 = time.time()

            # Receive 4-byte frame length
            len_bytes = receive_exact(cam_sock, 4)
            frame_len = struct.unpack('<I', len_bytes)[0]

            # Receive frame
            frame_data = receive_exact(cam_sock, frame_len)

            # Decode JPEG
            np_arr = np.frombuffer(frame_data, dtype=np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                cv2.imshow("ESP32 Camera", img)
            else:
                print("[!] Failed to decode JPEG")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            t1 = time.time()
            print(f"[📷] Frame: {frame_len} bytes in {(t1 - t0)*1000:.1f} ms")

    except Exception as e:
        print(f"[CAMERA] Error: {e}")
    finally:
        cam_sock.close()
        cv2.destroyAllWindows()

def control_recv_loop(ctrl_sock):
    try:
        while True:
            data = ctrl_sock.recv(4096)
            if not data:
                break
            print(f"[SENSOR] {data.decode(errors='ignore').strip()}")
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
    cam_sock = connect_socket(ESP32_IP, CAMERA_PORT)
    ctrl_sock = connect_socket(ESP32_IP, CONTROL_PORT)

    threading.Thread(target=camera_loop, args=(cam_sock,), daemon=True).start()
    threading.Thread(target=control_recv_loop, args=(ctrl_sock,), daemon=True).start()
    control_send_loop(ctrl_sock)  # Blocking

if __name__ == "__main__":
    main()
