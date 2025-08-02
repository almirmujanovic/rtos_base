import socket
import struct
import cv2
import numpy as np
import threading
import queue
import time

TCP_IP = "192.168.1.114"  # ESP32's IP address
TCP_PORT = 5001
MAX_QUEUE = 5  # Drop oldest frames if queue is full

frame_queue = queue.Queue(maxsize=MAX_QUEUE)
stop_event = threading.Event()

def receiver_thread(sock, frame_queue, stop_event):
    while not stop_event.is_set():
        try:
            # Read 4 bytes for frame length
            data = b''
            while len(data) < 4:
                more = sock.recv(4 - len(data))
                if not more:
                    print("Connection closed")
                    stop_event.set()
                    return
                data += more
            frame_len = struct.unpack('!I', data)[0]

            # Read JPEG data
            buf = b''
            while len(buf) < frame_len:
                more = sock.recv(frame_len - len(buf))
                if not more:
                    print("Connection closed")
                    stop_event.set()
                    return
                buf += more

            # Put frame in queue, drop oldest if full
            try:
                frame_queue.put_nowait(buf)
            except queue.Full:
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
                frame_queue.put_nowait(buf)
        except Exception as e:
            print(f"Receiver exception: {e}")
            stop_event.set()
            return

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((TCP_IP, TCP_PORT))
    print(f"Connected to {TCP_IP}:{TCP_PORT}")

    recv_thread = threading.Thread(target=receiver_thread, args=(sock, frame_queue, stop_event), daemon=True)
    recv_thread.start()

    last_time = time.time()
    frame_count = 0

    while not stop_event.is_set():
        try:
            buf = frame_queue.get(timeout=1)
            frame = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow("TCP Stream", frame)
                frame_count += 1
                now = time.time()
                print(f"Frame {frame_count} displayed ({now - last_time:.2f}s since last)")
                last_time = now
            else:
                print("Failed to decode JPEG")
            if cv2.waitKey(1) == 27:
                print("ESC pressed, exiting.")
                stop_event.set()
                break
        except queue.Empty:
            continue

    sock.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()