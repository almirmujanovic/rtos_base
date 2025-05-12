import websocket
import time
import threading
import socket

URL = "ws://192.168.1.15:81/ws"

def on_message(ws, message):
    print(f"📨 {message}")

def on_error(ws, error):
    print(f"❌ WebSocket error: {error}")

def on_close(ws, close_status_code, close_msg):
    print(f"🔌 WebSocket closed: {close_status_code} {close_msg}")

def on_open(ws):
    print("✅ WebSocket connection established")

def create_socket():
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp(
        URL,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )
    
    # Disable Nagle’s algorithm for low latency
    def set_tcp_nodelay(sock):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    wst = threading.Thread(target=lambda: ws.run_forever(
        ping_interval=20,
        ping_timeout=10,
        sslopt={"cert_reqs": 0},
        sockopt=[(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)],
        dispatcher=None,
        skip_utf8_validation=True
    ))
    wst.daemon = True
    wst.start()
    return ws

def auto_reconnect():
    backoff = 2
    ws = None
    while True:
        try:
            ws = create_socket()
            while True:
                time.sleep(1)
                if not ws.sock or not ws.sock.connected:
                    raise Exception("Lost connection")
            backoff = 2  # Reset backoff on success
        except Exception as e:
            print(f"🔁 Reconnecting in {backoff} seconds... ({e})")
            time.sleep(backoff)
            backoff = min(backoff * 2, 60)

if __name__ == "__main__":
    auto_reconnect()
