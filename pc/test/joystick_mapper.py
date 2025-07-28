import pygame
import paho.mqtt.client as mqtt
import time

# MQTT Configuration
BROKER = "192.168.137.1"
TOPIC_COMMAND = "/robot/commands"

# Smoothing configuration
MAX_ACCEL = 15  # Max speed change per cycle

# MQTT Callbacks
def on_connect(client, userdata, flags, rc):
    print("✅ MQTT connected")
    client.subscribe("/robot/sensors")

def on_message(client, userdata, msg):
    try:
        print(f"📡 ESP32 says: {msg.payload.decode('utf-8')}")
    except UnicodeDecodeError:
        print(f"📡 ESP32 sent non-UTF8 data: {msg.payload!r}")

# MQTT Setup
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, 1883)
client.loop_start()

# Initialize Joystick
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ No joystick found")
    exit()

js = pygame.joystick.Joystick(0)
js.init()

last_speed = 0
last_angle = 90

# Main loop
while True:
    pygame.event.pump()

    l2 = js.get_axis(4)  # Left trigger: forward
    r2 = js.get_axis(5)  # Right trigger: reverse
    rx = js.get_axis(2)  # Right stick X-axis: steering

    # Calculate signed speed
    if l2 > -1.0:
        target_speed = int(((l2 + 1) / 2) * 255)  # 0 to 255 (forward)
    elif r2 > -1.0:
        target_speed = -int(((r2 + 1) / 2) * 255)  # 0 to -255 (reverse)
    else:
        target_speed = 0

    # Apply acceleration ramp
    if target_speed > last_speed:
        speed = min(target_speed, last_speed + MAX_ACCEL)
    elif target_speed < last_speed:
        speed = max(target_speed, last_speed - MAX_ACCEL)
    else:
        speed = target_speed

    # Map angle (30 to 150)
    angle = int((rx + 1) * 60 + 30)

    # Publish speed if changed
    if abs(speed - last_speed) >= 1:
        client.publish(TOPIC_COMMAND, f"SPEED,{speed}")
        print(f"📤 SPEED → {speed}")
        last_speed = speed

    # Publish servo angle if changed
    if abs(angle - last_angle) >= 2:
        client.publish(TOPIC_COMMAND, f"SERVO,{angle}")
        print(f"📤 SERVO → {angle}")
        last_angle = angle

    time.sleep(0.033)
