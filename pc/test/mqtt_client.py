import paho.mqtt.client as mqtt

BROKER_IP = "192.168.1.87"  # Your PC IP
TOPIC_DATA = "/robot/sensors"
TOPIC_COMMAND = "/robot/commands"

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe(TOPIC_DATA)

def on_message(client, userdata, msg):
    print(f"📨 Topic: {msg.topic} | Data: {msg.payload.decode()}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_IP, 1883)
client.loop_start()

# To send a command:
while True:
    cmd = input("Send to Arduino: ")
    if cmd.strip():
        client.publish(TOPIC_COMMAND, cmd.strip())
