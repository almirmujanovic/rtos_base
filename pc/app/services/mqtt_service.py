from __future__ import annotations
import time
import threading
from PySide6.QtCore import QObject, Signal
import paho.mqtt.client as mqtt
from config import MqttConfig

class MqttService(QObject):
    sensorsUpdated = Signal(dict)            # {'vl53':int, 'hcsr04_1':int, 'hcsr04_2':int, 'hcsr04_3':int}
    statusUpdated = Signal(str)              # Arduino/ESP32 status lines (unchanged)
    connectedChanged = Signal(bool)          # MQTT connection state (unchanged)
    emergencyStopChanged = Signal(bool)      # Emergency/blocked/alert heuristic 
    safetyUpdated = Signal(dict)             # {'front_cm':int|255,'rear_cm':int|255,'front_soft':bool,'front_hard':bool,'rear_soft':bool,'rear_hard':bool,'force':bool,'estop':bool,'blocked_front':bool,'blocked_rear':bool}

    def __init__(self, cfg: MqttConfig):
        super().__init__()
        self.cfg = cfg

        self.cli = mqtt.Client()
        self.cli.on_connect = self._on_connect
        self.cli.on_disconnect = self._on_disconnect
        self.cli.on_message = self._on_message

        self.sensor_data = {"vl53": 0, "hcsr04_1": 0, "hcsr04_2": 0, "hcsr04_3": 0}
        self.last_command_time = 0
        self.last_sensor_time = 0
        self.arduino_status = "Unknown"

        self._last_speed = 0
        self._last_angle = 90
        self._connected = False
        self._running = False
        self._connection_thread = None

        self._message_count = 0
        self._last_timing_log = 0

        # Optional: last parsed safety snapshot
        self._last_safety = None

    def start(self):
        self._running = True
        self._connection_thread = threading.Thread(target=self._connect_mqtt, daemon=True)
        self._connection_thread.start()

    def _connect_mqtt(self):
        try:
            print(f"🔌 Connecting to MQTT broker at {self.cfg.host}:{self.cfg.port}")
            self.cli.connect(self.cfg.host, self.cfg.port, 60)
            self.cli.loop_start()
            print(" MQTT connection initiated")
            return True
        except Exception as e:
            print(f" MQTT connection failed: {e}")
            return False

    def stop(self):
        self._running = False
        try:
            if self.cli:
                self.cli.loop_stop()
                self.cli.disconnect()
        except Exception:
            pass

    def publish_move(self, speed: int, angle: int, force_override: bool = False):
        """
        Publish move command (backward compatible).
        If force_override=True, append ',F' to request per-frame crawl override.
        """
        if not self._connected:
            return

        if (abs(speed - self._last_speed) >= 5 or abs(angle - self._last_angle) >= 2):
            if force_override:
                command = f"MOVE,{int(speed)},{int(angle)},F"
            else:
                command = f"MOVE,{int(speed)},{int(angle)}"
            try:
                self.cli.publish(self.cfg.topic_commands, command, qos=0, retain=False)
                self.last_command_time = time.time()
                self._last_speed = speed
                self._last_angle = angle
                print(f" Published: {command}")
            except Exception as e:
                print(f" Failed to publish command: {e}")

    # NEW: sticky force enable/disable (maps to UNO 'FORCE,1/0')
    def publish_force(self, enable: bool):
        if not self._connected:
            return
        cmd = f"FORCE,{1 if enable else 0}"
        try:
            self.cli.publish(self.cfg.topic_commands, cmd, qos=0, retain=False)
            self.last_command_time = time.time()
            print(f" Published raw: {cmd}")
        except Exception as e:
            print(f" Failed to publish raw command: {e}")

    # --- MQTT Callbacks (unchanged base logic) ---
    def _on_connect(self, client, userdata, flags, rc):
        self._connected = (rc == 0)
        if rc == 0:
            print(" MQTT connected")
            client.subscribe("/robot/sensors", qos=0)
            client.subscribe("/robot/status", qos=0)
            print("📡 Subscribed to /robot/sensors and /robot/status")
        else:
            print(f" MQTT connection failed with code {rc}")
        self.connectedChanged.emit(self._connected)

    def _on_disconnect(self, client, userdata, rc):
        print(f" MQTT disconnected (code: {rc})")
        self._connected = False
        self.connectedChanged.emit(False)

    def _parse_safety_line(self, line: str) -> dict | None:
        """
        Parses 'SAFETY,FF:<frontMin>,FR:<soft><hard>,RR:<rear>,RS:<soft><hard>,FORCE:<0/1>,E:<0/1>,BF:<0/1>,BR:<0/1>'
        Returns a dict or None if format unexpected.
        """
        try:
            if not line.startswith("SAFETY,"):
                return None
            # Remove prefix and split key:value segments
            segs = line[7:].split(',')
            kv = {}
            for s in segs:
                if ':' in s:
                    k, v = s.split(':', 1)
                    kv[k.strip()] = v.strip()

            def bitpair_to_bools(s: str):
                # Accept "10", "01", "11", "00" or even single '1'/'0'
                if s is None: return (False, False)
                s = s.strip()
                if len(s) == 2:
                    return (s[0] == '1', s[1] == '1')
                elif len(s) == 1:
                    # interpret as soft only
                    return (s == '1', False)
                else:
                    return (False, False)

            front_soft, front_hard = bitpair_to_bools(kv.get('FR'))
            rear_soft, rear_hard   = bitpair_to_bools(kv.get('RS'))

            out = {
                "front_cm": int(kv.get("FF", "255")),
                "rear_cm":  int(kv.get("RR", "255")),
                "front_soft": front_soft,
                "front_hard": front_hard,
                "rear_soft": rear_soft,
                "rear_hard": rear_hard,
                "force": bool(int(kv.get("FORCE", "0"))),
                "estop": bool(int(kv.get("E", "0"))),
                "blocked_front": bool(int(kv.get("BF", "0"))),
                "blocked_rear":  bool(int(kv.get("BR", "0"))),
            }
            return out
        except Exception as e:
            print(f" SAFETY parse error: {e}")
            return None

    def _on_message(self, client, userdata, msg):
        receive_start = time.time()

        try:
            if msg.topic == "/robot/sensors":
                b = msg.payload
                try:
                    parse_start = time.time()
                    s = b.decode(errors='replace')
                    parts = s.split(',')
                    if len(parts) == 4:
                        self.sensor_data = {
                            'vl53': int(parts[0]) & 0xFF,
                            'hcsr04_1': int(parts[1]) & 0xFF,
                            'hcsr04_2': int(parts[2]) & 0xFF,
                            'hcsr04_3': int(parts[3]) & 0xFF,
                        }
                        parse_time = (time.time() - parse_start) * 1_000_000
                        signal_start = time.time()
                        self.last_sensor_time = receive_start
                        self.sensorsUpdated.emit(self.sensor_data)
                        signal_time = (time.time() - signal_start) * 1_000_000
                        self._message_count += 1
                        if self._message_count % 50 == 0:
                            total_time = (time.time() - receive_start) * 1_000_000
                            print(f"⏱️ PYTHON_MQTT: total={total_time:.0f}μs, parse={parse_time:.0f}μs, signal={signal_time:.0f}μs")
                            print(f"📊 Sensors: VL53={self.sensor_data['vl53']}, HC1={self.sensor_data['hcsr04_1']}, HC2={self.sensor_data['hcsr04_2']}, HC3={self.sensor_data['hcsr04_3']}")
                except Exception as e:
                    print(f" Sensor parse error: {e}")

            elif msg.topic == "/robot/status":
                self.arduino_status = msg.payload.decode(errors='replace')
                self.statusUpdated.emit(self.arduino_status)

                # Optional: parse SAFETY line if present
                if self.arduino_status.startswith("SAFETY,"):
                    safety = self._parse_safety_line(self.arduino_status)
                    if safety:
                        self._last_safety = safety
                        self.safetyUpdated.emit(safety)

                # Timing passthrough (unchanged)
                if self.arduino_status.startswith("TIMING,"):
                    print(f" DEVICE_TIMING: {self.arduino_status}")

                # Emergency heuristic (unchanged)
                is_emergency = ("emergency" in self.arduino_status.lower() or 
                                "blocked" in self.arduino_status.lower() or
                                "obstacle" in self.arduino_status.lower() or
                                "alert" in self.arduino_status.lower())
                self.emergencyStopChanged.emit(is_emergency)

        except Exception as e:
            print(f" Failed to process MQTT message: {e}")

        total_time = (time.time() - receive_start) * 1_000_000
        if total_time > 5000:
            print(f"SLOW_MESSAGE: {total_time:.0f}μs for topic {msg.topic}")

    @property
    def is_connected(self):
        return self._connected

    def get_sensor_age(self):
        current_time = time.time()
        return current_time - self.last_sensor_time if self.last_sensor_time > 0 else 999

    def get_command_age(self):
        current_time = time.time()
        return current_time - self.last_command_time if self.last_command_time > 0 else 999

    def is_sensor_live(self):
        return self.get_sensor_age() < 1

    def get_obstacles(self):
        obstacles = []
        if self.sensor_data['vl53'] < 8 and self.sensor_data['vl53'] > 0:
            obstacles.append("FRONT")
        if self.sensor_data['hcsr04_1'] < 8 and self.sensor_data['hcsr04_1'] > 0:
            obstacles.append("LEFT")
        if self.sensor_data['hcsr04_2'] < 8 and self.sensor_data['hcsr04_2'] > 0:
            obstacles.append("RIGHT")
        if self.sensor_data['hcsr04_3'] < 8 and self.sensor_data['hcsr04_3'] > 0:
            obstacles.append("BACK")
        return obstacles

    def has_obstacles(self):
        return len(self.get_obstacles()) > 0

    def get_current_sensor_data(self):
        return self.sensor_data.copy()

    def get_arduino_status(self):
        return self.arduino_status

    # Existing raw publisher (unchanged)
    def publish_command(self, command: str):
        if not self._connected:
            return
        try:
            self.cli.publish(self.cfg.topic_commands, command, qos=0, retain=False)
            self.last_command_time = time.time()
            print(f" Published raw: {command}")
        except Exception as e:
            print(f" Failed to publish raw command: {e}")

    def get_connection_status(self):
        return {
            'connected': self._connected,
            'broker': f"{self.cfg.host}:{self.cfg.port}",
            'sensor_age': self.get_sensor_age(),
            'command_age': self.get_command_age(),
            'message_count': self._message_count,
            'has_obstacles': self.has_obstacles(),
            'obstacles': self.get_obstacles()
        }

    def reset_stats(self):
        self._message_count = 0
        self._last_timing_log = time.time()
        print("MQTT statistics reset")
