from __future__ import annotations
import time
import threading
from PySide6.QtCore import QObject, Signal
import paho.mqtt.client as mqtt
from config import MqttConfig

class MqttService(QObject):
    sensorsUpdated = Signal(dict)   # {'vl53':int, 'hcsr04_1':int, 'hcsr04_2':int, 'hcsr04_3':int}
    statusUpdated = Signal(str)     # Arduino status messages
    connectedChanged = Signal(bool) # MQTT connection state
    emergencyStopChanged = Signal(bool)  # Emergency stop detection

    def __init__(self, cfg: MqttConfig):
        super().__init__()
        self.cfg = cfg
        
        # MQTT setup - IDENTICAL to optimized_mqtt.py
        self.cli = mqtt.Client()
        self.cli.on_connect = self._on_connect
        self.cli.on_disconnect = self._on_disconnect
        self.cli.on_message = self._on_message
        
        # Data storage - IDENTICAL to optimized_mqtt.py
        self.sensor_data = {"vl53": 0, "hcsr04_1": 0, "hcsr04_2": 0, "hcsr04_3": 0}
        self.last_command_time = 0
        self.last_sensor_time = 0
        self.arduino_status = "Unknown"
        
        # ✅ IDENTICAL command rate limiting to optimized_mqtt.py
        self._last_speed = 0
        self._last_angle = 90
        self._connected = False
        self._running = False
        self._connection_thread = None
        
        # ✅ Timing tracking for performance monitoring
        self._message_count = 0
        self._last_timing_log = 0

    def start(self):
        """Start MQTT service - IDENTICAL logic to optimized_mqtt.py"""
        self._running = True
        
        # Start connection in background thread
        self._connection_thread = threading.Thread(target=self._connect_mqtt, daemon=True)
        self._connection_thread.start()

    def _connect_mqtt(self):
        """Connect to MQTT broker - IDENTICAL to optimized_mqtt.py connect_mqtt()"""
        try:
            print(f"🔌 Connecting to MQTT broker at {self.cfg.host}:{self.cfg.port}")
            self.cli.connect(self.cfg.host, self.cfg.port, 60)  # ✅ Same 60s keepalive as optimized
            self.cli.loop_start()
            print("✅ MQTT connection initiated")
            return True
        except Exception as e:
            print(f"❌ MQTT connection failed: {e}")
            return False

    def stop(self):
        """Stop MQTT service"""
        self._running = False
        try:
            if self.cli:
                self.cli.loop_stop()
                self.cli.disconnect()
        except Exception:
            pass

    def publish_move(self, speed: int, angle: int):
        """Publish move command - IDENTICAL logic to optimized_mqtt.py send_command()"""
        if not self._connected:
            return
        
        # ✅ IDENTICAL change detection from optimized_mqtt.py
        if (abs(speed - self._last_speed) >= 5 or abs(angle - self._last_angle) >= 2):
            command = f"MOVE,{int(speed)},{int(angle)}"
            try:
                self.cli.publish(self.cfg.topic_commands, command, qos=0, retain=False)
                self.last_command_time = time.time()
                self._last_speed = speed
                self._last_angle = angle
                print(f"📤 Published: {command}")
            except Exception as e:
                print(f"❌ Failed to publish command: {e}")

    # --- MQTT Callbacks - IDENTICAL logic to optimized_mqtt.py ---
    def _on_connect(self, client, userdata, flags, rc):
        """MQTT connect callback - IDENTICAL to optimized_mqtt.py on_mqtt_connect"""
        self._connected = (rc == 0)
        if rc == 0:
            print("✅ MQTT connected")
            # Subscribe to EXACT same topics as optimized_mqtt.py
            client.subscribe("/robot/sensors", qos=0)
            client.subscribe("/robot/status", qos=0)
            print("📡 Subscribed to /robot/sensors and /robot/status")
        else:
            print(f"❌ MQTT connection failed with code {rc}")
        
        self.connectedChanged.emit(self._connected)

    def _on_disconnect(self, client, userdata, rc):
        """MQTT disconnect callback"""
        print(f"❌ MQTT disconnected (code: {rc})")
        self._connected = False
        self.connectedChanged.emit(False)

    def _on_message(self, client, userdata, msg):
        """Handle MQTT messages with timing - ENHANCED from optimized_mqtt.py"""
        receive_start = time.time()
        
        try:
            if msg.topic == "/robot/sensors":
                # ✅ IDENTICAL text-only detection logic from optimized_mqtt.py (since we fixed ESP32 to send text)
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
                        parse_time = (time.time() - parse_start) * 1000000  # microseconds
                        
                        signal_start = time.time()
                        self.last_sensor_time = receive_start
                        self.sensorsUpdated.emit(self.sensor_data)
                        signal_time = (time.time() - signal_start) * 1000000
                        
                        self._message_count += 1
                        
                        # ✅ Log timing every 50 messages like optimized script
                        if self._message_count % 50 == 0:
                            total_time = (time.time() - receive_start) * 1000000
                            print(f"⏱️ PYTHON_MQTT: total={total_time:.0f}μs, parse={parse_time:.0f}μs, signal={signal_time:.0f}μs")
                            print(f"📊 Sensors: VL53={self.sensor_data['vl53']}, HC1={self.sensor_data['hcsr04_1']}, HC2={self.sensor_data['hcsr04_2']}, HC3={self.sensor_data['hcsr04_3']}")
                        
                except Exception as e:
                    print(f"❌ Sensor parse error: {e}")
                        
            elif msg.topic == "/robot/status":
                # ✅ IDENTICAL status handling from optimized_mqtt.py
                self.arduino_status = msg.payload.decode(errors='replace')
                self.statusUpdated.emit(self.arduino_status)
                
                # ✅ Handle timing messages from ESP32/Arduino
                if self.arduino_status.startswith("TIMING,"):
                    print(f"📊 DEVICE_TIMING: {self.arduino_status}")
                
                # ✅ Emergency stop detection - enhanced from optimized_mqtt.py logic
                is_emergency = ("emergency" in self.arduino_status.lower() or 
                              "blocked" in self.arduino_status.lower() or
                              "obstacle" in self.arduino_status.lower() or
                              "alert" in self.arduino_status.lower())
                self.emergencyStopChanged.emit(is_emergency)
                
        except Exception as e:
            print(f"❌ Failed to process MQTT message: {e}")
        
        # ✅ Log slow message processing like optimized script
        total_time = (time.time() - receive_start) * 1000000
        if total_time > 5000:  # Log if >5ms (suspicious)
            print(f"⚠️ SLOW_MESSAGE: {total_time:.0f}μs for topic {msg.topic}")

    # --- Properties matching optimized_mqtt.py functionality ---
    @property
    def is_connected(self):
        """Check if MQTT is connected - IDENTICAL to optimized_mqtt.py mqtt_client.is_connected()"""
        return self._connected
    
    def get_sensor_age(self):
        """Get sensor data age - IDENTICAL calculation from optimized_mqtt.py update_gui()"""
        current_time = time.time()
        return current_time - self.last_sensor_time if self.last_sensor_time > 0 else 999
    
    def get_command_age(self):
        """Get command age - IDENTICAL calculation from optimized_mqtt.py update_gui()"""
        current_time = time.time()
        return current_time - self.last_command_time if self.last_command_time > 0 else 999
    
    def is_sensor_live(self):
        """Check if sensors are live - IDENTICAL logic from optimized_mqtt.py update_gui()"""
        return self.get_sensor_age() < 1
    
    def get_obstacles(self):
        """Get obstacle list - IDENTICAL logic from optimized_mqtt.py update_gui()"""
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
        """Check if any obstacles detected - IDENTICAL logic from optimized_mqtt.py"""
        return len(self.get_obstacles()) > 0
    
    def get_current_sensor_data(self):
        """Get current sensor data - for compatibility"""
        return self.sensor_data.copy()
    
    def get_arduino_status(self):
        """Get Arduino status - IDENTICAL to optimized_mqtt.py"""
        return self.arduino_status

    # ✅ NEW: Additional methods for compatibility with existing app code
    def publish_command(self, command: str):
        """Publish raw command - for app compatibility"""
        if not self._connected:
            return
        
        try:
            self.cli.publish(self.cfg.topic_commands, command, qos=0, retain=False)
            self.last_command_time = time.time()
            print(f"📤 Published raw: {command}")
        except Exception as e:
            print(f"❌ Failed to publish raw command: {e}")

    def get_connection_status(self):
        """Get detailed connection status - for app diagnostics"""
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
        """Reset performance statistics - for app maintenance"""
        self._message_count = 0
        self._last_timing_log = time.time()
        print("📊 MQTT statistics reset")