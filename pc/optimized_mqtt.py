import pygame
import paho.mqtt.client as mqtt
import time
import threading
from datetime import datetime
import tkinter as tk
from tkinter import ttk
import queue

# Configuration
BROKER_IP = "192.168.1.14"
MQTT_PORT = 1883
JOYSTICK_INDEX = 0

class RobotControlGUI:
    def __init__(self):
        # Create main window
        self.root = tk.Tk()
        self.root.title("🤖 ESP32 Robot Control Dashboard")
        self.root.geometry("800x600")
        self.root.configure(bg='#2b2b2b')
        
        # Data queues for thread-safe GUI updates
        self.update_queue = queue.Queue()
        
        # MQTT setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # Joystick setup
        pygame.init()
        pygame.joystick.init()
        
        # Data storage
        self.sensor_data = {"vl53": 0, "hcsr04_1": 0, "hcsr04_2": 0, "hcsr04_3": 0}
        self.joystick_data = {"speed": 0, "angle": 0, "raw_x": 0, "raw_y": 0, "r2": 0, "l2": 0}
        self.last_command_time = 0
        self.last_sensor_time = 0
        self.arduino_status = "Unknown"
        
        # Control settings
        self.deadzone = 0.1
        self.max_speed = 255
        self.min_angle = 55
        self.max_angle = 125
        
        # Initialize joystick
        self.setup_joystick()
        
        # Create GUI
        self.create_widgets()
        
        # Start background threads
        self.running = True
        self.start_threads()
        
    def setup_joystick(self):
        if pygame.joystick.get_count() == 0:
            self.joystick = None
            print("❌ No joystick found!")
            return
            
        self.joystick = pygame.joystick.Joystick(JOYSTICK_INDEX)
        self.joystick.init()
        print(f"✅ Joystick connected: {self.joystick.get_name()}")
    
    def create_widgets(self):
        # Main frame
        main_frame = tk.Frame(self.root, bg='#2b2b2b')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Title
        title_label = tk.Label(main_frame, text="🤖 ESP32 Robot Control Dashboard", 
                              font=('Arial', 16, 'bold'), fg='#ffffff', bg='#2b2b2b')
        title_label.pack(pady=(0, 20))
        
        # Create three columns
        columns_frame = tk.Frame(main_frame, bg='#2b2b2b')
        columns_frame.pack(fill=tk.BOTH, expand=True)
        
        # Left column - Sensor Data
        self.create_sensor_frame(columns_frame)
        
        # Middle column - Joystick Data
        self.create_joystick_frame(columns_frame)
        
        # Right column - Connection Status
        self.create_status_frame(columns_frame)
        
        # Bottom frame - Controls info
        self.create_controls_frame(main_frame)
        
    def create_sensor_frame(self, parent):
        sensor_frame = tk.LabelFrame(parent, text="📡 SENSOR DATA", font=('Arial', 12, 'bold'),
                                   fg='#ffffff', bg='#3b3b3b', relief=tk.RAISED, bd=2)
        sensor_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # Sensor labels
        self.vl53_label = tk.Label(sensor_frame, text="VL53L0X (Front):  000 cm", 
                                  font=('Courier', 10), fg='#00ff00', bg='#3b3b3b')
        self.vl53_label.pack(anchor=tk.W, padx=10, pady=5)
        
        self.hc1_label = tk.Label(sensor_frame, text="HCSR04-1 (Left):  000 cm", 
                                 font=('Courier', 10), fg='#00ff00', bg='#3b3b3b')
        self.hc1_label.pack(anchor=tk.W, padx=10, pady=5)
        
        self.hc2_label = tk.Label(sensor_frame, text="HCSR04-2 (Right): 000 cm", 
                                 font=('Courier', 10), fg='#00ff00', bg='#3b3b3b')
        self.hc2_label.pack(anchor=tk.W, padx=10, pady=5)
        
        self.hc3_label = tk.Label(sensor_frame, text="HCSR04-3 (Back):  000 cm", 
                                 font=('Courier', 10), fg='#00ff00', bg='#3b3b3b')
        self.hc3_label.pack(anchor=tk.W, padx=10, pady=5)
        
        # Obstacle warning
        self.obstacle_label = tk.Label(sensor_frame, text="✅ Path clear", 
                                     font=('Arial', 11, 'bold'), fg='#00ff00', bg='#3b3b3b')
        self.obstacle_label.pack(anchor=tk.W, padx=10, pady=10)
        
        # Sensor progress bars
        tk.Label(sensor_frame, text="Sensor Ranges:", font=('Arial', 10, 'bold'), 
                fg='#ffffff', bg='#3b3b3b').pack(anchor=tk.W, padx=10, pady=(10, 5))
        
        self.vl53_progress = ttk.Progressbar(sensor_frame, length=200, mode='determinate', maximum=255)
        self.vl53_progress.pack(anchor=tk.W, padx=10, pady=2)
        
        self.hc1_progress = ttk.Progressbar(sensor_frame, length=200, mode='determinate', maximum=255)
        self.hc1_progress.pack(anchor=tk.W, padx=10, pady=2)
        
        self.hc2_progress = ttk.Progressbar(sensor_frame, length=200, mode='determinate', maximum=255)
        self.hc2_progress.pack(anchor=tk.W, padx=10, pady=2)
        
        self.hc3_progress = ttk.Progressbar(sensor_frame, length=200, mode='determinate', maximum=255)
        self.hc3_progress.pack(anchor=tk.W, padx=10, pady=2)
    
    def create_joystick_frame(self, parent):
        joystick_frame = tk.LabelFrame(parent, text="🎮 JOYSTICK INPUT", font=('Arial', 12, 'bold'),
                                     fg='#ffffff', bg='#3b3b3b', relief=tk.RAISED, bd=2)
        joystick_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        # Raw axis values
        self.raw_x_label = tk.Label(joystick_frame, text="Left Stick X:  +0.000", 
                                   font=('Courier', 10), fg='#ffff00', bg='#3b3b3b')
        self.raw_x_label.pack(anchor=tk.W, padx=10, pady=5)
        
        self.r2_label = tk.Label(joystick_frame, text="R2 Trigger:   +0.000", 
                                font=('Courier', 10), fg='#ffff00', bg='#3b3b3b')
        self.r2_label.pack(anchor=tk.W, padx=10, pady=5)
        
        self.l2_label = tk.Label(joystick_frame, text="L2 Trigger:   +0.000", 
                                font=('Courier', 10), fg='#ffff00', bg='#3b3b3b')
        self.l2_label.pack(anchor=tk.W, padx=10, pady=5)
        
        # Processed values
        tk.Label(joystick_frame, text="Processed Values:", font=('Arial', 10, 'bold'), 
                fg='#ffffff', bg='#3b3b3b').pack(anchor=tk.W, padx=10, pady=(15, 5))
        
        self.speed_label = tk.Label(joystick_frame, text="Speed:      +000 (-255 to +255)", 
                                   font=('Courier', 10), fg='#00ffff', bg='#3b3b3b')
        self.speed_label.pack(anchor=tk.W, padx=10, pady=5)
        
        self.angle_label = tk.Label(joystick_frame, text="Angle:      090° (55-125, center=90)", 
                                   font=('Courier', 10), fg='#00ffff', bg='#3b3b3b')
        self.angle_label.pack(anchor=tk.W, padx=10, pady=5)
        
        # Progress bars
        tk.Label(joystick_frame, text="Visual Feedback:", font=('Arial', 10, 'bold'), 
                fg='#ffffff', bg='#3b3b3b').pack(anchor=tk.W, padx=10, pady=(15, 5))
        
        tk.Label(joystick_frame, text="Speed:", font=('Arial', 9), 
                fg='#ffffff', bg='#3b3b3b').pack(anchor=tk.W, padx=10)
        self.speed_progress = ttk.Progressbar(joystick_frame, length=200, mode='determinate', maximum=255)
        self.speed_progress.pack(anchor=tk.W, padx=10, pady=2)
        
        tk.Label(joystick_frame, text="Angle:", font=('Arial', 9), 
                fg='#ffffff', bg='#3b3b3b').pack(anchor=tk.W, padx=10)
        self.angle_progress = ttk.Progressbar(joystick_frame, length=200, mode='determinate', maximum=70)
        self.angle_progress.pack(anchor=tk.W, padx=10, pady=2)
    
    def create_status_frame(self, parent):
        status_frame = tk.LabelFrame(parent, text="🔗 CONNECTION STATUS", font=('Arial', 12, 'bold'),
                                   fg='#ffffff', bg='#3b3b3b', relief=tk.RAISED, bd=2)
        status_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        self.mqtt_status_label = tk.Label(status_frame, text="MQTT:       🔴 DISCONNECTED", 
                                         font=('Courier', 10), fg='#ff0000', bg='#3b3b3b')
        self.mqtt_status_label.pack(anchor=tk.W, padx=10, pady=5)
        
        self.sensor_status_label = tk.Label(status_frame, text="Sensors:    🔴 STALE", 
                                           font=('Courier', 10), fg='#ff0000', bg='#3b3b3b')
        self.sensor_status_label.pack(anchor=tk.W, padx=10, pady=5)
        
        self.command_status_label = tk.Label(status_frame, text="Commands:   Never sent", 
                                            font=('Courier', 10), fg='#ffff00', bg='#3b3b3b')
        self.command_status_label.pack(anchor=tk.W, padx=10, pady=5)
        
        self.arduino_status_label = tk.Label(status_frame, text="Arduino:    Unknown", 
                                            font=('Courier', 10), fg='#ffff00', bg='#3b3b3b')
        self.arduino_status_label.pack(anchor=tk.W, padx=10, pady=5)
        
        # Time display
        self.time_label = tk.Label(status_frame, text="", font=('Arial', 11, 'bold'), 
                                  fg='#ffffff', bg='#3b3b3b')
        self.time_label.pack(anchor=tk.W, padx=10, pady=15)
    
    def create_controls_frame(self, parent):
        controls_frame = tk.LabelFrame(parent, text="🎯 CONTROLS", font=('Arial', 12, 'bold'),
                                     fg='#ffffff', bg='#3b3b3b', relief=tk.RAISED, bd=2)
        controls_frame.pack(fill=tk.X, pady=(20, 0))
        
        controls_text = """
        R2 Trigger: Forward Speed (0 to +255)    |    L2 Trigger: Reverse Speed (0 to -255)
        Left Stick X: Servo Steering (55° to 125°, center=90°)
        Emergency Stop: Arduino automatically stops if obstacle < 8cm detected
        """
        
        tk.Label(controls_frame, text=controls_text, font=('Arial', 10), 
                fg='#ffffff', bg='#3b3b3b', justify=tk.CENTER).pack(pady=10)
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT connected")
            client.subscribe("/robot/sensors")
            client.subscribe("/robot/status")
        else:
            print(f"❌ MQTT connection failed with code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        current_time = time.time()
        
        if msg.topic == "/robot/sensors":
            self.parse_sensor_data(msg.payload.decode())
            self.last_sensor_time = current_time
        elif msg.topic == "/robot/status":
            status_msg = msg.payload.decode()
            self.arduino_status = status_msg
    
    def parse_sensor_data(self, message):
        """Parse 8-bit sensor data: '45,150,255,75'"""
        try:
            parts = message.split(',')
            if len(parts) == 4:
                self.sensor_data = {
                    'vl53': int(parts[0]),
                    'hcsr04_1': int(parts[1]),
                    'hcsr04_2': int(parts[2]),
                    'hcsr04_3': int(parts[3])
                }
        except (ValueError, IndexError):
            print(f"❌ Failed to parse sensor data: {message}")
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def process_joystick(self):
        """Process joystick input with R2/L2 triggers"""
        if not self.joystick:
            return 0, 90
            
        pygame.event.pump()
        
        # Get joystick values
        raw_x = self.joystick.get_axis(0)  # Left stick X (steering)
        r2_trigger = (self.joystick.get_axis(5) + 1) / 2  # R2 trigger (0 to 1)
        l2_trigger = (self.joystick.get_axis(4) + 1) / 2  # L2 trigger (0 to 1)
        
        # Apply deadzone to steering
        x = self.apply_deadzone(raw_x)
        
        # Calculate speed: R2 = forward, L2 = reverse
        forward_speed = r2_trigger * self.max_speed if r2_trigger > 0.1 else 0
        reverse_speed = -l2_trigger * self.max_speed if l2_trigger > 0.1 else 0
        
        # Use whichever trigger is pressed more
        if forward_speed > abs(reverse_speed):
            speed = int(forward_speed)
        else:
            speed = int(reverse_speed)
        
        # Calculate angle: 55 to 125 degrees
        angle = int(((x + 1) / 2) * (self.max_angle - self.min_angle) + self.min_angle)
        angle = max(self.min_angle, min(self.max_angle, angle))
        
        # Store for display
        self.joystick_data = {
            "speed": speed,
            "angle": angle,
            "raw_x": raw_x,
            "raw_y": 0,  # Not used anymore
            "r2": r2_trigger,
            "l2": l2_trigger
        }
        
        return speed, angle
    
    def send_command(self, speed, angle):
        """Send robot command"""
        command = f"MOVE,{speed},{angle}"
        self.mqtt_client.publish("/robot/commands", command)
        self.last_command_time = time.time()
        return command
    
    def update_gui(self):
        """Update GUI elements (called from main thread)"""
        # Update sensor data
        self.vl53_label.config(text=f"VL53L0X (Front):  {self.sensor_data['vl53']:3d} cm")
        self.hc1_label.config(text=f"HCSR04-1 (Left):  {self.sensor_data['hcsr04_1']:3d} cm")
        self.hc2_label.config(text=f"HCSR04-2 (Right): {self.sensor_data['hcsr04_2']:3d} cm")
        self.hc3_label.config(text=f"HCSR04-3 (Back):  {self.sensor_data['hcsr04_3']:3d} cm")
        
        # Update progress bars
        self.vl53_progress['value'] = self.sensor_data['vl53']
        self.hc1_progress['value'] = self.sensor_data['hcsr04_1']
        self.hc2_progress['value'] = self.sensor_data['hcsr04_2']
        self.hc3_progress['value'] = self.sensor_data['hcsr04_3']
        
        # Check obstacles
        obstacles = []
        if self.sensor_data['vl53'] < 8 and self.sensor_data['vl53'] > 0:
            obstacles.append("FRONT")
        if self.sensor_data['hcsr04_1'] < 8 and self.sensor_data['hcsr04_1'] > 0:
            obstacles.append("LEFT")
        if self.sensor_data['hcsr04_2'] < 8 and self.sensor_data['hcsr04_2'] > 0:
            obstacles.append("RIGHT")
        if self.sensor_data['hcsr04_3'] < 8 and self.sensor_data['hcsr04_3'] > 0:
            obstacles.append("BACK")
        
        if obstacles:
            self.obstacle_label.config(text=f"⚠️ OBSTACLES: {', '.join(obstacles)}", fg='#ff0000')
        else:
            self.obstacle_label.config(text="✅ Path clear", fg='#00ff00')
        
        # Update joystick data
        self.raw_x_label.config(text=f"Left Stick X:  {self.joystick_data['raw_x']:+6.3f}")
        self.r2_label.config(text=f"R2 Trigger:   {self.joystick_data['r2']:6.3f}")
        self.l2_label.config(text=f"L2 Trigger:   {self.joystick_data['l2']:6.3f}")
        
        self.speed_label.config(text=f"Speed:      {self.joystick_data['speed']:+4d} (-255 to +255)")
        self.angle_label.config(text=f"Angle:      {self.joystick_data['angle']:3d}° (55-125, center=90)")
        
        # Update progress bars
        self.speed_progress['value'] = abs(self.joystick_data['speed'])
        self.angle_progress['value'] = self.joystick_data['angle'] - self.min_angle
        
        # Update connection status
        current_time = time.time()
        sensor_age = current_time - self.last_sensor_time if self.last_sensor_time > 0 else 999
        command_age = current_time - self.last_command_time if self.last_command_time > 0 else 999
        
        mqtt_status = "🟢 CONNECTED" if self.mqtt_client.is_connected() else "🔴 DISCONNECTED"
        mqtt_color = '#00ff00' if self.mqtt_client.is_connected() else '#ff0000'
        
        sensor_status = "🟢 LIVE" if sensor_age < 1 else "🔴 STALE"
        sensor_color = '#00ff00' if sensor_age < 1 else '#ff0000'
        
        self.mqtt_status_label.config(text=f"MQTT:       {mqtt_status}", fg=mqtt_color)
        self.sensor_status_label.config(text=f"Sensors:    {sensor_status} ({sensor_age:.1f}s)", fg=sensor_color)
        self.command_status_label.config(text=f"Commands:   {command_age:.1f}s ago")
        self.arduino_status_label.config(text=f"Arduino:    {self.arduino_status}")
        
        # Update time
        self.time_label.config(text=datetime.now().strftime('%H:%M:%S'))
    
    def connect_mqtt(self):
        """Connect to MQTT broker"""
        try:
            self.mqtt_client.connect(BROKER_IP, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            return True
        except Exception as e:
            print(f"❌ MQTT connection failed: {e}")
            return False
    
    def joystick_thread(self):
        """Background thread for joystick processing"""
        last_speed = 0
        last_angle = 90
        
        while self.running:
            try:
                speed, angle = self.process_joystick()
                
                # Send command if values changed significantly
                if (abs(speed - last_speed) >= 5 or abs(angle - last_angle) >= 2):
                    self.send_command(speed, angle)
                    last_speed = speed
                    last_angle = angle
                
                time.sleep(0.05)  # 20Hz
                
            except Exception as e:
                print(f"Joystick thread error: {e}")
                time.sleep(0.1)
    
    def gui_update_thread(self):
        """Background thread for GUI updates"""
        while self.running:
            try:
                self.root.after(0, self.update_gui)  # Schedule GUI update on main thread
                time.sleep(0.1)  # 10Hz GUI updates
            except Exception as e:
                print(f"GUI update thread error: {e}")
                time.sleep(0.1)
    
    def start_threads(self):
        """Start background threads"""
        # Connect MQTT
        self.connect_mqtt()
        
        # Start joystick thread
        joystick_thread = threading.Thread(target=self.joystick_thread, daemon=True)
        joystick_thread.start()
        
        # Start GUI update thread
        gui_thread = threading.Thread(target=self.gui_update_thread, daemon=True)
        gui_thread.start()
    
    def on_closing(self):
        """Handle window closing"""
        self.running = False
        # Send stop command
        self.send_command(0, 90)
        self.mqtt_client.disconnect()
        pygame.quit()
        self.root.destroy()
    
    def run(self):
        """Start the GUI application"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        print("🚀 Starting robot control GUI...")
        self.root.mainloop()

if __name__ == "__main__":
    app = RobotControlGUI()
    app.run()