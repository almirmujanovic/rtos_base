#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

// Motor Driver TB6612 pins
#define PWMA 9
#define AIN1 10
#define AIN2 11
#define PWMB 3
#define BIN1 4
#define BIN2 5
#define STBY 12

// HCSR04 pins (shared trigger)
#define TRIG_PIN 2
#define ECHO_PIN_1 A0  // Left
#define ECHO_PIN_2 A1  // Back
#define ECHO_PIN_3 A2  // Right

// Servo pin
#define SERVO_PIN 6

// Constants - OPTIMIZED TIMING
#define MAX_DISTANCE_CM 100                    // ✅ Reduced for faster response
#define OBSTACLE_THRESHOLD_CM 8
#define SENSOR_TIMEOUT_US 6000                 // ✅ 100cm = ~5800μs
#define SENSOR_UPDATE_INTERVAL 50              // ✅ 20Hz for reliable timing
#define SENSOR_SEND_INTERVAL 50                // ✅ Match update rate
#define UART_BAUD 115200

#define SERVO_MIN_ANGLE 25
#define SERVO_MAX_ANGLE 155
#define BAD_READS_THRESHOLD 2

Servo steeringServo;
VL53L0X vl53Sensor;

struct SensorData {
    uint8_t vl53_distance;
    uint8_t hcsr04_left;
    uint8_t hcsr04_right;
    uint8_t hcsr04_back;
    bool obstacle_detected;
};

struct MotorControl {
    int speed;
    int angle;
    bool motors_enabled;
};

SensorData sensors = {255, 255, 255, 255, false};  // ✅ Initialize to "no obstacle"
MotorControl motors = {0, 90, true};
unsigned long lastSensorUpdate = 0;
unsigned long lastSensorSend = 0;                  // ✅ Separate timing for UART
unsigned long lastCommandTime = 0;
bool emergencyStop = false;
bool vl53_initialized = false;

// Immunity counters
uint8_t bad_front = 0;
uint8_t bad_left = 0;
uint8_t bad_right = 0;
uint8_t bad_back = 0;

// ✅ Timing variables for diagnostics
unsigned long timing_sensor_start = 0;
unsigned long timing_sensor_end = 0;
unsigned long timing_vl53_start = 0;
unsigned long timing_vl53_end = 0;

void setup() {
    Serial.begin(UART_BAUD);
    Serial.println("🤖 Arduino Robot Controller v1.2 (TIMING FIXED)");
    Serial1.begin(UART_BAUD);
    Serial1.println("STARTUP,Arduino Robot Controller v1.2");
    
    initializeMotors();
    steeringServo.attach(SERVO_PIN);
    steeringServo.write(90);
    initializeHCSR04();
    initializeVL53L0X();
    
    Serial.println("✅ All systems initialized");
    Serial1.println("STATUS,All systems initialized");
    delay(200);  // ✅ Longer startup delay
}

void loop() {
    unsigned long currentTime = millis();
    
    // Process UART commands first (highest priority)
    processUARTCommands();

    // ✅ Update sensors at controlled 20Hz rate
    if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
        updateAllSensorsSequential();  // ✅ NEW: Sequential reading
        lastSensorUpdate = currentTime;
    }

    // Check obstacles every loop for safety
    checkObstacles();

    // Command timeout
    if (currentTime - lastCommandTime > 100000 && lastCommandTime > 0) {
        setMotorSpeed(0);
        Serial.println("⚠️ Command timeout - motors stopped");
        Serial1.println("STATUS,Command timeout - motors stopped");
    }

    // ✅ Send sensor data at controlled rate (not every loop!)
    if (currentTime - lastSensorSend >= SENSOR_SEND_INTERVAL) {
        sendSensorData();
        lastSensorSend = currentTime;
    }

    // ✅ REMOVED: delay(1) - let task run at full speed
}

void initializeMotors() {
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
    setMotorSpeed(0);
    Serial.println("✅ TB6612 motor driver initialized");
}

void initializeHCSR04() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN_1, INPUT);
    pinMode(ECHO_PIN_2, INPUT);
    pinMode(ECHO_PIN_3, INPUT);
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);  // Ensure clean state
    Serial.println("✅ HCSR04 sensors initialized");
}

void initializeVL53L0X() {
    Wire.begin();
    Wire.setClock(400000);  // ✅ Fast I2C for VL53L0X
    
    // ✅ Try initialization with retries
    for (int attempt = 0; attempt < 3; attempt++) {
        if (vl53Sensor.init()) {
            vl53Sensor.setTimeout(30);                        // ✅ SHORTER: 30ms timeout
            vl53Sensor.setMeasurementTimingBudget(20000);     // ✅ FASTER: 20ms measurement
            vl53_initialized = true;
            Serial.println("✅ VL53L0X initialized (fast mode)");
            Serial1.println("STATUS,VL53L0X initialized");
            return;
        }
        Serial.print("❌ VL53L0X init attempt ");
        Serial.print(attempt + 1);
        Serial.println(" failed");
        delay(100);
    }
    
    Serial.println("❌ VL53L0X sensor not found - using fallback");
    Serial1.println("ERROR,VL53L0X sensor not found");
    vl53_initialized = false;
    sensors.vl53_distance = 255;
}

// ✅ NEW: Sequential sensor reading to prevent interference
void updateAllSensorsSequential() {
    timing_sensor_start = micros();
    
    // Read VL53L0X first (most critical and doesn't interfere)
    timing_vl53_start = micros();
    sensors.vl53_distance = readVL53L0X();
    timing_vl53_end = micros();
    
    // ✅ Sequential HC-SR04 reading with proper delays
    sensors.hcsr04_left = readHCSR04(ECHO_PIN_1);
    delay(20);  // ✅ PROPER: 20ms between sensors for acoustic isolation
    
    sensors.hcsr04_back = readHCSR04(ECHO_PIN_2);
    delay(20);  // ✅ PROPER: 20ms between sensors
    
    sensors.hcsr04_right = readHCSR04(ECHO_PIN_3);
    
    timing_sensor_end = micros();
    
    // ✅ Enhanced debug with timing every 2 seconds
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 2000) {
        unsigned long total_time = timing_sensor_end - timing_sensor_start;
        unsigned long vl53_time = timing_vl53_end - timing_vl53_start;
        
        Serial.print("⏱️ TIMING: Total=");
        Serial.print(total_time);
        Serial.print("μs, VL53=");
        Serial.print(vl53_time);
        Serial.print("μs | 📊 Values: VL53=");
        Serial.print(sensors.vl53_distance);
        Serial.print("cm, L=");
        Serial.print(sensors.hcsr04_left);
        Serial.print("cm, R=");
        Serial.print(sensors.hcsr04_right);
        Serial.print("cm, B=");
        Serial.print(sensors.hcsr04_back);
        Serial.println("cm");
        
        // Send timing to ESP32
        Serial1.print("TIMING,ARDUINO_SENSORS,");
        Serial1.println(total_time);
        
        lastDebug = millis();
    }
}

// ✅ OPTIMIZED: Faster VL53L0X reading
uint8_t readVL53L0X() {
    if (!vl53_initialized) return 255;
    
    // ✅ Non-blocking read with quick timeout
    uint16_t distance_mm = vl53Sensor.readRangeSingleMillimeters();
    
    if (vl53Sensor.timeoutOccurred()) {
        Serial.println("⚠️ VL53L0X timeout");
        return 255;  // Timeout indicates no obstacle
    }
    
    uint16_t distance_cm = distance_mm / 10;
    
    // ✅ Range validation for VL53L0X
    if (distance_cm < 2 || distance_cm > 200) {
        return 255;  // Out of valid range
    }
    
    return (distance_cm > 255) ? 255 : (uint8_t)distance_cm;
}

// ✅ OPTIMIZED: HC-SR04 with proper timeout for 100cm
uint8_t readHCSR04(int echoPin) {
    // ✅ Ensure trigger pin is clean
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    // Send 10μs trigger pulse
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // ✅ Read with proper timeout for 100cm range
    unsigned long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT_US);
    
    if (duration == 0) {
        return 255;  // No echo = no obstacle in range
    }
    
    // Convert to centimeters
    uint16_t distance_cm = duration / 58;
    
    // ✅ Validate range (HC-SR04 minimum ~2cm)
    if (distance_cm < 2) {
        return 2;   // Too close, clamp to minimum
    }
    if (distance_cm > MAX_DISTANCE_CM) {
        return 255; // Beyond our interest range
    }
    
    return (uint8_t)distance_cm;
}

void checkObstacles() {
    bool obstacle_detected = false;

    // ✅ Only check valid readings (not 255)
    if (sensors.vl53_distance < OBSTACLE_THRESHOLD_CM && sensors.vl53_distance > 0 && sensors.vl53_distance < 255)
        bad_front++;
    else
        bad_front = 0;

    if (sensors.hcsr04_left < OBSTACLE_THRESHOLD_CM && sensors.hcsr04_left > 0 && sensors.hcsr04_left < 255)
        bad_left++;
    else
        bad_left = 0;

    if (sensors.hcsr04_right < OBSTACLE_THRESHOLD_CM && sensors.hcsr04_right > 0 && sensors.hcsr04_right < 255)
        bad_right++;
    else
        bad_right = 0;

    if (sensors.hcsr04_back < OBSTACLE_THRESHOLD_CM && sensors.hcsr04_back > 0 && sensors.hcsr04_back < 255)
        bad_back++;
    else
        bad_back = 0;

    // Only trigger obstacle if threshold reached
    if (bad_front >= BAD_READS_THRESHOLD ||
        bad_left >= BAD_READS_THRESHOLD ||
        bad_right >= BAD_READS_THRESHOLD ||
        bad_back >= BAD_READS_THRESHOLD) {
        obstacle_detected = true;
    }

    // Emergency stop logic
    if (!obstacle_detected) {
        if (emergencyStop) {
            emergencyStop = false;
            Serial.println("✅ Path clear - motors enabled");
            Serial1.println("STATUS,Path clear - motors enabled");
        }
    } else {
        if (!emergencyStop) {
            emergencyStop = true;
            setMotorSpeed(0);
            
            // ✅ Enhanced obstacle reporting
            String obstacles = "🛑 Obstacles: ";
            if (bad_front >= BAD_READS_THRESHOLD) obstacles += "FRONT(" + String(sensors.vl53_distance) + "cm) ";
            if (bad_left >= BAD_READS_THRESHOLD) obstacles += "LEFT(" + String(sensors.hcsr04_left) + "cm) ";
            if (bad_right >= BAD_READS_THRESHOLD) obstacles += "RIGHT(" + String(sensors.hcsr04_right) + "cm) ";
            if (bad_back >= BAD_READS_THRESHOLD) obstacles += "BACK(" + String(sensors.hcsr04_back) + "cm) ";
            
            Serial.println(obstacles);
            Serial1.println("ALERT," + obstacles);
        }
    }
    sensors.obstacle_detected = obstacle_detected;
}

void sendSensorData() {
    // ✅ Consistent format that ESP32 can parse reliably
    Serial1.print("SENSORS,");
    Serial1.print(sensors.vl53_distance);
    Serial1.print(",");
    Serial1.print(sensors.hcsr04_left);
    Serial1.print(",");
    Serial1.print(sensors.hcsr04_right);
    Serial1.print(",");
    Serial1.println(sensors.hcsr04_back);
    
    // Debug output every 3 seconds (reduced frequency)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 3000) {
        Serial.print("📊 Sensors: VL53=");
        Serial.print(sensors.vl53_distance);
        Serial.print("cm, L=");
        Serial.print(sensors.hcsr04_left);
        Serial.print("cm, R=");
        Serial.print(sensors.hcsr04_right);
        Serial.print("cm, B=");
        Serial.print(sensors.hcsr04_back);
        Serial.println("cm");
        lastDebugTime = millis();
    }
}

// ✅ Rest of functions remain the same...
void processUARTCommands() {
    if (!Serial1.available()) return;
    String command = Serial1.readStringUntil('\n');
    command.trim();
    if (command.length() == 0) return;
    lastCommandTime = millis();
    Serial.println("📨 Received: " + command);
    if (command.startsWith("MOVE,")) {
        parseMoveCommand(command);
    } else if (command == "STOP") {
        setMotorSpeed(0);
        motors.speed = 0;
        Serial.println("🛑 STOP command received");
        Serial1.println("ACK,STOP");
    } else if (command == "STATUS") {
        sendStatusUpdate();
    } else if (command.startsWith("SERVO,")) {
        parseServoCommand(command);
    } else {
        Serial.println("❌ Unknown command: " + command);
        Serial1.println("ERROR,Unknown command: " + command);
    }
}

void parseMoveCommand(String command) {
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    if (firstComma == -1 || secondComma == -1) {
        Serial.println("❌ Invalid MOVE format");
        Serial1.println("ERROR,Invalid MOVE format");
        return;
    }
    int speed = command.substring(firstComma + 1, secondComma).toInt();
    int angle = command.substring(secondComma + 1).toInt();
    speed = constrain(speed, -255, 255);
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    motors.speed = speed;
    motors.angle = angle;
    steeringServo.write(angle);
    if (!emergencyStop) {
        setMotorSpeed(speed);
        Serial.println("🚗 MOVE: Speed=" + String(speed) + ", Angle=" + String(angle));
        Serial1.println("ACK,MOVE," + String(speed) + "," + String(angle));
    } else {
        Serial.println("🚫 BLOCKED: Emergency stop active");
        Serial1.println("BLOCKED,Emergency stop active");
    }
}

void parseServoCommand(String command) {
    int commaIndex = command.indexOf(',');
    if (commaIndex == -1) {
        Serial.println("❌ Invalid SERVO format");
        Serial1.println("ERROR,Invalid SERVO format");
        return;
    }
    int angle = command.substring(commaIndex + 1).toInt();
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    steeringServo.write(angle);
    motors.angle = angle;
    Serial.println("🎯 SERVO: Angle=" + String(angle));
    Serial1.println("ACK,SERVO," + String(angle));
}

void setMotorSpeed(int speed) {
    if (speed == 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMA, 0);
        analogWrite(PWMB, 0);
        return;
    }
    int pwm_value = abs(speed);
    bool forward = (speed > 0);
    
    if (forward) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    } else {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }
    analogWrite(PWMA, pwm_value);
    analogWrite(PWMB, pwm_value);
}

void sendStatusUpdate() {
    String status = "STATUS,Speed:" + String(motors.speed) +
                   ",Angle:" + String(motors.angle) +
                   ",Emergency:" + String(emergencyStop ? "true" : "false") +
                   ",Obstacle:" + String(sensors.obstacle_detected ? "true" : "false") +
                   ",VL53:" + String(vl53_initialized ? "OK" : "FAIL");
    Serial1.println(status);
    Serial.println("📋 " + status);
}