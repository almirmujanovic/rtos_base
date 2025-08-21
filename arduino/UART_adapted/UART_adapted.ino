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

// Constants
#define MAX_DISTANCE_CM 100
#define OBSTACLE_THRESHOLD_CM 8
#define SENSOR_TIMEOUT_US (MAX_DISTANCE_CM * 58)
#define SENSOR_UPDATE_INTERVAL 50  // Reduced to 20Hz for stability
#define UART_BAUD 115200

#define SERVO_MIN_ANGLE 48
#define SERVO_MAX_ANGLE 118

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

SensorData sensors = {0};
MotorControl motors = {0, 90, true};
unsigned long lastSensorUpdate = 0;
unsigned long lastCommandTime = 0;
unsigned long lastSensorSend = 0;
bool emergencyStop = false;
bool vl53_initialized = false;

// Immunity counters
uint8_t bad_front = 0;
uint8_t bad_left = 0;
uint8_t bad_right = 0;
uint8_t bad_back = 0;

void setup() {
    // CRITICAL: Initialize Serial1 FIRST before any other Serial operations
    Serial1.begin(UART_BAUD);
    delay(100);  // Let UART stabilize
    
    Serial.begin(UART_BAUD);
    Serial.println("🤖 Arduino Robot Controller v1.1");
    
    // Send startup message to ESP32
    Serial1.println("STARTUP,Arduino Robot Controller v1.1");
    
    initializeMotors();
    steeringServo.attach(SERVO_PIN);
    steeringServo.write(90);
    initializeHCSR04();
    initializeVL53L0X();
    
    Serial.println("✅ All systems initialized");
    Serial1.println("STATUS,All systems initialized");
    
    delay(100);
}

void loop() {
    unsigned long currentTime = millis();
    
    // Process UART commands first (highest priority)
    processUARTCommands();

    // Update sensors at 20Hz (reduced for stability)
    if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
        updateAllSensors();
        lastSensorUpdate = currentTime;
    }

    // Check obstacles every loop for safety
    checkObstacles();

    // Command timeout: only stop motors
    if (currentTime - lastCommandTime > 1000 && lastCommandTime > 0) {
        setMotorSpeed(0);
        Serial.println("⚠️ Command timeout - motors stopped");
        Serial1.println("STATUS,Command timeout - motors stopped");
    }

    // Send sensor data at 20Hz (matches update rate)
    if (currentTime - lastSensorSend >= SENSOR_UPDATE_INTERVAL) {
        sendSensorData();
        lastSensorSend = currentTime;
    }

    delay(2);  // Small delay for stability
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
    Serial.println("✅ HCSR04 sensors initialized");
}

void initializeVL53L0X() {
    Wire.begin();
    vl53_initialized = false;
    
    // Try to initialize VL53L0X with retries
    for (int i = 0; i < 3; i++) {
        if (vl53Sensor.init()) {
            vl53Sensor.setTimeout(50);
            vl53Sensor.setMeasurementTimingBudget(20000);
            vl53_initialized = true;
            Serial.println("✅ VL53L0X initialized");
            Serial1.println("STATUS,VL53L0X initialized");
            return;
        }
        delay(100);
    }
    
    Serial.println("❌ VL53L0X sensor not found - using fallback");
    Serial1.println("ERROR,VL53L0X sensor not found");
    sensors.vl53_distance = 255;
}

void updateAllSensors() {
    // Read VL53L0X first (most reliable)
    sensors.vl53_distance = readVL53L0X();
    
    // Read HC-SR04 sensors with small delays
    sensors.hcsr04_left = readHCSR04(ECHO_PIN_1);
    delay(5);
    sensors.hcsr04_back = readHCSR04(ECHO_PIN_2);
    delay(5);
    sensors.hcsr04_right = readHCSR04(ECHO_PIN_3);
    
    // Debug output every second
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 1000) {
        Serial.print("📊 Raw Sensors: VL53=");
        Serial.print(sensors.vl53_distance);
        Serial.print("cm, L=");
        Serial.print(sensors.hcsr04_left);
        Serial.print("cm, R=");
        Serial.print(sensors.hcsr04_right);
        Serial.print("cm, B=");
        Serial.print(sensors.hcsr04_back);
        Serial.println("cm");
        lastDebug = millis();
    }
}

uint8_t readVL53L0X() {
    if (!vl53_initialized) return 255;
    
    uint16_t distance_mm = vl53Sensor.readRangeSingleMillimeters();
    if (vl53Sensor.timeoutOccurred()) {
        return 255;
    }
    
    uint16_t distance_cm = distance_mm / 10;
    return (distance_cm > 255) ? 255 : (uint8_t)distance_cm;
}

uint8_t readHCSR04(int echoPin) {
    // Ensure trigger is low
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    // Send trigger pulse
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Read echo with timeout
    unsigned long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT_US);
    
    if (duration == 0) {
        return 255;  // Timeout
    }
    
    uint16_t distance_cm = duration / 58;
    return (distance_cm > 255) ? 255 : (uint8_t)distance_cm;
}

void checkObstacles() {
    bool obstacle_detected = false;

    // Immunity logic: increment counters if bad, reset if good
    if (sensors.vl53_distance < OBSTACLE_THRESHOLD_CM && sensors.vl53_distance > 0)
        bad_front++;
    else
        bad_front = 0;

    if (sensors.hcsr04_left < OBSTACLE_THRESHOLD_CM && sensors.hcsr04_left > 0)
        bad_left++;
    else
        bad_left = 0;

    if (sensors.hcsr04_right < OBSTACLE_THRESHOLD_CM && sensors.hcsr04_right > 0)
        bad_right++;
    else
        bad_right = 0;

    if (sensors.hcsr04_back < OBSTACLE_THRESHOLD_CM && sensors.hcsr04_back > 0)
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
            Serial.println("🛑 Obstacle detected - emergency stop");
            Serial1.println("ALERT,Obstacle detected - emergency stop");
        }
    }
    sensors.obstacle_detected = obstacle_detected;
}

void sendSensorData() {
    // CRITICAL: Use consistent format that ESP32 expects
    Serial1.print("SENSORS,");
    Serial1.print(sensors.vl53_distance);
    Serial1.print(",");
    Serial1.print(sensors.hcsr04_left);
    Serial1.print(",");
    Serial1.print(sensors.hcsr04_right);
    Serial1.print(",");
    Serial1.println(sensors.hcsr04_back);
}

void processUARTCommands() {
    // Check if data is available
    if (!Serial1.available()) return;
    
    String command = Serial1.readStringUntil('\n');
    command.trim();
    
    if (command.length() == 0) return;
    
    lastCommandTime = millis();
    Serial.println("📨 ESP32 → Arduino: " + command);
    
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