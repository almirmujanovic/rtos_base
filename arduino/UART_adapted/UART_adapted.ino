#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

// ---------------- PINS  ----------------
#define PWMA 9
#define AIN1 10
#define AIN2 11
#define PWMB 3
#define BIN1 4
#define BIN2 5
#define STBY 12

#define TRIG_PIN 2
#define ECHO_PIN_1 A0  // Front-Left
#define ECHO_PIN_2 A1  // Rear
#define ECHO_PIN_3 A2  // Front-Right

#define SERVO_PIN 6

// ---------------- CONSTANTS ----------------
#define MAX_DISTANCE_CM 100
#define OBSTACLE_THRESHOLD_CM 8
#define SENSOR_TIMEOUT_US 6000
#define SENSOR_UPDATE_INTERVAL 50
#define SENSOR_SEND_INTERVAL 50
#define UART_BAUD 115200

#define SERVO_MIN_ANGLE 25
#define SERVO_MAX_ANGLE 155
#define BAD_READS_THRESHOLD 2

// Safety tuning
#define DEBOUNCE_FRAMES 3          
#define HYSTERESIS_CLEAR_PCT 15    // % farther to clear a zone
#define BLOCKED_LATCH_MS 1500      // time commanding into hard stop before we flag "blocked"
#define CRAWL_PWM_CAP 40           // max PWM when forcing through SOFT zone (≈ 0.06–0.08 m/s on TT motors)
#define SOFT_FORWARD_PWM_CAP 64    // speed cap in soft zone if not forcing (gentle ramp)

// ---- Fixed zone thresholds (cm) ----
#define FC_SOFT_CM   25   // front center: half below this
#define FC_HARD_CM    8   // front center: stop below this

#define FS_SOFT_CM   15   // front sides (L/R): half below this
#define FS_HARD_CM    6   // front sides (L/R): stop below this

#define REAR_SOFT_CM 20   // rear: half below this
#define REAR_HARD_CM  8   // rear: stop below this

// Half-speed cap (PWM). "Half" of max.
#define HALF_PWM_CAP 127


// Soft/Hard distance functions (speed-scaled, in cm)
// v_scale = |pwm| / 255
inline uint8_t soft_distance_cm(uint8_t pwm_abs) {
  float v = pwm_abs / 255.0f;
  // base 12cm, scale with speed (tuneable)
  float d = max(12.0f, 40.0f * v + 8.0f);   // 12..~48cm
  if (d > 100.0f) d = 100.0f;
  return (uint8_t)d;
}
inline uint8_t hard_distance_cm(uint8_t pwm_abs) {
  float v = pwm_abs / 255.0f;
  // base 7cm, scale with speed (tuneable)
  float d = max(7.0f, 20.0f * v + 5.0f);    // 7..~25cm
  if (d > 100.0f) d = 100.0f;
  return (uint8_t)d;
}

// ---------------- TYPES/STATE ----------------
Servo steeringServo;
VL53L0X vl53Sensor;

struct SensorData {
  uint8_t vl53_distance;  // front center
  uint8_t hcsr04_left;    // front-left
  uint8_t hcsr04_right;   // front-right
  uint8_t hcsr04_back;    // rear
  bool obstacle_detected;
};

struct MotorControl {
  int speed;              // requested PWM (-255..255)
  int angle;              // servo angle
  bool motors_enabled;
};

struct SafetyState {
  // instantaneous distances
  uint8_t front_min;      // min(VL53, left, right) but using outlier rejection
  uint8_t rear_dist;

  // soft/hard thresholds derived from current |speed|
  uint8_t soft_front;
  uint8_t hard_front;
  uint8_t soft_rear;
  uint8_t hard_rear;

  // debounced zone states
  bool soft_front_on;
  bool hard_front_on;
  bool soft_rear_on;
  bool hard_rear_on;

  // counters for debounce
  uint8_t sf_cnt, hf_cnt, sr_cnt, hr_cnt;

  // latching / helpers
  bool force_override;      
  bool estop_latched;       
  bool blocked_front;
  bool blocked_rear;

  unsigned long hard_front_since_ms;
  unsigned long hard_rear_since_ms;
};

SensorData sensors = {255, 255, 255, 255, false};
MotorControl motors = {0, 90, true};
SafetyState safety = {};

unsigned long lastSensorUpdate = 0;
unsigned long lastSensorSend = 0;
unsigned long lastCommandTime = 0;
bool vl53_initialized = false;

// immunity counters (legacy quick noise guards)
uint8_t bad_front = 0;
uint8_t bad_left = 0;
uint8_t bad_right = 0;
uint8_t bad_back = 0;

// timing diag
unsigned long timing_sensor_start = 0;
unsigned long timing_sensor_end = 0;
unsigned long timing_vl53_start = 0;
unsigned long timing_vl53_end = 0;

// ---------------- FORWARD DECLS ----------------
void initializeMotors();
void initializeHCSR04();
void initializeVL53L0X();
void updateAllSensorsSequential();
uint8_t readVL53L0X();
uint8_t readHCSR04(int echoPin);
void processUARTCommands();
void parseMoveCommand(String command);
void parseServoCommand(String command);
void sendSensorData();
void sendStatusUpdate();
void setMotorSpeedRaw(int speed);

// Safety helpers
void updateSafetyThresholds(uint8_t pwm_abs);
uint8_t fusedFrontDistance();
void updateSafetyZones(uint8_t pwm_abs);
void applySafetyAndDrive();
int clampSpeedWithSafety(int requested);

// ---------------- SETUP/LOOP ----------------
void setup() {
  Serial.begin(UART_BAUD);
  Serial.println("Arduino Car Controller");
  Serial1.begin(UART_BAUD);
  Serial1.println("STARTUP,Arduino Car Controller");

  initializeMotors();
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90);
  initializeHCSR04();
  initializeVL53L0X();

  safety.force_override = false;
  safety.estop_latched = false;
  safety.blocked_front = false;
  safety.blocked_rear = false;
  safety.hard_front_since_ms = 0;
  safety.hard_rear_since_ms  = 0;

  Serial.println("All systems initialized");
  Serial1.println("STATUS,All systems initialized");
  delay(200);
}

void loop() {
  unsigned long now = millis();

  // 1) Commands first
  processUARTCommands();

  // 2) Sensors @20Hz
  if (now - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    updateAllSensorsSequential();
    lastSensorUpdate = now;
  }

  // 3) Update safety thresholds/zones using current requested speed
  updateSafetyZones((uint8_t)abs(motors.speed));

  // 4) Apply safety + drive outputs
  applySafetyAndDrive();

  // 5) Command timeout (unchanged)
  if (now - lastCommandTime > 100000 && lastCommandTime > 0) {
    motors.speed = 0;
  }

  // 6) Telemetry rate
  if (now - lastSensorSend >= SENSOR_SEND_INTERVAL) {
    sendSensorData();
    // Safety telemetry (lightweight)
    Serial1.print("SAFETY,FF:");
    Serial1.print(safety.front_min);
    Serial1.print(",FR:");
    Serial1.print((int)safety.soft_front_on);
    Serial1.print((int)safety.hard_front_on);
    Serial1.print(",RR:");
    Serial1.print(safety.rear_dist);
    Serial1.print(",RS:");
    Serial1.print((int)safety.soft_rear_on);
    Serial1.print((int)safety.hard_rear_on);
    Serial1.print(",FORCE:");
    Serial1.print((int)safety.force_override);
    Serial1.print(",E:");
    Serial1.print((int)safety.estop_latched);
    Serial1.print(",BF:");
    Serial1.print((int)safety.blocked_front);
    Serial1.print(",BR:");
    Serial1.println((int)safety.blocked_rear);

    lastSensorSend = now;
  }
}

// ---------------- INIT ----------------
void initializeMotors() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  setMotorSpeedRaw(0);
  Serial.println("TB6612 motor driver initialized");
}

void initializeHCSR04() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(ECHO_PIN_3, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  Serial.println("HCSR04 sensors initialized");
}

void initializeVL53L0X() {
  Wire.begin();
  Wire.setClock(400000);
  for (int attempt = 0; attempt < 3; attempt++) {
    if (vl53Sensor.init()) {
      vl53Sensor.setTimeout(30);
      vl53Sensor.setMeasurementTimingBudget(20000);
      vl53_initialized = true;
      Serial.println("VL53L0X initialized (fast mode)");
      Serial1.println("STATUS,VL53L0X initialized");
      return;
    }
    Serial.print("VL53L0X init attempt ");
    Serial.println(attempt + 1);
    delay(100);
  }
  Serial.println("VL53L0X sensor not found - using fallback");
  Serial1.println("ERROR,VL53L0X sensor not found");
  vl53_initialized = false;
  sensors.vl53_distance = 255;
}

// ---------------- SENSORS ----------------
void updateAllSensorsSequential() {
  timing_sensor_start = micros();

  timing_vl53_start = micros();
  sensors.vl53_distance = readVL53L0X();
  timing_vl53_end = micros();

  sensors.hcsr04_left = readHCSR04(ECHO_PIN_1);
  delay(20);
  sensors.hcsr04_back = readHCSR04(ECHO_PIN_2);
  delay(20);
  sensors.hcsr04_right = readHCSR04(ECHO_PIN_3);

  timing_sensor_end = micros();


  bad_front = (sensors.vl53_distance < OBSTACLE_THRESHOLD_CM && sensors.vl53_distance > 0 && sensors.vl53_distance < 255) ? min<uint8_t>(bad_front + 1, 255) : 0;
  bad_left  = (sensors.hcsr04_left   < OBSTACLE_THRESHOLD_CM && sensors.hcsr04_left   > 0 && sensors.hcsr04_left   < 255) ? min<uint8_t>(bad_left  + 1, 255) : 0;
  bad_right = (sensors.hcsr04_right  < OBSTACLE_THRESHOLD_CM && sensors.hcsr04_right  > 0 && sensors.hcsr04_right  < 255) ? min<uint8_t>(bad_right + 1, 255) : 0;
  bad_back  = (sensors.hcsr04_back   < OBSTACLE_THRESHOLD_CM && sensors.hcsr04_back   > 0 && sensors.hcsr04_back   < 255) ? min<uint8_t>(bad_back  + 1, 255) : 0;

  sensors.obstacle_detected = (bad_front >= BAD_READS_THRESHOLD) ||
                              (bad_left  >= BAD_READS_THRESHOLD) ||
                              (bad_right >= BAD_READS_THRESHOLD) ||
                              (bad_back  >= BAD_READS_THRESHOLD);

  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    unsigned long total_time = timing_sensor_end - timing_sensor_start;
    unsigned long vl53_time = timing_vl53_end - timing_vl53_start;

    Serial.print("TIMING: Total=");
    Serial.print(total_time);
    Serial.print("μs, VL53=");
    Serial.print(vl53_time);
    Serial.print("μs | Values: VL53=");
    Serial.print(sensors.vl53_distance);
    Serial.print("cm, L=");
    Serial.print(sensors.hcsr04_left);
    Serial.print("cm, R=");
    Serial.print(sensors.hcsr04_right);
    Serial.print("cm, B=");
    Serial.print(sensors.hcsr04_back);
    Serial.println("cm");

    Serial1.print("TIMING,ARDUINO_SENSORS,");
    Serial1.println(total_time);
    lastDebug = millis();
  }
}

uint8_t readVL53L0X() {
  if (!vl53_initialized) return 255;
  uint16_t distance_mm = vl53Sensor.readRangeSingleMillimeters();
  if (vl53Sensor.timeoutOccurred()) {
    Serial.println("VL53L0X timeout");
    return 255;
  }
  uint16_t distance_cm = distance_mm / 10;
  if (distance_cm > 200) return 255;
  return (distance_cm > 255) ? 255 : (uint8_t)distance_cm;
}

uint8_t readHCSR04(int echoPin) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT_US);
  if (duration == 0) return 255;
  uint16_t distance_cm = duration / 58;
  if (distance_cm < 2) return 2;
  if (distance_cm > MAX_DISTANCE_CM) return 255;
  return (uint8_t)distance_cm;
}

// ---------------- SAFETY MANAGER ----------------
uint8_t fusedMin(uint8_t a, uint8_t b) {
  if (a == 255) return b;
  if (b == 255) return a;
  return min(a, b);
}
uint8_t fusedFrontDistance() {

  uint8_t fl = sensors.hcsr04_left;
  uint8_t fr = sensors.hcsr04_right;
  uint8_t fc = sensors.vl53_distance;

  uint8_t lr_min = (fl == 255 && fr == 255) ? 255 : min(fl, fr);
  return fusedMin(fc, lr_min);
}

void updateSafetyThresholds(uint8_t /*pwm_abs*/) {

  safety.soft_front = FC_SOFT_CM;   
  safety.hard_front = FC_HARD_CM;
  safety.soft_rear  = REAR_SOFT_CM;
  safety.hard_rear  = REAR_HARD_CM;
}

void updateSafetyZones(uint8_t pwm_abs) {
  updateSafetyThresholds(pwm_abs);

  // Read distances
  uint8_t fc = sensors.vl53_distance;   // front center
  uint8_t fl = sensors.hcsr04_left;     // front-left
  uint8_t fr = sensors.hcsr04_right;    // front-right
  uint8_t rb = sensors.hcsr04_back;     // rear

  // For display/telemetry (fused front min)
  uint8_t lr_min = (fl == 255 && fr == 255) ? 255 : min(fl, fr);
  safety.front_min = (fc == 255) ? lr_min : ((lr_min == 255) ? fc : min(fc, lr_min));
  safety.rear_dist = rb;

  // --- FRONT soft/hard logic (any sensor triggers the harsher level) ---
  bool front_hard_now = false;
  bool front_soft_now = false;

  // center thresholds
  if (fc != 255) {
    if (fc <= FC_HARD_CM) front_hard_now = true;
    else if (fc <= FC_SOFT_CM) front_soft_now = true;
  }
  // left thresholds
  if (fl != 255) {
    if (fl <= FS_HARD_CM) front_hard_now = true;
    else if (fl <= FS_SOFT_CM) front_soft_now = true;
  }
  // right thresholds
  if (fr != 255) {
    if (fr <= FS_HARD_CM) front_hard_now = true;
    else if (fr <= FS_SOFT_CM) front_soft_now = true;
  }

  // --- REAR soft/hard (single sensor) ---
  bool rear_hard_now = false;
  bool rear_soft_now = false;
  if (rb != 255) {
    if (rb <= REAR_HARD_CM) rear_hard_now = true;
    else if (rb <= REAR_SOFT_CM) rear_soft_now = true;
  }

  // Debounce counters
  if (front_soft_now) safety.sf_cnt = min<uint8_t>(safety.sf_cnt + 1, 255); else if (safety.sf_cnt > 0) safety.sf_cnt--;
  if (front_hard_now) safety.hf_cnt = min<uint8_t>(safety.hf_cnt + 1, 255); else if (safety.hf_cnt > 0) safety.hf_cnt--;

  if (rear_soft_now)  safety.sr_cnt = min<uint8_t>(safety.sr_cnt + 1, 255); else if (safety.sr_cnt > 0) safety.sr_cnt--;
  if (rear_hard_now)  safety.hr_cnt = min<uint8_t>(safety.hr_cnt + 1, 255); else if (safety.hr_cnt > 0) safety.hr_cnt--;


  if (safety.sf_cnt >= DEBOUNCE_FRAMES) safety.soft_front_on = true;
  if (safety.sf_cnt == 0)               safety.soft_front_on = false;

  if (safety.hf_cnt >= DEBOUNCE_FRAMES) {
    if (!safety.hard_front_on) safety.hard_front_since_ms = millis();
    safety.hard_front_on = true;
  }
  if (safety.hf_cnt == 0) safety.hard_front_on = false;

  if (safety.sr_cnt >= DEBOUNCE_FRAMES) safety.soft_rear_on = true;
  if (safety.sr_cnt == 0)               safety.soft_rear_on = false;

  if (safety.hr_cnt >= DEBOUNCE_FRAMES) {
    if (!safety.hard_rear_on) safety.hard_rear_since_ms = millis();
    safety.hard_rear_on = true;
  }
  if (safety.hr_cnt == 0) safety.hard_rear_on = false;

  // Blocked flags (for telemetry only)
  unsigned long now = millis();
  safety.blocked_front = safety.hard_front_on && (motors.speed > 5) &&
                         (now - safety.hard_front_since_ms > BLOCKED_LATCH_MS);
  safety.blocked_rear  = safety.hard_rear_on  && (motors.speed < -5) &&
                         (now - safety.hard_rear_since_ms  > BLOCKED_LATCH_MS);
}


int clampSpeedWithSafety(int requested) {
  if (safety.estop_latched) return 0;

  int req = requested;

  // Forward gating
  if (req > 0) {
    if (safety.hard_front_on) return 0;  // hard stop
    if (safety.soft_front_on) {
      if (safety.force_override) {
        // FORCE bypasses the half-speed cap in soft zone
        return req;
      } else {
        // Half-speed cap (±127)
        int capped = min(req, (int)HALF_PWM_CAP);
        return capped;
      }
    }
    return req; // clear → full
  }

  // Reverse gating
  if (req < 0) {
    if (safety.hard_rear_on) return 0;   // hard stop
    if (safety.soft_rear_on) {
      // Mirror front behavior: half-speed cap in reverse
      int mag = (-req);
      int capped = min(mag, (int)HALF_PWM_CAP);
      return -capped;
    }
    return req; // clear → full
  }

  return 0;
}


void applySafetyAndDrive() {
  // Apply steering immediately (unchanged)
  steeringServo.write(motors.angle);

  // Gated speed
  int safe_pwm = clampSpeedWithSafety(motors.speed);

  // Drive the motors
  setMotorSpeedRaw(safe_pwm);
}

// ---------------- UART / COMMANDS ----------------
void processUARTCommands() {
  if (!Serial1.available()) return;
  String command = Serial1.readStringUntil('\n');
  command.trim();
  if (command.length() == 0) return;
  lastCommandTime = millis();
  Serial.println("Received: " + command);

  if (command.startsWith("MOVE,")) {
    parseMoveCommand(command);
  } else if (command == "STOP") {
    motors.speed = 0;
    safety.estop_latched = true;  // STOP latches e-stop until next MOVE/SERVO clears it
    setMotorSpeedRaw(0);
    Serial.println("STOP command received (Stop latched)");
    Serial1.println("ACK,STOP");
  } else if (command == "STATUS") {
    sendStatusUpdate();
  } else if (command.startsWith("SERVO,")) {
    parseServoCommand(command);
  } else if (command.startsWith("FORCE,")) {
    // new simple override toggle
    int val = command.substring(6).toInt();
    safety.force_override = (val != 0);
    Serial1.println(String("ACK,FORCE,") + (safety.force_override ? "1" : "0"));
  } else {
    Serial.println("Unknown command: " + command);
    Serial1.println("ERROR,Unknown command: " + command);
  }
}

// MOVE,speed,angle[,flags]
// flags may include 'F' to enable force override just for this command frame
void parseMoveCommand(String command) {
  // split
  int c1 = command.indexOf(',');
  int c2 = command.indexOf(',', c1 + 1);
  if (c1 == -1 || c2 == -1) {
    Serial.println("Invalid MOVE format");
    Serial1.println("ERROR,Invalid MOVE format");
    return;
  }

  int speed = command.substring(c1 + 1, c2).toInt();
  int nextComma = command.indexOf(',', c2 + 1);
  String angleStr, flagsStr;
  if (nextComma == -1) {
    angleStr = command.substring(c2 + 1);
  } else {
    angleStr = command.substring(c2 + 1, nextComma);
    flagsStr = command.substring(nextComma + 1);
    flagsStr.trim();
  }

  speed = constrain(speed, -255, 255);
  int angle = angleStr.toInt();
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  // if flags include 'F', momentarily enable force override
  bool force_from_move = (flagsStr.length() > 0 && flagsStr.indexOf('F') != -1);

  // clear e-stop on active drive command
  safety.estop_latched = false;

  motors.speed = speed;
  motors.angle = angle;
  steeringServo.write(angle);

  // apply temporary force only for this cycle (non-latching)
  bool previous_force = safety.force_override;
  if (force_from_move) safety.force_override = true;

  // drive via safety layer
  int safe_pwm = clampSpeedWithSafety(motors.speed);
  setMotorSpeedRaw(safe_pwm);

  // restore persistent force flag
  if (force_from_move) safety.force_override = previous_force;

  Serial.println("MOVE: Speed=" + String(speed) + ", Angle=" + String(angle) + (force_from_move ? " [F]" : ""));
  Serial1.println("ACK,MOVE," + String(speed) + "," + String(angle));
}

void parseServoCommand(String command) {
  int commaIndex = command.indexOf(',');
  if (commaIndex == -1) {
    Serial.println("Invalid SERVO format");
    Serial1.println("ERROR,Invalid SERVO format");
    return;
  }
  int angle = command.substring(commaIndex + 1).toInt();
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  motors.angle = angle;
  steeringServo.write(angle);

  // touching servo does not clear e-stop by itself (keeps STOP behavior predictable)
  Serial.println("SERVO: Angle=" + String(angle));
  Serial1.println("ACK,SERVO," + String(angle));
}

// ---------------- MOTOR DRIVER (RAW) ----------------
void setMotorSpeedRaw(int speed) {
  if (speed == 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    analogWrite(PWMA, 0); analogWrite(PWMB, 0);
    return;
  }
  int pwm_value = abs(speed);
  bool forward = (speed > 0);

  if (forward) {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  } else {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMA, pwm_value);
  analogWrite(PWMB, pwm_value);
}

// ---------------- TELEMETRY ----------------
void sendSensorData() {
  Serial1.print("SENSORS,");
  Serial1.print(sensors.vl53_distance);
  Serial1.print(",");
  Serial1.print(sensors.hcsr04_left);
  Serial1.print(",");
  Serial1.print(sensors.hcsr04_right);
  Serial1.print(",");
  Serial1.println(sensors.hcsr04_back);

  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 3000) {
    Serial.print("Sensors: VL53=");
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

void sendStatusUpdate() {
  String status = "STATUS,Speed:" + String(motors.speed) +
                  ",Angle:" + String(motors.angle) +
                  ",Emergency:" + String(safety.estop_latched ? "true" : "false") +
                  ",Obstacle:" + String(sensors.obstacle_detected ? "true" : "false") +
                  ",VL53:" + String(vl53_initialized ? "OK" : "FAIL");
  Serial1.println(status);
  Serial.println(" " + status);
}
