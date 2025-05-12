#include <Adafruit_VL53L0X.h>
#include <Servo.h>

// VL53L0X Sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
const int XSHUT_PIN = 8;  // XSHUT for hardware reset

// HC-SR04 Pins
const int trigPins[3] = {2, 4, 6};
const int echoPins[3] = {3, 5, 7};

// sg90
Servo sg90;
const int SERVO_PIN = 10;

void setup() {
  Serial1.begin(115200);  // Only using Serial1 (pins 0=RX, 1=TX)

  // Reset VL53L0X via XSHUT
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);
  delay(10);
  digitalWrite(XSHUT_PIN, HIGH);
  delay(10);

  // Init VL53L0X
  if (!lox.begin()) {
    Serial1.println(F("Failed to boot VL53L0X"));
    while (1);
  }


  // Init HC-SR04
  for (int i = 0; i < 3; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  sg90.attach(SERVO_PIN);
  sg90.write(180);
}

void sendStructuredData(uint16_t vl53, uint16_t hc1, uint16_t hc2, uint16_t hc3) {
  String payload = "{";
  payload += "\"vl53\":" + String(vl53) + ",";
  payload += "\"ultrasonic\":[" + String(hc1) + "," + String(hc2) + "," + String(hc3) + "]";
  payload += "}";
  Serial1.println(payload);
}

void logData(const String& msg) {
  Serial1.println("[" + String(millis()) + "ms] " + msg);
}

void checkSerial1Commands(){
  static String input = "";

  while (Serial1.available()){
    char c = Serial1.read();
    if (c=='\n' || c=='\r'){
      input.trim();

      if(input.startsWith("SERVO")){
        int angle = input.substring(6).toInt();
        angle = constrain(angle, 0, 180);
        sg90.write(angle);
      } else {
        logData("Unknown command: "+input);
      }
      input = "";
    } else {
      input+=c;
    }
  } 
}

void loop() {
  static String input = "";
  unsigned long start = millis();

  // === 1. Handle incoming UART commands ===
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n' || c == '\r') {
      input.trim();

      if (input.length() > 0) {
        if (input.startsWith("SERVO")) {
          int angle = input.substring(6).toInt();
          angle = constrain(angle, 0, 180);
          sg90.write(angle);
        } else {
          Serial1.println("{\"error\":\"Unknown command: " + input + "\"}");
        }
      }

      input = "";
    } else {
      input += c;
    }
  }

  // === 2. Read VL53L0X sensor ===
  uint16_t vl53_val = 0xFFFF;
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    vl53_val = measure.RangeMilliMeter;
  }

  // === 3. Read 3x HC-SR04 ===
  long distances[3] = {0};

  for (int i = 0; i < 3; i++) {
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);

    long duration = pulseIn(echoPins[i], HIGH, 20000);  // timeout 20ms
    distances[i] = duration * 0.034 / 2;
  }

  // === 4. Send JSON over UART ===
  String payload = "{";
  payload += "\"status\":\"alive\",";
  payload += "\"vl53\":" + String(vl53_val) + ",";
  payload += "\"ultrasonic\":[" + String(distances[0]) + "," + String(distances[1]) + "," + String(distances[2]) + "]";
  payload += "}";

  Serial1.println(payload);
  Serial1.flush();  // ensure it's fully sent

  // === 5. Wait to maintain 100ms loop ===
  unsigned long elapsed = millis() - start;
  if (elapsed < 100) {
    delay(100 - elapsed);
  }
}


