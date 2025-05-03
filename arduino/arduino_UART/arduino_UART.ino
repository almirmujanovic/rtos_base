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
  sg90.write(90);
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

  checkSerial1Commands(); // read tcp -> uart

  // VL53L0X reading
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    logData("VL53L0X  (mm): " + String(measure.RangeMilliMeter));
  } else {
    logData("VL53L0X  (mm): Izvan dometa");
  }

  // HC-SR04 readings
  for (int i = 0; i < 3; i++) {
    long duration, distance;

    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);

    duration = pulseIn(echoPins[i], HIGH);
    distance = duration * 0.034 / 2;

    logData("HC-SR04 #" + String(i + 1) + " (cm): " + String(distance));
  }

  logData("-------------------------------");
  delay(500);
}
