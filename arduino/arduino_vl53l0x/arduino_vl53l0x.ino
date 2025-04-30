#include <Adafruit_VL53L0X.h>

// VL53L0X Sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// HC-SR04 Pins
const int trigPins[3] = {2, 4, 6};
const int echoPins[3] = {3, 5, 7};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1);  // Wait for native USB

  // Initialize VL53L0X
  Serial.println("VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  Serial.println(F("VL53L0X API Simple Ranging \n\n"));

  // Setup HC-SR04 Pins
  for (int i = 0; i < 3; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

void loop() {
  // VL53L0X Reading
  VL53L0X_RangingMeasurementData_t measure;
  Serial.print("VL53L0X  (mm): ");
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println("Izvan dometa");
  }

  // HC-SR04 Readings
  for (int i = 0; i < 3; i++) {
    long duration, distance;

    // Trigger the sensor
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);

    // Read echo
    duration = pulseIn(echoPins[i], HIGH);

    //  distance in cm
    distance = duration * 0.034 / 2;

    // output cleanup
    Serial.write(27);       
    Serial.print("[2J");     
    Serial.write(27);
    Serial.print("[H");      

    // output
    Serial.print("HC-SR04 #");
    Serial.print(i + 1);
    Serial.print("  (cm): ");
    Serial.println(distance);
  }

  Serial.println("-------------------------------");
  delay(300);
}
