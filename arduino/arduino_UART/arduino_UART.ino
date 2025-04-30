void setup() {
  Serial1.begin(9600);
}

void loop() {
  Serial1.println("PING FROM ARDUINO");
  delay(1000);
}
