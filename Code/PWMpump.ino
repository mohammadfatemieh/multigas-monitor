void setup() {
  pinMode(10,OUTPUT);
  analogWrite(10,255); // high
  delay(3000); // keep high for 3 seconds
}

void loop() {
  analogWrite(10,100); // 50% duty cycle
  delay(5);
}
