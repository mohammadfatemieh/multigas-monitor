void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Serial setup successful");

}

void loop() {
  // put your main code here, to run repeatedly:
  pinMode(32,OUTPUT);
  digitalWrite(32,HIGH);
  delay(500);
  digitalWrite(32,LOW);
  delay(500);
  Serial.println("Setting IO32 HIGH");
  Serial.println("THIS BOARD IS SEX");
  delay(1000);
  
}
