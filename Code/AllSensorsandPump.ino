/*
  ReadAnalogVoltage

  Reads an analog input on pin 0, converts it to voltage, and prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/ReadAnalogVoltage
*/

float ALPHS3_ADC;
float ALPHS4_ADC;
float ALPHS5_ADC;
float ALPHS6_ADC;
float ALPHS7_ADC;
float ALPHS8_ADC;

float ALPHS3_volts;
float ALPHS4_volts;
float ALPHS5_volts;
float ALPHS6_volts;
float ALPHS7_volts;
float ALPHS8_volts;

#define PUMP 26
#define ALPHS_EN 15

const int freq = 500;
const int channel = 0;
const int resolution = 8;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(PUMP,OUTPUT);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(PUMP, channel);
  pinMode(ALPHS_EN,OUTPUT); // set enable pin as output
  
  pinMode(33,OUTPUT);
  pinMode(32,OUTPUT);
  pinMode(36,OUTPUT); // test the SENSOR_VN pin
  ledcWrite(channel,255); // set to max duty cycle
  delay(3000); // keep high for 3 seconds
}

// the loop routine runs over and over again forever:
void loop() {
  ledcWrite(channel,255); // 50% duty cycle
  delay(2000);
  //digitalWrite(ALPHS_EN,HIGH);
  //digitalWrite(ALPHS_EN,LOW);

  ledcWrite(channel,200);
  delay(2000);
  while (millis() < 5000) {
    digitalWrite(ALPHS_EN,LOW); // turn off the sensors
    Serial.println("Sensors turned off...");
  }
  
  digitalWrite(ALPHS_EN,HIGH); // turn on the sensors
  digitalWrite(36,HIGH);
  Serial.println("Sensors turned on....");
  
  digitalWrite(32,HIGH); // test the IO pins
  Serial.println("Turning LED on...");
  delay(1000);
  digitalWrite(32,LOW); 
  Serial.println("Turning LED off...");
  delay(1000);
  digitalWrite(33,HIGH); // test the IO pins
  Serial.println("Turning LED on...");
  delay(1000);
  digitalWrite(33,LOW); 
  Serial.println("Turning LED off...");
  delay(1000);
  
  // read the input on analog GPIO pins
  ALPHS3_ADC = analogRead(4);
  ALPHS4_ADC = analogRead(12);
  ALPHS5_ADC = analogRead(13);
  //ALPHS6_ADC = analogRead(14);
  //ALPHS7_ADC = analogRead(15);
  ALPHS8_ADC = analogRead(25);
  
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.3V):
  ALPHS3_volts = ALPHS3_ADC * (3.3 / 1023.0);
  ALPHS4_volts = ALPHS4_ADC * (3.3 / 1023.0);
  ALPHS5_volts = ALPHS5_ADC * (3.3 / 1023.0);
  ALPHS6_volts = ALPHS6_ADC * (3.3 / 1023.0);
  ALPHS7_volts = ALPHS7_ADC * (3.3 / 1023.0);
  ALPHS8_volts = ALPHS8_ADC * (3.3 / 1023.0);

  // print out the value you read:
  Serial.print("Sensor 3 is: ");
  Serial.println(ALPHS3_volts);
  
  Serial.print("Sensor 4 is: ");
  Serial.println(ALPHS4_volts);
  
  Serial.print("Sensor 5 is: ");
  Serial.println(ALPHS5_volts);
  
//  Serial.print("Sensor 6 is: ");
//  Serial.println(ALPHS6_volts);
//  
//  Serial.print("Sensor 7 is: ");
//  Serial.println(ALPHS7_volts);  

  Serial.print("Sensor 8 is: ");
  Serial.println(ALPHS8_volts);
  delay(1000);
}
