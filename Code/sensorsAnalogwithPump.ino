/*
  ReadAnalogVoltage

  Reads an analog input on pin 0, converts it to voltage, and prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/ReadAnalogVoltage
*/

float ALPHS4_ADC;
float ALPHS5_ADC;
float ALPHS4_volts;
float ALPHS5_volts;
#define PUMP 26
#define ALPHS_EN 35

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
  delay(1000);
  digitalWrite(ALPHS_EN,LOW); // turn off the sensors
  Serial.println("Sensors turned off...");
  delay(2000);
  digitalWrite(ALPHS_EN,HIGH); // turn on the sensors
  Serial.println("Sensors turned on....");
  digitalWrite(32,HIGH); // test the IO pins
  Serial.println("Turning IO36 on...");
  delay(1000);
  digitalWrite(32,LOW); 
  Serial.println("Turning IO36 off...");
  delay(1000);
  digitalWrite(33,HIGH); // test the IO pins
  Serial.println("Turning IO36 on...");
  delay(1000);
  digitalWrite(33,LOW); 
  Serial.println("Turning IO36 off...");
  delay(1000);
  
  // read the input on analog pin
  ALPHS4_ADC = analogRead(13);
  ALPHS5_ADC = analogRead(14);
  
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.3V):
  ALPHS4_volts = ALPHS4_ADC * (3.3 / 1023.0);
  ALPHS5_volts = ALPHS5_ADC * (3.3 / 1023.0);
  
  // print out the value you read:
  Serial.print("Sensor 4 is: ");
  Serial.println(ALPHS4_volts);
  Serial.print("Sensor 5 is: ");
  Serial.println(ALPHS5_volts);
  delay(1000);
}
