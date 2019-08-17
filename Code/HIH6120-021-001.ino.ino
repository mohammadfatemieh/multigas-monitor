#include <Wire.h>
#include <HIH61XX.h>

//  Create an HIH61XX with I2C address 0x27, powered by 3.3V pin

HIH61XX hih(0x27);

void setup()

{
  Serial.begin(9600);
  Wire.begin();
}

void loop()

{
  //  start the sensor
  hih.start();
  //  request an update of the humidity and temperature

  hih.update();
  Serial.println("Humidity: ");
  Serial.println(hih.humidity(), 5);
  Serial.println(" RH (");
  Serial.println(hih.humidity_Raw());
  Serial.println(")");
  Serial.println("Temperature: ");
  Serial.println(hih.temperature(), 5);
  Serial.println(" C (");
  Serial.println(hih.temperature_Raw());
  Serial.println(")");
  while(true);
}
