/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>

//BME280 libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//DS18B20 libraries
#include <OneWire.h>
#include <DallasTemperature.h>

//HIH6120 library
#include <HIH61XX.h>

//  Create an HIH61XX with I2C address 0x27, powered by 3.3V pin
HIH61XX hih(0x27);

// Data wire is connected to ESP32 GPIO 27
#define ONE_WIRE_BUS 27
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// Temperature Sensor variables
float ext_temp;

#define BME_SCK 22
//#define BME_MISO 12
#define BME_MOSI 21
//#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

// pin defintions for GPS UART
#define RXD2 16
#define TXD2 17

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

void setup() {
    // SETTING UP THE GPS
    // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
    Serial.begin(115200);
    //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    Serial.println("Serial Txd is on pin: "+String(TX));
    Serial.println("Serial Rxd is on pin: "+String(RX));
  
    //Serial.begin(9600);
    while(!Serial);    // time to get serial running
    //Serial.println(F("BME280 test"));
    
    // SETTING UP THE DS18B20 SENSOR
    sensors.begin();
 
    Wire.begin();

    unsigned status;
    
    // SETTING UP THE BME280
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1);
    }

    // SETTING UP THE HIH SENSOR
    hih.start();

    delayTime = 1000;
    Serial.println();
}

void loop() {
    // Printing the output of the UART2 for the GPS
    //Choose Serial1 or Serial2 as required
    while (Serial2.available()) {
      Serial.print(char(Serial2.read()));
      }

    // Printing the output of the environmental sensors
    BME280_Readings();
    DS18B20_Readings();
    HIH6120_Readings();

    //while(true);

    delay(delayTime);
}

void BME280_Readings() {
    Serial.print("BME280 Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("BME280 Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("BME280 Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("BME280 Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void DS18B20_Readings(){
  sensors.requestTemperatures(); 
  ext_temp = sensors.getTempCByIndex(0); // Temperature in Celsius
  //ext_temp = sensors.getTempFByIndex(0); // Temperature in Fahrenheit
  Serial.print("DS18B20 Temp: ");
  Serial.print(ext_temp);
  Serial.println(" *C");
  
  Serial.println();
}

void HIH6120_Readings(){
    hih.update();
    
    Serial.print("HIH6120 Humidity: ");
    Serial.print(hih.humidity()*100, 5);
    Serial.println(" %");
    Serial.print("HIH6120 RH Raw: ");
    Serial.println(hih.humidity_Raw());
    
    Serial.print("Ext HIH Temperature: ");
    Serial.print(hih.temperature(), 5);
    Serial.println(" *C");
    Serial.print("HIH6120 Temp Raw: ");
    Serial.println(hih.temperature_Raw());
}
