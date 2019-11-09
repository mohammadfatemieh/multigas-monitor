/*
 * Logs all data from environmental sensors to SD card
 * Last tested: 18-August-2019 with VAMoS RevC
 */

// Libraries for SD card
#include "FS.h"
#include "SD.h"
#include <SPI.h>

//DS18B20 libraries
#include <OneWire.h>
#include <DallasTemperature.h>

// Libraries to get time from NTP Server
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include <Wire.h>

// HIH6120 library
#include <HIH61XX.h>

// GPS library
#include <Adafruit_GPS.h>

// BME280 libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Define deep sleep options
uint64_t uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds
// Sleep for 1 minutes = 60 seconds
uint64_t TIME_TO_SLEEP = 60;

// Replace with your network credentials
const char* ssid     = "xiami";
const char* password = "viv19952019";

// Define CS pin for the SD card module
#define SD_CS 5

#define BME_SCK 22
//#define BME_MISO 12
#define BME_MOSI 21
//#define BME_CS 10

Adafruit_BME280 bme; // I2C

#define SEALEVELPRESSURE_HPA (1013.25)
#define PUMP 26
#define ALPHS_EN 15

// Save reading number on RTC memory
RTC_DATA_ATTR int readingID = 0;
RTC_DATA_ATTR int fileNumber = 0;

String dataMessage;

// Create an HIH61XX with I2C address 0x27, powered by 3.3V pin
HIH61XX hih(0x27);

// Data wire is connected to ESP32 GPIO 27
#define ONE_WIRE_BUS 27
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

/***** GPS DEFINITIONS ****/
#define GPSSerial Serial2
#define RXD2 16
#define TXD2 17
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();
int delayTime = 1000;

// Environmental Sensor variables
float DS18B20_temp;
float bme_t;
float bme_rh;
float bme_p;
float hih_rh;
float hih_t;

// Analog sensor variables
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

// Parameters for pump PWM
const int freq = 500;
const int channel = 0;
const int resolution = 8;

// Define NTP Client to get time
// VIV'S NOTE: Will we actually have access to an NTP server on the glacier?
// Probably not, hence will have to use the GPS
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

void setup() {
  // Start serial communication for debugging purposes
  Serial.begin(115200);
  pinMode(PUMP,OUTPUT);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(PUMP, channel);
  pinMode(ALPHS_EN,OUTPUT); // set enable pin as output
  ledcWrite(channel,255); // set pump to max duty cycle
  delay(3000); // keep high for 3 seconds
  
  // Note the format for setting a serial port is as follows:
  //Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz
    
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);
    delay(delayTime);

//  // Connect to Wi-Fi network with SSID and password
//  /*Serial.print("Connecting to ");
//  Serial.println(ssid);
//  WiFi.begin(ssid, password); // want access point to be open, so password set to NULL
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
//  Serial.println("");
//  Serial.println("WiFi connected.");*/
//
//  // Initialize a NTPClient to get time
//  /*
//  timeClient.begin();
//  // Set offset time in seconds to adjust for your timezone, for example:
//  // GMT +1 = 3600
//  // GMT +8 = 28800
//  // GMT -1 = -3600
//  // GMT 0 = 0
//  timeClient.setTimeOffset(-25200); // PST is behing GMT by 7 hours (7x3600=25200)*/
//  initializeSD();
//
//  // If the data.txt file doesn't exist
//  // Create a file on the SD card and write the data labels
//  File file = SD.open("/VAMoS EnvData.txt");
//  if(!file) {
//    Serial.println("File doens't exist");
//    Serial.println("Creating file...");
//    //writeFile(SD, "/VAMoS EnvData.txt", "Reading ID, Date, Hour, Temperature \r\n");
//    writeFile(SD, "/VAMoS EnvData.txt", "Reading ID, DS18B20, BME280_T, HIH_T, BME280_RH, HIH_RH, BME280_P \r\n");
//  }
//  else {
//    Serial.println("File already exists");  
//  }
//  file.close();
//
//  // Enable Timer wake_up
//  //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
//
//  // Start the DallasTemperature library
//  sensors.begin(); 
//  Wire.begin();
//
//  unsigned status;
//
//  // SETTING UP THE BME280
//  status = bme.begin();
//
//  // SETTING UP THE HIH SENSOR
//  hih.start();
//
//  DS18B20_Readings();
//  BME280_Readings();
//  HIH6120_Readings();
//  //getTimeStamp();
//  logSDCard();
//  
//  // Increment readingID on every new reading
//  readingID++;
//  
//  // Start deep sleep
//  //Serial.println("DONE! Going to sleep now.");
//  //esp_deep_sleep_start(); 
}

void loop() {
  // Enable Timer wake_up
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  while (millis() < 10000){
      ParseGPS(); 
  }

  initializeSD();

  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/VAMoS EnvData.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    //writeFile(SD, "/VAMoS EnvData.txt", "Reading ID, Date, Hour, Temperature \r\n");
    writeFile(SD, "/VAMoS EnvData.txt", "Reading ID, Date, Time [UTC], DS18B20 [C], BME280_T [C], HIH_T [C], BME280_RH [%], HIH_RH [%], BME280_P [hPa] \r\n");
  }
  else {
    //fileNumber++;
    Serial.println("File already exists");
    //writeFile(SD, "/VAMoS EnvData.txt" + fileNumber, "Reading ID, Date, Time [UTC], DS18B20 [C], BME280_T [C], HIH_T [C], BME280_RH [%], HIH_RH [%], BME280_P [hPa] \r\n");       
  }
  file.close();

  // Start the DallasTemperature library
  sensors.begin(); 
  Wire.begin();

  unsigned status;

  // SETTING UP THE BME280
  status = bme.begin();

  // SETTING UP THE HIH SENSOR
  hih.start();

  DS18B20_Readings();
  BME280_Readings();
  HIH6120_Readings();
  //getTimeStamp();
  delay(5000);
  ParseGPS();
  logSDCard();
  
  // Increment readingID on every new reading
  readingID++;
  
  // Start deep sleep
  Serial.println("DONE! Going to sleep now.");
  esp_deep_sleep_start(); 
}

void initializeSD(){
  // Initialize SD card
  SD.begin(SD_CS);  
  if(!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
    return;    // init failed
  }
}

// Function to get temperature
void DS18B20_Readings(){
  sensors.requestTemperatures(); 
  DS18B20_temp = sensors.getTempCByIndex(0); // Temperature in Celsius
  //temperature = sensors.getTempFByIndex(0); // Temperature in Fahrenheit
  Serial.print("DS18B20 Temp: ");
  Serial.println(DS18B20_temp);
}

void BME280_Readings() {
    Serial.print("BME280 Temperature = ");
    bme_t = bme.readTemperature();
    Serial.print(bme_t);
    Serial.println(" *C");

    Serial.print("BME280 Pressure = ");
    bme_p = bme.readPressure() / 100.0F;
    Serial.print(bme_p);
    Serial.println(" hPa");

    Serial.print("BME280 Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("BME280 Humidity = ");
    bme_rh = bme.readHumidity();
    Serial.print(bme_rh);
    Serial.println(" %");

    Serial.println();
}

void HIH6120_Readings(){
    hih.update();
    
    Serial.print("HIH6120 Humidity: ");
    hih_rh = hih.humidity()*100;
    Serial.print(hih_rh, 5);
    Serial.println(" %");
    Serial.print("HIH6120 RH Raw: ");
    Serial.println(hih.humidity_Raw());
    
    Serial.print("Ext HIH Temperature: ");
    hih_t = hih.temperature();
    Serial.print(hih_t, 5);
    Serial.println(" *C");
    Serial.print("HIH6120 Temp Raw: ");
    Serial.println(hih.temperature_Raw());

    Serial.println();
}

// Function to get date and time from NTPClient
void getTimeStamp() {
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = timeClient.getFormattedDate();
  Serial.println(formattedDate);

  // Extract date
  int splitT = formattedDate.indexOf("T");
  dayStamp = formattedDate.substring(0, splitT);
  Serial.println(dayStamp);
  // Extract time
  timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);
  Serial.println(timeStamp);
}

// Write the sensor readings on the SD card
void logSDCard() {
  //dataMessage = String(readingID) + "," + String(dayStamp) + "," + String(timeStamp) + "," + String(temperature) + "\r\n";
  dataMessage = String(readingID) + "," + String(GPS.day) + "-" + String(GPS.month) + "-" + String(GPS.year) + "," + String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + ":" + String(GPS.milliseconds) + "," 
  + String(DS18B20_temp) + "," + String(bme_t) + "," + String(hih_t) + "," + String(bme_rh) + "," + String(hih_rh) + "," + String(bme_p) + "\r\n";
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/VAMoS EnvData.txt", dataMessage.c_str());
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void ParseGPS(){
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.print(GPS.milliseconds);
    Serial.println(" UTC");
    
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
 }
