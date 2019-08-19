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

// BME280 libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Define deep sleep options
uint64_t uS_TO_S_FACTOR = 1000000;  // Conversion factor for micro seconds to seconds
// Sleep for 10 minutes = 600 seconds
uint64_t TIME_TO_SLEEP = 600;

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

// Save reading number on RTC memory
RTC_DATA_ATTR int readingID = 0;

String dataMessage;

// Create an HIH61XX with I2C address 0x27, powered by 3.3V pin
HIH61XX hih(0x27);

// Data wire is connected to ESP32 GPIO 27
#define ONE_WIRE_BUS 27
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// Environmental Sensor variables
float DS18B20_temp;
float bme_t;
float bme_rh;
float bme_p;
float hih_rh;
float hih_t;

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

  // Connect to Wi-Fi network with SSID and password
  /*Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password); // want access point to be open, so password set to NULL
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");*/

  // Initialize a NTPClient to get time
  /*
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(-25200); // PST is behing GMT by 7 hours (7x3600=25200)*/
  initializeSD();

  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/VAMoS EnvData.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    //writeFile(SD, "/VAMoS EnvData.txt", "Reading ID, Date, Hour, Temperature \r\n");
    writeFile(SD, "/VAMoS EnvData.txt", "Reading ID, DS18B20, BME280_T, HIH_T, BME280_RH, HIH_RH, BME280_P \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();

  // Enable Timer wake_up
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

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
  logSDCard();
  
  // Increment readingID on every new reading
  readingID++;
  
  // Start deep sleep
  Serial.println("DONE! Going to sleep now.");
  esp_deep_sleep_start(); 
}

void loop() {
  // The ESP32 will be in deep sleep
  // it never reaches the loop()
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
  dataMessage = String(readingID) + "," + String(DS18B20_temp) + "," + String(bme_t) + "," + String(hih_t) + "," + String(bme_rh) + "," + String(hih_rh) + "," + String(bme_p) + "\r\n";
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
