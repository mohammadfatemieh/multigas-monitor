//-----------------------------------------------------------------------------
// Filename:                    ESP32_AccessPoint_Download.ino
// Author:                      Howard Huang
// Created (DD/MM/YY):          26/08/19
// Last modified (DD/MM/YY):    26/08/19
// Last modified by:            Howard Huang
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Libraries
//-----------------------------------------------------------------------------

#include    <WiFi.h>
#include    <ESP32WebServer.h>  // https://github.com/Pedroalbuquerque/ESP32WebServer (place in lib folder)
#include    <ESPmDNS.h>         // from ESP package
#include    "FS.h"              // File server from ESP package (Needed before SPIFFS.h)
#include    "ESP32_AccessPoint_Download_Vars.h"            // Global variables
#include    "ESP32_AccessPoint_Download_css.h"             // css file for web page
#include    <SPI.h>
#include    "SPIFFS.h"

#include "SD.h"
#include <Wire.h>

//DS18B20 libraries
#include <OneWire.h>
#include <DallasTemperature.h>

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
#define WARMUP_TIME 5000 // 5 seconds to warm up sensors (should be 25 mins)
#define ACTIVE_TIME 30000 // 30 seconds
#define GREEN 32
#define RED 33

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
uint32_t start_time = millis();
int delayTime = 1000;
int GPSwait = 3000;

// Environmental Sensor variables
float DS18B20_temp;
float bme_t;
float bme_rh;
float bme_p;
float hih_rh;
float hih_t;

// Analog sensor variables
int ALPHS3_ADC;
int ALPHS4_ADC;
int ALPHS5_ADC;
int ALPHS6_ADC;
int ALPHS7_ADC;
int ALPHS8_ADC;

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

//-----------------------------------------------------------------------------
// Setup and main loop
//-----------------------------------------------------------------------------

ESP32WebServer  server(80);     // Initialize server at port 80

void setup(void)
{
    Serial.begin(115200);
    pinMode(PUMP,OUTPUT);
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(PUMP, channel);

    pinMode(GREEN,OUTPUT);
    pinMode(RED,OUTPUT);

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

    // Setup access point
    Serial.print("Setting AP (Access Point)...");
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();         // TODO: Find out what this does
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // SD card setup
    // SD card needs pull-up on MISO
    Serial.println(MISO);
    pinMode(19, INPUT_PULLUP);
    Serial.print(F("Initializing SD card..."));

    // Mount SPIFFS (SPI Flash File System)
    if(!SPIFFS.begin(true))
    {
        Serial.println("Error mounting SPIFFS");
        return;
    }

    // Web server setup
    server.on("/",          HomePage);
    server.on("/download",  File_Download);

    server.begin();
    Serial.println("HTTP server started");

}

void loop()
{
  // Enable Timer wake_up
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  start_time = millis();

  while(millis() - start_time < WARMUP_TIME){
      digitalWrite(ALPHS_EN,HIGH); // turn on the sensors
      Serial.println("Turning on analog sensors...");
      delay(WARMUP_TIME); // wait for sensors to warm up
      ParseGPS();
   }
  start_time = millis();
  
  while(millis() - start_time < ACTIVE_TIME){
      Serial.println("Sensors are warm. Starting logging...");
      digitalWrite(GREEN,HIGH);
      ledcWrite(channel,255); // 100% duty cycle
      delay(3000);
      ledcWrite(channel,155);
      Serial.println("Pump running...");
      server.handleClient();      // Listen for client connections
}

//-----------------------------------------------------------------------------
// Server functions
//-----------------------------------------------------------------------------

void HomePage()
{
    SendHTML_Header();
    SendHTML_Content();
    SendHTML_Stop();        // Stop needed because no content length was sent
}

void File_Download()
{
    File dataFile = SPIFFS.open("/ESP_Data_file.txt");
    if (dataFile)
    {
        // Send download information to client
        server.sendHeader("Content-Type", "text/text");
        server.sendHeader("Content-Disposition", "attachment; filename=ESP_Data_File.txt");
        server.sendHeader("Connection", "close");
        server.streamFile(dataFile, "application/octet-stream");
        dataFile.close();
    }
    else
        ReportFileNotPresent("download");
}


//-----------------------------------------------------------------------------
// SendHTML functions
//  Not sure how most of these functions work but it seems like they update the
//  webpage for the client. First send a header, then update the content and
//  send the content. Then close the webpage update with the client.
//-----------------------------------------------------------------------------
void SendHTML_Header()  // Not exactly sure what this does
{
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-chache");
    server.sendHeader("Expires", "-1");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");
    append_page_header();                           // from css.h
    server.sendContent(webpage);
    webpage = "";
}

void SendHTML_Content()
{
    server.sendContent(webpage);
    webpage = "";
}

void SendHTML_Stop()    // Not sure what this does either
{
    server.sendContent("");
    server.client().stop();     // Stop needed because no content length was sent
}

// Let client know that the file could not be found
void ReportFileNotPresent(String target)
{
    SendHTML_Header();
    webpage += F("<h3>File does not exist</h3>");
    webpage += F("<a href='/");
    webpage += target + "'>[Back]</a><br><br>";
    append_page_footer();                           // from css.h
    SendHTML_Content();
    SendHTML_Stop();
}
