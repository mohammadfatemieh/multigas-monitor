/*
 * Logs all data from environmental sensors to SD card hosts a web server for data download
 * Includes analog sensors, outputs them to SD card in ADC values
 * Runs the set cycle time for warmup, active, and sleep
 * 
 * Last tested: 18-August-2019 with VAMoS RevC
 */

// Libraries for SD card
#include "FS.h"
#include "SD.h"
#include <SPI.h>

//DS18B20 libraries
#include <OneWire.h>            // https://github.com/PaulStoffregen/OneWire
#include <DallasTemperature.h>  // https://github.com/milesburton/Arduino-Temperature-Control-Library

// Libraries to get time from NTP Server
#include <WiFi.h>
#include <NTPClient.h>          // https://github.com/taranais/NTPClient
#include <WiFiUdp.h>

#include <Wire.h>

// HIH6120 library
#include <HIH61XX.h>

// GPS library
#include <Adafruit_GPS.h>

// BME280 libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Wifi and web server libraries
#include <WiFi.h>
#include <ESP32WebServer.h>         // https://github.com/Pedroalbuquerque/ESP32WebServer
#include <ESPmDNS.h>                // From ESP package
// Web server header files
#include "ESP32_Logging_DL_vars.h"
#include "ESP32_Logging_DL_css.h"

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
#define ALPHS_EN 13
#define WARMUP_TIME 5000 // 5 seconds to warm up sensors (should be 25 mins)
#define ACTIVE_TIME 60000 // 30 seconds
#define PUMP_MAX_DUTY_MIN_TIME 3000 // 3 seconds for pump to run at 100% duty cycle
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
uint32_t active_start_time = millis();
uint32_t pump_max_duty_start_time = millis();
int delayTime = 1000;
int GPSwait = 3000;
bool warm_init_flag = 0;            // Flag so warm up only runs once per wakeup
bool active_init_flag = 0;          // Flag so pump runs 100% duty cycle only once (when it enters active mode)

// Environmental Sensor variables
float DS18B20_temp;
float bme_t;
float bme_rh;
float bme_p;
float hih_rh;
float hih_t;

// Analog sensor variables
int ALPHS1_ADC;
int ALPHS2_ADC;
int ALPHS3_ADC;
int ALPHS4_ADC;
int ALPHS5_ADC;
int ALPHS6_ADC;
int ALPHS7_ADC;
int ALPHS8_ADC;

float ALPHS1_volts;
float ALPHS2_volts;
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

// Initialize server at port 80
ESP32WebServer  server(80);

void setup() {
  // Start serial communication for debugging purposes
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

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // Web server setup
    server.on("/",          HomePage);
    server.on("/download",  File_Download);

    server.begin();
    Serial.println("HTTP server started");
}

void loop() {
    // Enable Timer wake_up
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    start_time = millis();

    // Set init flags to 0 on wake
    warm_init_flag = 0;
    active_init_flag = 0;

    // Begin handle client loop
    while(millis() - start_time < WARMUP_TIME + ACTIVE_TIME)
    {
        // Sensor functions
        // If statements are used because we need to run SendHTML functions with every loop
        if (millis() - start_time < WARMUP_TIME)
        {
            warmSensor();

            // Set active start_time at the end of the warmup time
            // Active time won't run unless warmup is already done (else if statement)
            active_start_time = millis();
        }
        else if (millis() - active_start_time < ACTIVE_TIME)
            runSensor();

        // HTTP stuff
        server.handleClient();
    }

    // Go back to deep sleep
    Serial.println("Done! Going to sleep now.");

    digitalWrite(ALPHS_EN, LOW);

    // Turn off pump
    ledcWrite(channel, 0);   

    digitalWrite(GREEN, HIGH);
    digitalWrite(RED, HIGH);

    delay(500);

    esp_deep_sleep_start();

    // END MAIN LOOP
}


//-----------------------------------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Peripheral functions
//-----------------------------------------------------------------------------

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
  + String(DS18B20_temp) + "," + String(bme_t) + "," + String(hih_t) + "," + String(bme_rh) + "," + String(hih_rh) + "," + String(bme_p) + "," + String(ALPHS1_ADC) + "," + String(ALPHS2_ADC) + "," + String(ALPHS3_ADC) + "," 
  + String(ALPHS4_ADC) + "," + String(ALPHS5_ADC) + "," + String(ALPHS6_ADC) + "," + String(ALPHS7_ADC) + "," + String(ALPHS8_ADC) + "\r\n";
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/VAMoS_EnvData.txt", dataMessage.c_str());
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

void readAlphasense(){
    // read the input on analog GPIO pins
  ALPHS1_ADC = analogRead(35);
  ALPHS2_ADC = analogRead(2);
  ALPHS3_ADC = analogRead(12);
  ALPHS4_ADC = analogRead(4);
  ALPHS5_ADC = analogRead(34);
  ALPHS6_ADC = analogRead(36);
  ALPHS7_ADC = analogRead(39);
  ALPHS8_ADC = analogRead(25);
  
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.3V):
  ALPHS1_volts = ALPHS1_ADC * (3.3 / 1023.0);
  ALPHS2_volts = ALPHS2_ADC * (3.3 / 1023.0);
  ALPHS3_volts = ALPHS3_ADC * (3.3 / 1023.0);
  ALPHS4_volts = ALPHS4_ADC * (3.3 / 1023.0);
  ALPHS5_volts = ALPHS5_ADC * (3.3 / 1023.0);
  ALPHS6_volts = ALPHS6_ADC * (3.3 / 1023.0);
  ALPHS7_volts = ALPHS7_ADC * (3.3 / 1023.0);
  ALPHS8_volts = ALPHS8_ADC * (3.3 / 1023.0);

  // print out the value you read:
  Serial.print("Sensor 1 is: ");
  Serial.print(ALPHS1_volts);
  Serial.println(" V");
  
  Serial.print("Sensor 2 is: ");
  Serial.print(ALPHS2_volts);
  Serial.println(" V");
  
  Serial.print("Sensor 3 is: ");
  Serial.print(ALPHS3_volts);
  Serial.println(" V");
  
  Serial.print("Sensor 4 is: ");
  Serial.print(ALPHS4_volts);
  Serial.println(" V");
  
  Serial.print("Sensor 5 is: ");
  Serial.print(ALPHS5_volts);
  Serial.println(" V");
  
  Serial.print("Sensor 6 is: ");
  Serial.println(ALPHS6_volts);
  
  Serial.print("Sensor 7 is: ");
  Serial.println(ALPHS7_volts);  

  Serial.print("Sensor 8 is: ");
  Serial.print(ALPHS8_volts);
  Serial.println(" V");
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

//-----------------------------------------------------------------------------
// Sensor functions
// Warmup/Run operations
//-----------------------------------------------------------------------------

void warmSensor()
{
    // Warm up sensors
    // Run these operations only once then wait for sensors to warm up
    if (!warm_init_flag)
    {
        digitalWrite(ALPHS_EN, HIGH);   // turn on sensors
        Serial.println("Turning on analog sensors...");
        digitalWrite(GREEN, LOW);
        digitalWrite(RED, HIGH);

        // Set flag because sensors area already warmed up
        warm_init_flag = 1;
    }
}

void runSensor()
{
    // Perform logging functions
    digitalWrite(RED, LOW);
    digitalWrite(GREEN, HIGH);

    // On initial run, run pump at 100% duty cycle for 3 seconds
    if (!active_init_flag)
    {
        pump_max_duty_start_time = millis();    // set start time (should only be set once per wake)
        ledcWrite(channel, 255);                // 100% duty cycle)

        // Debugging messages
        Serial.println("Pump running at 100% duty cycle...");

        active_init_flag = 1;                   // Set flag so we don't do this again until next wake
    }
    // Let pump run in 100% duty cycle
    if (millis() - pump_max_duty_start_time < PUMP_MAX_DUTY_MIN_TIME)
        return;

    // Otherwise continue code at less % duty cycle
    
    ledcWrite(channel, 155);
    Serial.println("Pump running...");

    // Initialize SD mount
    initializeSD();                 // Consider moving to setup

    // Create data file if doesn't already exist
    File file = SD.open("/VAMoS_EnvData.txt");

    if (!file)
    {
        Serial.println("File doesn't exist");
        Serial.println("Creating file...");

        writeFile(SD, "/VAMoS_EnvData.txt", "Reading ID, Date, Time [UTC], DS18B20 [C], BME280_T [C], HIH_T [C], BME280_RH[%], HIH_RH [%], BME280_P [hPa], s3 [ADC], s4 [ADC], s5 [ADC], s6 [ADC] \r\n");
    }
    else
        Serial.println("File already exists");

    file.close();

    // Start DallasTemperature library
    sensors.begin();
    Wire.begin();

    // Setting up BME 280
    unsigned status;
    status = bme.begin();

    // Setting up HIH sensor
    hih.start();

    // ParseGPS();
    DS18B20_Readings();
    BME280_Readings();
    HIH6120_Readings();
    //getTimeStamp();
    logSDCard();

    // Increment ID for logging
    readingID++;
}

//-----------------------------------------------------------------------------
// Web server functions
//-----------------------------------------------------------------------------

// Update client with home page
void HomePage()
{
    SendHTML_Header();
    SendHTML_Content();
    SendHTML_Stop();
}

// function to send the data file to the client
// This assumes that there's always an SD card in the slot
void File_Download()
{
    File dataFile = SD.open("/VAMoS_EnvData.txt");
    if (dataFile)
    {
        // Send download info to client
        server.sendHeader("Content-Type", "text/text");
        server.sendHeader("Content-Disposition", "attachment; filename=VAMoS_EnvData.txt");
        server.sendHeader("Connection", "close");
        server.streamFile(dataFile, "application/octet-stream");
        dataFile.close();
    }
    else
        ReportFileNotPresent("download");
}

// SendHTML functions
void SendHTML_Header()
{
    // I think this sends information to the client so they know that this is a text/html web server
    server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.sendHeader("Expires", "-1");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");
    append_page_header();                   // from css.h
    server.sendContent(webpage);            // Send client the HTML
    webpage = "";
}

// Updates the page for the client
void SendHTML_Content()
{
    server.sendContent(webpage);
    webpage = "";
}

void SendHTML_Stop()        // Send blank to the client to signal that the server is done sending HTML?
{
    server.sendContent("");
    server.client().stop(); // Stop needed because no content length was sent
}

// Let client know that file could not be found
// This should probably be some sort of fatal error because there should always be a data file?
void ReportFileNotPresent(String target)
{
    SendHTML_Header();
    webpage += F("<h3>File does not exist</h3>");
    webpage += F("<a href='/");
    webpage += target + "'>[Back]</a><br><br>";
    append_page_footer();       // from css.h
    SendHTML_Content();
    SendHTML_Stop();
}
