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

//-----------------------------------------------------------------------------
// Setup and main loop
//-----------------------------------------------------------------------------

ESP32WebServer  server(80);     // Initialize server at port 80

void setup(void)
{
    Serial.begin(115200);

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

    // Mount SPIFFS (SPI Flahs File System)
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
