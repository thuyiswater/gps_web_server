#include <TinyGPS++.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <HTTPClient.h>

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

TinyGPSPlus gps;
const char* ssid = "cindy"; //ssid of your wifi
const char* password = "sharingiscaring"; //password of your wifi
String lat, lng;

void gps_setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(3000);
  WiFiServer server(80);

  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); //connecting to wifi
  while (WiFi.status() != WL_CONNECTED)// while wifi not connected
  {
    delay(500);
    Serial.print("Connecting... \n"); //print "...."
  }
  Serial.println("");
  Serial.println("WiFi connected");
  server.begin();
  Serial.println("Server started");
  Serial.println(WiFi.localIP());  // Print the IP address
}

void displayLocation() {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) {
        Serial.print(F("Location: \n"));
        Serial.print("Lat: ");
        Serial.print(gps.location.lat(), 6);
        lat = String(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print("Lng: ");
        Serial.print(gps.location.lng(), 6);
        lng = String(gps.location.lng(), 6);
        Serial.println();
      }
        
      if (gps.date.isValid() && gps.time.isValid()) {
        Serial.print(F("GPS date & time: "));
        Serial.print(gps.date.year());
        Serial.print(F("-"));
        Serial.print(gps.date.month());
        Serial.print(F("-"));
        Serial.print(gps.date.day());
        Serial.print(F(" "));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        Serial.println(gps.time.second());
      } else {
        Serial.println(F("INVALID"));
      }

      // Google script Web_App_URL.
      String Web_App_URL = "https://script.google.com/macros/s/AKfycbxLUGZN9K9sI74frTvSbUemLhrzcRhOVEKBopuQPCR8ATEpGsX-4DcX44dVuFx3KyXr/exec";
      // Create a URL for sending or writing data to Google Sheets.
      String Send_Data_URL = Web_App_URL + "?sts=write";
      Send_Data_URL += "&lat=" + lat;
      Send_Data_URL += "&long=" + lng;

      Serial.println();
      Serial.println("-------------");
      Serial.println("Send data to Google Spreadsheet...");
      Serial.print("URL : ");
      Serial.println(Send_Data_URL);

      // Initialize HTTPClient as "http".
      HTTPClient http;
  
      // HTTP GET Request.
      http.begin(Send_Data_URL.c_str());
      http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  
      // Gets the HTTP status code.
      int httpCode = http.GET(); 
      Serial.print("HTTP Status Code : ");
      Serial.println(httpCode);
  
      // Getting response from google sheets.
      String payload;
      if (httpCode > 0) {
        payload = http.getString();
        Serial.println("Payload : " + payload);
        Serial.println();   
      }
      
      http.end();

      if (millis() > 5000 && gps.charsProcessed() < 10) {
          Serial.println(F("No GPS detected: check wiring."));
      }
    }
  }
}

