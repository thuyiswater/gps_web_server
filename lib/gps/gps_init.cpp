#include <TinyGPS++.h>
#include <WiFi.h>
#include <HardwareSerial.h>

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

TinyGPSPlus gps;

const char* ssid = "Bibo"; //ssid of your wifi
const char* password = "hchk2016"; //password of your wifi
WiFiServer server(80);
float latArr[100];
int currIndex = 0;

void gps_setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(3000);

  Serial.println(ssid);
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

// Function to check if a value already exists in the array
void addUniqueValue(float value) {
  for (int i = 0; i < currIndex; i++) {
    if (latArr[i] == value) {
      return; // Value already exists in the array
    }
  }
  latArr[currIndex] = value;
  currIndex++;
}

void displayLocation() {
    while (Serial2.available() > 0) {
        // if (gps.encode(Serial2.read())) {
        //     if (gps.location.isValid()) {
        //         Serial.print(F("Location: \n"));
        //         Serial.print("Lat: ");
        //         Serial.print(gps.location.lat(), 6);
        //         addUniqueValue(gps.location.lat());
        //         Serial.print(F(","));
        //         Serial.print("Lng: ");
        //         Serial.print(gps.location.lng(), 6);
        //         Serial.println();   
        //     }  
        //     else {
        //         Serial.print(F("Connecting to gps... \n"));
        //         delay(1000);
        //     }
             
        //     if (gps.date.isValid() && gps.time.isValid()) {
        //         Serial.print(F("GPS date & time: "));
        //         Serial.print(gps.date.year());
        //         Serial.print(F("-"));
        //         Serial.print(gps.date.month());
        //         Serial.print(F("-"));
        //         Serial.print(gps.date.day());
        //         Serial.print(F(" "));
        //         Serial.print(gps.time.hour());
        //         Serial.print(F(":"));
        //         Serial.print(gps.time.minute());
        //         Serial.print(F(":"));
        //         Serial.println(gps.time.second());
        //     } else {
        //         Serial.println(F("INVALID"));
        //     }

        //     if (millis() > 5000 && gps.charsProcessed() < 10) {
        //         Serial.println(F("No GPS detected: check wiring."));
        //     }
        // }

        WiFiClient client = server.available(); // Check if a client has connected
        if (!client) {
        return;
        }
        
        const char index_html[] PROGMEM = R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
            <title>Web Server</title>
            <meta http-equiv='Content-Type' content='text/html; charset=utf-8'/>
            <style type='text/css'>body{margin:0;padding:0;overflow:hidden;font-family:'Segoe UI',Helvetica,Arial,Sans-Serif}</style>
        </head>
        <script type='text/javascript'>
            function GetMap() {
        var map = new Microsoft.Maps.Map('#myMap', {
        });

        //Request the user's location
        navigator.geolocation.getCurrentPosition(function (position) {
            var loc = new Microsoft.Maps.Location(
                position.coords.latitude,
                position.coords.longitude);

            //Add a pushpin at the user's location.
            var pin = new Microsoft.Maps.Pushpin(loc);
            map.entities.push(pin);

            //Center the map on the user's location.
            map.setView({ center: loc, zoom: 15 });
        });
    }
        </script>
        <script type='text/javascript' src='http://www.bing.com/api/maps/mapcontrol?callback=GetMap&key=Aj7OfvaPB9EGj83kqNUt8vGZhtCbG6_8mv-1roU3PzIm4rdXZTayRLk9vBLhJeAy' async defer></script>
        </head>
        <body>
            <div id="myMap" style="position:relative;width:800px;height:600px;"></div>
        </body>
        </html>)rawliteral";

        client.print(index_html); // all the values are send to the webpage
        delay(100); 
    }
}

