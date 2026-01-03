// Import required libraries
#include <WiFi.h>
//#include "ESPAsyncWebServer.h"
#include <WiFiClient.h>
#include <WebServer.h>
#include <WiFiAP.h>
#include "SPIFFS.h"

// Replace with your network credentials
const char* ssid = "ssid";
const char* password = "password";
const String styleFile = "/min.css";
const String scriptFile = "/main.js";
const String htmlIndex = "/index.html";

// Create AsyncWebServer object on port 80
//AsyncWebServer server(80);
WebServer server(80);

void handleRoot() {
  String myFile = htmlIndex;
  if (SPIFFS.exists(myFile)) {
    Serial.println(F("myFile founded on   SPIFFS"));   //ok
    File file = SPIFFS.open(myFile, "r");
    //server.send(200, "text/html", file);
    size_t sent = server.streamFile(file, "text/html" );
  }
  else
  {
    Serial.println(F("Index not found on SPIFFS"));
    handleNotFound;
  }
}
void handleJS() {
  String myFile = scriptFile;
  if (SPIFFS.exists(myFile)) {
    Serial.println(F("myFile founded on   SPIFFS"));   //ok
    File file = SPIFFS.open(myFile, "r");
    //server.send(200, "text/css", file);
    size_t sent = server.streamFile(file, "text/javascript" );
    file.close();
  }
  else
  {
    Serial.println(F("JS not found on SPIFFS"));
    handleNotFound;
  }
}
void handleCSS() {
  String myFile = styleFile;
  if (SPIFFS.exists(myFile)) {
    Serial.println(F("myFile founded on   SPIFFS"));   //ok
    File file = SPIFFS.open(myFile, "r");
    //server.send(200, "text/css", file);
    size_t sent = server.streamFile(file, "text/css" );
    file.close();
  }
  else
  {
    Serial.println(F("Stylsheet not found on SPIFFS"));
    handleNotFound;
  }
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  //  pinMode(ledPin, OUTPUT);

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Connect to Wi-Fi
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // Route for root / web page
  server.on("/", handleRoot);

  // Route to load style.css file
  server.on(styleFile, handleCSS);

  // Route to load main.js file
  server.on(scriptFile, handleJS);

  server.onNotFound(handleNotFound);

  // Start server
  server.begin();
}



void loop() {
  server.handleClient();
}
