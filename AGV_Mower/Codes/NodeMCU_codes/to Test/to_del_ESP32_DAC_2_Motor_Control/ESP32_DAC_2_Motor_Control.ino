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

#define DAC1 25
#define DAC2 26

int analogValue = 0;
String powerState = "OFF";

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

//LED state by Vedha
void handlePowerstate() {  
  String t_state = server.arg("Powerstate"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
  //Serial.println(t_state);
  if (t_state == "true")
  {
    powerState = "true"; //Feedback parameter
  }
  else
  {
    powerState = "false"; //Feedback parameter
    analogValue = 0;
    speedControl(analogValue,'b');
  }
  server.send(200, "text/plane", powerState ); //Send web page
}
void handleSpeedstate() {
  String speedState = "";
  String t_state = server.arg("Speedstate"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
  //Serial.println(t_state);
  if (t_state == "5" && powerState=="true")
  {
    analogValue = 128;
    speedState = "half"; //Feedback parameter
    speedControl(analogValue,'b');
  }
  else if (t_state == "10" && powerState=="true")
  {
    analogValue = 255;
    speedState = "full"; //Feedback parameter
    speedControl(analogValue,'b');
  }
  else
  {
    analogValue = 0;
    speedState = "0"; //Feedback parameter
    speedControl(analogValue,'b');
  }
  server.send(200, "text/plane", speedState );
}
void handleMotionstate()  {
  String motionState = "";
  String t_state = server.arg("Motionstate"); //Refer  xhttp.open("GET", "setLED?LEDstate="+led, true);
  //Serial.println(t_state);
  if (t_state == "fwd" && powerState=="true")
  {  
    speedControl(analogValue,'b');
    motionState = "fwd"; //Feedback parameter
  }
  else if (t_state == "bwd" && powerState=="true")
  {      
    speedControl(analogValue,'b');
    motionState = "bwd"; //Feedback parameter
  }
  else if (t_state == "rgt" && powerState=="true")
  {
    speedControl(analogValue,'r');
    motionState = "rgt"; //Feedback parameter
  }
  else if (t_state == "lft" && powerState=="true")
  {
    speedControl(analogValue,'l');
    motionState = "lft"; //Feedback parameter
  }
  else if (t_state == "stop")
  {
    motionState = "stop"; //Feedback parameter    
    analogValue = 0;
    speedControl(analogValue,'b');
  }  
  server.send(200, "text/plane", motionState ); //Send web page
}

void speedControl(int value,char side){
  if(side == 'b'){
    dacWrite(DAC1, value);
    dacWrite(DAC2, value);
  }
  else if(side == 'r'){
    dacWrite(DAC1, value/2);
  }
  else if(side == 'l'){
    dacWrite(DAC2, value/2);
  }  
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);  
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
  server.on("/setPower", handlePowerstate);
  server.on("/setDIR", handleMotionstate);
  server.on("/setSpeed", handleSpeedstate);
  server.onNotFound(handleNotFound);

  // Start server
  server.begin();
}
void loop() {
  server.handleClient();  
  //Serial.println(analogRead(34));
}
