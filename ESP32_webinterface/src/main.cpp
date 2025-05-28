#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>

const char* ssid = "ESP32_Rover_Control";
const char* password = "12345678";

WebServer server(80);

//streams the spiffs file into the server for the browser to display

void handleRoot() {
  File file = SPIFFS.open("/Joystick.html", "r");
  server.streamFile(file, "text/html");
  file.close();
}


void direction() {
  int x = server.arg("x").toInt(); 
  int y = server.arg("y").toInt();  
  if (x == 0 && y == 0) {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    server.send(200, "text/plain", "stopped"); // part of a interupt cycle on the 'end' event of the joystick
    return;
  }

  int left_motor  = y + x;
  int right_motor = y - x;

  //handles motors simulataneously

  if (left_motor < 0) {
    digitalWrite(15, HIGH);  // Reverse
  } 
  else {
    digitalWrite(15, LOW);   // Forward
  }
  ledcWrite(0, min(abs(left_motor), 255)); //makes sure PWM out is 255=<

  if (right_motor < 0) {
    digitalWrite(4, HIGH);   // Reverse
  } else {
    digitalWrite(4, LOW);    // Forward
  }
  ledcWrite(1, min(abs(right_motor), 255));  //makes sure PWM out is 255=<
}

//void read_IR(){  }

void read_data(){
  int input_1_state = digitalRead(13);

  //these are purely test inputs for the framework of sending accross the data between server and client
  //these http requests are parsed in the javascript in the html file
  if (input_1_state == 1){
    server.send(200, "text/plain", "radio=100&infared=100&magnetic=up&species=duck1&name=joey");
    return;
  }
  server.send(200, "text/plain", "radio=0&infared=0&magnetic=down&species=duck2&name=bob");
}

void setup() {
  Serial.begin(115200);
  ledcSetup(0, 5000, 8); // 5kHz, 8-bit resolution
  pinMode(15, OUTPUT);
  pinMode(4, OUTPUT);
  ledcAttachPin(2, 0); 
  ledcAttachPin(16, 1); 

  SPIFFS.begin();

  WiFi.softAP(ssid, password);    
  Serial.println("AP started. IP address: " + WiFi.softAPIP().toString());  //setup access point

  server.on("/", handleRoot);

  //serves the static javascript lib from the esp storage
  server.on("/assets/nipplejs.min.js", HTTP_GET, []() {
    File file = SPIFFS.open("/assets/nipplejs.min.js", "r");
    server.streamFile(file, "application/javascript");
    file.close();
  });
  //serves the styling file to the browser again from esp
  server.on("/assets/styles.css", HTTP_GET, [](){
    File file = SPIFFS.open("/assets/styles.css", "r");
    server.streamFile(file, "text/css");
    file.close();
  });

  server.on("/joystick", HTTP_GET, direction);
  server.on("/read_data", HTTP_GET, read_data);     //handles the server http roots 

  server.begin();
}

void loop() {
  server.handleClient();
}
