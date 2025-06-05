#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
// ——— CONFIG ———
static const int PWM_FREQ = 50;                  // 50 Hz for servos
static const int PWM_RES_BITS = 16;              // 16-bit resolution
static const int PERIOD_US = 1000000 / PWM_FREQ; // 20 000 µs

static const int PULSE_MIN_US = 1000;
static const int PULSE_MAX_US = 2000;

const char *ssid = "ESP32_TEST1";
const char *password = "12345678";
TaskHandle_t Task0;

WebServer server(80);

// streams the spiffs file into the server for the browser to display
static const uint8_t servoPins[4] = {32, 33, 25, 26};
static const uint8_t servoChannels[4] = {0, 1, 2, 3};
void handleRoot()
{
  File file = SPIFFS.open("/Joystick.html", "r");
  server.streamFile(file, "text/html");
  file.close();
}

void direction()
{
  int x = server.arg("x").toInt();
  int y = server.arg("y").toInt();
  if (x == 0 && y == 0)
  {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    server.send(200, "text/plain", "stopped"); // part of a interupt cycle on the 'end' event of the joystick
    return;
  }

  int left_motor = y + x;
  int right_motor = y - x;

  // handles motors simulataneously

  if (left_motor < 0)
  {
    digitalWrite(15, HIGH); // Reverse
  }
  else
  {
    digitalWrite(15, LOW); // Forward
  }
  ledcWrite(0, min(abs(left_motor), 255)); // makes sure PWM out is 255=<

  if (right_motor < 0)
  {
    digitalWrite(4, HIGH); // Reverse
  }
  else
  {
    digitalWrite(4, LOW); // Forward
  }
  ledcWrite(1, min(abs(right_motor), 255)); // makes sure PWM out is 255=<
}

// void read_IR(){  }

void read_data()
{
  int input_1_state = digitalRead(13);

  // these are purely test inputs for the framework of sending accross the data between server and client
  // these http requests are parsed in the javascript in the html file
  if (input_1_state == 1)
  {
    server.send(200, "text/plain", "radio=100&infared=100&magnetic=up&species=duck1&name=joey");
    return;
  }
  server.send(200, "text/plain", "radio=0&infared=0&magnetic=down&species=duck2&name=bob");
}

void Task0code(void *pvParameters)
{
  Serial.print("Task0 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    read_data();
    Serial.println("Data has been read");
    delay(1000);
  }
}
void setupContinuousServo(uint8_t pin, uint8_t ledcChannel)
{
  // 1) configure timer (only need once per timer; here we use TIMER 0 on HIGH-speed)
  ledcSetup(ledcChannel, PWM_FREQ, PWM_RES_BITS);
  // 2) attach the pin to that channel
  ledcAttachPin(pin, ledcChannel);
}
void rotateContinuousServo(uint8_t ledcChannel, int speedPercent)
{
  speedPercent = constrain(speedPercent, -100, 100);
  // map to 1000–2000 µs
  int pulseUs = map(speedPercent, -100, 100, PULSE_MIN_US, PULSE_MAX_US);

  // convert µs to duty count:
  //    duty = pulseUs / periodUs * (2^res – 1)
  uint32_t maxDuty = (1UL << PWM_RES_BITS) - 1;
  uint32_t duty = (uint64_t)pulseUs * maxDuty / PERIOD_US;

  // write it out
  ledcWrite(ledcChannel, duty);
}
void setup()
{
  Serial.begin(115200);
  /*
  ledcSetup(0, 5000, 8); // 5kHz, 8-bit resolution
  pinMode(15, OUTPUT);
  pinMode(4, OUTPUT);
  ledcAttachPin(2, 0);
  ledcAttachPin(16, 1);
*/
  SPIFFS.begin();
  for (uint8_t i = 0; i < 4; ++i)
  {
    setupContinuousServo(servoPins[i], servoChannels[i]);
  }
  WiFi.softAP(ssid, password);
  Serial.println("AP started. IP address: " + WiFi.softAPIP().toString()); // setup access point

  server.on("/", handleRoot);

  // serves the static javascript lib from the esp storage
  server.on("/assets/nipplejs.min.js", HTTP_GET, []()
            {
    File file = SPIFFS.open("/assets/nipplejs.min.js", "r");
    server.streamFile(file, "application/javascript");
    file.close(); });
  // serves the styling file to the browser again from esp
  server.on("/assets/styles.css", HTTP_GET, []()
            {
    File file = SPIFFS.open("/assets/styles.css", "r");
    server.streamFile(file, "text/css");
    file.close(); });

  server.on("/joystick", HTTP_GET, direction);
  server.on("/read_data", HTTP_GET, read_data); // handles the server http roots
  xTaskCreatePinnedToCore(Task0code, "Task0", 10000, NULL, 1, &Task0, 0);
  server.begin();
}

void loop()
{
  delay(2000);
  // FL
  rotateContinuousServo(servoChannels[0], 100);
  delay(2000);
  // RR
  rotateContinuousServo(servoChannels[1], 100);
  delay(2000);
  // RL
  rotateContinuousServo(servoChannels[2], 100);
  delay(2000);
  // FR
  rotateContinuousServo(servoChannels[3], 100);
  delay(2000);
  // server.handleClient();
}
