#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <vector>
#include <driver/pcnt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Pinmap

// Motors
#define PWM_FREQ_MOTOR 5000
#define PWM_RES_MOTOR 8
#define MOTOR_CH0 0
#define MOTOR_PIN0 25
#define MOTOR_CH1 1
#define MOTOR_PIN1 33
#define MOTOR_CH2 2
#define MOTOR_PIN2 27
#define MOTOR_CH3 3
#define MOTOR_PIN3 26

// Sensors
#define IR_INPUT_PIN 18
#define RF_INPUT_PIN 23
#define HALL_SENSOR_PIN 35

//  Pulsecounters
#define IR_PULSECOUNTER PCNT_UNIT_0
#define RF_PULSECOUNTER PCNT_UNIT_1

//  Timings
#define SENSOR_POLL_TIME 250
#define FREQ_MEASURE_TIME 200
#define MOTOR_DELAY 50
#define UART_READ_DELAY_MS 10

// find frequency multiplier
const float FREQ_MULTIPLIER = 1000.0 / FREQ_MEASURE_TIME;

// Hall sensor threasholds
const int UPPER_THRESHOLD = 3100; // 2.8V equivalent
const int LOWER_THRESHOLD = 2730; // 2.2V equivalent

// Wifi creds
const char *ssid = "ESP32_Rover_Control";
const char *password = "12345678";

// Server
AsyncWebServer server(80);

// Duck data
std::vector<String> duck_names = {"Wibbo", "Gribbit", "Snorkle", "Zapple"};
std::vector<int> duck_ir_freqs = {457, 0, 293, 0};
std::vector<int> duck_rf_freqs = {0, 100, 0, 150};

// Motor global vars
int motor_x_val = 0;
int motor_y_val = 0;

// Global sensor data vars for web ui
int hall = 0;
float IR_freq = 0;
float RF_freq = 0;
String speciesName = "NULL";
String ultrasonic_name1 = "NULL";
String ultrasonic_name2 = "NULL";

// Setup was created by using https://github.com/pycom/pycom-esp-idf/blob/master/examples/peripherals/pcnt/main/pcnt_example_main.c to prompt Deepseek R1 Qwen distilled for examples.
void setup_pcnt()
{
  // IR pulsecounter
  pcnt_config_t pcnt_config_ir = {
      .pulse_gpio_num = IR_INPUT_PIN,
      .ctrl_gpio_num = PCNT_PIN_NOT_USED,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_KEEP,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DIS,
      .counter_h_lim = 32767,
      .counter_l_lim = -32768,
      .unit = IR_PULSECOUNTER,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config_ir);
  pcnt_set_filter_value(IR_PULSECOUNTER, 10);
  pcnt_filter_enable(IR_PULSECOUNTER);

  // RF pulsecounter
  pcnt_config_t pcnt_config_rf = {
      .pulse_gpio_num = RF_INPUT_PIN,
      .ctrl_gpio_num = PCNT_PIN_NOT_USED,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_KEEP,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DIS,
      .counter_h_lim = 32767,
      .counter_l_lim = -32768,
      .unit = RF_PULSECOUNTER,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config_rf);
  pcnt_set_filter_value(RF_PULSECOUNTER, 10);
  pcnt_filter_enable(RF_PULSECOUNTER);
}

// motor control task for PWM on TC1508 (Or whichever aliexpress special this one is)
void motorTask(void *parameter)
{
  while (1)
  {
    if (motor_x_val == 0 && motor_y_val == 0)
    {
      // Stopped
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      ledcWrite(2, 0);
      ledcWrite(3, 0);
    }
    else
    {
      int left_val = motor_y_val + motor_x_val;
      int right_val = motor_y_val - motor_x_val;
      int left_speed = min(abs(left_val), 255);
      int right_speed = min(abs(right_val), 255);

      if (right_val >= 0)
      {
        ledcWrite(1, right_speed);
        ledcWrite(0, 0);
      }
      else
      {
        ledcWrite(0, right_speed);
        ledcWrite(1, 0);
      }
      if (left_val >= 0)
      {
        ledcWrite(2, left_speed);
        ledcWrite(3, 0);
      }
      else
      {
        ledcWrite(3, left_speed);
        ledcWrite(2, 0);
      }
    }
    // Small delay to not oversaturate task
    vTaskDelay(pdMS_TO_TICKS(MOTOR_DELAY));
  }
}

// get hall state and raw value
String read_hall_value(int &hall_raw)
{
  hall_raw = analogRead(HALL_SENSOR_PIN);
  String hall_state;
  if (hall_raw >= UPPER_THRESHOLD)
  {
    hall_state = "up";
  }
  else if (hall_raw <= LOWER_THRESHOLD)
  {
    hall_state = "down";
  }
  else
  {
    hall_state = "N/A";
  }
  return hall_state;
}

// matching logic for speecies
String matchSpecies(int ir_freq, int rf_freq)
{

  String match = "None";
  bool ir_active = (ir_freq > 50);
  bool rf_active = (rf_freq > 50);

  // Match IR ducks
  if (ir_active && !rf_active)
  {
    int wibbo_delta = abs(ir_freq - 457);
    int snorkle_delta = abs(ir_freq - 293);

    if (wibbo_delta < snorkle_delta)
    {
      match = "Wibbo";
    }
    else
    {
      match = "Snorkle";
    }
  }
  else if (!ir_active && rf_active)
  {
    // Match RF ducks
    int gribbit_diff = abs(rf_freq - 100);
    int zapple_diff = abs(rf_freq - 150);

    if (gribbit_diff < zapple_diff)
    {
      match = "Gribbit";
    }
    else
    {
      match = "Zapple";
    }
  }
  return match;
}

// Sensor reading and pulse counting task
void sensor_task(void *parameter)
{
  while (1)
  {
    // Same as pulse counter initialization, Deepseek R1 Qwen distilled was used to explain and give examples for which functions needed to be called for the pulsecounter.
    vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_TIME));

    // Measure frequency
    pcnt_counter_clear(IR_PULSECOUNTER);
    pcnt_counter_clear(RF_PULSECOUNTER);
    pcnt_counter_resume(IR_PULSECOUNTER);
    pcnt_counter_resume(RF_PULSECOUNTER);

    vTaskDelay(pdMS_TO_TICKS(FREQ_MEASURE_TIME));

    pcnt_counter_pause(IR_PULSECOUNTER);
    pcnt_counter_pause(RF_PULSECOUNTER);

    int16_t ir_count, rf_count;
    pcnt_get_counter_value(IR_PULSECOUNTER, &ir_count);
    pcnt_get_counter_value(RF_PULSECOUNTER, &rf_count);

    // scale frequency from pulsecounter
    float ir_freq = ir_count * FREQ_MULTIPLIER;
    float rf_freq = rf_count * FREQ_MULTIPLIER;

    // get hall sensor value
    int hall_raw = 0;
    String hall_state = read_hall_value(hall_raw);

    // global var update
    IR_freq = ir_freq;
    RF_freq = rf_freq;
    hall = hall_raw;

    speciesName = "";
    speciesName = matchSpecies(ir_freq, rf_freq);
  }
}

// Heavily referenced example by UKHeliBob: https://forum.arduino.cc/t/receive-string-from-serial-monitor/643266/2
void uart_reading_task1(void *parameter)
{
  String name = "";

  while (1)
  {
    if (Serial2.available())
    {
      char c = Serial2.read();
      // wait for # and start reading next 3 chars
      if (c == '#')
      {
        name = "#";
      }
      else if (name.length() > 0)
      {
        name += c;
        if (name.length() == 4)
        {
          ultrasonic_name1 = name;
          name = "";
        }
      }
    }
    // delay to not oversaturate task
    vTaskDelay(pdMS_TO_TICKS(UART_READ_DELAY_MS));
  }
}

void uart_reading_task2(void *parameter)
{
  String name = "";

  while (1)
  {
    if (Serial2.available())
    {
      char c = Serial2.read();

      // wait for # and start reading next 3 chars
      if (c == '#')
      {
        name = "#";
      }
      else if (name.length() > 0)
      {
        name += c;
        if (name.length() == 4)
        {
          ultrasonic_name1 = name;
          name = "";
        }
      }
    }
    // delay to not oversaturate task
    vTaskDelay(pdMS_TO_TICKS(UART_READ_DELAY_MS));
  }
}

// webserver handlers
void handleRoot(AsyncWebServerRequest *req)
{
  req->send(SPIFFS, "/Joystick.html", "text/html");
}

void handleJoystick(AsyncWebServerRequest *req)
{
  motor_x_val = req->getParam("x")->value().toInt();
  motor_y_val = req->getParam("y")->value().toInt();
  req->send(200, "text/plain", "command received");
}

// Create query string
void handleReadData(AsyncWebServerRequest *req)
{
  String query_string = "radio=" + String(RF_freq) +
                        "&infared=" + String(IR_freq) +
                        "&magnetic=" + String(hall) +
                        "&species=" + speciesName +
                        "&name=" + ultrasonic_name1 + ultrasonic_name2;
  req->send(200, "text/plain", query_string);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // motor setup
  pinMode(HALL_SENSOR_PIN, INPUT);

  ledcSetup(MOTOR_CH0, PWM_FREQ_MOTOR, PWM_RES_MOTOR);
  ledcAttachPin(MOTOR_PIN0, MOTOR_CH0);

  ledcSetup(MOTOR_CH1, PWM_FREQ_MOTOR, PWM_RES_MOTOR);
  ledcAttachPin(MOTOR_PIN1, MOTOR_CH1);

  ledcSetup(MOTOR_CH2, PWM_FREQ_MOTOR, PWM_RES_MOTOR);
  ledcAttachPin(MOTOR_PIN2, MOTOR_CH2);

  ledcSetup(MOTOR_CH3, PWM_FREQ_MOTOR, PWM_RES_MOTOR);
  ledcAttachPin(MOTOR_PIN3, MOTOR_CH3);

  // esp32 hardware setup
  setup_pcnt();
  Serial1.begin(600, SERIAL_8N1, 16);
  Serial2.begin(600, SERIAL_8N1, 17);

  // creating tasks

  xTaskCreate(
      motorTask,
      "Motor",
      4096,
      NULL,
      1,
      NULL);

  xTaskCreate(
      sensor_task,
      "Sensor",
      8192,
      NULL,
      2,
      NULL);

  xTaskCreate(
      uart_reading_task1,
      "UART_read1",
      4096,
      NULL,
      3,
      NULL);

  xTaskCreate(
      uart_reading_task2,
      "UART_read",
      4096,
      NULL,
      3,
      NULL);

  // start wifi and webserver
  SPIFFS.begin(true);

  // WIFI
  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  // Async webserver setup
  server.on("/", HTTP_GET, handleRoot);
  server.serveStatic("/assets", SPIFFS, "/assets/");
  server.on("/joystick", HTTP_GET, handleJoystick);
  server.on("/read_data", HTTP_GET, handleReadData);
  server.begin();
}

void loop()
{
}
