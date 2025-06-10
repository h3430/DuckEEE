#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <string>
#include <Arduino.h>

struct DuckSpecies {
  String name;
  bool has_IR;
  int IR_freq, RF_freq;
};

const char* ssid = "ESP32_Rover_Control";
const char* password = "12345678";

const int HALL_SENSOR_PIN = 33;  // Hall sensor connected to analog pin A0         
const int LEFT_WHEEL_DIRECTION_PIN = 15;
const int RIGHT_WHEEL_DIRECTION_PIN = 4;
const int LEFT_WHEEL_CONTROL_PIN = 2;
const int RIGHT_WHEEL_CONTROL_PIN =16;
//const int RF_pin = 2;
//const int IR_pin= 3;

volatile unsigned int pulse_count_1, pulse_count_2;
unsigned int measurement_period_ms = 1000;
unsigned long t = 0;


float IR_freq;
float RF_freq;


std::vector<DuckSpecies> ducks = {
  {"Wibbo", true, 457, 0},
  {"Gribbit", false, 0, 100},
  {"Snorkle", true, 293, 0},
  {"Zapple", false, 0, 150}
};


// Analog thresholds calculated for 2.2V and 2.8V (based on 5V reference)
const int UPPER_THRESHOLD = 3100;  // 2.8V = (2.8 / 3.3) * 4095 ≈ 573
const int LOWER_THRESHOLD = 2730;  // 2.2V = (2.2 / 3.3) * 4095 ≈ 450
TaskHandle_t Task0;


WebServer server(80);

//streams the spiffs file to the client for the browser to display

void handleRoot() {
  File file = SPIFFS.open("/Joystick.html", "r");
  server.streamFile(file, "text/html");
  file.close();
}
float pulse_count_to_freq(unsigned int pulse_count, unsigned int period_ms) {
  return pulse_count / (period_ms * 0.001);
}

bool is_valid_freq(float freq, float lower_thres, float higher_thres) {
  return (freq >= lower_thres) && (freq <= higher_thres);
}

void pulse_ISR_1() {
  pulse_count_1++;
}

void pulse_ISR_2() {
  pulse_count_2++;
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
    digitalWrite(LEFT_WHEEL_DIRECTION_PIN, HIGH);  // Reverse
  } 
  else {
    digitalWrite(LEFT_WHEEL_DIRECTION_PIN, LOW);   // Forward
  }
  ledcWrite(0, min(abs(left_motor), 255)); //makes sure PWM out is 255=<

  if (right_motor < 0) {
    digitalWrite(RIGHT_WHEEL_DIRECTION_PIN, HIGH);   // Reverse
  } else {
    digitalWrite(RIGHT_WHEEL_DIRECTION_PIN, LOW);    // Forward
  }
  ledcWrite(1, min(abs(right_motor), 255));  //makes sure PWM out is 255=<
}

String no_species(){
  return "";
}

void measure_freq() {
  noInterrupts();
  pulse_count_1 = 0;
  pulse_count_2 = 0;
  interrupts();
  
  delay(measurement_period_ms);

  noInterrupts();

  float ir_freq = pulse_count_to_freq(pulse_count_1, measurement_period_ms);
  float rf_freq = pulse_count_to_freq(pulse_count_2, measurement_period_ms);

  bool ir_signal_present = is_valid_freq(ir_freq, 112, 1200);
  bool rf_signal_present = is_valid_freq(rf_freq, 83, 180);
  

  if (ir_signal_present || rf_signal_present) {
    if (ir_signal_present) {
      IR_freq = ir_freq;
    }

    if (rf_signal_present) {
      RF_freq = rf_freq;
    }

  } else {
    IR_freq = 0;
    RF_freq = 0;
  }

  interrupts();
}

String find_matching_species( int meas_IR_freq, int meas_RF_freq) {
  String closest_match = "No Match";
  bool has_IR = is_valid_freq(IR_freq, 83, 180);
  int ir_freq_diff = INT_MAX;
  int rf_freq_diff = INT_MAX;

  for (auto d : ducks) {
    if (d.has_IR == has_IR) {
      if (d.has_IR) {
        if (abs(meas_IR_freq - d.IR_freq) < ir_freq_diff) {
          ir_freq_diff = abs(meas_IR_freq - d.IR_freq);
          closest_match = d.name;
        }
      } else {
        if (abs(meas_RF_freq - d.RF_freq) < rf_freq_diff) {
          rf_freq_diff = abs(meas_RF_freq - d.RF_freq);
          closest_match = d.name;
        }
      }
    }
  }

  return closest_match;
}



std::string hall_read(){
  int hallSensorValue = analogRead(HALL_SENSOR_PIN);
  Serial.println(hallSensorValue);
  if (hallSensorValue >= UPPER_THRESHOLD) {
    return "up" ; // Turn LED ON when voltage ≥ 2.8V
  } 
  else if (hallSensorValue <= LOWER_THRESHOLD) {
    return "down" ;  // Turn LED OFF when voltage ≤ 2.2V
  }
  else{
    return "N/A";
  }

}

std::string ultrasonic_read() {
  static char nameBuffer[5]; // 4 chars + null terminator
  static int charCount = 0;


  while (Serial2.available()) {
    char incomingChar = Serial2.read();

    // '#' is start of new name
    if (incomingChar == '#') {
      charCount = 0;
      nameBuffer[charCount++] = incomingChar;
    } else if (charCount > 0 && charCount < 4) {
      nameBuffer[charCount++] = incomingChar;
    }

    // If we've read 4 characters, null terminate and process name
    if (charCount == 4) {
      nameBuffer[4] = '\0'; // Null terminate the string
      charCount = 0; // Reset for the next name
      return std::string(nameBuffer);
    }
  }
  return std::string(nameBuffer);
}


void read_data(){
  //these are purely test inputs for the framework of sending accross the data between server and client
  //these http requests are parsed in the javascript in the html file

  measure_freq();
  
  std::string http_send = "radio=" + std::to_string(RF_freq) +
                          "&infrared=" + std::to_string(IR_freq) +
                          "&magnetic=" + hall_read() +
                          "&species=" + "" +
                          "&name=" + ultrasonic_read();
  server.send(200, "text/plain", http_send.c_str());
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

void setup() {
  Serial.begin(115200);
  Serial2.begin(600, SERIAL_8N1, 16, -1); // RX on GPIO 16, TX disabled
  //pinMode(RF_input_pin, INPUT_PULLUP);
  //pinMode(IR-input_pin, INPUT_PULLUP);

  //attachInterrupt(digitalPinToInterrupt(input_pin_1), pulse_ISR_1, RISING);
  //attachInterrupt(digitalPinToInterrupt(input_pin_2), pulse_ISR_2, RISING);
  ledcSetup(0, 5000, 8); // 5kHz, 8-bit resolution
  pinMode(LEFT_WHEEL_DIRECTION_PIN, OUTPUT);
  pinMode(RIGHT_WHEEL_DIRECTION_PIN, OUTPUT);
  ledcAttachPin(LEFT_WHEEL_CONTROL_PIN, 0); 
  ledcAttachPin(RIGHT_WHEEL_CONTROL_PIN, 1); 

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
