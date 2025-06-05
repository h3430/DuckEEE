#include <Arduino.h>

void setup() {
  Serial.begin(9600); // PIO default 9600 baud rate
  Serial2.begin(600, SERIAL_8N1, 16, -1); // RX on GPIO 16, TX disabled
}

void loop() {
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
      Serial.print("Duck name received: ");
      Serial.println(nameBuffer);
      charCount = 0; // Reset for the next name
    }
  }
}