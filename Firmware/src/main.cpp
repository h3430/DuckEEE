#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
    Serial.begin(9600);
      delay(100);
        Serial.println("HelloWorld");
}

void loop() {
        delay(1000);
        Serial.println("HelloWorld");
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}