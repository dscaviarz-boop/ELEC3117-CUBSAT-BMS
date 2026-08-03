#include <Arduino.h>

const int PIN_STATUS_LED = 18;

void setup()
{
  Serial.begin(115200);

  pinMode(PIN_STATUS_LED, OUTPUT);

  Serial.println("ESP32 board test started");
}

void loop()
{
  digitalWrite(PIN_STATUS_LED, HIGH);
  Serial.println("LED ON");
  delay(500);

  digitalWrite(PIN_STATUS_LED, LOW);
  Serial.println("LED OFF");
  delay(500);
}