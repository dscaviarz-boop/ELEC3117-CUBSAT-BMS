#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP32 MCU board boot successful");
  Serial.println("================================");

  Serial.printf("Chip model: %s\n", ESP.getChipModel());
  Serial.printf("Chip revision: %d\n", ESP.getChipRevision());
  Serial.printf("CPU frequency: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash size: %u bytes\n", ESP.getFlashChipSize());
  Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
}

void loop()
{
  static uint32_t counter = 0;

  Serial.printf(
      "MCU running | Count: %lu | Uptime: %lu ms | Free heap: %u\n",
      counter++,
      millis(),
      ESP.getFreeHeap());

  delay(1000);
}