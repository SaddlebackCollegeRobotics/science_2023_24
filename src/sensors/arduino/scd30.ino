// Basic demo for readings from Adafruit SCD30
#include <Adafruit_SCD30.h>
#include <ArduinoJson.h>

Adafruit_SCD30 scd30;

void setup(void) {
  Serial.begin(9600); // 115200
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Try to initialize!
  if (!scd30.begin()) {
    Serial.println("Failed to find SCD30 chip");
    while (1) {
      delay(10);
    }
  }

  scd30.setMeasurementInterval(2); // Set interval to 10 seconds
}

void loop() {
  if (scd30.dataReady()) {
    if (!scd30.read()) {
      Serial.println("Error reading sensor data");
      return;
    }

    StaticJsonDocument<200> doc;
    doc["temperature"] = scd30.temperature;
    doc["relative_humidity"] = scd30.relative_humidity;
    doc["co2"] = scd30.CO2;

    serializeJson(doc, Serial);
    Serial.println();
  }

  delay(100);
}