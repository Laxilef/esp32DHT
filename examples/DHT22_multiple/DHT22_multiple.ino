#include <Arduino.h>
#include <Ticker.h>
#include <esp32DHT.h>

Ticker ticker;
DHT22 sensor;
uint8_t pins[] = {12, 13};
uint8_t currentPin = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // check if busy
  if (sensor.getStatus() != DHT::Status::NONE && sensor.getStatus() != DHT::Status::READY && sensor.getStatus() != DHT::Status::FAIL_ON_DRIVER) {
    delay(100);
    return;
  }

  constexpr size_t pinsCount = sizeof(pins) / sizeof(pins[0]);
  auto newPin = pins[random(pinsCount)];
  if (newPin != currentPin) {
    // clear old data before new setup
    sensor.end();

    sensor.onData([newPin](float humidity, float temperature) {
      Serial.printf("[%lu,\t%hhu] Temp: %g°C\nHumid: %g%%\n", millis(), newPin, temperature, humidity);
    });
    sensor.onError([newPin](DHT::Status status) {
      Serial.printf("[%lu,\t%hhu] Sensor error: %s\n", millis(), newPin, DHT::statusToString(status));
    });

    if (sensor.setup(newPin)) {
      currentPin = newPin;
      Serial.printf("[%lu,\t%hhu] Changed pin\n", millis(), newPin);

    } else {
      Serial.printf("[%lu,\t%hhu] ERROR: %s\n", millis(), newPin, DHT::statusToString(sensor.getStatus()));
    }
  }

  // check if ready
  if (sensor.getStatus() == DHT::Status::READY) {
    sensor.read();
    delay(5000);
  }
}
