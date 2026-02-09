/*

Copyright 2018 Bert Melis

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/


#include <Arduino.h>
#include <Ticker.h>
#include <esp32DHT.h>

Ticker ticker;
DHT22 sensor;
// DHT11 sensor;  // DHT11 also works!
DHT::Status prevStatus = DHT::Status::NONE;

void readDHT() {
  sensor.read();
}

void setup() {
  Serial.begin(74880);
  sensor.setup(12); // pin 23 is DATA
  sensor.onData([](float humidity, float temperature) {
    Serial.printf("[%lu] Temp: %g°C\nHumid: %g%%\n", millis(), temperature, humidity);
  });
  sensor.onError([](DHT::Status status) {
    Serial.printf("[%lu] Sensor error: %s\n", millis(), DHT::statusToString(status));
  });
  ticker.attach(30, readDHT);
}

void loop() {
  auto currentStatus = sensor.getStatus();
  if (currentStatus != prevStatus) {
    Serial.printf("[%lu] Status '%s' => '%s'\n", millis(), DHT::statusToString(prevStatus), DHT::statusToString(currentStatus));
    prevStatus = currentStatus;
  }
}
