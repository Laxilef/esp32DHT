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

#pragma once

extern "C" {
  #include <freertos/FreeRTOS.h>
  #include <freertos/task.h>
  #include <freertos/queue.h>
  #include <esp32-hal-gpio.h>
  #include <driver/rmt_rx.h>
  #include <esp_timer.h>
}
#include <functional>

class DHT {
 public:
  enum class Status {
    NONE,
    WAITING,
    REQUESTING,
    RECEIVING,
    RECEIVED,
    TIMEOUT,
    BAD_DATA,
    BAD_CHECKSUM,
    UNDERFLOW_DATA,
    OVERFLOW_DATA,
    NACK,
    FAIL_ON_DRIVER
  };
  typedef std::function<void(float humidity, float temperature)> DataCallback;
  typedef std::function<void(Status status)> ErrorCallback;

  DHT();
  ~DHT();
  void end();
  bool setup(uint8_t pin);
  void onData(DataCallback callback);
  void onError(ErrorCallback callback);
  void read();
  Status getStatus() const;
  static const char* statusToString(const Status status);

 protected:
  Status _status;
  uint8_t _data[5];

 private:
  static void _readSensor(DHT* instance);
  static bool _onRxDone(rmt_channel_handle_t, const rmt_rx_done_event_data_t*, void*);
  void _decode(const rmt_symbol_word_t* data, const size_t numItems);
  void _tryCallback();
  virtual float _getTemperature() = 0;
  virtual float _getHumidity() = 0;

 private:
  uint8_t _pin;
  rmt_channel_handle_t _channel;
  DataCallback _onData;
  ErrorCallback _onError;
  TaskHandle_t _task;
  rmt_symbol_word_t _raw[128];
  QueueHandle_t _queue;
};

class DHT11 : public DHT {
 private:
  float _getTemperature() override;
  float _getHumidity() override;
};

class DHT22 : public DHT {
 private:
  float _getTemperature() override;
  float _getHumidity() override;
};
