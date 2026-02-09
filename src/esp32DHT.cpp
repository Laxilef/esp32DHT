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
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONDHTTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include "esp32DHT.hpp"  // NOLINT

DHT::DHT() :
  _status(DHT::Status::NONE),
  _data{0},
  _pin(0),
  _channel(nullptr),
  _onData(nullptr),
  _onError(nullptr),
  _task(nullptr),
  _queue(nullptr) {}

DHT::~DHT() {
  end();
}

void DHT::end() {
  if (_channel != nullptr) {
    rmt_disable(_channel);
    rmt_del_channel(_channel);
    _channel = nullptr;
  }

  if (_task != nullptr) {
    vTaskDelete(_task);
    _task = nullptr;
  }

  if (_queue != nullptr) {
    vQueueDelete(_queue);
    _queue = nullptr;
  }

  _status = Status::NONE;
}

bool DHT::setup(uint8_t pin) {
  _pin = pin;
  rmt_rx_channel_config_t rx_conf = {
    .gpio_num = static_cast<gpio_num_t>(_pin),
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,
    .mem_block_symbols = 128
  };

  if (rmt_new_rx_channel(&rx_conf, &_channel) != ESP_OK) {
    _status = Status::FAIL_ON_DRIVER;
    return false;
  }

  if (rmt_enable(_channel) != ESP_OK) {
    rmt_del_channel(_channel);
    _status = Status::FAIL_ON_DRIVER;
    return false;
  }

  rmt_rx_event_callbacks_t cbs = {
    .on_recv_done = _onRxDone
  };
  if (rmt_rx_register_event_callbacks(_channel, &cbs, this) != ESP_OK) {
    rmt_disable(_channel);
    rmt_del_channel(_channel);
    _status = Status::FAIL_ON_DRIVER;
    return false;
  }
  
  _queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
  xTaskCreate((TaskFunction_t)&_readSensor, "esp32DHT", 3072, this, 5, &_task);

  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, HIGH);
}

void DHT::onData(DHT::DataCallback callback) {
  _onData = callback;
}

void DHT::onError(DHT::ErrorCallback callback) {
  _onError = callback;
}

void DHT::read() {
  xTaskNotifyGive(_task);
}

DHT::Status DHT::getStatus() const {
  return _status;
}

const char* DHT::statusToString(const Status status) {
  switch (status) {
    case Status::NONE:
      return "NONE";
      break;

    case Status::WAITING:
      return "WAITING";
      break;

    case Status::REQUESTING:
      return "REQUESTING";
      break;

    case Status::RECEIVING:
      return "RECEIVING";
      break;

    case Status::RECEIVED:
      return "RECEIVED";
      break;

    case Status::TIMEOUT:
      return "TIMEOUT";
      break;

    case Status::BAD_DATA:
      return "BAD_DATA";
      break;

    case Status::BAD_CHECKSUM:
      return "BAD_CHECKSUM";
      break;

    case Status::UNDERFLOW_DATA:
      return "UNDERFLOW_DATA";
      break;

    case Status::OVERFLOW_DATA:
      return "OVERFLOW_DATA";
      break;

    case Status::NACK:
      return "NACK";
      break;

    case Status::FAIL_ON_DRIVER:
      return "FAIL_ON_DRIVER";
      break;

    default:
      return "UNKNOWN";
      break;
  }
}

void DHT::_readSensor(DHT* instance) {
  while (1) {
    // reset
    memset(instance->_data, 0, sizeof(instance->_data));
    instance->_status = Status::WAITING;

    // block and wait for notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // set status
    instance->_status = Status::REQUESTING;

    // give start signal to sensor
    digitalWrite(instance->_pin, LOW);
    vTaskDelay(18 / portTICK_PERIOD_MS);
    pinMode(instance->_pin, INPUT);

    // set status
    instance->_status = Status::RECEIVING;

    rmt_receive_config_t rx_config = {
      .signal_range_min_ns = 3000,
      .signal_range_max_ns = 1000000,
    };
    rmt_receive(instance->_channel, instance->_raw, sizeof(instance->_raw), &rx_config);

    // blocks until data is available or timeouts after 1s
    rmt_rx_done_event_data_t eData = {};
    if (xQueueReceive(instance->_queue, &eData, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
      instance->_decode(eData.received_symbols, eData.num_symbols);

    } else {
      instance->_status = Status::TIMEOUT;
    }

    pinMode(instance->_pin, OUTPUT);
    digitalWrite(instance->_pin, HIGH);

    // return results
    instance->_tryCallback();
  }
}

bool IRAM_ATTR DHT::_onRxDone(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *eData, void *uCtx) {
  BaseType_t wakeup = pdFALSE;
  DHT* instance = static_cast<DHT*>(uCtx);
  xQueueSendFromISR(instance->_queue, eData, &wakeup);
  return (wakeup == pdTRUE);
}

void DHT::_decode(const rmt_symbol_word_t* data, const size_t numItems) {
  uint8_t pulse = data[0].duration0 + data[0].duration1;

  if (numItems < 41) {
    _status = Status::UNDERFLOW_DATA;

  } else if (numItems > 42) {
    _status = Status::OVERFLOW_DATA;

  } else if (pulse < 130 || pulse > 180) {
    _status = Status::NACK;

  } else {
    for (uint8_t i = 1; i < 41; ++i) {  // don't include tail >40
      pulse = data[i].duration0 + data[i].duration1;

      if (pulse > 55 && pulse < 145) {
        _data[(i - 1) / 8] <<= 1;  // shift left
        if (pulse > 110) {
          _data[(i - 1) / 8] |= 1;
        }

      } else {
        _status = Status::BAD_DATA;
        return;
      }
    }

    if (_data[4] == ((_data[0] + _data[1] + _data[2] + _data[3]) & 0xFF)) {
      _status = Status::RECEIVED;

    } else {
      _status = Status::BAD_CHECKSUM;
    }
  }
}

void DHT::_tryCallback() {
  if (_status == Status::RECEIVED && _onData) {
    _onData(_getHumidity(), _getTemperature());

  } else if (_status != Status::RECEIVED && _onError) {
    _onError(_status);
  }
}

float DHT11::_getTemperature() {
  return _status == Status::RECEIVED
    ? static_cast<float>(_data[2])
    : NAN;
}

float DHT11::_getHumidity() {
  return _status == Status::RECEIVED
    ? static_cast<float>(_data[0])
    : NAN;
}

float DHT22::_getTemperature() {
  if (_status != Status::RECEIVED) {
    return NAN;
  }

  float temp = (((_data[2] & 0x7F) << 8) | _data[3]) * 0.1;
  if (_data[2] & 0x80) {  // negative temperature
    temp = -temp;
  }

  return temp;
}

float DHT22::_getHumidity() {
  return _status == Status::RECEIVED
    ? (((_data[0] << 8) | _data[1]) * 0.1)
    : NAN;
}
