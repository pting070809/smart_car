#pragma once

#include <Arduino.h>

#if defined(ESP32)
#include <esp_arduino_version.h>
#endif

/**
 * @brief ESP32 原生硬體 PWM (LEDC) 舵機控制類別
 * @details 支援 ESP32 Arduino Core 3.x 與 2.x，免安裝額外第三方函式庫。
 *          適用 SG90, MG90S, MG995, MG996R 等標準舵機。
 */
class ServoDriver {
public:
    ServoDriver(uint8_t pin = 8, uint16_t minUs = 500, uint16_t maxUs = 2500, int minAngle = 0, int maxAngle = 180)
        : _pin(pin),
          _minUs(minUs),
          _maxUs(maxUs),
          _minAngle(minAngle),
          _maxAngle(maxAngle),
          _attached(false),
          _currentAngle(90),
          _currentUs(1500),
          _channel(255) {}

    ~ServoDriver() {
        detach();
    }

    bool attach(uint8_t pin, uint16_t minUs = 500, uint16_t maxUs = 2500) {
        _pin = pin;
        _minUs = minUs;
        _maxUs = maxUs;

        const uint32_t freq = 50;        // 標準舵機頻率 50Hz (20ms 週期)
        const uint8_t resolution = 14;   // 14-bit 解析度 (0 ~ 16383)

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        // ESP32 Arduino Core 3.x
        if (!ledcAttach(_pin, freq, resolution)) {
            return false;
        }
#elif defined(ESP32)
        // ESP32 Arduino Core 2.x
        static uint8_t s_channelCounter = 8;
        if (_channel == 255) {
            _channel = s_channelCounter++;
            if (s_channelCounter > 15) s_channelCounter = 8;
        }
        ledcSetup(_channel, freq, resolution);
        ledcAttachPin(_pin, _channel);
#endif
        _attached = true;
        write(_currentAngle);
        return true;
    }

    void detach() {
        if (!_attached) return;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcDetach(_pin);
#elif defined(ESP32)
        if (_channel != 255) {
            ledcDetachPin(_pin);
        }
#endif
        _attached = false;
    }

    bool isAttached() const {
        return _attached;
    }

    void write(int angle) {
        if (!_attached) {
            attach(_pin, _minUs, _maxUs);
        }

        if (angle < _minAngle) angle = _minAngle;
        if (angle > _maxAngle) angle = _maxAngle;
        _currentAngle = angle;

        uint16_t us = _minUs + ((uint32_t)(angle - _minAngle) * (_maxUs - _minUs)) / (_maxAngle - _minAngle);
        writeMicroseconds(us);
    }

    void writeMicroseconds(uint16_t us) {
        if (!_attached) {
            attach(_pin, _minUs, _maxUs);
        }

        if (us < _minUs) us = _minUs;
        if (us > _maxUs) us = _maxUs;
        _currentUs = us;

        uint32_t duty = ((uint32_t)us * 16383UL) / 20000UL;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(_pin, duty);
#elif defined(ESP32)
        ledcWrite(_channel, duty);
#endif
    }

    void easeTo(int targetAngle, uint16_t stepDelayMs = 12) {
        if (targetAngle < _minAngle) targetAngle = _minAngle;
        if (targetAngle > _maxAngle) targetAngle = _maxAngle;

        int step = (targetAngle > _currentAngle) ? 1 : -1;
        while (_currentAngle != targetAngle) {
            write(_currentAngle + step);
            delay(stepDelayMs);
        }
    }

    int read() const {
        return _currentAngle;
    }

    uint16_t readMicroseconds() const {
        return _currentUs;
    }

    uint8_t getPin() const {
        return _pin;
    }

private:
    uint8_t _pin;
    uint16_t _minUs;
    uint16_t _maxUs;
    int _minAngle;
    int _maxAngle;
    bool _attached;
    int _currentAngle;
    uint16_t _currentUs;
    uint8_t _channel;
};
