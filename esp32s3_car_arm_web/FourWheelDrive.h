#pragma once

#include <Arduino.h>

/**
 * @brief 直流馬達控制類別 (適用於具備 1A/1B、2A/2B 腳位之雙 H 橋驅動模組，如 MX1508 / L9110 / HS-F09)
 */
class Motor {
public:
    Motor(uint8_t pinA, uint8_t pinB, bool inverted = false);

    void begin();
    void drive(int speed);        // -255 ~ +255
    void forward(uint8_t speed);
    void backward(uint8_t speed);
    void stop();
    void brake();
    void setInverted(bool inverted);
    bool isInverted() const;

private:
    uint8_t _pinA;
    uint8_t _pinB;
    bool _inverted;
};

/**
 * @brief 4WD 底盤全車控制類別
 */
class FourWheelDrive {
public:
#if defined(ESP32)
    FourWheelDrive(
        uint8_t flPinA = 4,  uint8_t flPinB = 5,
        uint8_t frPinA = 6,  uint8_t frPinB = 7,
        uint8_t rlPinA = 15, uint8_t rlPinB = 16,
        uint8_t rrPinA = 17, uint8_t rrPinB = 18
    );
#else
    FourWheelDrive(
        uint8_t flPinA = 2, uint8_t flPinB = 3,
        uint8_t frPinA = 4, uint8_t frPinB = 5,
        uint8_t rlPinA = 6, uint8_t rlPinB = 7,
        uint8_t rrPinA = 8, uint8_t rrPinB = 9
    );
#endif

    void begin();
    void moveForward(uint8_t speed);
    void moveBackward(uint8_t speed);
    void spinLeft(uint8_t speed);
    void spinRight(uint8_t speed);
    void setSpeeds(int fl, int fr, int rl, int rr);
    void stop();
    void brake();
    void setInversions(bool flInvert, bool frInvert, bool rlInvert, bool rrInvert);

    Motor& fl() { return _fl; }
    Motor& fr() { return _fr; }
    Motor& rl() { return _rl; }
    Motor& rr() { return _rr; }

private:
    Motor _fl;
    Motor _fr;
    Motor _rl;
    Motor _rr;
};
