#include "FourWheelDrive.h"

Motor::Motor(uint8_t pinA, uint8_t pinB, bool inverted)
    : _pinA(pinA), _pinB(pinB), _inverted(inverted) {}

void Motor::begin() {
    pinMode(_pinA, OUTPUT);
    pinMode(_pinB, OUTPUT);
    stop();
}

void Motor::drive(int speed) {
    if (_inverted) {
        speed = -speed;
    }

    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        analogWrite(_pinA, speed);
        analogWrite(_pinB, 0);
    } else if (speed < 0) {
        analogWrite(_pinA, 0);
        analogWrite(_pinB, -speed);
    } else {
        stop();
    }
}

void Motor::forward(uint8_t speed) {
    drive((int)speed);
}

void Motor::backward(uint8_t speed) {
    drive(-(int)speed);
}

void Motor::stop() {
    analogWrite(_pinA, 0);
    analogWrite(_pinB, 0);
    digitalWrite(_pinA, LOW);
    digitalWrite(_pinB, LOW);
}

void Motor::brake() {
    analogWrite(_pinA, 255);
    analogWrite(_pinB, 255);
    digitalWrite(_pinA, HIGH);
    digitalWrite(_pinB, HIGH);
}

void Motor::setInverted(bool inverted) {
    _inverted = inverted;
}

bool Motor::isInverted() const {
    return _inverted;
}

FourWheelDrive::FourWheelDrive(
    uint8_t flPinA, uint8_t flPinB,
    uint8_t frPinA, uint8_t frPinB,
    uint8_t rlPinA, uint8_t rlPinB,
    uint8_t rrPinA, uint8_t rrPinB
) : _fl(flPinA, flPinB),
    _fr(frPinA, frPinB),
    _rl(rlPinA, rlPinB),
    _rr(rrPinA, rrPinB) {}

void FourWheelDrive::begin() {
    _fl.begin();
    _fr.begin();
    _rl.begin();
    _rr.begin();
}

void FourWheelDrive::moveForward(uint8_t speed) {
    _fl.forward(speed);
    _fr.forward(speed);
    _rl.forward(speed);
    _rr.forward(speed);
}

void FourWheelDrive::moveBackward(uint8_t speed) {
    _fl.backward(speed);
    _fr.backward(speed);
    _rl.backward(speed);
    _rr.backward(speed);
}

void FourWheelDrive::spinLeft(uint8_t speed) {
    _fl.backward(speed);
    _rl.backward(speed);
    _fr.forward(speed);
    _rr.forward(speed);
}

void FourWheelDrive::spinRight(uint8_t speed) {
    _fl.forward(speed);
    _rl.forward(speed);
    _fr.backward(speed);
    _rr.backward(speed);
}

void FourWheelDrive::setSpeeds(int fl, int fr, int rl, int rr) {
    _fl.drive(fl);
    _fr.drive(fr);
    _rl.drive(rl);
    _rr.drive(rr);
}

void FourWheelDrive::stop() {
    _fl.stop();
    _fr.stop();
    _rl.stop();
    _rr.stop();
}

void FourWheelDrive::brake() {
    _fl.brake();
    _fr.brake();
    _rl.brake();
    _rr.brake();
}

void FourWheelDrive::setInversions(bool flInvert, bool frInvert, bool rlInvert, bool rrInvert) {
    _fl.setInverted(flInvert);
    _fr.setInverted(frInvert);
    _rl.setInverted(rlInvert);
    _rr.setInverted(rrInvert);
}
