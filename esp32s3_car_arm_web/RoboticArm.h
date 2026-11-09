#pragma once

#include <Arduino.h>
#include <math.h>
#include "ServoDriver.h"

/**
 * ==============================================================================
 *  4-DOF 車載機械手臂控制與運動學類別 (Car-Mounted Robotic Arm)
 * ==============================================================================
 *  結構特點：
 *   - 無底座旋轉舵機（由小車輪胎轉向充當 Yaw 方向）
 *   - 關節 1 (Shoulder): 大臂俯仰角度 (前後抬升)
 *   - 關節 2 (Elbow)   : 小臂俯仰角度 (前後伸展)
 *   - 關節 3 (Wrist)   : 手腕滾轉角度 (爪子水平/垂直旋轉 Roll)
 *   - 關節 4 (Gripper) : 夾爪開合 (抓取/放開)
 * ==============================================================================
 */
class RoboticArm {
public:
    /**
     * @brief 建構子
     * @param pinShoulder 大臂舵機腳位 (預設 GPIO 8)
     * @param pinElbow    小臂舵機腳位 (預設 GPIO 9)
     * @param pinWrist    手腕滾轉腳位 (預設 GPIO 10)
     * @param pinGripper  夾爪腳位     (預設 GPIO 11)
     */
    RoboticArm(
        uint8_t pinShoulder = 8,
        uint8_t pinElbow    = 9,
        uint8_t pinWrist    = 10,
        uint8_t pinGripper  = 11
    ) : _pinShoulder(pinShoulder),
        _pinElbow(pinElbow),
        _pinWrist(pinWrist),
        _pinGripper(pinGripper),
        _l1(105.0f),  // 大臂長度預設 105mm (可透過 setArmDimensions 自訂)
        _l2(120.0f),  // 小臂至爪尖預設 120mm
        _h(65.0f),    // 肩部離地高度預設 65mm
        _angleShoulder(90),
        _angleElbow(90),
        _angleWrist(90),
        _angleGripper(40),
        _currentX(0.0f),
        _currentZ(0.0f) {}

    void begin() {
        _shoulder.attach(_pinShoulder);
        _elbow.attach(_pinElbow);
        _wrist.attach(_pinWrist);
        _gripper.attach(_pinGripper);

        // 預設開機姿態 (回待命位)
        setAngles(90, 90, 90, 45);
    }

    /**
     * @brief 設定機械臂幾何尺寸 (單位: 毫米 mm)
     * @param l1 大臂長度 (肩軸心到肘軸心)
     * @param l2 小臂長度 (肘軸心到爪尖)
     * @param h  肩部離地高度
     */
    void setArmDimensions(float l1, float l2, float h) {
        _l1 = l1;
        _l2 = l2;
        _h = h;
        calculateFK();
    }

    /**
     * @brief 直接設定 4 個舵機的角度 (0 ~ 180 度)
     */
    void setAngles(int shoulder, int elbow, int wrist, int gripper) {
        _angleShoulder = constrain(shoulder, 10, 175);
        _angleElbow    = constrain(elbow, 10, 175);
        _angleWrist    = constrain(wrist, 0, 180);
        _angleGripper  = constrain(gripper, 0, 180);

        _shoulder.write(_angleShoulder);
        _elbow.write(_angleElbow);
        _wrist.write(_angleWrist);
        _gripper.write(_angleGripper);

        calculateFK();
    }

    void setShoulder(int angle) {
        _angleShoulder = constrain(angle, 10, 175);
        _shoulder.write(_angleShoulder);
        calculateFK();
    }

    void setElbow(int angle) {
        _angleElbow = constrain(angle, 10, 175);
        _elbow.write(_angleElbow);
        calculateFK();
    }

    void setWrist(int angle) {
        _angleWrist = constrain(angle, 0, 180);
        _wrist.write(_angleWrist);
    }

    void setGripper(int angle) {
        _angleGripper = constrain(angle, 0, 180);
        _gripper.write(_angleGripper);
    }

    /**
     * @brief 正向運動學 (Forward Kinematics, FK)
     * 計算爪子相對於車體/肩部的前方距離 X 與離地高度 Z
     * 幾何關係：
     *   - 設大臂水平朝前為 0°，垂直朝上為 90°，朝後為 180° (一般舵機以 90° 為中位垂直)
     *   - rad1 = (shoulderAngle - 0) 轉弧度
     *   - rad2 = (shoulderAngle + elbowAngle - 180) 轉弧度 (相對於水平面)
     */
    void calculateFK() {
        float rad1 = radians((float)_angleShoulder);
        // 小臂相對於水平面的絕對傾角：假設 90/90 時大臂垂直、小臂水平向前
        float rad2 = radians((float)(_angleShoulder + _angleElbow - 90));

        _currentX = _l1 * cos(rad1) + _l2 * cos(rad2);
        _currentZ = _h + _l1 * sin(rad1) + _l2 * sin(rad2);
    }

    /**
     * @brief 逆向運動學 (Inverse Kinematics, IK)
     * @details 給定目標前方距離 targetX 與離地高度 targetZ，反推大臂與小臂角度
     * @return true 若目標在可達範圍內且成功解算，false 若超出工作半徑
     */
    bool moveToCoordinate(float targetX, float targetZ) {
        float dx = targetX;
        float dz = targetZ - _h; // 相對於肩部的高度
        float distSq = dx * dx + dz * dz;
        float dist = sqrt(distSq);

        // 工作空間半徑檢查 (餘弦定理成立條件)
        if (dist > (_l1 + _l2) || dist < fabs(_l1 - _l2) || dist < 1.0f) {
            return false; // 超出工作半徑 (Unreachable)
        }

        // 餘弦定理求解三角形內角
        float cosA = (_l1 * _l1 + distSq - _l2 * _l2) / (2.0f * _l1 * dist);
        cosA = constrain(cosA, -1.0f, 1.0f);
        float angleA = acos(cosA); // rad

        float cosB = (_l1 * _l1 + _l2 * _l2 - distSq) / (2.0f * _l1 * _l2);
        cosB = constrain(cosB, -1.0f, 1.0f);
        float angleB = acos(cosB); // rad

        // 目標點相對於肩部的極角
        float angleBase = atan2(dz, dx); // rad

        // 計算對應的舵機角 (度)
        float sDeg = degrees(angleBase + angleA);
        float eDeg = 180.0f - degrees(angleB);

        int targetShoulder = (int)round(sDeg);
        int targetElbow = (int)round(eDeg);

        // 安全限位檢查
        if (targetShoulder < 10 || targetShoulder > 175 || targetElbow < 10 || targetElbow > 175) {
            return false;
        }

        setShoulder(targetShoulder);
        setElbow(targetElbow);
        return true;
    }

    // ================== [預設姿態巨集] ==================
    void home() {
        setAngles(90, 90, 90, 45); // 待命垂直折疊姿態
    }

    void groundPick() {
        setAngles(45, 120, 90, 80); // 俯身貼地抓取
    }

    void liftUp() {
        setAngles(110, 60, 90, 20); // 抬高並收縮夾緊
    }

    void openGripper() {
        setGripper(90); // 完全張開
    }

    void closeGripper() {
        setGripper(25); // 閉合夾緊
    }

    void setWristHorizontal() {
        setWrist(90);  // 橫向水平夾
    }

    void setWristVertical() {
        setWrist(0);   // 垂直豎向夾
    }

    // ================== [狀態讀取] ==================
    int getShoulder() const { return _angleShoulder; }
    int getElbow() const    { return _angleElbow; }
    int getWrist() const    { return _angleWrist; }
    int getGripper() const  { return _angleGripper; }
    float getX() const      { return _currentX; }
    float getZ() const      { return _currentZ; }
    float getL1() const     { return _l1; }
    float getL2() const     { return _l2; }
    float getH() const      { return _h; }

private:
    uint8_t _pinShoulder, _pinElbow, _pinWrist, _pinGripper;
    ServoDriver _shoulder;
    ServoDriver _elbow;
    ServoDriver _wrist;
    ServoDriver _gripper;

    float _l1, _l2, _h;
    int _angleShoulder, _angleElbow, _angleWrist, _angleGripper;
    float _currentX, _currentZ;
};
