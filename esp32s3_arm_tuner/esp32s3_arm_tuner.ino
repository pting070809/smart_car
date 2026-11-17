#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_VL53L0X.h"

/**
 * ==============================================================================
 *    ESP32-S3 4WD 戰術小車 + 4-DOF 機械手臂 + ToF 雷射測距 + AI JSON API 中控系統
 * ==============================================================================
 *
 * 【腳位與硬體資源分配總表 (ESP32-S3)】:
 * ------------------------------------------------------------------------------
 *  [ToF 雷射測距感測器] (I2C 通訊, 100kHz):
 *   - SCL   -> GPIO 10 (硬體 I2C 專用)
 *   - SDA   -> GPIO 11 (硬體 I2C 專用)
 *   - XSHUT -> 5V 或 3.3V, VIN -> 3.3V, GND -> GND
 *
 *  [機械手臂 4 顆舵機] (50Hz PWM, 14-bit 解析度, LEDC Ch 0~3):
 *   - 舵機 1 (大臂 Shoulder) : GPIO 8   -> Channel 0
 *   - 舵機 2 (小臂 Elbow)    : GPIO 9   -> Channel 1
 *   - 舵機 3 (手腕 Wrist)    : GPIO 12  -> Channel 2
 *   - 舵機 4 (夾爪 Gripper)  : GPIO 13  -> Channel 3
 *
 *  [底盤 4WD 馬達] (1000Hz PWM, 8-bit 解析度, LEDC Ch 4~7):
 *   - 左前輪 (FL): 1A -> GPIO 4,  1B -> GPIO 5
 *   - 左後輪 (RL): 1A -> GPIO 15, 1B -> GPIO 16
 *     => 左側前進綁定 Channel 4, 左側後退綁定 Channel 5
 *   - 右前輪 (FR): 2A -> GPIO 6,  2B -> GPIO 7
 *   - 右後輪 (RR): 2A -> GPIO 17, 2B -> GPIO 18
 *     => 右側前進綁定 Channel 6, 右側後退綁定 Channel 7
 * ------------------------------------------------------------------------------
 * 【對外 API 接收接口】:
 *   - POST /api/command  : AI / 上位機標準指令接口 (支援車輛、手臂角度、空間座標 IK)
 *   - GET  /api/tof      : 即時雷射測距查詢 (JSON: distance_mm, distance_cm, valid)
 *   - GET  /api/status   : 全車全臂完整感測器狀態
 *   - GET  /api/last     : 最後一筆收到的 API 指令原文 (回顯監控)
 *   - GET  /car?act=&spd=: 輕量車控 GET 路由
 *   - GET  /set?s=&e=&w=: 輕量臂控 GET 路由
 * ==============================================================================
 */

// ===================== [WiFi 設定] =====================
const char* STA_SSID = "TING";
const char* STA_PASS = "0966022557";
const char* AP_SSID  = "ESP32-CAR-ARM";
const char* AP_PASS  = "12345678";

// ===================== [硬體腳位定義] =====================
// ToF 雷射測距
#define TOF_I2C_SCL 10
#define TOF_I2C_SDA 11

// 舵機腳位
const uint8_t PIN_SERVO_SHOULDER = 8;
const uint8_t PIN_SERVO_ELBOW    = 9;
const uint8_t PIN_SERVO_WRIST    = 12;
const uint8_t PIN_SERVO_GRIPPER  = 13;

const uint8_t CH_SERVO_SHOULDER  = 0;
const uint8_t CH_SERVO_ELBOW     = 1;
const uint8_t CH_SERVO_WRIST     = 2;
const uint8_t CH_SERVO_GRIPPER   = 3;

// 馬達腳位
const uint8_t PIN_MOTOR_FL_A = 4;
const uint8_t PIN_MOTOR_FL_B = 5;
const uint8_t PIN_MOTOR_RL_A = 15;
const uint8_t PIN_MOTOR_RL_B = 16;

const uint8_t PIN_MOTOR_FR_A = 6;
const uint8_t PIN_MOTOR_FR_B = 7;
const uint8_t PIN_MOTOR_RR_A = 17;
const uint8_t PIN_MOTOR_RR_B = 18;

const uint8_t CH_MOTOR_L_FWD = 4;
const uint8_t CH_MOTOR_L_BWD = 5;
const uint8_t CH_MOTOR_R_FWD = 6;
const uint8_t CH_MOTOR_R_BWD = 7;

// 舵機規格
const uint16_t SERVO_MIN_US = 500;
const uint16_t SERVO_MAX_US = 2500;

// 機械手臂實測尺寸 (mm)
const float ARM_L1 = 160.0f;
const float ARM_L2 = 115.0f;
const float ARM_H  = 113.0f;

// ===================== [ToF 雷射感測器實例] =====================
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool tofOnline = false;
float tofDistanceMm = -1.0f;
float tofDistanceCm = -1.0f;
bool tofValid = false;
unsigned long lastTofRead = 0;
const unsigned long TOF_INTERVAL_MS = 60;

// ===================== [全域狀態變數] =====================
int angleShoulder = 90;
int angleElbow    = 90;
int angleWrist    = 90;
int angleGripper  = 45;

int carSpeed = 200;
String currentCarAction = "stop";
unsigned long carAutoStopTime = 0;

// 上位機 / AI API 監控追蹤
String lastIncomingJson = "{\"status\":\"none\"}";
uint32_t incomingCmdCount = 0;
unsigned long lastIncomingTime = 0;

WebServer server(80);

// ===================== [ToF 雷射感測器驅動] =====================
bool initTof() {
    Wire.begin(TOF_I2C_SDA, TOF_I2C_SCL);
    Wire.setClock(100000);
    tofOnline = lox.begin(VL53L0X_I2C_ADDR, false, &Wire);
    if (tofOnline) {
        Serial.println(F("[ToF] ✅ VL53L0X/L1X 雷射測距模組初始化成功！(SCL:10, SDA:11)"));
    } else {
        Serial.println(F("[ToF] ⚠️ 雷射測距模組初始化暫時失敗，稍後自動重連..."));
    }
    return tofOnline;
}

void updateTof() {
    if (millis() - lastTofRead < TOF_INTERVAL_MS) return;
    lastTofRead = millis();

    if (!tofOnline) {
        static unsigned long lastRetry = 0;
        if (millis() - lastRetry > 2000) {
            lastRetry = millis();
            initTof();
        }
        return;
    }

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 8190 && measure.RangeMilliMeter >= 20) {
        tofDistanceMm = (float)measure.RangeMilliMeter;
        tofDistanceCm = tofDistanceMm / 10.0f;
        tofValid = true;
    } else {
        tofDistanceMm = -1.0f;
        tofDistanceCm = -1.0f;
        tofValid = false;
    }
}

// ===================== [機械手臂驅動] =====================
void setServoPulse(uint8_t pin, uint8_t channel, uint16_t us) {
    us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
    uint32_t duty = (uint32_t)us * 16383UL / 20000UL;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    if (!ledcWrite(pin, duty)) {
        ledcWriteChannel(channel, duty);
    }
#else
    ledcWrite(channel, duty);
#endif
}

void writeAngle(uint8_t pin, uint8_t channel, int angle, int minAng = 0, int maxAng = 180) {
    angle = constrain(angle, minAng, maxAng);
    uint16_t us = SERVO_MIN_US + ((uint32_t)(angle - minAng) * (SERVO_MAX_US - SERVO_MIN_US)) / (maxAng - minAng);
    setServoPulse(pin, channel, us);
}

void applyArmAngles(int s, int e, int w, int g) {
    angleShoulder = constrain(s, 15, 165);
    angleElbow    = constrain(e, 15, 165);
    angleWrist    = constrain(w, 0, 180);
    angleGripper  = constrain(g, 10, 100);

    writeAngle(PIN_SERVO_SHOULDER, CH_SERVO_SHOULDER, angleShoulder, 0, 180);
    writeAngle(PIN_SERVO_ELBOW,    CH_SERVO_ELBOW,    angleElbow,    0, 180);
    writeAngle(PIN_SERVO_WRIST,    CH_SERVO_WRIST,    angleWrist,    0, 180);
    writeAngle(PIN_SERVO_GRIPPER,  CH_SERVO_GRIPPER,  angleGripper,  0, 180);

    float rad1 = radians((float)angleShoulder);
    float rad2 = radians((float)(angleShoulder - angleElbow));
    float curX = ARM_L1 * cos(rad1) + ARM_L2 * cos(rad2);
    float curZ = ARM_H + ARM_L1 * sin(rad1) + ARM_L2 * sin(rad2);

    Serial.printf("[ARM] 肩:%3d° | 肘:%3d° | 腕:%3d° | 爪:%3d°  => [X=%.1f mm, Z=%.1f mm]\n",
                  angleShoulder, angleElbow, angleWrist, angleGripper, curX, curZ);
}

// 空間座標逆運動學 (IK)
bool moveArmToCoordinate(float targetX, float targetZ) {
    float dx = targetX;
    float dz = targetZ - ARM_H;
    float distSq = dx * dx + dz * dz;
    float dist = sqrt(distSq);

    if (dist > (ARM_L1 + ARM_L2) || dist < fabs(ARM_L1 - ARM_L2) || dist < 1.0f) {
        Serial.printf("[IK] 目標 (%.1f, %.1f) 超出機械臂最大半徑 (%.1f mm)\n", targetX, targetZ, ARM_L1 + ARM_L2);
        return false;
    }

    float cosA = (ARM_L1 * ARM_L1 + distSq - ARM_L2 * ARM_L2) / (2.0f * ARM_L1 * dist);
    cosA = constrain(cosA, -1.0f, 1.0f);
    float angleA = acos(cosA);

    float cosB = (ARM_L1 * ARM_L1 + ARM_L2 * ARM_L2 - distSq) / (2.0f * ARM_L1 * ARM_L2);
    cosB = constrain(cosB, -1.0f, 1.0f);
    float angleB = acos(cosB);

    float angleBase = atan2(dz, dx);

    float sDeg = degrees(angleBase + angleA);
    float eDeg = 180.0f - degrees(angleB);

    int targetShoulder = (int)round(sDeg);
    int targetElbow = (int)round(eDeg);

    if (targetShoulder < 15 || targetShoulder > 165 || targetElbow < 15 || targetElbow > 165) {
        Serial.printf("[IK] 逆解角度超限: 肩=%d°, 肘=%d°\n", targetShoulder, targetElbow);
        return false;
    }

    applyArmAngles(targetShoulder, targetElbow, angleWrist, angleGripper);
    return true;
}

void initServos() {
    const uint32_t freq = 50;
    const uint8_t resolution = 14;

    pinMode(PIN_SERVO_SHOULDER, OUTPUT);
    pinMode(PIN_SERVO_ELBOW,    OUTPUT);
    pinMode(PIN_SERVO_WRIST,    OUTPUT);
    pinMode(PIN_SERVO_GRIPPER,  OUTPUT);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(PIN_SERVO_SHOULDER, freq, resolution);
    ledcAttach(PIN_SERVO_ELBOW,    freq, resolution);
    ledcAttach(PIN_SERVO_WRIST,    freq, resolution);
    ledcAttach(PIN_SERVO_GRIPPER,  freq, resolution);
#else
    ledcSetup(CH_SERVO_SHOULDER, freq, resolution); ledcAttachPin(PIN_SERVO_SHOULDER, CH_SERVO_SHOULDER);
    ledcSetup(CH_SERVO_ELBOW,    freq, resolution); ledcAttachPin(PIN_SERVO_ELBOW,    CH_SERVO_ELBOW);
    ledcSetup(CH_SERVO_WRIST,    freq, resolution); ledcAttachPin(PIN_SERVO_WRIST,    CH_SERVO_WRIST);
    ledcSetup(CH_SERVO_GRIPPER,  freq, resolution); ledcAttachPin(PIN_SERVO_GRIPPER,  CH_SERVO_GRIPPER);
#endif

    applyArmAngles(90, 90, 90, 45);
    Serial.println(F("[INIT] 4 顆舵機硬體 PWM (LEDC 50Hz) 初始化就緒"));
}

// ===================== [4WD 底盤馬達驅動] =====================
void carDrive(int leftSpeed, int rightSpeed) {
    leftSpeed  = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (leftSpeed > 0) {
        ledcWriteChannel(CH_MOTOR_L_FWD, leftSpeed);
        ledcWriteChannel(CH_MOTOR_L_BWD, 0);
    } else if (leftSpeed < 0) {
        ledcWriteChannel(CH_MOTOR_L_FWD, 0);
        ledcWriteChannel(CH_MOTOR_L_BWD, -leftSpeed);
    } else {
        ledcWriteChannel(CH_MOTOR_L_FWD, 0);
        ledcWriteChannel(CH_MOTOR_L_BWD, 0);
    }

    if (rightSpeed > 0) {
        ledcWriteChannel(CH_MOTOR_R_FWD, rightSpeed);
        ledcWriteChannel(CH_MOTOR_R_BWD, 0);
    } else if (rightSpeed < 0) {
        ledcWriteChannel(CH_MOTOR_R_FWD, 0);
        ledcWriteChannel(CH_MOTOR_R_BWD, -rightSpeed);
    } else {
        ledcWriteChannel(CH_MOTOR_R_FWD, 0);
        ledcWriteChannel(CH_MOTOR_R_BWD, 0);
    }
}

void carStop() {
    carDrive(0, 0);
    currentCarAction = "stop";
}

void carBrake() {
    ledcWriteChannel(CH_MOTOR_L_FWD, 255);
    ledcWriteChannel(CH_MOTOR_L_BWD, 255);
    ledcWriteChannel(CH_MOTOR_R_FWD, 255);
    ledcWriteChannel(CH_MOTOR_R_BWD, 255);
    currentCarAction = "brake";
}

void executeCar(const String& action, int speed, int durationMs = 0) {
    speed = constrain(speed, 60, 255);
    currentCarAction = action;

    if (action == "forward") {
        carDrive(speed, speed);
    } else if (action == "backward") {
        carDrive(-speed, -speed);
    } else if (action == "left") {
        carDrive(speed / 3, speed);
    } else if (action == "right") {
        carDrive(speed, speed / 3);
    } else if (action == "spin_left") {
        carDrive(-speed, speed);
    } else if (action == "spin_right") {
        carDrive(speed, -speed);
    } else if (action == "brake") {
        carBrake();
    } else {
        carStop();
    }

    if (durationMs > 0 && action != "stop" && action != "brake") {
        carAutoStopTime = millis() + durationMs;
    } else {
        carAutoStopTime = 0;
    }

    Serial.printf("[CAR] 動作: %-10s | 速度: %3d | 限時: %d ms\n", action.c_str(), speed, durationMs);
}

void initMotors() {
    const uint32_t motorFreq = 1000;
    const uint8_t motorRes = 8;

    pinMode(PIN_MOTOR_FL_A, OUTPUT);
    pinMode(PIN_MOTOR_FL_B, OUTPUT);
    pinMode(PIN_MOTOR_RL_A, OUTPUT);
    pinMode(PIN_MOTOR_RL_B, OUTPUT);

    pinMode(PIN_MOTOR_FR_A, OUTPUT);
    pinMode(PIN_MOTOR_FR_B, OUTPUT);
    pinMode(PIN_MOTOR_RR_A, OUTPUT);
    pinMode(PIN_MOTOR_RR_B, OUTPUT);

    ledcAttachChannel(PIN_MOTOR_FL_A, motorFreq, motorRes, CH_MOTOR_L_FWD);
    ledcAttachChannel(PIN_MOTOR_RL_A, motorFreq, motorRes, CH_MOTOR_L_FWD);
    ledcAttachChannel(PIN_MOTOR_FL_B, motorFreq, motorRes, CH_MOTOR_L_BWD);
    ledcAttachChannel(PIN_MOTOR_RL_B, motorFreq, motorRes, CH_MOTOR_L_BWD);

    ledcAttachChannel(PIN_MOTOR_FR_A, motorFreq, motorRes, CH_MOTOR_R_FWD);
    ledcAttachChannel(PIN_MOTOR_RR_A, motorFreq, motorRes, CH_MOTOR_R_FWD);
    ledcAttachChannel(PIN_MOTOR_FR_B, motorFreq, motorRes, CH_MOTOR_R_BWD);
    ledcAttachChannel(PIN_MOTOR_RR_B, motorFreq, motorRes, CH_MOTOR_R_BWD);

    carStop();
    Serial.println(F("[INIT] 4WD 底盤馬達硬體 PWM (LEDC 1000Hz, Ch 4~7) 初始化就緒"));
}

// ===================== [一體化控制網頁 (ToF + 車控 + 臂控 + API 沙盒)] =====================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-TW">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>ESP32-S3 戰術車臂控制中樞</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; user-select: none; }
    body { background: #090d16; color: #f8fafc; padding: 12px; display: flex; flex-direction: column; align-items: center; }
    .container { width: 100%; max-width: 500px; }
    h1 { font-size: 18px; text-align: center; margin-bottom: 10px; color: #38bdf8; letter-spacing: 1px; }
    .card { background: rgba(30, 41, 59, 0.85); border-radius: 14px; padding: 14px; margin-bottom: 12px; border: 1px solid rgba(56, 189, 248, 0.2); box-shadow: 0 4px 14px rgba(0,0,0,0.4); }
    .card-title { font-size: 14px; font-weight: 700; color: #38bdf8; margin-bottom: 10px; display: flex; align-items: center; gap: 6px; }

    /* ToF 雷射測距卡片 */
    .tof-card { background: linear-gradient(135deg, rgba(6,182,212,0.15) 0%, rgba(15,23,42,0.9) 100%); border-color: rgba(6,182,212,0.4); }
    .tof-display { display: flex; align-items: baseline; justify-content: center; gap: 8px; margin: 6px 0; }
    .tof-num { font-size: 32px; font-weight: 800; color: #06b6d4; font-family: monospace; }
    .tof-unit { font-size: 14px; color: #94a3b8; font-weight: 600; }
    .tof-bar-bg { width: 100%; height: 8px; background: #0b1120; border-radius: 4px; overflow: hidden; margin-top: 6px; }
    .tof-bar-fill { height: 100%; width: 0%; background: #06b6d4; transition: width 0.1s linear, background 0.15s; }

    /* 車控九宮格 */
    .dpad-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-bottom: 10px; }
    .d-btn { aspect-ratio: 1.15; border-radius: 12px; background: #1e293b; border: 1px solid rgba(255,255,255,0.12); color: #f8fafc; font-size: 13px; font-weight: 700; cursor: pointer; display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 2px; transition: all 0.08s; touch-action: none; }
    .d-btn:active, .d-btn.active { background: #0284c7; color: #fff; transform: scale(0.95); border-color: #38bdf8; box-shadow: 0 0 12px rgba(6,182,212,0.6); }
    .d-btn.stop { background: #7f1d1d; color: #fca5a5; border-color: #ef4444; }
    .d-btn.stop:active, .d-btn.stop.active { background: #dc2626; color: #fff; }
    .d-btn.spin { color: #fbbf24; border-color: rgba(245,158,11,0.3); }

    /* 手臂座標與滑桿 */
    .coord-box { display: flex; justify-content: space-around; background: #050811; padding: 8px; border-radius: 8px; margin-bottom: 10px; border: 1px solid rgba(255,255,255,0.06); }
    .coord-item { text-align: center; }
    .coord-lbl { font-size: 11px; color: #94a3b8; }
    .coord-val { font-size: 16px; font-weight: 700; color: #06b6d4; font-family: monospace; }
    .slider-row { margin-bottom: 12px; }
    .slider-header { display: flex; justify-content: space-between; font-size: 12px; font-weight: 600; margin-bottom: 4px; }
    .slider-val { color: #38bdf8; font-family: monospace; }
    .slider-ctl { display: flex; align-items: center; gap: 8px; }
    input[type=range] { flex: 1; height: 8px; border-radius: 4px; accent-color: #0284c7; cursor: pointer; }
    .step-btn { width: 30px; height: 30px; border-radius: 6px; border: 1px solid #475569; background: #334155; color: #fff; font-size: 15px; font-weight: 700; cursor: pointer; display: flex; align-items: center; justify-content: center; }
    .step-btn:active { background: #0284c7; }

    /* 姿態捷徑 */
    .preset-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; margin-top: 8px; }
    .btn-preset { padding: 8px 4px; border-radius: 8px; border: 1px solid rgba(56,189,248,0.4); background: rgba(56,189,248,0.1); color: #38bdf8; font-size: 12px; font-weight: 600; cursor: pointer; }
    .btn-preset:active { background: #0284c7; color: #fff; }

    /* API 沙盒 */
    .code-box { background: #050811; padding: 8px; border-radius: 8px; font-family: monospace; font-size: 11px; color: #34d399; border: 1px solid rgba(255,255,255,0.08); white-space: pre-wrap; word-break: break-all; max-height: 90px; overflow-y: auto; user-select: text; }

    .key-tip { font-size: 11px; color: #64748b; text-align: center; margin-top: 4px; }
    .status-bar { text-align: center; font-size: 11px; color: #10b981; margin-top: 8px; font-family: monospace; }
  </style>
</head>
<body>
  <div class="container">
    <h1>🎯 戰術車臂 & API 中控系統</h1>

    <!-- 0. ToF 雷射即時測距卡片 -->
    <div class="card tof-card">
      <div class="card-title" style="color:#06b6d4;">
        <span>🎯</span> 前方 ToF 雷射測距
        <span id="tofStatusTag" style="margin-left:auto; font-size:11px; color:#94a3b8; font-family:monospace;">連線中...</span>
      </div>
      <div class="tof-display">
        <span class="tof-num" id="tofValMm">--</span>
        <span class="tof-unit">mm</span>
        <span style="color:#64748b; margin: 0 4px;">|</span>
        <span class="tof-num" id="tofValCm" style="font-size:24px; color:#38bdf8;">--</span>
        <span class="tof-unit">cm</span>
      </div>
      <div class="tof-bar-bg">
        <div class="tof-bar-fill" id="tofBar"></div>
      </div>
    </div>

    <!-- 1. 小車 4WD 控制面板 -->
    <div class="card">
      <div class="card-title">
        <span>🚗</span> 4WD 底盤移動控制
        <span id="carActionBadge" style="margin-left:auto; font-size:11px; color:#94a3b8; font-family:monospace;">停止 (STOP)</span>
      </div>

      <div style="display:flex; justify-content:space-between; font-size:12px; margin-bottom:6px;">
        <span style="color:#94a3b8;">馬達動力 PWM</span>
        <span id="txtCarSpeed" style="color:#38bdf8; font-family:monospace;">200 (78%)</span>
      </div>
      <input type="range" id="rngCarSpeed" min="80" max="255" value="200" style="width:100%; margin-bottom:12px;" oninput="onSpeedChange(this.value)">

      <div class="dpad-grid">
        <button class="d-btn spin" id="btnSpinL"
                onpointerdown="carCmd('spin_left')" onpointerup="carStop()" onpointerleave="carStop()">
          <span>↺</span><span>左旋</span>
        </button>
        <button class="d-btn" id="btnFwd"
                onpointerdown="carCmd('forward')" onpointerup="carStop()" onpointerleave="carStop()">
          <span>▲</span><span>前進 (W)</span>
        </button>
        <button class="d-btn spin" id="btnSpinR"
                onpointerdown="carCmd('spin_right')" onpointerup="carStop()" onpointerleave="carStop()">
          <span>↻</span><span>右旋</span>
        </button>

        <button class="d-btn" id="btnLeft"
                onpointerdown="carCmd('left')" onpointerup="carStop()" onpointerleave="carStop()">
          <span>◀</span><span>左轉 (A)</span>
        </button>
        <button class="d-btn stop" id="btnStop" onclick="carBrake()">
          <span>🛑</span><span>急煞 (Space)</span>
        </button>
        <button class="d-btn" id="btnRight"
                onpointerdown="carCmd('right')" onpointerup="carStop()" onpointerleave="carStop()">
          <span>▶</span><span>右轉 (D)</span>
        </button>

        <div></div>
        <button class="d-btn" id="btnBwd"
                onpointerdown="carCmd('backward')" onpointerup="carStop()" onpointerleave="carStop()">
          <span>▼</span><span>後退 (S)</span>
        </button>
        <div></div>
      </div>

      <div class="key-tip">鍵盤支援：<b>W / A / S / D</b> 移動，<b>空白鍵</b> 煞車</div>
    </div>

    <!-- 2. 機械手臂 4-DOF 控制面板 -->
    <div class="card">
      <div class="card-title">
        <span>🦾</span> 機械手臂 4-DOF 調校
      </div>

      <div class="coord-box">
        <div class="coord-item">
          <div class="coord-lbl">前方延伸 X</div>
          <div class="coord-val" id="valX">115.0 mm</div>
        </div>
        <div class="coord-item">
          <div class="coord-lbl">離地高度 Z</div>
          <div class="coord-val" id="valZ">273.0 mm</div>
        </div>
      </div>

      <!-- 舵機 1: 肩部 -->
      <div class="slider-row">
        <div class="slider-header">
          <span>大臂俯仰 Shoulder (GPIO 8)</span>
          <span class="slider-val" id="txtS">90°</span>
        </div>
        <div class="slider-ctl">
          <button class="step-btn" onclick="stepArm('s', -1)">-</button>
          <input type="range" id="rngS" min="15" max="165" value="90" oninput="onArmSlide('s', this.value)">
          <button class="step-btn" onclick="stepArm('s', 1)">+</button>
        </div>
      </div>

      <!-- 舵機 2: 肘部 -->
      <div class="slider-row">
        <div class="slider-header">
          <span>小臂俯仰 Elbow (GPIO 9)</span>
          <span class="slider-val" id="txtE">90°</span>
        </div>
        <div class="slider-ctl">
          <button class="step-btn" onclick="stepArm('e', -1)">-</button>
          <input type="range" id="rngE" min="15" max="165" value="90" oninput="onArmSlide('e', this.value)">
          <button class="step-btn" onclick="stepArm('e', 1)">+</button>
        </div>
      </div>

      <!-- 舵機 3: 手腕 -->
      <div class="slider-row">
        <div class="slider-header">
          <span>手腕滾轉 Wrist (GPIO 12)</span>
          <span class="slider-val" id="txtW">90°</span>
        </div>
        <div class="slider-ctl">
          <button class="step-btn" onclick="stepArm('w', -1)">-</button>
          <input type="range" id="rngW" min="0" max="180" value="90" oninput="onArmSlide('w', this.value)">
          <button class="step-btn" onclick="stepArm('w', 1)">+</button>
        </div>
      </div>

      <!-- 舵機 4: 夾爪 -->
      <div class="slider-row">
        <div class="slider-header">
          <span>夾爪開合 Gripper (GPIO 13)</span>
          <span class="slider-val" id="txtG">45°</span>
        </div>
        <div class="slider-ctl">
          <button class="step-btn" onclick="stepArm('g', -1)">-</button>
          <input type="range" id="rngG" min="10" max="100" value="45" oninput="onArmSlide('g', this.value)">
          <button class="step-btn" onclick="stepArm('g', 1)">+</button>
        </div>
      </div>

      <div class="preset-grid">
        <button class="btn-preset" onclick="setArmAll(90, 90, 90, 45)">🏠 待命 90°</button>
        <button class="btn-preset" onclick="setArmAll(35, 115, 90, 80)">🎯 地面抓取</button>
        <button class="btn-preset" onclick="setArmAll(110, 60, 90, 20)">📦 抬升收回</button>
        <button class="btn-preset" onclick="onArmSlide('g', 90)">👐 張開爪子</button>
        <button class="btn-preset" onclick="onArmSlide('g', 25)">✊ 閉合夾緊</button>
        <button class="btn-preset" onclick="toggleWrist()">🔄 翻轉手腕</button>
      </div>
    </div>

    <!-- 3. AI / 外部 API 即時測試沙盒 -->
    <div class="card" style="border-color:rgba(16,185,129,0.35);">
      <div class="card-title" style="color:#10b981;">
        <span>🧪</span> AI / 外部 API 指令測試沙盒
      </div>
      <div style="font-size:11px; color:#94a3b8; margin-bottom:6px;">
        點擊模板發送至 <code>POST /api/command</code>：
      </div>
      <div style="display:flex; flex-wrap:wrap; gap:4px; margin-bottom:8px;">
        <button class="btn-preset" style="font-size:11px; padding:4px 8px;" onclick="fillTpl('fwd')">🚗 前進1秒</button>
        <button class="btn-preset" style="font-size:11px; padding:4px 8px;" onclick="fillTpl('ik')">🎯 空間抓取</button>
        <button class="btn-preset" style="font-size:11px; padding:4px 8px;" onclick="fillTpl('arm')">🦾 手臂角度</button>
        <button class="btn-preset" style="font-size:11px; padding:4px 8px;" onclick="fillTpl('home')">🏠 復位待命</button>
        <button class="btn-preset" style="font-size:11px; padding:4px 8px; background:#7f1d1d; color:#fca5a5;" onclick="fillTpl('brake')">🛑 急煞</button>
      </div>
      <textarea id="jsonInput" style="width:100%; height:70px; background:#050811; color:#38bdf8; border:1px solid rgba(255,255,255,0.1); border-radius:6px; padding:6px; font-family:monospace; font-size:11px; outline:none; user-select:text;"></textarea>
      <button class="btn-preset" style="width:100%; margin-top:6px; padding:8px; background:#0284c7; color:#fff;" onclick="sendApiTest()">
        🚀 發送 POST 測試指令
      </button>
      <div style="font-size:11px; color:#94a3b8; margin:8px 0 4px 0;">最新收到的 API 指令回顯：</div>
      <div class="code-box" id="lastIncomingBox">尚未收到外部 API 指令</div>
    </div>

    <div class="status-bar" id="statusBar">系統連線正常</div>
  </div>

  <script>
    // ==================== [ToF 測距即時輪詢] ====================
    let tofPolling = false;
    async function pollTof() {
      if (tofPolling) return;
      tofPolling = true;
      try {
        const res = await fetch('/api/tof');
        if (res.ok) {
          const data = await res.json();
          const tag = document.getElementById('tofStatusTag');
          const valMm = document.getElementById('tofValMm');
          const valCm = document.getElementById('tofValCm');
          const bar = document.getElementById('tofBar');

          if (!data.online) {
            tag.textContent = '感測器未就緒';
            tag.style.color = '#f59e0b';
            valMm.textContent = '--';
            valCm.textContent = '--';
            bar.style.width = '0%';
          } else if (data.valid && data.distance_mm > 0) {
            tag.textContent = '測距中 (有效)';
            tag.style.color = '#10b981';
            valMm.textContent = data.distance_mm.toFixed(0);
            valCm.textContent = data.distance_cm.toFixed(1);

            let pct = Math.min(100, Math.max(0, (data.distance_cm / 80) * 100));
            bar.style.width = pct + '%';
            bar.style.background = data.distance_cm < 15 ? '#ef4444' : (data.distance_cm < 30 ? '#f59e0b' : '#06b6d4');
          } else {
            tag.textContent = '超出量程';
            tag.style.color = '#94a3b8';
            valMm.textContent = '>1200';
            valCm.textContent = '>120';
            bar.style.width = '100%';
            bar.style.background = '#334155';
          }
        }
      } catch (e) {
        document.getElementById('tofStatusTag').textContent = '離線';
      } finally {
        tofPolling = false;
      }
    }
    setInterval(pollTof, 150);

    // ==================== [車控前端邏輯] ====================
    let carSpeed = 200;
    let activeCarAct = 'stop';

    function onSpeedChange(val) {
      carSpeed = parseInt(val);
      const pct = Math.round((carSpeed / 255) * 100);
      document.getElementById('txtCarSpeed').textContent = `${carSpeed} (${pct}%)`;
    }

    function carCmd(act) {
      activeCarAct = act;
      document.getElementById('carActionBadge').textContent = '動作: ' + act.toUpperCase();
      highlightCarBtn(act, true);
      fetch(`/car?act=${act}&spd=${carSpeed}`).catch(e => console.error(e));
    }

    function carStop() {
      if (activeCarAct === 'stop') return;
      activeCarAct = 'stop';
      document.getElementById('carActionBadge').textContent = '停止 (STOP)';
      clearCarHighlights();
      fetch(`/car?act=stop`).catch(e => console.error(e));
    }

    function carBrake() {
      activeCarAct = 'brake';
      document.getElementById('carActionBadge').textContent = '煞車 (BRAKE)';
      highlightCarBtn('brake', true);
      setTimeout(clearCarHighlights, 400);
      fetch(`/car?act=brake`).catch(e => console.error(e));
    }

    function highlightCarBtn(act, on) {
      const map = {
        forward: 'btnFwd', backward: 'btnBwd',
        left: 'btnLeft', right: 'btnRight',
        spin_left: 'btnSpinL', spin_right: 'btnSpinR',
        brake: 'btnStop'
      };
      const id = map[act];
      if (id) {
        const el = document.getElementById(id);
        if (el) {
          if (on) el.classList.add('active');
          else el.classList.remove('active');
        }
      }
    }

    function clearCarHighlights() {
      document.querySelectorAll('.d-btn').forEach(b => b.classList.remove('active'));
    }

    const keyMap = {
      'KeyW': 'forward', 'ArrowUp': 'forward',
      'KeyS': 'backward', 'ArrowDown': 'backward',
      'KeyA': 'left', 'ArrowLeft': 'left',
      'KeyD': 'right', 'ArrowRight': 'right',
      'KeyQ': 'spin_left', 'KeyE': 'spin_right',
      'Space': 'brake'
    };
    let pressedKeys = new Set();

    window.addEventListener('keydown', (e) => {
      if (e.repeat) return;
      const act = keyMap[e.code];
      if (act) {
        e.preventDefault();
        pressedKeys.add(e.code);
        if (act === 'brake') carBrake();
        else carCmd(act);
      }
    });

    window.addEventListener('keyup', (e) => {
      const act = keyMap[e.code];
      if (act) {
        e.preventDefault();
        pressedKeys.delete(e.code);
        highlightCarBtn(act, false);
        if (pressedKeys.size === 0) {
          carStop();
        } else {
          const lastKey = Array.from(pressedKeys).pop();
          carCmd(keyMap[lastKey]);
        }
      }
    });

    // ==================== [機械手臂前端邏輯] ====================
    let armSendTimer = null;

    function onArmSlide(joint, val) {
      document.getElementById('txt' + joint.toUpperCase()).textContent = val + '°';
      document.getElementById('rng' + joint.toUpperCase()).value = val;
      queueArmSend();
    }

    function stepArm(joint, delta) {
      const rng = document.getElementById('rng' + joint.toUpperCase());
      let next = parseInt(rng.value) + delta;
      next = Math.max(parseInt(rng.min), Math.min(parseInt(rng.max), next));
      onArmSlide(joint, next);
    }

    function setArmAll(s, e, w, g) {
      document.getElementById('rngS').value = s; document.getElementById('txtS').textContent = s + '°';
      document.getElementById('rngE').value = e; document.getElementById('txtE').textContent = e + '°';
      document.getElementById('rngW').value = w; document.getElementById('txtW').textContent = w + '°';
      document.getElementById('rngG').value = g; document.getElementById('txtG').textContent = g + '°';
      queueArmSend();
    }

    function toggleWrist() {
      const cur = parseInt(document.getElementById('rngW').value);
      onArmSlide('w', cur > 45 ? 0 : 90);
    }

    function queueArmSend() {
      clearTimeout(armSendTimer);
      armSendTimer = setTimeout(sendArmToServer, 35);
    }

    function sendArmToServer() {
      const s = document.getElementById('rngS').value;
      const e = document.getElementById('rngE').value;
      const w = document.getElementById('rngW').value;
      const g = document.getElementById('rngG').value;

      fetch(`/set?s=${s}&e=${e}&w=${w}&g=${g}`)
        .then(r => r.json())
        .then(data => {
          document.getElementById('valX').textContent = data.x + ' mm';
          document.getElementById('valZ').textContent = data.z + ' mm';
          document.getElementById('statusBar').textContent = `手臂狀態: 肩${data.s}° 肘${data.e}° 腕${data.w}° 爪${data.g}°`;
        })
        .catch(err => {
          document.getElementById('statusBar').textContent = '通訊異常: ' + err;
        });
    }

    // ==================== [API 沙盒測試邏輯] ====================
    const tpls = {
      fwd: '{\n  "action": "forward",\n  "speed": 200,\n  "duration": 1000\n}',
      ik: '{\n  "x": 150.0,\n  "z": 45.0,\n  "wrist": 90,\n  "gripper": 25\n}',
      arm: '{\n  "shoulder": 85,\n  "elbow": 110,\n  "wrist": 90,\n  "gripper": 30\n}',
      home: '{\n  "preset": "home"\n}',
      brake: '{\n  "action": "brake"\n}'
    };

    function fillTpl(k) {
      document.getElementById('jsonInput').value = tpls[k] || '';
    }
    fillTpl('fwd');

    async function sendApiTest() {
      const raw = document.getElementById('jsonInput').value;
      try { JSON.parse(raw); } catch (e) { alert('JSON 語法錯誤: ' + e.message); return; }
      try {
        const res = await fetch('/api/command', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: raw
        });
        const data = await res.json();
        document.getElementById('lastIncomingBox').textContent = JSON.stringify(data, null, 2);
        // 同步滑桿
        if (data.arm) {
          document.getElementById('rngS').value = data.arm.shoulder; document.getElementById('txtS').textContent = data.arm.shoulder + '°';
          document.getElementById('rngE').value = data.arm.elbow;    document.getElementById('txtE').textContent = data.arm.elbow + '°';
          document.getElementById('rngW').value = data.arm.wrist;    document.getElementById('txtW').textContent = data.arm.wrist + '°';
          document.getElementById('rngG').value = data.arm.gripper;  document.getElementById('txtG').textContent = data.arm.gripper + '°';
          document.getElementById('valX').textContent = data.arm.x + ' mm';
          document.getElementById('valZ').textContent = data.arm.z + ' mm';
        }
      } catch (err) {
        alert('API 發送失敗: ' + err);
      }
    }

    // 輪詢最新 API 指令
    async function pollLastIncoming() {
      try {
        const res = await fetch('/api/last');
        if (res.ok) {
          const txt = await res.text();
          if (txt && txt.length > 2) {
            document.getElementById('lastIncomingBox').textContent = txt;
          }
        }
      } catch (e) {}
    }
    setInterval(pollLastIncoming, 1000);
  </script>
</body>
</html>
)rawliteral";

// ===================== [Web 路由處理] =====================
void handleRoot() {
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML, sizeof(INDEX_HTML) - 1);
}

// 獨立 ToF 雷射測距 API: GET /api/tof
void handleTofApi() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    String json = "{";
    json += "\"status\":\"ok\",";
    json += "\"online\":" + String(tofOnline ? "true" : "false") + ",";
    json += "\"valid\":" + String(tofValid ? "true" : "false") + ",";
    json += "\"distance_mm\":" + String(tofDistanceMm, 1) + ",";
    json += "\"distance_cm\":" + String(tofDistanceCm, 1);
    json += "}";
    server.send(200, "application/json", json);
}

// 最後一筆收到的 API 指令原文: GET /api/last
void handleLastApi() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", lastIncomingJson);
}

// ==============================================================================
// 核心 AI / 上位機控制接口: POST /api/command (相容 /api/json, /api/ai)
// 支援底盤動作、手臂角度、空間座標 (IK)、一鍵姿態巨集
// ==============================================================================
void handleCommandApi() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "*");
    if (server.method() == HTTP_OPTIONS) {
        server.send(204);
        return;
    }

    String body = "";
    if (server.hasArg("plain")) {
        body = server.arg("plain");
    }

    // 記錄最新指令
    if (body.length() > 0) {
        lastIncomingJson = body;
        incomingCmdCount++;
        lastIncomingTime = millis();
        Serial.printf("\n[API #%d] 收到指令: %s\n", incomingCmdCount, body.c_str());
    }

    // 輕量 JSON 解析輔助 lambda
    auto getInt = [&](const String& key, int defVal) -> int {
        if (server.hasArg(key)) return server.arg(key).toInt();
        int idx = body.indexOf("\"" + key + "\"");
        if (idx == -1) return defVal;
        int colon = body.indexOf(':', idx);
        int comma = body.indexOf(',', colon);
        int brace = body.indexOf('}', colon);
        int end = (comma != -1 && comma < brace) ? comma : brace;
        if (colon != -1 && end != -1) {
            String sub = body.substring(colon + 1, end);
            sub.trim();
            return sub.toInt();
        }
        return defVal;
    };

    auto getFloat = [&](const String& key, float defVal) -> float {
        if (server.hasArg(key)) return server.arg(key).toFloat();
        int idx = body.indexOf("\"" + key + "\"");
        if (idx == -1) return defVal;
        int colon = body.indexOf(':', idx);
        int comma = body.indexOf(',', colon);
        int brace = body.indexOf('}', colon);
        int end = (comma != -1 && comma < brace) ? comma : brace;
        if (colon != -1 && end != -1) {
            String sub = body.substring(colon + 1, end);
            sub.trim();
            return sub.toFloat();
        }
        return defVal;
    };

    auto getStr = [&](const String& key, const String& defVal) -> String {
        if (server.hasArg(key)) return server.arg(key);
        int idx = body.indexOf("\"" + key + "\"");
        if (idx == -1) return defVal;
        int colon = body.indexOf(':', idx);
        int q1 = body.indexOf('\"', colon);
        int q2 = body.indexOf('\"', q1 + 1);
        if (q1 != -1 && q2 != -1) {
            return body.substring(q1 + 1, q2);
        }
        return defVal;
    };

    bool carExecuted = false;
    bool armExecuted = false;
    bool ikSuccess = false;

    // 1. 車輛控制解析
    String act = getStr("action", "");
    if (act.length() > 0) {
        int spd = getInt("speed", carSpeed);
        int dur = getInt("duration", 0);
        executeCar(act, spd, dur);
        carExecuted = true;
    }

    // 2. 一鍵姿態解析
    String preset = getStr("preset", "");
    if (preset.length() > 0) {
        if (preset == "home") applyArmAngles(90, 90, 90, 45);
        else if (preset == "ground") applyArmAngles(35, 115, 90, 80);
        else if (preset == "lift") applyArmAngles(110, 60, 90, 20);
        else if (preset == "open") applyArmAngles(angleShoulder, angleElbow, angleWrist, 90);
        else if (preset == "close") applyArmAngles(angleShoulder, angleElbow, angleWrist, 25);
        armExecuted = true;
    }

    // 3. 空間座標 (IK) 解析
    if ((body.indexOf("\"x\"") != -1 && body.indexOf("\"z\"") != -1) || (server.hasArg("x") && server.hasArg("z"))) {
        float tx = getFloat("x", 115.0f);
        float tz = getFloat("z", 273.0f);
        if (body.indexOf("\"wrist\"") != -1 || server.hasArg("wrist")) angleWrist = getInt("wrist", angleWrist);
        if (body.indexOf("\"gripper\"") != -1 || server.hasArg("gripper")) angleGripper = getInt("gripper", angleGripper);
        ikSuccess = moveArmToCoordinate(tx, tz);
        armExecuted = true;
    }

    // 4. 關節直接角度解析
    bool hasJoint = false;
    int s = angleShoulder, e = angleElbow, w = angleWrist, g = angleGripper;
    if (body.indexOf("\"shoulder\"") != -1 || server.hasArg("shoulder")) { s = getInt("shoulder", s); hasJoint = true; }
    if (body.indexOf("\"elbow\"") != -1    || server.hasArg("elbow"))    { e = getInt("elbow", e);    hasJoint = true; }
    if (body.indexOf("\"wrist\"") != -1    || server.hasArg("wrist"))    { w = getInt("wrist", w);    hasJoint = true; }
    if (body.indexOf("\"gripper\"") != -1  || server.hasArg("gripper"))  { g = getInt("gripper", g);  hasJoint = true; }
    if (hasJoint) {
        applyArmAngles(s, e, w, g);
        armExecuted = true;
    }

    // 計算當前空間座標
    float rad1 = radians((float)angleShoulder);
    float rad2 = radians((float)(angleShoulder - angleElbow));
    float curX = ARM_L1 * cos(rad1) + ARM_L2 * cos(rad2);
    float curZ = ARM_H + ARM_L1 * sin(rad1) + ARM_L2 * sin(rad2);

    // 組織標準 JSON 回應
    String resp = "{";
    resp += "\"status\":\"success\",";
    resp += "\"code\":200,";
    resp += "\"car\":{";
    resp += "\"action\":\"" + currentCarAction + "\",";
    resp += "\"speed\":" + String(carSpeed) + ",";
    resp += "\"moving\":" + String(currentCarAction != "stop" && currentCarAction != "brake" ? "true" : "false");
    resp += "},";
    resp += "\"arm\":{";
    resp += "\"shoulder\":" + String(angleShoulder) + ",";
    resp += "\"elbow\":" + String(angleElbow) + ",";
    resp += "\"wrist\":" + String(angleWrist) + ",";
    resp += "\"gripper\":" + String(angleGripper) + ",";
    resp += "\"x\":" + String(curX, 1) + ",";
    resp += "\"z\":" + String(curZ, 1) + ",";
    resp += "\"ikSuccess\":" + String(ikSuccess ? "true" : "false");
    resp += "},";
    resp += "\"tof\":{";
    resp += "\"online\":" + String(tofOnline ? "true" : "false") + ",";
    resp += "\"valid\":" + String(tofValid ? "true" : "false") + ",";
    resp += "\"distance_mm\":" + String(tofDistanceMm, 1) + ",";
    resp += "\"distance_cm\":" + String(tofDistanceCm, 1);
    resp += "}";
    resp += "}";

    server.send(200, "application/json", resp);
}

// 車控快捷 API: GET /car?act=forward&spd=200&dur=500
void handleCarApi() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    String act = server.hasArg("act") ? server.arg("act") : "stop";
    int spd = server.hasArg("spd") ? server.arg("spd").toInt() : carSpeed;
    int dur = server.hasArg("dur") ? server.arg("dur").toInt() : 0;

    carSpeed = constrain(spd, 60, 255);
    executeCar(act, carSpeed, dur);

    String json = "{";
    json += "\"status\":\"ok\",";
    json += "\"action\":\"" + currentCarAction + "\",";
    json += "\"speed\":" + String(carSpeed);
    json += "}";
    server.send(200, "application/json", json);
}

// 臂控快捷 API: GET /set?s=90&e=90&w=90&g=45
void handleSetApi() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (server.hasArg("s")) angleShoulder = server.arg("s").toInt();
    if (server.hasArg("e")) angleElbow    = server.arg("e").toInt();
    if (server.hasArg("w")) angleWrist    = server.arg("w").toInt();
    if (server.hasArg("g")) angleGripper  = server.arg("g").toInt();

    applyArmAngles(angleShoulder, angleElbow, angleWrist, angleGripper);

    float rad1 = radians((float)angleShoulder);
    float rad2 = radians((float)(angleShoulder - angleElbow));
    float curX = ARM_L1 * cos(rad1) + ARM_L2 * cos(rad2);
    float curZ = ARM_H + ARM_L1 * sin(rad1) + ARM_L2 * sin(rad2);

    String json = "{";
    json += "\"status\":\"ok\",";
    json += "\"s\":" + String(angleShoulder) + ",";
    json += "\"e\":" + String(angleElbow) + ",";
    json += "\"w\":" + String(angleWrist) + ",";
    json += "\"g\":" + String(angleGripper) + ",";
    json += "\"x\":" + String(curX, 1) + ",";
    json += "\"z\":" + String(curZ, 1);
    json += "}";
    server.send(200, "application/json", json);
}

// 狀態查詢 API: GET /status
void handleStatusApi() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    float rad1 = radians((float)angleShoulder);
    float rad2 = radians((float)(angleShoulder - angleElbow));
    float curX = ARM_L1 * cos(rad1) + ARM_L2 * cos(rad2);
    float curZ = ARM_H + ARM_L1 * sin(rad1) + ARM_L2 * sin(rad2);

    String json = "{";
    json += "\"tof\":{";
    json += "\"online\":" + String(tofOnline ? "true" : "false") + ",";
    json += "\"valid\":" + String(tofValid ? "true" : "false") + ",";
    json += "\"distance_mm\":" + String(tofDistanceMm, 1) + ",";
    json += "\"distance_cm\":" + String(tofDistanceCm, 1);
    json += "},";
    json += "\"car\":{";
    json += "\"action\":\"" + currentCarAction + "\",";
    json += "\"speed\":" + String(carSpeed);
    json += "},";
    json += "\"arm\":{";
    json += "\"s\":" + String(angleShoulder) + ",";
    json += "\"e\":" + String(angleElbow) + ",";
    json += "\"w\":" + String(angleWrist) + ",";
    json += "\"g\":" + String(angleGripper) + ",";
    json += "\"x\":" + String(curX, 1) + ",";
    json += "\"z\":" + String(curZ, 1);
    json += "}}";
    server.send(200, "application/json", json);
}

// ===================== [序列埠指令解析] =====================
void handleSerialCommands() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    Serial.printf("\n[SERIAL CMD] 收到指令: \"%s\"\n", cmd.c_str());

    // --- ToF 查詢 ---
    if (cmd.equalsIgnoreCase("tof")) {
        Serial.printf("[ToF] 狀態:%s | 距離: %.1f mm (%.1f cm) | 有效:%s\n",
                      tofOnline ? "在線" : "離線", tofDistanceMm, tofDistanceCm, tofValid ? "YES" : "NO");
        return;
    }

    // --- 車控指令 ---
    if (cmd.equalsIgnoreCase("f") || cmd.equalsIgnoreCase("forward")) {
        executeCar("forward", carSpeed);
        return;
    }
    if (cmd.equalsIgnoreCase("b") || cmd.equalsIgnoreCase("backward")) {
        executeCar("backward", carSpeed);
        return;
    }
    if (cmd.equalsIgnoreCase("l") || cmd.equalsIgnoreCase("left")) {
        executeCar("left", carSpeed);
        return;
    }
    if (cmd.equalsIgnoreCase("r") || cmd.equalsIgnoreCase("right")) {
        executeCar("right", carSpeed);
        return;
    }
    if (cmd.equalsIgnoreCase("sl") || cmd.equalsIgnoreCase("spin_left")) {
        executeCar("spin_left", carSpeed);
        return;
    }
    if (cmd.equalsIgnoreCase("sr") || cmd.equalsIgnoreCase("spin_right")) {
        executeCar("spin_right", carSpeed);
        return;
    }
    if (cmd.equalsIgnoreCase("stop")) {
        executeCar("stop", 0);
        return;
    }
    if (cmd.equalsIgnoreCase("brake")) {
        executeCar("brake", 0);
        return;
    }
    if (cmd.startsWith("spd ") || cmd.startsWith("SPD ")) {
        carSpeed = constrain(cmd.substring(4).toInt(), 60, 255);
        Serial.printf("[CAR] 設定車速: %d\n", carSpeed);
        return;
    }

    // --- 臂控指令 ---
    if (cmd.equalsIgnoreCase("home")) {
        applyArmAngles(90, 90, 90, 45);
        return;
    }
    if (cmd.startsWith("s ") || cmd.startsWith("S ")) {
        applyArmAngles(cmd.substring(2).toInt(), angleElbow, angleWrist, angleGripper);
        return;
    }
    if (cmd.startsWith("e ") || cmd.startsWith("E ")) {
        applyArmAngles(angleShoulder, cmd.substring(2).toInt(), angleWrist, angleGripper);
        return;
    }
    if (cmd.startsWith("w ") || cmd.startsWith("W ")) {
        applyArmAngles(angleShoulder, angleElbow, cmd.substring(2).toInt(), angleGripper);
        return;
    }
    if (cmd.startsWith("g ") || cmd.startsWith("G ")) {
        applyArmAngles(angleShoulder, angleElbow, angleWrist, cmd.substring(2).toInt());
        return;
    }

    int s, e, w, g;
    if (sscanf(cmd.c_str(), "%d %d %d %d", &s, &e, &w, &g) == 4) {
        applyArmAngles(s, e, w, g);
        return;
    }

    Serial.println(F("[HELP] 支援指令:"));
    Serial.println(F("  測距: tof"));
    Serial.println(F("  車控: f, b, l, r, sl, sr, stop, brake, spd <60~255>"));
    Serial.println(F("  臂控: s <度>, e <度>, w <度>, g <度>, home, 或 <肩 肘 腕 爪>"));
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println(F("========================================================"));
    Serial.println(F("  ESP32-S3 戰術小車 + 機械手臂 + ToF 雷射 + API 中控系統"));
    Serial.println(F("========================================================"));

    // 1. 初始化 ToF 雷射測距感測器 (SCL: 10, SDA: 11)
    initTof();

    // 2. 初始化 4 顆舵機 (Ch 0~3)
    initServos();

    // 3. 初始化 4WD 底盤馬達 (Ch 4~7)
    initMotors();

    // 4. Wi-Fi 連線設定
    WiFi.persistent(false);
    WiFi.disconnect(true, true);
    delay(100);

    bool staOk = false;
    if (strlen(STA_SSID) > 0) {
        WiFi.mode(WIFI_STA);
        WiFi.enableAP(false);
        Serial.printf("[Wi-Fi] 優先連線 WiFi: %s ...\n", STA_SSID);
        WiFi.begin(STA_SSID, STA_PASS);

        int count = 0;
        while (WiFi.status() != WL_CONNECTED && count < 25) {
            delay(300);
            Serial.print(".");
            count++;
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED) {
            staOk = true;
            Serial.println(F("--------------------------------------------------------"));
            Serial.printf(" [Wi-Fi STA] ✅ 連線成功！\n");
            Serial.printf(" [控制網址 IP] 👉  http://%s\n", WiFi.localIP().toString().c_str());
            Serial.println(F("--------------------------------------------------------"));
        }
    }

    if (!staOk) {
        WiFi.disconnect(true, true);
        delay(100);
        WiFi.mode(WIFI_AP);
        WiFi.enableSTA(false);
        WiFi.softAP(AP_SSID, AP_PASS);
        Serial.println(F("--------------------------------------------------------"));
        Serial.printf(" [Wi-Fi 備援 AP] 基地台已啟動: %s (密碼: %s)\n", AP_SSID, AP_PASS);
        Serial.printf(" [控制網址 IP] 👉  http://%s\n", WiFi.softAPIP().toString().c_str());
        Serial.println(F("--------------------------------------------------------"));
    }

    MDNS.begin("esp32car");

    // 5. 註冊 Web 與 API 路由
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/command", handleCommandApi); // 標準 AI JSON 指令端點 (支援 GET, POST, OPTIONS)
    server.on("/api/json", handleCommandApi);    // 相容別名
    server.on("/api/ai", handleCommandApi);      // 相容別名
    server.on("/api/last", HTTP_GET, handleLastApi); // 最後一筆指令原文回顯
    server.on("/api/tof", HTTP_GET, handleTofApi);
    server.on("/car", handleCarApi);
    server.on("/set", handleSetApi);
    server.on("/status", handleStatusApi);

    server.enableCORS(true);
    server.begin();

    Serial.println(F("[SYSTEM] 系統初始化完成，小車、手臂、雷射測距與 API 接口已就緒！\n"));
}

void loop() {
    server.handleClient();
    updateTof(); // 高速非阻塞刷新 ToF 雷射測距
    handleSerialCommands();

    // 車輛限時自動停車檢查
    if (carAutoStopTime > 0 && millis() >= carAutoStopTime) {
        carStop();
        carAutoStopTime = 0;
        Serial.println(F("[CAR] 到達指定限時，自動停車"));
    }
}
