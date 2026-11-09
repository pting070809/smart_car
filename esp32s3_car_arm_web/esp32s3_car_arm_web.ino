#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "FourWheelDrive.h"
#include "RoboticArm.h"

/**
 * ==============================================================================
 *     ESP32-S3 4WD 戰術小車 + 4-DOF 機械手臂 - 網頁雙控制與空間座標調試系統
 * ==============================================================================
 *
 * 【ESP32-S3 硬體引腳分配總表】
 * ------------------------------------------------------------------------------
 * 【小車 4WD 馬達 (兩塊雙路 H 橋驅動板 MX1508 / L9110)】:
 *   - 左前輪 (FL): 1A -> GPIO 4,   1B -> GPIO 5
 *   - 右前輪 (FR): 2A -> GPIO 6,   2B -> GPIO 7
 *   - 左後輪 (RL): 1A -> GPIO 15,  1B -> GPIO 16
 *   - 右後輪 (RR): 2A -> GPIO 17,  2B -> GPIO 18
 *
 * 【機械手臂 4 顆舵機 (PWM 訊號線)】:
 *   - 舵機 1 (大臂 Shoulder) : GPIO 8   (大臂垂直平面仰角抬升)
 *   - 舵機 2 (小臂 Elbow)    : GPIO 9   (小臂垂直平面伸展俯仰)
 *   - 舵機 3 (手腕 Wrist Roll): GPIO 10  (爪子橫向滾轉/水平豎直切換)
 *   - 舵機 4 (夾爪 Gripper)  : GPIO 11  (爪子張開 / 夾緊閉合)
 * ------------------------------------------------------------------------------
 * 【⚠️ 重要電源與共地規範】:
 *   1. 外部 5V / 7.4V 電源的負極 (GND)，必須與 ESP32-S3 的 GND 短接共地！
 *   2. 舵機電源與馬達電源請使用外部獨立降壓或電池供電，絕對不可直接抽 ESP32 的 3.3V。
 * ==============================================================================
 */

// ===================== [硬體物件實例] =====================
FourWheelDrive car(4, 5, 6, 7, 15, 16, 17, 18);
RoboticArm arm(8, 9, 10, 11);
WebServer server(80);

// ===================== [Wi-Fi 設定] =====================
const char* AP_SSID = "ESP32-S3-CAR-ARM";
const char* AP_PASS = "12345678"; // 至少 8 碼，設為 "" 則為開放熱點

const char* STA_SSID = ""; // 可填家用或手機分享 Wi-Fi
const char* STA_PASS = "";

// ===================== [全域狀態] =====================
String carAction = "stop";
int carSpeed = 200;
bool isMoving = false;
unsigned long lastCarCmdTime = 0;
const unsigned long CAR_FAILSAFE_MS = 1500; // 超過 1.5 秒未收到車控指令自動安全停車

// 上位機 / AI 外部傳入指令追蹤監控
String lastIncomingJson = "{\"status\":\"none\"}";
uint32_t incomingCmdCount = 0;
unsigned long lastIncomingTime = 0;

// ===================== [極致科技感 Web 雙控制面板 HTML/CSS/JS] =====================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-TW">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>ESP32-S3 戰術小車與機械手臂控制中心</title>
  <style>
    :root {
      --bg: #090d16;
      --card-bg: rgba(18, 25, 41, 0.85);
      --card-border: rgba(56, 189, 248, 0.2);
      --cyan: #06b6d4;
      --sky: #38bdf8;
      --indigo: #6366f1;
      --red: #ef4444;
      --amber: #f59e0b;
      --green: #10b981;
      --purple: #a855f7;
      --text: #f8fafc;
      --muted: #94a3b8;
      --btn-bg: #1e293b;
      --btn-active: #0284c7;
    }
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      user-select: none;
      -webkit-user-select: none;
      touch-action: manipulation;
    }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
      background: radial-gradient(circle at 50% 10%, #1e1b4b 0%, #090d16 90%);
      color: var(--text);
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 14px;
      overflow-x: hidden;
    }
    header {
      text-align: center;
      margin-bottom: 12px;
      width: 100%;
      max-width: 900px;
    }
    .badge {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      background: rgba(16, 185, 129, 0.15);
      border: 1px solid rgba(16, 185, 129, 0.4);
      color: var(--green);
      padding: 4px 14px;
      border-radius: 9999px;
      font-size: 12px;
      font-weight: 700;
      margin-bottom: 6px;
    }
    .pulse-dot {
      width: 8px;
      height: 8px;
      background: var(--green);
      border-radius: 50%;
      box-shadow: 0 0 8px var(--green);
      animation: pulse 1.5s infinite;
    }
    @keyframes pulse {
      0%, 100% { opacity: 1; transform: scale(1); }
      50% { opacity: 0.3; transform: scale(0.8); }
    }
    h1 {
      font-size: 22px;
      font-weight: 800;
      letter-spacing: 1px;
      background: linear-gradient(135deg, #38bdf8 0%, #c084fc 100%);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
    }
    
    /* 空間座標大儀表卡 */
    .radar-card {
      width: 100%;
      max-width: 900px;
      background: linear-gradient(135deg, rgba(30, 41, 59, 0.7) 0%, rgba(15, 23, 42, 0.9) 100%);
      border: 1px solid rgba(168, 85, 247, 0.35);
      border-radius: 16px;
      padding: 12px 18px;
      margin-bottom: 14px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.5);
      display: flex;
      flex-wrap: wrap;
      justify-content: space-around;
      align-items: center;
      gap: 12px;
    }
    .radar-item {
      text-align: center;
      min-width: 110px;
    }
    .radar-label {
      font-size: 11px;
      font-weight: 600;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }
    .radar-val {
      font-size: 20px;
      font-weight: 800;
      font-family: monospace;
      color: #38bdf8;
      text-shadow: 0 0 10px rgba(56, 189, 248, 0.5);
      margin-top: 2px;
    }
    .radar-val.purple { color: #c084fc; text-shadow: 0 0 10px rgba(192, 132, 252, 0.5); }
    .radar-val.amber  { color: #fbbf24; text-shadow: 0 0 10px rgba(251, 191, 36, 0.5); }
    .radar-val.green  { color: #34d399; text-shadow: 0 0 10px rgba(52, 211, 153, 0.5); }

    /* 雙區域主佈局 (左右或上下響應式) */
    .dashboard-container {
      width: 100%;
      max-width: 900px;
      display: grid;
      grid-template-columns: 1fr;
      gap: 14px;
      margin-bottom: 14px;
    }
    @media(min-width: 768px) {
      .dashboard-container {
        grid-template-columns: 1fr 1.2fr;
      }
    }

    .panel {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 18px;
      padding: 16px;
      backdrop-filter: blur(12px);
      box-shadow: 0 12px 32px rgba(0,0,0,0.4);
    }
    .panel-title {
      font-size: 14px;
      font-weight: 800;
      color: var(--sky);
      margin-bottom: 12px;
      display: flex;
      align-items: center;
      gap: 6px;
      border-bottom: 1px solid rgba(255,255,255,0.08);
      padding-bottom: 8px;
    }

    /* 車控九宮格 */
    .pad-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      width: 100%;
      max-width: 280px;
      margin: 0 auto 12px auto;
    }
    .btn-ctl {
      aspect-ratio: 1;
      border-radius: 14px;
      background: var(--btn-bg);
      border: 1px solid rgba(255,255,255,0.1);
      color: var(--text);
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 3px;
      font-size: 12px;
      font-weight: 600;
      cursor: pointer;
      box-shadow: 0 4px 10px rgba(0,0,0,0.3);
      transition: all 0.1s ease;
      touch-action: none;
    }
    .btn-ctl svg {
      width: 24px;
      height: 24px;
      stroke: currentColor;
      stroke-width: 2.2;
      fill: none;
      stroke-linecap: round;
      stroke-linejoin: round;
    }
    .btn-ctl:active, .btn-ctl.active {
      background: var(--btn-active);
      box-shadow: 0 0 16px rgba(6, 182, 212, 0.6);
      transform: scale(0.94);
      border-color: #38bdf8;
    }
    .btn-stop {
      background: rgba(239, 68, 68, 0.15);
      border-color: rgba(239, 68, 68, 0.4);
      color: #f87171;
    }
    .btn-stop:active, .btn-stop.active {
      background: var(--red);
      color: #fff;
    }
    .btn-spin {
      background: rgba(245, 158, 11, 0.12);
      border-color: rgba(245, 158, 11, 0.3);
      color: #fbbf24;
    }
    .btn-spin:active, .btn-spin.active {
      background: var(--amber);
      color: #000;
    }

    /* 滑桿通用 */
    .slider-group {
      margin-bottom: 12px;
    }
    .slider-info {
      display: flex;
      justify-content: space-between;
      font-size: 12px;
      font-weight: 600;
      color: var(--muted);
      margin-bottom: 5px;
    }
    .slider-info span.v {
      color: var(--sky);
      font-family: monospace;
      font-size: 13px;
      font-weight: 700;
    }
    input[type=range] {
      width: 100%;
      height: 7px;
      border-radius: 4px;
      background: #1e293b;
      outline: none;
      -webkit-appearance: none;
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 22px;
      height: 22px;
      border-radius: 50%;
      background: #38bdf8;
      cursor: pointer;
      box-shadow: 0 0 10px #38bdf8;
      border: 2px solid #fff;
    }
    input[type=range].purple::-webkit-slider-thumb {
      background: #c084fc;
      box-shadow: 0 0 10px #c084fc;
    }
    input[type=range].amber::-webkit-slider-thumb {
      background: #fbbf24;
      box-shadow: 0 0 10px #fbbf24;
    }
    input[type=range].green::-webkit-slider-thumb {
      background: #34d399;
      box-shadow: 0 0 10px #34d399;
    }

    /* 姿態預設捷徑按鈕 */
    .preset-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(80px, 1fr));
      gap: 8px;
      margin-top: 10px;
    }
    .btn-preset {
      padding: 8px 6px;
      border-radius: 10px;
      background: #1e293b;
      border: 1px solid rgba(255,255,255,0.08);
      color: var(--text);
      font-size: 12px;
      font-weight: 700;
      cursor: pointer;
      text-align: center;
      transition: all 0.15s;
    }
    .btn-preset:active {
      background: #38bdf8;
      color: #000;
      transform: scale(0.95);
    }

    /* 底部 JSON 封包監控 */
    .json-panel {
      width: 100%;
      max-width: 900px;
      background: rgba(15, 23, 42, 0.7);
      border: 1px solid rgba(255,255,255,0.08);
      border-radius: 14px;
      padding: 12px 16px;
      font-size: 12px;
    }
    .code-box {
      background: #050811;
      padding: 10px;
      border-radius: 8px;
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      color: #38bdf8;
      border: 1px solid rgba(255,255,255,0.05);
      white-space: pre-wrap;
      word-break: break-all;
      max-height: 120px;
      overflow-y: auto;
    }
    .key-hint {
      text-align: center;
      color: var(--muted);
      font-size: 11px;
      margin-top: 8px;
    }
  </style>
</head>
<body>

  <header>
    <div class="badge">
      <span class="pulse-dot"></span>
      <span id="connStatus">ESP32-S3 連線正常</span>
    </div>
    <h1>4WD CAR & 4-DOF ROBOTIC ARM</h1>
  </header>

  <!-- 空間運動學雷達卡片 -->
  <section class="radar-card">
    <div class="radar-item">
      <div class="radar-label">前方伸展距離 X</div>
      <div class="radar-val" id="valX">150.0 mm</div>
    </div>
    <div class="radar-item">
      <div class="radar-label">垂直離地高度 Z</div>
      <div class="radar-val purple" id="valZ">85.0 mm</div>
    </div>
    <div class="radar-item">
      <div class="radar-label">爪子滾轉 (Roll)</div>
      <div class="radar-val amber" id="valRoll">90° 水平</div>
    </div>
    <div class="radar-item">
      <div class="radar-label">夾爪開合 (Claw)</div>
      <div class="radar-val green" id="valClaw">45° 半開</div>
    </div>
  </section>

  <div class="dashboard-container">

    <!-- 左半部: 車體推進控制區 -->
    <section class="panel">
      <div class="panel-title">
        <span>🚗</span> 4WD 底盤推進操控
      </div>

      <!-- 速度滑桿 -->
      <div class="slider-group">
        <div class="slider-info">
          <span>底盤動力速度 (PWM)</span>
          <span class="v" id="carSpeedText">200 (78%)</span>
        </div>
        <input type="range" min="80" max="255" value="200" oninput="onCarSpeedChange(this.value)">
      </div>

      <!-- 方向盤九宮格 (按住移動、放開停車) -->
      <div class="pad-grid">
        <button class="btn-ctl btn-spin" data-action="spin_left" id="btnSpinL" title="原地左旋轉">
          <svg viewBox="0 0 24 24"><path d="M2.5 2v6h6M2.66 15.57a10 10 0 1 0 .57-8.38L2.5 8"/></svg>
          <span>左旋轉</span>
        </button>
        <button class="btn-ctl" data-action="forward" id="btnFwd" title="前進 (W / ↑)">
          <svg viewBox="0 0 24 24"><path d="M12 19V5M5 12l7-7 7 7"/></svg>
          <span>前進</span>
        </button>
        <button class="btn-ctl btn-spin" data-action="spin_right" id="btnSpinR" title="原地右旋轉">
          <svg viewBox="0 0 24 24"><path d="M21.5 2v6h-6M21.34 15.57a10 10 0 1 1-.57-8.38L21.5 8"/></svg>
          <span>右旋轉</span>
        </button>

        <button class="btn-ctl" data-action="left" id="btnLeft" title="左前轉 (A / ←)">
          <svg viewBox="0 0 24 24"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
          <span>左轉</span>
        </button>
        <button class="btn-ctl btn-stop" data-action="stop" id="btnStop" title="停車 (Space)">
          <svg viewBox="0 0 24 24"><rect x="4" y="4" width="16" height="16" rx="3"/></svg>
          <span>停車</span>
        </button>
        <button class="btn-ctl" data-action="right" id="btnRight" title="右前轉 (D / →)">
          <svg viewBox="0 0 24 24"><path d="M5 12h14M12 5l7 7-7 7"/></svg>
          <span>右轉</span>
        </button>

        <div></div>
        <button class="btn-ctl" data-action="backward" id="btnBwd" title="倒退 (S / ↓)">
          <svg viewBox="0 0 24 24"><path d="M12 5v14M19 12l-7 7-7-7"/></svg>
          <span>後退</span>
        </button>
        <div></div>
      </div>

      <div style="display:flex; gap:8px;">
        <button class="btn-preset" style="flex:1; background:#7f1d1d; color:#fca5a5;" onclick="sendDirectCar('brake')">🛑 主動急煞</button>
        <button class="btn-preset" style="flex:1;" onclick="sendDirectCar('stop')">⚪ 斷電滑行</button>
      </div>

      <div class="key-hint">
        支援鍵盤 <b>W A S D / 方向鍵</b> 移動，<b>空白鍵</b> 急煞
      </div>
    </section>

    <!-- 右半部: 機械手臂 4 軸即時調校與空間座標區 -->
    <section class="panel">
      <div class="panel-title">
        <span>🦾</span> 機械手臂 4-DOF 調校與空間運動學
      </div>

      <!-- 舵機 1: 大臂 Shoulder -->
      <div class="slider-group">
        <div class="slider-info">
          <span>大臂仰角 Shoulder (GPIO 8)</span>
          <span class="v" id="txtShoulder">90°</span>
        </div>
        <input type="range" id="rngShoulder" min="15" max="165" value="90" oninput="onArmSliderChange()">
      </div>

      <!-- 舵機 2: 小臂 Elbow -->
      <div class="slider-group">
        <div class="slider-info">
          <span>小臂俯仰 Elbow (GPIO 9)</span>
          <span class="v" id="txtElbow">90°</span>
        </div>
        <input type="range" class="purple" id="rngElbow" min="15" max="165" value="90" oninput="onArmSliderChange()">
      </div>

      <!-- 舵機 3: 手腕滾轉 Wrist Roll -->
      <div class="slider-group">
        <div class="slider-info">
          <span>手腕滾轉 Wrist Roll (GPIO 10)</span>
          <span class="v" id="txtWrist">90° (水平)</span>
        </div>
        <input type="range" class="amber" id="rngWrist" min="0" max="180" value="90" oninput="onArmSliderChange()">
      </div>

      <!-- 舵機 4: 夾爪開合 Gripper -->
      <div class="slider-group">
        <div class="slider-info">
          <span>夾爪開合 Gripper (GPIO 11)</span>
          <span class="v" id="txtGripper">45°</span>
        </div>
        <input type="range" class="green" id="rngGripper" min="10" max="95" value="45" oninput="onArmSliderChange()">
      </div>

      <!-- 一鍵姿態巨集 -->
      <div style="font-size:12px; font-weight:700; color:var(--muted); margin: 10px 0 6px 0;">一鍵姿態設定:</div>
      <div class="preset-grid">
        <button class="btn-preset" onclick="applyPreset('home')">🏠 待命復位</button>
        <button class="btn-preset" onclick="applyPreset('ground')">🎯 地面抓取</button>
        <button class="btn-preset" onclick="applyPreset('lift')">📦 抬升收回</button>
        <button class="btn-preset" onclick="applyPreset('open')">👐 張開爪子</button>
        <button class="btn-preset" onclick="applyPreset('close')">✊ 閉合夾緊</button>
        <button class="btn-preset" onclick="toggleWristRoll()">🔄 橫/直翻轉</button>
      </div>
    </section>

  </div>

  <!-- 團隊網站 / AI 外部 API 即時監控與手動測試區 -->
  <div class="dashboard-container" style="margin-top: 10px;">

    <!-- 1. 外部 API 即時接收監控 -->
    <section class="panel" style="border-color: rgba(16, 185, 129, 0.35);">
      <div class="panel-title" style="color: var(--green);">
        <span>📥</span> 外部 API 即時接收監控 (AI 指令即時回顯)
        <span id="incomingBadge" style="margin-left:auto; font-size:11px; background:rgba(16,185,129,0.2); border:1px solid var(--green); color:var(--green); padding:2px 8px; border-radius:10px;">待命中 #0</span>
      </div>
      <div style="font-size:12px; color:var(--muted); margin-bottom:6px;">
        當 AI 或外部腳本 POST 指令到 <code>/api/command</code> 時，此處會自動即時刷新：
      </div>
      <div class="code-box" id="incomingDisplay" style="color:#34d399; min-height:80px;">尚未收到外部 API 指令</div>
    </section>

    <!-- 2. 手動 API 測試沙盒 -->
    <section class="panel" style="border-color: rgba(56, 189, 248, 0.35);">
      <div class="panel-title" style="color: var(--sky);">
        <span>🧪</span> 手動 JSON API 測試沙盒 (不用 AI 也能自己測)
      </div>
      <div style="font-size:12px; color:var(--muted); margin-bottom:6px;">
        點擊快捷模板或直接編輯 JSON，發送給 ESP32-S3 接收端口：
      </div>

      <div style="display:flex; flex-wrap:wrap; gap:6px; margin-bottom:8px;">
        <button class="btn-preset" style="font-size:11px; padding:4px 8px;" onclick="fillTemplate('fwd')">🚗 前進1秒</button>
        <button class="btn-preset" style="font-size:11px; padding:4px 8px;" onclick="fillTemplate('ik')">🎯 空間抓取</button>
        <button class="btn-preset" style="font-size:11px; padding:4px 8px;" onclick="fillTemplate('arm')">🦾 手臂角度</button>
        <button class="btn-preset" style="font-size:11px; padding:4px 8px;" onclick="fillTemplate('home')">🏠 復位待命</button>
        <button class="btn-preset" style="font-size:11px; padding:4px 8px; background:#7f1d1d; color:#fca5a5;" onclick="fillTemplate('brake')">🛑 急煞</button>
      </div>

      <textarea id="manualJsonInput" style="width:100%; height:85px; background:#050811; color:#38bdf8; border:1px solid rgba(255,255,255,0.1); border-radius:8px; padding:8px; font-family:monospace; font-size:12px; outline:none; resize:vertical;">{
  "action": "forward",
  "speed": 200,
  "duration": 1000
}</textarea>

      <button class="btn-preset" style="width:100%; margin-top:8px; padding:10px; background:#0284c7; color:#fff; font-weight:800;" onclick="sendManualApiTest()">
        🚀 發送 POST 測試指令到 /api/command
      </button>
    </section>

  </div>

  <script>
    let carSpeed = 200;
    let activeCarAction = null;
    let armDebounceTimer = null;
    let lastKnownIncomingCount = 0;

    // ==================== [車控邏輯] ====================
    function onCarSpeedChange(v) {
      carSpeed = parseInt(v);
      const pct = Math.round((carSpeed / 255) * 100);
      document.getElementById('carSpeedText').textContent = `${carSpeed} (${pct}%)`;
      if (activeCarAction && activeCarAction !== 'stop') {
        sendCar(activeCarAction);
      }
    }

    async function sendCar(action) {
      activeCarAction = action;
      try {
        await fetch(`/api/car?action=${action}&speed=${carSpeed}`);
      } catch (e) {
        console.error('車控通訊失敗:', e);
      }
    }

    function sendDirectCar(action) {
      sendCar(action);
      if (action === 'stop' || action === 'brake') {
        activeCarAction = null;
        document.querySelectorAll('.btn-ctl').forEach(b => b.classList.remove('active'));
      }
    }

    // 虛擬九宮格按鈕按住/放開事件
    const carBtns = document.querySelectorAll('.btn-ctl[data-action]');
    carBtns.forEach(btn => {
      const act = btn.getAttribute('data-action');
      if (act === 'stop') {
        btn.addEventListener('click', () => sendDirectCar('stop'));
        return;
      }

      const start = (e) => {
        e.preventDefault();
        btn.classList.add('active');
        sendCar(act);
      };

      const end = (e) => {
        e.preventDefault();
        btn.classList.remove('active');
        if (activeCarAction === act) sendDirectCar('stop');
      };

      btn.addEventListener('touchstart', start, { passive: false });
      btn.addEventListener('touchend', end, { passive: false });
      btn.addEventListener('touchcancel', end, { passive: false });
      btn.addEventListener('mousedown', start);
      btn.addEventListener('mouseup', end);
      btn.addEventListener('mouseleave', end);
    });

    // 鍵盤監聽 WASD
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
        sendCar(act);
        highlightKey(act, true);
      }
    });

    window.addEventListener('keyup', (e) => {
      const act = keyMap[e.code];
      if (act) {
        e.preventDefault();
        pressedKeys.delete(e.code);
        highlightKey(act, false);
        if (pressedKeys.size === 0) {
          sendDirectCar('stop');
        } else {
          const lastKey = Array.from(pressedKeys).pop();
          sendCar(keyMap[lastKey]);
        }
      }
    });

    function highlightKey(action, on) {
      const b = document.querySelector(`.btn-ctl[data-action="${action}"]`);
      if (b) {
        if (on) b.classList.add('active');
        else b.classList.remove('active');
      }
    }

    // ==================== [機械手臂邏輯] ====================
    function onArmSliderChange() {
      const s = parseInt(document.getElementById('rngShoulder').value);
      const e = parseInt(document.getElementById('rngElbow').value);
      const w = parseInt(document.getElementById('rngWrist').value);
      const g = parseInt(document.getElementById('rngGripper').value);

      updateArmUI(s, e, w, g);

      clearTimeout(armDebounceTimer);
      armDebounceTimer = setTimeout(() => {
        sendArmAngles(s, e, w, g);
      }, 50);
    }

    function updateArmUI(s, e, w, g) {
      document.getElementById('txtShoulder').textContent = s + '°';
      document.getElementById('txtElbow').textContent = e + '°';
      document.getElementById('txtWrist').textContent = w + '° ' + (w >= 75 && w <= 105 ? '(水平)' : '(傾斜/垂直)');
      document.getElementById('txtGripper').textContent = g + '° ' + (g <= 25 ? '(閉合)' : (g >= 80 ? '(全開)' : ''));

      document.getElementById('valRoll').textContent = w + '°';
      document.getElementById('valClaw').textContent = g + '°';

      const rad1 = s * Math.PI / 180;
      const rad2 = (s + e - 90) * Math.PI / 180;
      const x = 105.0 * Math.cos(rad1) + 120.0 * Math.cos(rad2);
      const z = 65.0 + 105.0 * Math.sin(rad1) + 120.0 * Math.sin(rad2);

      document.getElementById('valX').textContent = x.toFixed(1) + ' mm';
      document.getElementById('valZ').textContent = z.toFixed(1) + ' mm';
    }

    async function sendArmAngles(s, e, w, g) {
      try {
        await fetch(`/api/arm?shoulder=${s}&elbow=${e}&wrist=${w}&gripper=${g}`);
      } catch (err) {
        console.error('手臂通訊失敗:', err);
      }
    }

    function applyPreset(name) {
      if (name === 'home')   setArmControls(90, 90, 90, 45);
      if (name === 'ground') setArmControls(45, 120, 90, 80);
      if (name === 'lift')   setArmControls(110, 60, 90, 20);
      if (name === 'open')  { document.getElementById('rngGripper').value = 90; onArmSliderChange(); }
      if (name === 'close') { document.getElementById('rngGripper').value = 20; onArmSliderChange(); }
    }

    let isWristH = true;
    function toggleWristRoll() {
      isWristH = !isWristH;
      document.getElementById('rngWrist').value = isWristH ? 90 : 0;
      onArmSliderChange();
    }

    function setArmControls(s, e, w, g) {
      document.getElementById('rngShoulder').value = s;
      document.getElementById('rngElbow').value = e;
      document.getElementById('rngWrist').value = w;
      document.getElementById('rngGripper').value = g;
      onArmSliderChange();
    }

    // ==================== [手動 API 沙盒測試器] ====================
    const templates = {
      fwd: '{\n  "action": "forward",\n  "speed": 220,\n  "duration": 1000\n}',
      ik: '{\n  "x": 145.0,\n  "z": 45.0,\n  "wrist": 90,\n  "gripper": 25\n}',
      arm: '{\n  "shoulder": 80,\n  "elbow": 110,\n  "wrist": 90,\n  "gripper": 35\n}',
      home: '{\n  "preset": "home"\n}',
      brake: '{\n  "action": "brake"\n}'
    };

    function fillTemplate(key) {
      document.getElementById('manualJsonInput').value = templates[key] || '';
    }

    async function sendManualApiTest() {
      const raw = document.getElementById('manualJsonInput').value;
      try {
        JSON.parse(raw); // 驗證 JSON 語法
      } catch (e) {
        alert('JSON 格式有誤，請檢查引號與逗號！\n錯誤: ' + e.message);
        return;
      }

      try {
        const res = await fetch('/api/command', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: raw
        });
        const data = await res.json();
        console.log('手動 API 測試回應:', data);
        pollStatus(); // 立即輪詢一次更新畫面
      } catch (err) {
        alert('發送失敗: ' + err);
      }
    }

    // ==================== [外部 API 接收即時同步輪詢] ====================
    async function pollStatus() {
      try {
        const res = await fetch('/api/status');
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const data = await res.json();

        document.getElementById('connStatus').textContent = 'ESP32-S3 連線正常';
        document.getElementById('connStatus').parentElement.style.borderColor = 'rgba(16, 185, 129, 0.4)';

        // 同步手臂滑桿 (若有新狀態)
        if (data.arm) {
          document.getElementById('rngShoulder').value = data.arm.shoulder;
          document.getElementById('rngElbow').value = data.arm.elbow;
          document.getElementById('rngWrist').value = data.arm.wrist;
          document.getElementById('rngGripper').value = data.arm.gripper;
          updateArmUI(data.arm.shoulder, data.arm.elbow, data.arm.wrist, data.arm.gripper);
        }

        // 若有新的外部 API 指令，改用 /api/last 單獨取出乾淨的 JSON (避免雙引號轉義問題)
        if (data.incoming && data.incoming.count !== lastKnownIncomingCount) {
          lastKnownIncomingCount = data.incoming.count;

          const badge = document.getElementById('incomingBadge');
          badge.textContent = `⚡ 收到指令 #${data.incoming.count}`;
          badge.style.background = 'rgba(6, 182, 212, 0.3)';
          badge.style.borderColor = '#06b6d4';
          badge.style.color = '#06b6d4';

          // 獨立 fetch /api/last → ESP32 直接回傳原始 JSON，100% 不會有轉義問題
          try {
            const lastRes = await fetch('/api/last');
            if (lastRes.ok) {
              const lastText = await lastRes.text();
              // 嘗試美化顯示
              try {
                const parsed = JSON.parse(lastText);
                document.getElementById('incomingDisplay').textContent = JSON.stringify(parsed, null, 2);
              } catch (e) {
                document.getElementById('incomingDisplay').textContent = lastText;
              }
            }
          } catch (e2) {
            document.getElementById('incomingDisplay').textContent = '(取得指令內容失敗)';
          }
        }
      } catch (e) {
        document.getElementById('connStatus').textContent = '連線中斷 / 離線';
        document.getElementById('connStatus').parentElement.style.borderColor = 'rgba(239, 68, 68, 0.6)';
      }
    }

    // 每 800ms 輪詢一次
    setInterval(pollStatus, 800);

    window.onload = () => {
      onArmSliderChange();
      pollStatus();
    };
  </script>
</body>
</html>
)rawliteral";

// ===================== [車體動作核心執行] =====================
void executeCarCommand(const String& action, int speed) {
    speed = constrain(speed, 0, 255);
    carAction = action;
    carSpeed = speed;
    lastCarCmdTime = millis();

    if (action == "forward") {
        car.moveForward(speed);
        isMoving = true;
    } else if (action == "backward") {
        car.moveBackward(speed);
        isMoving = true;
    } else if (action == "left") {
        car.setSpeeds(speed / 3, speed, speed / 3, speed);
        isMoving = true;
    } else if (action == "right") {
        car.setSpeeds(speed, speed / 3, speed, speed / 3);
        isMoving = true;
    } else if (action == "spin_left") {
        car.spinLeft(speed);
        isMoving = true;
    } else if (action == "spin_right") {
        car.spinRight(speed);
        isMoving = true;
    } else if (action == "brake") {
        car.brake();
        isMoving = false;
    } else { // "stop"
        car.stop();
        isMoving = false;
    }

    Serial.printf("[CAR] 動作: %-10s | 速度: %3d | 行駛中: %s\n",
                  carAction.c_str(), carSpeed, isMoving ? "YES" : "NO");
}

// ===================== [Web 路由處理] =====================

void handleRoot() {
    server.send(200, "text/html", INDEX_HTML);
}

// 車體 API: GET /api/car?action=forward&speed=200
void handleCarApi() {
    String action = server.hasArg("action") ? server.arg("action") : "stop";
    int speed = server.hasArg("speed") ? server.arg("speed").toInt() : carSpeed;
    if (speed <= 0) speed = carSpeed;

    executeCarCommand(action, speed);

    String json = "{\"status\":\"ok\",\"action\":\"" + carAction + "\",\"speed\":" + String(carSpeed) + "}";
    server.send(200, "application/json", json);
}

// 機械手臂 API: GET /api/arm?shoulder=90&elbow=90&wrist=90&gripper=45
void handleArmApi() {
    if (server.hasArg("shoulder")) arm.setShoulder(server.arg("shoulder").toInt());
    if (server.hasArg("elbow"))    arm.setElbow(server.arg("elbow").toInt());
    if (server.hasArg("wrist"))    arm.setWrist(server.arg("wrist").toInt());
    if (server.hasArg("gripper"))  arm.setGripper(server.arg("gripper").toInt());

    Serial.printf("[ARM] 角度更新 -> S:%d° E:%d° W:%d° G:%d° | 空間位置 -> X:%.1fmm Z:%.1fmm\n",
                  arm.getShoulder(), arm.getElbow(), arm.getWrist(), arm.getGripper(),
                  arm.getX(), arm.getZ());

    String json = "{";
    json += "\"status\":\"ok\",";
    json += "\"shoulder\":" + String(arm.getShoulder()) + ",";
    json += "\"elbow\":" + String(arm.getElbow()) + ",";
    json += "\"wrist\":" + String(arm.getWrist()) + ",";
    json += "\"gripper\":" + String(arm.getGripper()) + ",";
    json += "\"x\":" + String(arm.getX(), 1) + ",";
    json += "\"z\":" + String(arm.getZ(), 1);
    json += "}";
    server.send(200, "application/json", json);
}

// 逆向運動學 API: GET /api/ik?x=150&z=80
void handleIkApi() {
    if (!server.hasArg("x") || !server.hasArg("z")) {
        server.send(400, "application/json", "{\"status\":\"error\",\"msg\":\"缺少 x 或 z\"}");
        return;
    }
    float tx = server.arg("x").toFloat();
    float tz = server.arg("z").toFloat();

    bool reachable = arm.moveToCoordinate(tx, tz);
    String json = "{";
    json += "\"status\":\"" + String(reachable ? "ok" : "unreachable") + "\",";
    json += "\"x\":" + String(arm.getX(), 1) + ",";
    json += "\"z\":" + String(arm.getZ(), 1) + ",";
    json += "\"shoulder\":" + String(arm.getShoulder()) + ",";
    json += "\"elbow\":" + String(arm.getElbow());
    json += "}";
    server.send(200, "application/json", json);
}

// 全系統狀態 API: GET /api/status
// 注意: 不在這裡嵌入 lastIncomingJson 字串 (避免 JSON 格式破損)
// 若要讀取最後一筆 AI 指令，請改用 GET /api/last
void handleStatusApi() {
    String json = "{";
    json += "\"status\":\"online\",";
    json += "\"car\":{";
    json += "\"action\":\"" + carAction + "\",";
    json += "\"speed\":" + String(carSpeed) + ",";
    json += "\"moving\":" + String(isMoving ? "true" : "false");
    json += "},";
    json += "\"arm\":{";
    json += "\"shoulder\":" + String(arm.getShoulder()) + ",";
    json += "\"elbow\":" + String(arm.getElbow()) + ",";
    json += "\"wrist\":" + String(arm.getWrist()) + ",";
    json += "\"gripper\":" + String(arm.getGripper()) + ",";
    json += "\"x\":" + String(arm.getX(), 1) + ",";
    json += "\"z\":" + String(arm.getZ(), 1);
    json += "},";
    // 只回傳計數器與時間戳，不嵌入 raw 字串 (避免雙引號轉義破壞 JSON)
    json += "\"incoming\":{";
    json += "\"count\":" + String(incomingCmdCount) + ",";
    json += "\"time\":" + String(lastIncomingTime);
    json += "}";
    json += "}";
    server.send(200, "application/json", json);
}

// 最後一筆外部 API 指令原文 (直接回傳 raw JSON，不再包一層) GET /api/last
// 瀏覽器收到的就是乾淨的 JSON，不會有轉義問題
void handleLastIncoming() {
    if (incomingCmdCount == 0) {
        server.send(200, "application/json", "{\"status\":\"none\",\"message\":\"尚未收到任何外部指令\"}");
        return;
    }
    // 直接把原始 JSON 字串送回去，完全不做任何轉義
    server.send(200, "application/json", lastIncomingJson);
}

// ==============================================================================
// 供上位機 / AI 智能大腦傳遞 JSON 指令的接收 API 端點
// 支援路由: POST /api/command  以及  POST /api/json  以及  POST /api/ai
// ==============================================================================
void handleJsonApi() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"status\":\"error\",\"code\":400,\"message\":\"缺少 JSON Request Body\"}");
        return;
    }

    String body = server.arg("plain");
    lastIncomingJson = body;
    incomingCmdCount++;
    lastIncomingTime = millis();

    Serial.println("\n------------------------------------------------");
    Serial.printf("[AI API #%d] 收到上位機/AI 傳入指令 JSON:\n", incomingCmdCount);
    Serial.println(body);
    Serial.println("------------------------------------------------");

    auto extractInt = [&](const String& key, int defaultVal) -> int {
        int idx = body.indexOf("\"" + key + "\"");
        if (idx == -1) return defaultVal;
        int colon = body.indexOf(':', idx);
        int comma = body.indexOf(',', colon);
        int brace = body.indexOf('}', colon);
        int end = (comma != -1 && comma < brace) ? comma : brace;
        if (colon != -1 && end != -1) {
            String sub = body.substring(colon + 1, end);
            sub.trim();
            return sub.toInt();
        }
        return defaultVal;
    };

    auto extractFloat = [&](const String& key, float defaultVal) -> float {
        int idx = body.indexOf("\"" + key + "\"");
        if (idx == -1) return defaultVal;
        int colon = body.indexOf(':', idx);
        int comma = body.indexOf(',', colon);
        int brace = body.indexOf('}', colon);
        int end = (comma != -1 && comma < brace) ? comma : brace;
        if (colon != -1 && end != -1) {
            String sub = body.substring(colon + 1, end);
            sub.trim();
            return sub.toFloat();
        }
        return defaultVal;
    };

    auto extractStr = [&](const String& key, const String& defaultVal) -> String {
        int idx = body.indexOf("\"" + key + "\"");
        if (idx == -1) return defaultVal;
        int colon = body.indexOf(':', idx);
        int q1 = body.indexOf('\"', colon);
        int q2 = body.indexOf('\"', q1 + 1);
        if (q1 != -1 && q2 != -1) {
            return body.substring(q1 + 1, q2);
        }
        return defaultVal;
    };

    bool carExecuted = false;
    bool armExecuted = false;
    bool ikExecuted = false;

    // 1. 車輛動作解析 (支援 {"action":"forward", "speed":200} 或嵌套 {"car":{"action":"forward"}} )
    String action = "";
    if (body.indexOf("\"action\"") != -1) {
        action = extractStr("action", "");
    }
    int speed = extractInt("speed", carSpeed);
    if (speed <= 0) speed = carSpeed;

    if (action.length() > 0 && action != "ik" && action != "arm") {
        executeCarCommand(action, speed);
        carExecuted = true;

        // 若指定了行駛時間 duration (毫秒)，例如 {"action":"forward", "duration":800}
        int duration = extractInt("duration", 0);
        if (duration > 0 && action != "stop" && action != "brake") {
            delay(duration);
            executeCarCommand("stop", speed);
        }
    }

    // 2. 機械手臂空間目標座標解析 (IK 逆運動學，例如 {"x":150.0, "z":80.0} 或 {"type":"ik", "x":150, "z":80})
    if (body.indexOf("\"x\"") != -1 && body.indexOf("\"z\"") != -1) {
        float tx = extractFloat("x", arm.getX());
        float tz = extractFloat("z", arm.getZ());
        ikExecuted = arm.moveToCoordinate(tx, tz);
        armExecuted = true;
    }

    // 3. 機械手臂各軸角度直接控制 (例如 {"shoulder":90, "elbow":90, "wrist":90, "gripper":40})
    if (body.indexOf("\"shoulder\"") != -1 || body.indexOf("\"elbow\"") != -1 ||
        body.indexOf("\"wrist\"") != -1    || body.indexOf("\"gripper\"") != -1) {
        if (body.indexOf("\"shoulder\"") != -1) arm.setShoulder(extractInt("shoulder", arm.getShoulder()));
        if (body.indexOf("\"elbow\"") != -1)    arm.setElbow(extractInt("elbow", arm.getElbow()));
        if (body.indexOf("\"wrist\"") != -1)    arm.setWrist(extractInt("wrist", arm.getWrist()));
        if (body.indexOf("\"gripper\"") != -1)  arm.setGripper(extractInt("gripper", arm.getGripper()));
        armExecuted = true;
    }

    // 4. 機械手臂預設姿態 (例如 {"preset":"home"} 或 {"preset":"ground"} 或 {"preset":"open"})
    String preset = extractStr("preset", "");
    if (preset.length() > 0) {
        if (preset == "home")        arm.home();
        else if (preset == "ground") arm.groundPick();
        else if (preset == "lift")   arm.liftUp();
        else if (preset == "open")   arm.openGripper();
        else if (preset == "close")  arm.closeGripper();
        armExecuted = true;
    }

    // 構造回傳給 AI 的詳細執行結果 JSON
    String resp = "{";
    resp += "\"status\":\"success\",";
    resp += "\"code\":200,";
    resp += "\"car\":{";
    resp += "\"action\":\"" + carAction + "\",";
    resp += "\"speed\":" + String(carSpeed) + ",";
    resp += "\"moving\":" + String(isMoving ? "true" : "false");
    resp += "},";
    resp += "\"arm\":{";
    resp += "\"shoulder\":" + String(arm.getShoulder()) + ",";
    resp += "\"elbow\":" + String(arm.getElbow()) + ",";
    resp += "\"wrist\":" + String(arm.getWrist()) + ",";
    resp += "\"gripper\":" + String(arm.getGripper()) + ",";
    resp += "\"x\":" + String(arm.getX(), 1) + ",";
    resp += "\"z\":" + String(arm.getZ(), 1) + ",";
    resp += "\"ikSuccess\":" + String(ikExecuted ? "true" : "false");
    resp += "},";
    resp += "\"timestamp\":" + String(millis());
    resp += "}";

    server.send(200, "application/json", resp);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n========================================================"));
    Serial.println(F(" ESP32-S3 4WD 戰術小車 + 4-DOF 機械手臂系統啟動中"));
    Serial.println(F("========================================================"));

    // 1. 初始化底盤馬達與機械手臂
    car.begin();
    car.stop();

    arm.begin();
    Serial.printf("[ARM] 機械手臂初始化就緒 (預設尺寸 L1:%.0fmm, L2:%.0fmm, H:%.0fmm)\n",
                  arm.getL1(), arm.getL2(), arm.getH());

    // 2. 啟動 Wi-Fi AP 模式
    WiFi.mode(WIFI_AP_STA);
    bool apOk = (strlen(AP_PASS) > 0) ? WiFi.softAP(AP_SSID, AP_PASS) : WiFi.softAP(AP_SSID);
    if (apOk) {
        Serial.print(F("[Wi-Fi AP] 基地台已就緒！熱點名稱: "));
        Serial.println(AP_SSID);
        Serial.print(F("[Wi-Fi AP] 控制網址: http://"));
        Serial.println(WiFi.softAPIP());
    }

    // 3. 連接現有外部 Wi-Fi STA (若有填寫)
    if (strlen(STA_SSID) > 0) {
        WiFi.begin(STA_SSID, STA_PASS);
        unsigned long t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 5000) {
            delay(300);
            Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.print(F("\n[Wi-Fi STA] 外部連線成功！IP: http://"));
            Serial.println(WiFi.localIP());
        }
    }

    // 4. mDNS 啟用
    if (MDNS.begin("esp32car")) {
        Serial.println(F("[mDNS] 網域就緒: http://esp32car.local"));
    }

    // 5. 註冊 Web 路由
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/car", HTTP_GET, handleCarApi);
    server.on("/api/arm", HTTP_GET, handleArmApi);
    server.on("/api/ik", HTTP_GET, handleIkApi);
    server.on("/api/command", HTTP_POST, handleJsonApi); // 推薦: AI 上位機標準 POST 端點
    server.on("/api/json", HTTP_POST, handleJsonApi);    // 相容別名
    server.on("/api/ai", HTTP_POST, handleJsonApi);      // 相容別名
    server.on("/api/status", HTTP_GET, handleStatusApi); // 全系統狀態 (含計數器)
    server.on("/api/last", HTTP_GET, handleLastIncoming); // 最後一筆收到的外部指令原文 (乾淨 JSON)

    server.enableCORS(true);
    server.begin();
    Serial.println(F("[HTTP] Web 伺服器啟動成功！請開啟瀏覽器開始操控！\n"));
}

void loop() {
    server.handleClient();

    // 車輛安全看門狗：若處於前進/轉彎狀態且逾時未收到新指令，安全停車
    if (isMoving && (millis() - lastCarCmdTime > CAR_FAILSAFE_MS)) {
        Serial.println(F("[FAIL-SAFE] 超時未收到車控指令，主動安全停車！"));
        car.stop();
        isMoving = false;
        carAction = "stop";
    }
}
