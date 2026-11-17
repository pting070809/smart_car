# 🚀 ESP32-S3 戰術小車 & 機械手臂 & 雷射測距 - API 使用與接入完整教學

> 本教學專為**團隊成員（AI 演算法、視覺辨識 YOLO/OpenCV、ROS、網頁前端、上位機開發者）**編寫。  
> 透過本教學，任何團隊成員都能在 **3 分鐘內** 使用 Python、JavaScript 或終端機成功控制小車、操作機械手臂，並讀取即時前方雷射測距數據。

---

## 📑 目錄
1. [快速入門 (3 分鐘上手)](#1-快速入門-3-分鐘上手)
2. [API 端點總覽清單](#2-api-端點總覽清單)
3. [如何讀取前方雷射測距 (ToF)](#3-如何讀取前方雷射測距-tof)
4. [如何控制底盤車輛移動](#4-如何控制底盤車輛移動)
5. [如何控制機械手臂](#5-如何控制機械手臂)
6. [複合指令 (邊開車邊動手臂)](#6-複合指令-邊開車邊動手臂)
7. [開箱即用 Python 完整封裝類別庫](#7-開箱即用-python-完整封裝類別庫)
8. [cURL 終端機一秒測試](#8-curl-終端機一秒測試)
9. [常見問題與排錯指南 (FAQ)](#9-常見問題與排錯指南-faq)

---

## 1. 快速入門 (3 分鐘上手)

### 步驟 1：確保在同一區域網路 (WiFi)
- **方案 A（推薦）**：將你的電腦/設備連至手機熱點 `TING`（小車開機會自動連線）。
- **方案 B**：直接用電腦 WiFi 搜尋並連上小車本身的基地台：
  - **SSID**：`ESP32-CAR-ARM`
  - **密碼**：`12345678`

### 步驟 2：獲取小車 IP
- 連上手機熱點時，小車 IP 通常為 `192.168.x.x`（開機時 Arduino 序列埠會印出，例如 `192.168.43.50`）。
- 連上小車基地台時，預設固定 IP 為 `192.168.4.1`。
- 亦可直接使用 mDNS 網址：`http://esp32car.local`。

### 步驟 3：在瀏覽器打開網頁或測試
在瀏覽器輸入 `http://<ESP32_IP>`，即可看到整合控制面板與雷射即時儀表。

---

## 2. API 端點總覽清單

基礎網址：`http://<ESP32_IP>`（例如 `http://192.168.4.1`）

| 功能 | 端點 (Endpoint) | Method | 描述 |
| :--- | :--- | :---: | :--- |
| **讀取雷射測距** | `/api/tof` | `GET` | 取得前方毫米/公分距離與有效標記 |
| **全能控制中樞** | `/api/command` | `POST` | 支援車輛、手臂角度、空間座標 IK、姿態 |
| **查詢全車狀態** | `/status` | `GET` | 查詢車輛動作、4軸角度、雷射距離 |
| **最後指令回顯** | `/api/last` | `GET` | 查看 ESP32 最後收到的原始指令 JSON |
| **快捷車控** | `/car?act=&spd=` | `GET` | 輕量 URL 參數控制小車 |
| **快捷臂控** | `/set?s=&e=&w=&g=` | `GET` | 輕量 URL 參數設定手臂 4 軸角度 |

> 💡 **CORS 跨域支援**：全端點已開啟 `Access-Control-Allow-Origin: *`，網頁前端（Vue, React, 原生 JS `fetch`）直連不會報錯！

---

## 3. 如何讀取前方雷射測距 (ToF)

ESP32-S3 搭載高精度 ToF 雷射測距模組（每秒 16 次高速採樣，有效範圍 2cm ~ 120cm+）。

### 請求方式
* **URL**: `GET http://<ESP32_IP>/api/tof`

### 回傳 JSON 範例
```json
{
  "status": "ok",
  "online": true,
  "valid": true,
  "distance_mm": 152.0,
  "distance_cm": 15.2
}
```

### 欄位解析
| 欄位 | 類型 | 說明 |
| :--- | :--- | :--- |
| `online` | bool | 感測器是否連線正常 (`true` / `false`) |
| `valid` | bool | 數值是否有效。前方 2cm~120cm 有物體為 `true`；若超出量程或空曠無物為 `false` |
| `distance_mm` | float | 距離（公釐 mm） |
| `distance_cm` | float | 距離（公分 cm） |

### 建議 AI 決策邏輯
- `distance_cm < 12.0`：進入**危險急停區**，小車應立刻停車或後退。
- `12.0 <= distance_cm <= 25.0`：進入**機械手臂最佳抓取區**，可啟動爪子進行夾取。
- `distance_cm > 25.0` 或 `valid == false`：前方開闊，可正常行駛。

---

## 4. 如何控制底盤車輛移動

* **URL**: `POST http://<ESP32_IP>/api/command`
* **Header**: `Content-Type: application/json`

### JSON 範例：前進 1 秒後自動安全停車
```json
{
  "action": "forward",
  "speed": 200,
  "duration": 1000
}
```

### 參數詳細說明
* **`action`**（必填，字串）：
  - `"forward"`：前進
  - `"backward"`：後退
  - `"left"`：左前差速微轉
  - `"right"`：右前差速微轉
  - `"spin_left"`：原地向左旋轉
  - `"spin_right"`：原地向右旋轉
  - `"stop"`：滑行停止（斷電）
  - `"brake"`：主動急煞（電機反接制動）
* **`speed`**（可選，整數）：
  - 範圍 `60 ~ 255`，預設為 `200`（動力充沛）。
* **`duration`**（可選，整數）：
  - 持續行駛的毫秒數（例如 `1000` = 1秒）。
  - **強烈推薦填寫**：ESP32-S3 會在時間到達時**自動煞停**，即使通訊中斷小車也不會失控暴衝！
  - 填 `0` 或不填代表「持續移動直到收到 stop 指令」。

---

## 5. 如何控制機械手臂

* **URL**: `POST http://<ESP32_IP>/api/command`
* **Header**: `Content-Type: application/json`

### 方式 A：4 關節直接角度控制
```json
{
  "shoulder": 85,
  "elbow": 110,
  "wrist": 90,
  "gripper": 30
}
```
* **關節限制說明**：
  - `shoulder`（大臂俯仰）：`15° ~ 165°`（90° 為直立）
  - `elbow`（小臂俯仰）：`15° ~ 165°`（90° 為水平向前）
  - `wrist`（手腕旋轉）：`0° ~ 180°`（90° 為水平夾，0° 為垂直夾）
  - `gripper`（夾爪開合）：`10°` (完全閉合夾緊) ~ `100°` (完全張開)

---

### 方式 B：空間毫米座標逆運動學 (IK) — 【視覺 AI 最愛】
當視覺（YOLO / 深度相機）計算出物體在車前的相對座標 $(X, Z)$ 時，直接傳入毫米值，ESP32 自動逆解舵機角度：
```json
{
  "x": 150.0,
  "z": 45.0,
  "wrist": 90,
  "gripper": 25
}
```
* **座標系定義**：
  - $X$：爪尖距離肩關節的前方水平距離（毫米 mm，有效範圍約 $50 \sim 250\text{ mm}$）。
  - $Z$：爪尖離地面的垂直高度（毫米 mm，肩軸心離地高 $H=113\text{ mm}$）。
  - 若座標在機械手臂工作半徑內，ESP32 會回傳 `"ikSuccess": true`。

---

### 方式 C：一鍵姿態巨集 (Presets)
快速調用常用姿態：
```json
{
  "preset": "ground"
}
```
* **可選姿態**：
  - `"home"`：回 90° 基準待命位
  - `"ground"`：地面低位抓取姿態
  - `"lift"`：抬高收回姿態
  - `"open"`：爪子完全張開
  - `"close"`：爪子閉合夾緊

---

## 6. 複合指令 (邊開車邊動手臂)

你可以在同一個 JSON 封包中**同時**下達小車與機械手臂指令：
```json
{
  "action": "forward",
  "speed": 180,
  "duration": 800,
  "shoulder": 70,
  "elbow": 105,
  "gripper": 90
}
```
ESP32 會在同一時刻驅動輪子前進並旋轉舵機，並在 800 毫秒後自動停車。

---

## 7. 開箱即用 Python 完整封裝類別庫

團隊可直接將以下程式碼存成 `car_sdk.py`，直接在演算法腳本中引用：

```python
"""
ESP32-S3 戰術車臂 Python SDK 客戶端
安裝相依套件: pip install requests
"""
import time
import requests

class ESP32CarClient:
    def __init__(self, ip: str = "192.168.4.1", timeout: float = 2.0):
        self.base_url = f"http://{ip}"
        self.timeout = timeout

    def get_distance(self):
        """
        讀取前方 ToF 雷射測距
        回傳: (distance_cm, valid)
        """
        try:
            r = requests.get(f"{self.base_url}/api/tof", timeout=self.timeout)
            if r.status_code == 200:
                data = r.json()
                return data.get("distance_cm", -1.0), data.get("valid", False)
        except Exception as e:
            print(f"[SDK] 讀取測距失敗: {e}")
        return -1.0, False

    def send_command(self, payload: dict):
        """發送 POST 指令到 /api/command"""
        try:
            r = requests.post(
                f"{self.base_url}/api/command",
                json=payload,
                headers={"Content-Type": "application/json"},
                timeout=self.timeout
            )
            if r.status_code == 200:
                return r.json()
        except Exception as e:
            print(f"[SDK] 發送指令失敗: {e}")
        return None

    def car_move(self, action: str, speed: int = 200, duration_ms: int = 0):
        """控制小車移動"""
        return self.send_command({
            "action": action,
            "speed": speed,
            "duration": duration_ms
        })

    def car_stop(self):
        """小車停止"""
        return self.send_command({"action": "stop"})

    def arm_set_angles(self, shoulder: int = None, elbow: int = None, wrist: int = None, gripper: int = None):
        """設定機械手臂關節角度"""
        payload = {}
        if shoulder is not None: payload["shoulder"] = shoulder
        if elbow is not None:    payload["elbow"] = elbow
        if wrist is not None:    payload["wrist"] = wrist
        if gripper is not None:  payload["gripper"] = gripper
        return self.send_command(payload)

    def arm_move_to_xyz(self, x: float, z: float, wrist: int = 90, gripper: int = None):
        """以毫米空間座標導引機械手臂 (IK)"""
        payload = {"x": x, "z": z, "wrist": wrist}
        if gripper is not None: payload["gripper"] = gripper
        return self.send_command(payload)

    def arm_preset(self, preset_name: str):
        """執行手臂姿態巨集 ('home', 'ground', 'lift', 'open', 'close')"""
        return self.send_command({"preset": preset_name})


# ==================== 實戰示範 ====================
if __name__ == "__main__":
    # 建立連線 (請替換為你的小車 IP)
    car = ESP32CarClient(ip="192.168.4.1")

    print("1. 讀取雷射測距...")
    dist, valid = car.get_distance()
    print(f"   當前距離: {dist} cm (狀態有效: {valid})")

    print("\n2. 手臂回待命位並張開爪子...")
    car.arm_preset("home")
    time.sleep(0.5)
    car.arm_preset("open")
    time.sleep(0.5)

    print("\n3. 若前方距離大於 20cm，則向前推進 800ms...")
    if valid and dist > 20.0:
        car.car_move("forward", speed=200, duration_ms=800)
        time.sleep(1.0)
    else:
        print("   距離過近，安全避障不前進！")

    print("\n4. 手臂切換為地面抓取姿態...")
    car.arm_preset("ground")
    time.sleep(1.0)

    print("\n5. 夾緊爪子抓取物品...")
    car.arm_preset("close")
    time.sleep(0.8)

    print("\n6. 抬高收納並倒退 500ms...")
    car.arm_preset("lift")
    car.car_move("backward", speed=180, duration_ms=500)
    print("任務完成！")
```

---

## 8. cURL 終端機一秒測試

若要在 Mac / Linux 終端機直接測試：

```bash
# 1. 取得雷射測距
curl http://192.168.4.1/api/tof

# 2. 小車前進 1 秒
curl -X POST http://192.168.4.1/api/command \
  -H "Content-Type: application/json" \
  -d '{"action":"forward","speed":200,"duration":1000}'

# 3. 空間抓取座標 (X=150mm, Z=45mm)
curl -X POST http://192.168.4.1/api/command \
  -H "Content-Type: application/json" \
  -d '{"x":150.0,"z":45.0,"wrist":90,"gripper":25}'

# 4. 手臂復位
curl -X POST http://192.168.4.1/api/command \
  -H "Content-Type: application/json" \
  -d '{"preset":"home"}'
```

---

## 9. 常見問題與排錯指南 (FAQ)

### Q1: 發送請求一直出現 `Connection refused` 或 `Timeout`？
* **檢查**：確認你的電腦是否跟小車在同一個 Wi-Fi 網段。
* **測試**：在終端機輸入 `ping 192.168.4.1`，確認網路封包能正常通暢。

### Q2: 讀取 `/api/tof` 回傳 `"valid": false` 是什麼意思？
* **原因**：ToF 雷射感測器在前方「無任何障礙物」或「障礙物超出 1.2 公尺量程」時會回傳 `false`。
* **處置**：這是正常現象，代表前方開闊無碰撞風險。

### Q3: 空間座標 IK 指令發過去，手臂沒有反應？
* **原因**：傳入的 $(X, Z)$ 超出了手臂的幾何極限（大臂 $160\text{ mm}$ + 小臂至爪尖 $115\text{ mm}$ = 最大理論半徑 $275\text{ mm}$）。
* **處置**：回傳的 JSON 裡會帶 `"ikSuccess": false`，請將 $X$ 設在 $80 \sim 220\text{ mm}$ 區間、$Z$ 設在 $20 \sim 200\text{ mm}$ 區間即可。

### Q4: 如何在網頁即時看到別人發了什麼 API 指令？
* 直接打開小車控制網頁，下方有「**AI / 外部 API 指令測試沙盒**」卡片，會即時回顯收到的最新 API 指令內容與數值。
