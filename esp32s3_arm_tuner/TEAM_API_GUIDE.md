# 🚗 ESP32-S3 戰術小車 & 機械手臂 & 雷射測距 - 團隊 API 對接手冊

本手冊供 **AI 決策、視覺辨識 (Python / YOLO / ROS / OpenCV)、Web 前端或上位機團隊成員** 快速接入使用。

---

## 📌 1. 連線基礎與前置準備

1. **同網域連線**：
   - 確保你的電腦 / 樹莓派 / 上位機 與 ESP32-S3 連在同一個 Wi-Fi（例如熱點 `TING`，或直接連小車 AP `ESP32-CAR-ARM`）。
2. **取得小車 IP**：
   - 預設 IP 通常為 `192.168.x.x`（若連小車 AP 則為 `192.168.4.1`）。
   - 可用網域名稱訪問：`http://esp32car.local`
3. **跨域支援 (CORS)**：
   - 已全域啟用 `Access-Control-Allow-Origin: *`，網頁端前端（React / Vue / 原生 JS）可直接 `fetch`，不會有跨域阻擋。

---

## 🎯 2. 如何讀取前方雷射測距數據？

上位機有 **2 種方式** 讀取 ToF 雷射測距：

### 方法 A：獨立測距查詢端點（最推薦、最輕量）
* **網址**：`GET http://<ESP32_IP>/api/tof`
* **回傳範例 (JSON)**：
```json
{
  "status": "ok",
  "online": true,
  "valid": true,
  "distance_mm": 152.0,
  "distance_cm": 15.2
}
```
* **欄位說明**：
  - `online`: 感測器是否正常運作 (`true` / `false`)
  - `valid`: 當前數值是否有效（障礙物在 20mm ~ 1200mm 範圍內為 `true`，超出量程為 `false`）
  - `distance_mm`: 距離（公釐 mm）
  - `distance_cm`: 距離（公分 cm）

### 方法 B：每次發送控制指令時自動附帶
每次發送 `POST /api/command`（控制小車或手臂）時，回傳的 JSON 裡就**已經自動包含了最新測距數據**，無需分開請求！

---

## 🎮 3. 如何發送控制指令（車輛移動 + 手臂姿態）？

* **核心端點**：`POST http://<ESP32_IP>/api/command`
* **Headers**：`Content-Type: application/json`

### 模式 1：小車底盤運動（支援限時自動煞停）
```json
{
  "action": "forward",   // 可選: "forward"(前進), "backward"(後退), "left"(左轉), "right"(右轉), "spin_left"(左原地旋轉), "spin_right"(右原地旋轉), "stop"(停止), "brake"(急煞)
  "speed": 200,          // PWM 速度: 80 ~ 255 (預設 200)
  "duration": 1000       // 可選毫秒: 執行 1000ms 後自動安全停車 (填 0 或不填代表持續移動)
}
```

### 模式 2：機械手臂 4 關節直接角度控制
```json
{
  "shoulder": 85,        // 大臂仰角: 15 ~ 165 度
  "elbow": 110,          // 小臂俯仰: 15 ~ 165 度
  "wrist": 90,           // 手腕旋轉: 0 ~ 180 度 (90度為水平)
  "gripper": 30          // 夾爪開合: 10 (閉合) ~ 100 (全開) 度
}
```

### 模式 3：機械手臂空間座標逆運動學 (IK)
當 AI 視覺辨識到目標物品在車前時，直接傳入毫米座標，ESP32-S3 會自動解算出舵機角度：
```json
{
  "x": 150.0,            // 車前水平伸展距離 (mm)
  "z": 45.0,             // 離地垂直高度 (mm)
  "wrist": 90,           // 可選: 手腕角度
  "gripper": 25          // 可選: 夾爪夾緊
}
```

### 模式 4：一鍵姿態巨集
```json
{
  "preset": "home"       // 可選: "home"(待命90度), "ground"(地面抓取), "lift"(抬起收納), "open"(張開爪子), "close"(閉合夾緊)
}
```

---

## 💻 4. 團隊程式碼範例 (Python 完整範例)

請將下列腳本存為 `car_client.py`，安裝 `requests` 即可直接執行：

```python
import time
import requests

# 填入小車的實際 IP
ESP32_IP = "192.168.x.x"  # 例如 192.168.4.1 或 192.168.50.123
BASE_URL = f"http://{ESP32_IP}"

def get_distance():
    """讀取 ToF 雷射測距"""
    try:
        res = requests.get(f"{BASE_URL}/api/tof", timeout=1.0)
        if res.status_code == 200:
            data = res.json()
            if data.get("valid"):
                return data.get("distance_cm"), data.get("distance_mm")
            else:
                return None, "超出測量範圍"
    except Exception as e:
        print(f"測距讀取失敗: {e}")
    return None, None

def send_command(payload):
    """發送控制指令給小車/手臂"""
    try:
        res = requests.post(f"{BASE_URL}/api/command", json=payload, timeout=2.0)
        return res.json()
    except Exception as e:
        print(f"指令發送失敗: {e}")
        return None

if __name__ == "__main__":
    print("=== 1. 測試讀取雷射測距 ===")
    cm, mm = get_distance()
    print(f"當前前方障礙物距離: {cm} cm ({mm} mm)\n")

    print("=== 2. 測試控制小車前進 1 秒 ===")
    resp = send_command({
        "action": "forward",
        "speed": 200,
        "duration": 1000
    })
    print(f"車控回應: {resp}\n")
    time.sleep(1.2)

    print("=== 3. 測試機械手臂地面抓取姿態 ===")
    resp = send_command({"preset": "ground"})
    print(f"手臂姿態回應: {resp}\n")
    time.sleep(1.0)

    print("=== 4. 測試夾緊爪子 ===")
    resp = send_command({"gripper": 25})
    print(f"夾爪回應: {resp}\n")
```

---

## ⚡ 5. cURL 終端機測試命令

可以直接在 Mac / Linux 終端機一行指令測試：

```bash
# 1. 讀取雷射距離
curl -s http://192.168.x.x/api/tof

# 2. 讓車子前進 800 毫秒後自停
curl -X POST http://192.168.x.x/api/command \
  -H "Content-Type: application/json" \
  -d '{"action":"forward","speed":200,"duration":800}'

# 3. 讓手臂移動到待命姿態
curl -X POST http://192.168.x.x/api/command \
  -H "Content-Type: application/json" \
  -d '{"preset":"home"}'
```
