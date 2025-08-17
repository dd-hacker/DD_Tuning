// DD_Tuning V1.1.0(Public)

// 後期型コントローラ(トルクセンサーのカプラーがDF62Wのもの)に対応
// DD_Hackが個人的に使用してるものからいくつかの機能が取り除かれたバージョンです。
// DD_Hackはこのスケッチの使用に対して保証を行いません。自己責任で利用してください。

// テスト済み車両(コントローラ型番)
// TB7B41(TB70-T)

// 少し下に進むとセットアップ項目があります。全て確認してから書き込みを行ってください。

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#if defined(CONFIG_BLUEDROID_ENABLED) && __has_include("esp_bt_main.h")
#include "esp_bt_main.h"
#endif
#if defined(CONFIG_BLUEDROID_ENABLED) && __has_include("esp_bt_device.h")
#include "esp_bt_device.h"
#endif
#if defined(CONFIG_BLUEDROID_ENABLED) && __has_include("esp_gap_ble_api.h")
#include "esp_gap_ble_api.h"
#endif
#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <Update.h>
#include <EEPROM.h>
#include <mbedtls/cipher.h>
#include <esp_sleep.h>
#include "driver/temperature_sensor.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_flash.h"

// セットアップ(Step1)
#define KEY_ID "64133933-2f7e-e94b-2025-717fd65b61cd" // このUUIDをアドバタイズするとロック解除されます。ダブルクォーテーションは消さないでください。
#define PIN_CODE "Disable123" // お好きなPINコードを設定してください。オフモードに使います。ダブルクォーテーションは消さないでください。
#define MFG_NUMBER 200 // お好きな数字を設定してください。0から255が使えると思います
#define LOCK_MODE 1 // ロック機能を使う:1 使わない:0

// セットアップ(Step2)
// これらはAESキーというもので、OTAアップデートの際に使用します。添付のツールで変更することを推奨します。
const uint8_t aesKey[32] = {
    0x60, 0xc2, 0x89, 0x17, 0x67, 0x29, 0x1b, 0x3f,
    0x6a, 0x45, 0x79, 0x83, 0x54, 0x86, 0xfe, 0x5a,
    0x76, 0x5e, 0xe4, 0xb0, 0xd0, 0x90, 0x15, 0xc9,
    0x40, 0x21, 0x2f, 0x5d, 0x91, 0x39, 0x29, 0xae
};

const uint8_t aesIv[16] = {
    0x12, 0x9d, 0xc6, 0x01, 0xf5, 0xd1, 0x4b, 0x15,
    0xfc, 0x05, 0xb4, 0x8f, 0xb6, 0x44, 0x8c, 0x50
};

// セットアップ(Step3)
// もしこのGPIO配置が合わない場合は変更してください。
const uint8_t INPUT_PIN_GREEN = 0;
const uint8_t INPUT_PIN_BLUE = 1;
const uint8_t OUTPUT_PIN_GREEN = 2;
const uint8_t OUTPUT_PIN_BLUE = 3;
const uint8_t NOTIFY_LED_PIN = 8;

// セットアップ(Step4)
// OTAアップデートに使用します。SSIDとパスワードを任意のものに変更することを推奨します。
const char* AP_SSID = "Update";
const char* AP_PASSWORD = "ddtuningupdate";

// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// --------------------------------------※以下は自分が何をやってるか理解している人だけいじってください。-------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------

// デバッグシリアル 0:オフ 1:オン
#define DEBUG 0

// ターゲットシステム別にコンパイル
// ESP32-C3を推奨していますが、それ以外でも動作するようにしているつもりです。保証はできません。
// このシステムでは基本的にSoCの性能を余しているので、クロックは低めに制御されます
#if CONFIG_IDF_TARGET_ESP32
  #define ULTRA_HIGH_CLOCK_FREQ 240
  #define HIGH_CLOCK_FREQ 80
  #define MEDIUM_CLOCK_FREQ 40
  #define LOW_CLOCK_FREQ 10
  
#elif CONFIG_IDF_TARGET_ESP32S3
  #define ULTRA_HIGH_CLOCK_FREQ 240
  #define HIGH_CLOCK_FREQ 80
  #define MEDIUM_CLOCK_FREQ 40
  #define LOW_CLOCK_FREQ 10
  
#elif CONFIG_IDF_TARGET_ESP32C2
  #define ULTRA_HIGH_CLOCK_FREQ 120
  #define HIGH_CLOCK_FREQ 80
  #define MEDIUM_CLOCK_FREQ 40
  #define LOW_CLOCK_FREQ 10
  
#elif CONFIG_IDF_TARGET_ESP32C3
  #define ULTRA_HIGH_CLOCK_FREQ 160
  #define HIGH_CLOCK_FREQ 80
  #define MEDIUM_CLOCK_FREQ 40
  #define LOW_CLOCK_FREQ 10

#elif CONFIG_IDF_TARGET_ESP32C6
  #define ULTRA_HIGH_CLOCK_FREQ 160
  #define HIGH_CLOCK_FREQ 80
  #define MEDIUM_CLOCK_FREQ 40
  #define LOW_CLOCK_FREQ 10

#endif

#define FIRMWARE_VERSION "1.1.0"

#define EEPROM_SIZE 32
#define MFG_NUMBER_ADDR 0
#define MFG_NUMBER_INITIALIZED_FLAG 0xAA
#define CONTROL_MODE_ADDR 16
#define OPERATION_MODE_ADDR 17

#define MODE_INITIALIZED_FLAG 0xBB

#define SERVICE_UUID "a6679495-4ebf-ef08-1139-66c1f226c70b"
#define PRESET_CHAR_UUID "bd6d5a2a-c6e3-335a-6d0e-6511fdeac94d"
#define MODE_CHAR_UUID "3eb13d83-c0fc-79fd-e335-e321c5f7ed11"
#define UPDATE_MODE_CHAR_UUID "15cceef4-bf7e-35c3-fe51-a9487ef5e459"
#define LOCK_CHAR_UUID "600b44f2-94d6-ecbb-8f7b-0039a7937353"
#define DIAG_UUID "d80be7f9-ab88-8c69-f4cb-4dd863db8cb5"

#define BLE_APPEARANCE 0x0410

temperature_sensor_handle_t temp_sensor = NULL;
mbedtls_cipher_context_t cipher_ctx;

// プリセット
uint32_t BLINK_INTERVAL;
uint32_t DEBOUNCE_DELAY;
uint32_t CHANGE_TIMEOUT;
uint32_t BOOST_DURATION;
uint8_t currentPresetIndex = 0;
bool modesInitialized = false;
struct TimingPreset {
  uint32_t blinkInterval;
  uint32_t debounceDelay;
  uint32_t changeTimeout;
  uint32_t boostDuration;
};

// 走行制御
const uint8_t BOOST_TRIGGER_COUNT = 3;
const uint32_t QUICK_BOOST_DETECTION_TIME = 500000;
uint32_t lastDebounceTimeGreen = 0;
uint32_t lastDebounceTimeBlue = 0;
uint32_t lastChangeTime = 0;
uint32_t lastBlinkTime = 0;
uint32_t boostStartTime = 0;
uint8_t inputChangeCount = 0;
uint32_t firstInputTime = 0;
uint8_t lastInputStateGreen = HIGH;
uint8_t lastInputStateBlue = LOW;
uint8_t inputStateGreen = HIGH;
uint8_t inputStateBlue = LOW;
bool isBlinking = false;
bool OutputState = LOW;
bool inputChangedDuringTimeout = false;
bool lastStableState = HIGH;
bool isInBoostMode = false;
bool inputStateChanged = false;
bool isTurboMode = false;
bool offMode = false;
bool updateMode = false;
bool isSystemLocked = false;

// 通知LED
#define BLINK_COUNT 6
#define NOTIFY_BLINK_INTERVAL 500
#define SYSTEM_LOCKED_BLINK_INTERVAL 200
uint8_t notifyBlinkCounter = 0;
uint32_t notifyLastBlinkTime = 0;
uint32_t notifyLEDOnTime = 0;
bool notifyBlinkActive = false;
bool notifyLEDState = false;

// BLE
#define ADVERTISE_TIMEOUT 20000
uint32_t advertiseStartTime = 0;
uint32_t lastScanStartTime = 0;
uint32_t lastScanTime = 0;
bool isAdvertising = false;
bool wasConnected = false;
bool deviceConnected = false;
bool isSetupblestep = false;
bool isScanning = false;

// クロック制御(クロックを下げる用意が整ってから1秒後に下げる)
#define CLOCK_REDUCTION_DELAY 1000
uint32_t lastClockChangeTime = 0;
enum ClockMode {
    ULTRA_HIGH_CLOCK,
    HIGH_CLOCK,
    MEDIUM_CLOCK,
    LOW_CLOCK
};

// 温度保護(15秒に1回チェック、センサ温度が80℃超えてたら停止)
#define TEMPERATURE_THRESHOLD 80
#define TEMP_CHECK_INTERVAL 15000
uint32_t lastCheckTime = 0;

// Debug
const uint16_t MEMORY_CHECK_INTERVAL = 5000;
uint32_t lastMemoryCheckTime = 0;
const uint16_t interval = 50; 
uint32_t lastsendMillis = 0;
float currentRPMForDiag = 0.0f; // 診断データ送信用
float currentTemperatureForDiag = 0.0f; // 診断データ用温度

BLEServer* pServer = nullptr;
BLECharacteristic* pPresetCharacteristic = nullptr;
BLECharacteristic* pModeCharacteristic = nullptr;
BLECharacteristic* pUpdateModeCharacteristic = nullptr;
BLECharacteristic* pLockCharacteristic = nullptr;
BLEScan* pBLEScan = nullptr;
BLECharacteristic* pCountCharacteristic = nullptr;
BLECharacteristic *pDiagCharacteristic = nullptr;

ClockMode requiredMode = LOW_CLOCK;
ClockMode currentClockMode = LOW_CLOCK;
WebServer server(80);

TimingPreset presets[] = {
  { 5300, 3800, 80000, 0 },        // Normal       0
  { 5300, 3800, 200000, 0 },       // Cruise       1
  { 5270, 2500, 45000, 1000000 },  // Fun          2
  { 0, 3800, 80000, 0 },           // Auto         3
  { 0, 3800, 80000, 0 },           // Eco-Charging 4 (利用不可)
  { 5270, 2500, 100000, 5000000 }, // Turbo        5
};

const char* serverIndex = R"(
<!DOCTYPE html>
<html lang="ja">
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>DD_Tuning OTA アップデート</title>
    <style>
        :root {
            --primary: #00d4aa;
            --primary-dark: #00b894;
            --primary-light: #00e6c7;
            --secondary: #6c5ce7;
            --accent: #fd79a8;
            --surface: #ffffff;
            --surface-dark: #f8f9fa;
            --background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            --text: #2d3436;
            --text-light: #636e72;
            --text-white: #ffffff;
            --success: #00b894;
            --error: #e17055;
            --warning: #fdcb6e;
            --shadow: 0 8px 25px rgba(0, 0, 0, 0.1);
            --shadow-lg: 0 15px 50px rgba(0, 0, 0, 0.15);
            --border-radius: 12px;
            --transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
        }

        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            -webkit-tap-highlight-color: transparent;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Helvetica Neue', 'Arial', 'Noto Sans', sans-serif;
            background: var(--background);
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 15px;
            line-height: 1.6;
            -webkit-font-smoothing: antialiased;
            -moz-osx-font-smoothing: grayscale;
        }

        .container {
            background: var(--surface);
            border-radius: var(--border-radius);
            box-shadow: var(--shadow-lg);
            padding: clamp(20px, 5vw, 40px);
            max-width: 500px;
            width: 100%;
            position: relative;
            overflow: hidden;
            margin: auto;
        }

        .container::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 4px;
            background: linear-gradient(90deg, var(--primary), var(--secondary), var(--accent));
        }

        .header {
            text-align: center;
            margin-bottom: clamp(20px, 5vw, 30px);
        }

        .logo {
            width: clamp(50px, 12vw, 60px);
            height: clamp(50px, 12vw, 60px);
            background: linear-gradient(135deg, var(--primary), var(--secondary));
            border-radius: 50%;
            margin: 0 auto 15px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: clamp(18px, 4vw, 24px);
            color: var(--text-white);
            font-weight: bold;
        }

        h1 {
            color: var(--text);
            font-size: clamp(20px, 5vw, 24px);
            font-weight: 700;
            margin-bottom: 8px;
            line-height: 1.3;
        }

        .version {
            color: var(--text-light);
            font-size: clamp(12px, 3vw, 14px);
            margin-bottom: 20px;
        }

        .alert {
            background: linear-gradient(135deg, var(--warning), #e8b923);
            color: var(--text-white);
            padding: clamp(12px, 3vw, 15px);
            border-radius: var(--border-radius);
            margin-bottom: clamp(20px, 4vw, 25px);
            display: flex;
            align-items: center;
            gap: 10px;
            font-weight: 500;
            font-size: clamp(13px, 3.5vw, 16px);
        }

        .alert-icon {
            font-size: clamp(16px, 4vw, 20px);
            flex-shrink: 0;
        }

        .upload-area {
            border: 2px dashed #ddd;
            border-radius: var(--border-radius);
            padding: clamp(20px, 5vw, 30px);
            text-align: center;
            margin-bottom: clamp(20px, 4vw, 25px);
            transition: var(--transition);
            cursor: pointer;
            background: var(--surface-dark);
            min-height: 120px;
            display: flex;
            flex-direction: column;
            justify-content: center;
            touch-action: manipulation;
        }

        .upload-area:hover, .upload-area.dragover {
            border-color: var(--primary);
            background: rgba(0, 212, 170, 0.05);
        }

        .upload-area:active {
            transform: scale(0.98);
        }

        .upload-icon {
            width: clamp(40px, 10vw, 50px);
            height: clamp(40px, 10vw, 50px);
            background: var(--primary);
            border-radius: 50%;
            margin: 0 auto 12px;
            display: flex;
            align-items: center;
            justify-content: center;
            color: var(--text-white);
            font-size: clamp(16px, 4vw, 20px);
        }

        .upload-text {
            color: var(--text);
            font-weight: 500;
            margin-bottom: 5px;
            font-size: clamp(14px, 3.5vw, 16px);
        }

        .upload-subtext {
            color: var(--text-light);
            font-size: clamp(12px, 3vw, 14px);
            line-height: 1.4;
        }

        .file-info {
            background: rgba(0, 212, 170, 0.1);
            padding: clamp(12px, 3vw, 15px);
            border-radius: var(--border-radius);
            margin-top: 15px;
            display: none;
            color: var(--primary-dark);
            font-weight: 500;
            font-size: clamp(13px, 3vw, 14px);
            word-break: break-word;
        }

        .btn {
            width: 100%;
            padding: clamp(12px, 3vw, 15px);
            border: none;
            border-radius: var(--border-radius);
            font-size: clamp(14px, 3.5vw, 16px);
            font-weight: 600;
            cursor: pointer;
            transition: var(--transition);
            text-transform: uppercase;
            letter-spacing: 0.5px;
            touch-action: manipulation;
            min-height: 48px;
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .btn-primary {
            background: linear-gradient(135deg, var(--primary), var(--primary-dark));
            color: var(--text-white);
        }

        .btn-primary:hover:not(:disabled) {
            transform: translateY(-1px);
            box-shadow: var(--shadow);
        }

        .btn-primary:active:not(:disabled) {
            transform: translateY(0);
        }

        .btn:disabled {
            opacity: 0.6;
            cursor: not-allowed;
            transform: none !important;
        }

        .progress-container {
            display: none;
            margin-top: clamp(20px, 4vw, 25px);
        }

        .progress-bar {
            width: 100%;
            height: 8px;
            background: #e9ecef;
            border-radius: 4px;
            overflow: hidden;
            margin-bottom: 15px;
        }

        .progress-fill {
            height: 100%;
            background: linear-gradient(90deg, var(--primary), var(--secondary));
            width: 0%;
            transition: width 0.3s ease;
            position: relative;
        }

        .progress-fill::after {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.3), transparent);
            animation: shimmer 1.5s infinite;
        }

        @keyframes shimmer {
            0% { transform: translateX(-100%); }
            100% { transform: translateX(100%); }
        }

        .progress-text {
            text-align: center;
            color: var(--text-light);
            font-weight: 500;
            font-size: clamp(13px, 3vw, 14px);
        }

        .message {
            padding: clamp(15px, 4vw, 20px);
            border-radius: var(--border-radius);
            margin-top: 20px;
            text-align: center;
            font-weight: 500;
            display: none;
            animation: slideUp 0.3s ease;
            font-size: clamp(14px, 3.5vw, 16px);
        }

        @keyframes slideUp {
            from {
                opacity: 0;
                transform: translateY(20px);
            }
            to {
                opacity: 1;
                transform: translateY(0);
            }
        }

        .message.success {
            background: linear-gradient(135deg, var(--success), #00a085);
            color: var(--text-white);
        }

        .message.error {
            background: linear-gradient(135deg, var(--error), #d63031);
            color: var(--text-white);
        }

        .reboot-info {
            margin-top: 12px;
            font-size: clamp(12px, 3vw, 14px);
            opacity: 0.9;
            line-height: 1.4;
        }

        .credits {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            padding: clamp(12px, 3vw, 15px);
            border-radius: var(--border-radius);
            box-shadow: var(--shadow);
            font-size: clamp(11px, 2.5vw, 13px);
            color: var(--text-light);
            margin-top: 20px;
            text-align: center;
        }

        .credits p {
            margin: 3px 0;
        }

        .credits .highlight {
            color: var(--primary-dark);
            font-weight: 600;
        }

        .claude-tooltip {
            position: relative;
            cursor: help;
            text-decoration: underline;
            text-decoration-style: dotted;
        }

        .claude-tooltip:hover::after {
            content: "このシステムはAnthropic社のClaude Sonnet 4 AIアシスタントの支援により作成されました";
            position: absolute;
            bottom: 100%;
            right: 0;
            background: var(--text);
            color: var(--text-white);
            padding: 10px;
            border-radius: 6px;
            font-size: 12px;
            white-space: nowrap;
            z-index: 1000;
            margin-bottom: 5px;
            opacity: 0;
            animation: fadeIn 0.2s ease forwards;
        }

        @keyframes fadeIn {
            to { opacity: 1; }
        }

        @media (max-width: 600px) {
            .container {
                padding: 30px 20px;
                margin: 10px;
            }
            
            .credits {
                position: static;
                margin-top: 30px;
                width: 100%;
                text-align: center;
            }
        }

        .hidden {
            display: none !important;
        }

        /* モバイル専用の最適化 */
        @media (max-width: 768px) {
            body {
                padding: 10px;
            }
            
            .container {
                margin: 0;
                min-height: auto;
                border-radius: 8px;
            }
            
            .upload-area {
                min-height: 100px;
                padding: 20px 15px;
            }
            
            .claude-tooltip:hover::after {
                display: none;
            }
        }

        /* 極小画面への対応 */
        @media (max-width: 360px) {
            .container {
                padding: 15px;
            }
            
            .header {
                margin-bottom: 15px;
            }
            
            .alert {
                padding: 10px;
                gap: 8px;
            }
            
            .upload-area {
                padding: 15px 10px;
                min-height: 90px;
            }
        }

        /* 高解像度画面への対応 */
        @media (min-width: 1200px) {
            .container {
                max-width: 600px;
            }
        }

        /* ダークモード対応（システム設定に従う） */
        @media (prefers-color-scheme: dark) {
            :root {
                --surface: #2d3748;
                --surface-dark: #4a5568;
                --text: #e2e8f0;
                --text-light: #a0aec0;
                --background: linear-gradient(135deg, #2d3748 0%, #4a5568 100%);
            }
            
            .upload-area {
                border-color: #4a5568;
                background: var(--surface-dark);
            }
            
            .credits {
                background: rgba(45, 55, 72, 0.95);
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>DD_Tuning OTA アップデート</h1>
            <div class="version">バージョン 1.1.0</div>
        </div>

        <div class="alert">
            <span class="alert-icon">⚠️</span>
            <span>アップデート中は絶対に電源を切らないでください</span>
        </div>

        <div class="message success" id="success-message">
            ✅ アップデートが正常に完了しました！
            <div class="reboot-info">
                デバイスが自動的に再起動されます。<br>
                このウィンドウを閉じてください。
            </div>
        </div>

        <div class="message error" id="error-message">
            ❌ アップデートに失敗しました
            <div id="error-details"></div>
        </div>

        <form id="upload-form" method="POST" action="/update" enctype="multipart/form-data">
            <div class="upload-area" id="upload-area">
                <input type="file" id="file-input" name="update" accept=".bin" style="display: none;">
                <div class="upload-icon">📤</div>
                <div class="upload-text">ファームウェアファイルを選択</div>
                <div class="upload-subtext">ファイルをタップして選択してください</div>
                <div class="file-info" id="file-info"></div>
            </div>
            
            <button type="submit" class="btn btn-primary" id="upload-btn" disabled>
                アップデート開始
            </button>
        </form>

        <div class="progress-container" id="progress-container">
            <div class="progress-bar">
                <div class="progress-fill" id="progress-fill"></div>
            </div>
            <div class="progress-text" id="progress-text">準備中...</div>
        </div>

        <div class="credits">
            <p>Developed by <span class="highlight">DD_Hack</span></p>
            <p>AI Assistant: <span class="highlight">Claude Sonnet 4</span></p>
            <p>Smart Bicycle Tuning System</p>
        </div>
    </div>

    <script>
        const uploadArea = document.getElementById('upload-area');
        const fileInput = document.getElementById('file-input');
        const fileInfo = document.getElementById('file-info');
        const uploadBtn = document.getElementById('upload-btn');
        const uploadForm = document.getElementById('upload-form');
        const progressContainer = document.getElementById('progress-container');
        const progressFill = document.getElementById('progress-fill');
        const progressText = document.getElementById('progress-text');
        const successMessage = document.getElementById('success-message');
        const errorMessage = document.getElementById('error-message');
        const errorDetails = document.getElementById('error-details');

        // ファイルドラッグ&ドロップ処理（デスクトップのみ）
        uploadArea.addEventListener('click', () => fileInput.click());
        
        if (!('ontouchstart' in window)) {
            uploadArea.addEventListener('dragover', (e) => {
                e.preventDefault();
                uploadArea.classList.add('dragover');
            });
            
            uploadArea.addEventListener('dragleave', () => {
                uploadArea.classList.remove('dragover');
            });
            
            uploadArea.addEventListener('drop', (e) => {
                e.preventDefault();
                uploadArea.classList.remove('dragover');
                const files = e.dataTransfer.files;
                if (files.length > 0) {
                    fileInput.files = files;
                    handleFileSelect();
                }
            });
        } else {
            // モバイルデバイスではドラッグ&ドロップテキストを変更
            document.querySelector('.upload-subtext').textContent = 'ファイルをタップして選択してください';
        }

        fileInput.addEventListener('change', handleFileSelect);

        function handleFileSelect() {
            const file = fileInput.files[0];
            if (file) {
                if (file.name.endsWith('.bin')) {
                    fileInfo.textContent = `選択されたファイル: ${file.name} (${formatFileSize(file.size)})`;
                    fileInfo.style.display = 'block';
                    uploadBtn.disabled = false;
                    hideMessages();
                } else {
                    showError('無効なファイル形式です。.binファイルを選択してください。');
                    resetForm();
                }
            }
        }

        function formatFileSize(bytes) {
            if (bytes === 0) return '0 Bytes';
            const k = 1024;
            const sizes = ['Bytes', 'KB', 'MB', 'GB'];
            const i = Math.floor(Math.log(bytes) / Math.log(k));
            return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
        }

        function hideMessages() {
            successMessage.style.display = 'none';
            errorMessage.style.display = 'none';
        }

        function showSuccess() {
            hideMessages();
            successMessage.style.display = 'block';
        }

        function showError(message) {
            hideMessages();
            errorDetails.textContent = message;
            errorMessage.style.display = 'block';
        }

        function resetForm() {
            fileInput.value = '';
            fileInfo.style.display = 'none';
            uploadBtn.disabled = true;
            progressContainer.style.display = 'none';
            progressFill.style.width = '0%';
        }

        uploadForm.addEventListener('submit', (e) => {
            e.preventDefault();
            
            const file = fileInput.files[0];
            if (!file) return;

            uploadForm.style.display = 'none';
            progressContainer.style.display = 'block';
            hideMessages();

            const formData = new FormData();
            formData.append('update', file);

            const xhr = new XMLHttpRequest();

            xhr.upload.addEventListener('progress', (e) => {
                if (e.lengthComputable) {
                    const percent = Math.round((e.loaded / e.total) * 100);
                    progressFill.style.width = percent + '%';
                    progressText.textContent = `アップロード中... ${percent}%`;
                }
            });

            xhr.addEventListener('load', () => {
                if (xhr.status === 200) {
                    progressFill.style.width = '100%';
                    progressText.textContent = '完了！デバイスを再起動中...';
                    setTimeout(() => {
                        showSuccess();
                        progressContainer.style.display = 'none';
                    }, 1000);
                } else {
                    showError(xhr.responseText || 'アップデートに失敗しました');
                    uploadForm.style.display = 'block';
                    progressContainer.style.display = 'none';
                    uploadBtn.disabled = false;
                }
            });

            xhr.addEventListener('error', () => {
                showError('通信エラーが発生しました。接続を確認してください。');
                uploadForm.style.display = 'block';
                progressContainer.style.display = 'none';
                uploadBtn.disabled = false;
            });

            xhr.open('POST', '/update');
            xhr.send(formData);
        });

        document.addEventListener('DOMContentLoaded', () => {
            resetForm();
        });

        function adjustViewport() {
            const viewport = document.querySelector('meta[name=viewport]');
            if (viewport && window.innerWidth <= 768) {
                viewport.setAttribute('content', 'width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no');
            }
        }

        window.addEventListener('resize', adjustViewport);
        adjustViewport();
    </script>
</body>
</html>
)";

void IRAM_ATTR processInput(uint32_t currentMicros) {
    static uint32_t lastValidChangeTime = 0;
    static uint32_t intervalHistory[12] = {0};
    static int historyIndex = 0;
    static float avgRPM = 0.0f;
    static float lastRawRPM = 0.0f;
    static uint32_t avgInterval = 0;
    static uint32_t steadyRpmStartTime = 0;
    static bool isStableRpm = false;
    static uint32_t instabilityDetectedTime = 0;
    static uint8_t stableSampleCount = 0;
    static float rpmHistoryTrend[5] = {0};
    static int rpmHistoryTrendIndex = 0;
    static uint32_t lastTrendCheckTime = 0;
    static bool decreasingTrend = false;
    static uint8_t decreaseTrendCount = 0;
    static uint32_t lastBlinkInterval = 5300;
    static uint32_t lastRpmCheckTime = 0;

    // --- ピン入力読み取り & デバウンス ---
    int readingGreen = digitalRead(INPUT_PIN_GREEN);
    int readingBlue = digitalRead(INPUT_PIN_BLUE);
    bool greenChanged = (readingGreen != lastInputStateGreen);
    bool blueChanged = (readingBlue != lastInputStateBlue);
    bool stateChangedSinceDebounce = false;

    if (greenChanged) {
        lastDebounceTimeGreen = currentMicros;
    }
    if ((currentMicros - lastDebounceTimeGreen) > DEBOUNCE_DELAY) {
        if (readingGreen != inputStateGreen) {
            inputStateGreen = readingGreen;
            inputStateChanged = true;
            stateChangedSinceDebounce = true;
            lastDebounceTimeGreen = currentMicros;
        }
    }

    if (blueChanged) {
        lastDebounceTimeBlue = currentMicros;
    }
    if ((currentMicros - lastDebounceTimeBlue) > DEBOUNCE_DELAY) {
        if (readingBlue != inputStateBlue) {
            inputStateBlue = readingBlue;
            inputStateChanged = true;
            stateChangedSinceDebounce = true;
            lastDebounceTimeBlue = currentMicros;
        }
    }

    // ケイデンス(RPM)計算 & 状態判定
    if (stateChangedSinceDebounce) {
        if (lastValidChangeTime != 0) {
            uint32_t interval = currentMicros - lastValidChangeTime;

            // 有効な範囲 (2000-500000μs ≒ 5-600rpm on 48ppr)
            if (interval > 2000 && interval < 500000) {
                intervalHistory[historyIndex] = interval;
                historyIndex = (historyIndex + 1) % 12;

                // --- 平均間隔計算 (重み付き) ---
                float weightedAvg = 0; float totalWeight = 0; uint8_t validCount = 0;
                for (int i = 0; i < 12; i++) { if (intervalHistory[i] > 0) validCount++; }
                for (int i = 0; i < 12; i++) {
                    int idx = (historyIndex - 1 - i + 12) % 12;
                    if (intervalHistory[idx] > 0) {
                        float weight = validCount - i; if (i >= validCount) weight = 1;
                        weightedAvg += intervalHistory[idx] * weight; totalWeight += weight;
                    }
                }
                avgInterval = (totalWeight > 0) ? (uint32_t)(weightedAvg / totalWeight) : 0;
                if (avgInterval > 0) {
                    lastRawRPM = 60000000.0f / ((float)avgInterval * 48.0f);
                    
                    if (avgRPM == 0) {
                        avgRPM = lastRawRPM;
                    }
                    else {
                        avgRPM = avgRPM * 0.5f + lastRawRPM * 0.5f;
                    }
                }
                else {
                    lastRawRPM = 0;
                }

                currentRPMForDiag = avgRPM;
                
                const float RPM_TOLERANCE_FOR_STABLE = 8.0f;
                float rpmDiff = fabs(lastRawRPM - avgRPM);

                if (rpmDiff > RPM_TOLERANCE_FOR_STABLE) {
                    steadyRpmStartTime = 0;
                    stableSampleCount = 0;
                    if (isStableRpm) {
                        isStableRpm = false;
                        instabilityDetectedTime = currentMicros;
                        lastBlinkInterval = BLINK_INTERVAL;
                        #if DEBUG
                        Serial.printf("RPM Unstable (Diff: %.1f)\n", rpmDiff);
                        #endif
                    }
                }
                else {
                    if (!isStableRpm) {
                        if (steadyRpmStartTime == 0) {
                            steadyRpmStartTime = currentMicros;
                            stableSampleCount = 1;
                        }
                        else {
                            stableSampleCount++;
                            const uint32_t STABLE_TIME_THRESHOLD = 200000;
                            const uint8_t STABLE_COUNT_THRESHOLD = 3;
                            if ((currentMicros - steadyRpmStartTime >= STABLE_TIME_THRESHOLD) && (stableSampleCount >= STABLE_COUNT_THRESHOLD)) {
                                isStableRpm = true;
                                lastRpmCheckTime = currentMicros;
                                lastBlinkInterval = BLINK_INTERVAL;
                                instabilityDetectedTime = 0;
                                #if DEBUG
                                Serial.print("RPM Stable at: "); Serial.println(avgRPM);
                                #endif
                                decreasingTrend = false;
                                decreaseTrendCount = 0;
                            }                        }
                    }
                    else {
                        lastRpmCheckTime = currentMicros;
                    }
                }
                
                // --- 下降トレンド検出 (100ms毎にチェック - 要調整) ---
                const uint32_t TREND_CHECK_INTERVAL = 100000;
                const float DECREASE_TREND_THRESHOLD = -6.0f;
                const uint8_t TREND_COUNT_THRESHOLD = 3;

                if (currentMicros - lastTrendCheckTime >= TREND_CHECK_INTERVAL && avgRPM > 1.0f) {
                    rpmHistoryTrend[rpmHistoryTrendIndex] = avgRPM;
                    rpmHistoryTrendIndex = (rpmHistoryTrendIndex + 1) % 5;
                    lastTrendCheckTime = currentMicros;

                    bool hasEnoughData = true;
                    for (int i = 0; i < 5; i++) { if (rpmHistoryTrend[i] == 0) hasEnoughData = false; }

                    if (hasEnoughData) {
                        float sumDiff = 0;
                        for (int i = 0; i < 4; i++) {
                            int currentIdx = (rpmHistoryTrendIndex - 1 - i + 5) % 5;
                            int prevIdx = (rpmHistoryTrendIndex - 1 - (i + 1) + 5) % 5;
                            sumDiff += (rpmHistoryTrend[currentIdx] - rpmHistoryTrend[prevIdx]);
                        }
                        float avgDiff = sumDiff / 4.0f;

                        if (avgDiff < DECREASE_TREND_THRESHOLD) {
                            decreaseTrendCount++;
                            if (decreaseTrendCount >= TREND_COUNT_THRESHOLD) {
                                if (!decreasingTrend) {
                                    decreasingTrend = true;
                                    if (isStableRpm) {
                                        isStableRpm = false;
                                        instabilityDetectedTime = currentMicros;
                                        lastBlinkInterval = BLINK_INTERVAL;
                                        #if DEBUG
                                        Serial.println("RPM Decreasing Trend Detected -> Unstable");
                                        #endif
                                    }
                                    if (instabilityDetectedTime == 0) {
                                         instabilityDetectedTime = currentMicros;
                                         lastBlinkInterval = BLINK_INTERVAL;
                                     }
                                }
                            }
                        }
                        else {
                            decreaseTrendCount = 0;
                            if (decreasingTrend) {
                                decreasingTrend = false;
                                #if DEBUG
                                Serial.println("RPM Decreasing Trend Ended");
                                #endif
                            }
                        }
                    }
                }
            }
            else {
                avgInterval = 0;
            }
        }
        lastValidChangeTime = currentMicros;
        
        #if DEBUG
        static uint32_t lastStabilityDebugTime = 0;
        if (currentMicros - lastStabilityDebugTime > 500000) {
            Serial.printf("RPM: %.1f, Diff: %.1f, Stable: %s, Duration: %dms, Samples: %d, AvgInt: %d\n",
                          avgRPM, 
                          avgRPM > 0 ? fabs(lastRawRPM - avgRPM) : 0,
                          isStableRpm ? "YES" : "NO",
                          isStableRpm ? (currentMicros - steadyRpmStartTime) / 1000 : 0,
                          stableSampleCount,
                          avgInterval);
            lastStabilityDebugTime = currentMicros;
        }
        #endif
    }
    if (currentPresetIndex == 3) { // Autoモードの場合のみ実行
        const uint32_t AUTO_BASE_INTERVAL = 5300;
        const uint32_t AUTO_TRANSITION_TIME_TO_FOLLOW = 300000;
        const uint32_t AUTO_TRANSITION_TIME_TO_BASE = 100000;

        const uint32_t AUTO_FOLLOW_MIN_INTERVAL = 5300; 
        const uint32_t AUTO_FOLLOW_MAX_INTERVAL = 100000; 

        uint32_t targetBlinkInterval;
        uint32_t transitionTime;
        uint32_t transitionStartTime = 0;
        uint32_t previousInterval;

        if (isStableRpm && !decreasingTrend) {
            transitionTime = AUTO_TRANSITION_TIME_TO_FOLLOW;
            transitionStartTime = lastRpmCheckTime;
            previousInterval = lastBlinkInterval;

            if (avgInterval > 0) {
                targetBlinkInterval = avgInterval;

                targetBlinkInterval = max(AUTO_FOLLOW_MIN_INTERVAL, targetBlinkInterval);
                targetBlinkInterval = min(AUTO_FOLLOW_MAX_INTERVAL, targetBlinkInterval);

            }
            else {
                targetBlinkInterval = AUTO_BASE_INTERVAL;
            }

            #if DEBUG
            if(BLINK_INTERVAL != targetBlinkInterval) Serial.println("State: Stable -> Target FOLLOW Calc");
            #endif

        }
        else {
            targetBlinkInterval = AUTO_BASE_INTERVAL;
            transitionTime = AUTO_TRANSITION_TIME_TO_BASE;
            if (instabilityDetectedTime != 0) {
                 transitionStartTime = instabilityDetectedTime;
                 previousInterval = lastBlinkInterval;
            }
            else {
                 transitionStartTime = 0;
                 previousInterval = AUTO_BASE_INTERVAL;
            }

            #if DEBUG
            if(BLINK_INTERVAL != targetBlinkInterval) {
                if (!isStableRpm) Serial.println("State: Unstable -> Target BASE");
                else if (decreasingTrend) Serial.println("State: Decreasing -> Target BASE");
                else Serial.println("State: Initial? -> Target BASE");
            }
            #endif
        }

        // --- BLINK_INTERVAL の更新 (滑らかな移行) ---
        if (transitionStartTime != 0 && previousInterval != targetBlinkInterval) {
            uint32_t transitionElapsed = currentMicros - transitionStartTime;

            if (transitionElapsed <= transitionTime) {
                float t = (float)transitionElapsed / transitionTime;
                if (transitionTime == 0) t = 1.0f;

                BLINK_INTERVAL = (uint32_t)(previousInterval * (1.0f - t) + targetBlinkInterval * t);

                uint32_t currentMin = min(AUTO_BASE_INTERVAL, AUTO_FOLLOW_MIN_INTERVAL);
                uint32_t currentMax = max(AUTO_BASE_INTERVAL, AUTO_FOLLOW_MAX_INTERVAL);
                BLINK_INTERVAL = max(currentMin, BLINK_INTERVAL);
                BLINK_INTERVAL = min(currentMax, BLINK_INTERVAL);

            }
            else {
                BLINK_INTERVAL = targetBlinkInterval;

                if (targetBlinkInterval == AUTO_BASE_INTERVAL) {
                     instabilityDetectedTime = 0;
                }
                lastBlinkInterval = BLINK_INTERVAL;
            }
             #if DEBUG
             static uint32_t lastDebugPrintTime = 0;
             if (currentMicros - lastDebugPrintTime > 100000) {
                Serial.printf("Target=%d, Prev=%d, Curr=%d, Stable=%d, Decr=%d, t=%.2f, AvgInt=%d, RPM=%.1f\n",
                              targetBlinkInterval, previousInterval, BLINK_INTERVAL, isStableRpm, decreasingTrend, min(1.0f, (float)transitionElapsed / transitionTime), avgInterval, avgRPM);
                lastDebugPrintTime = currentMicros;
             }
             #endif

        }
        else {
            BLINK_INTERVAL = targetBlinkInterval;
            lastBlinkInterval = BLINK_INTERVAL;
        }
    }
    lastInputStateGreen = readingGreen;
    lastInputStateBlue = readingBlue;
}

void IRAM_ATTR processInputCount(uint32_t currentMicros) {
    if (inputStateChanged) {
        inputChangeCount++;
        #if DEBUG
        Serial.printf("Input Count: %d\n", inputChangeCount);
        #endif
        
        if (inputChangeCount == 1) {
            firstInputTime = currentMicros;
        }
        
        if (!isBlinking && !isInBoostMode) {
            isBlinking = true;
            inputChangedDuringTimeout = true;
        }
        
        lastChangeTime = currentMicros;
        if (!isInBoostMode && inputChangeCount >= BOOST_TRIGGER_COUNT && BOOST_DURATION > 0) {
            uint32_t timeSinceFirst = currentMicros - firstInputTime;
            if (timeSinceFirst <= QUICK_BOOST_DETECTION_TIME) {
                isInBoostMode = true;
                boostStartTime = currentMicros;
                #if DEBUG
                Serial.println("Quick Boost Mode Activation");
                #endif
            }
        }
        if (isInBoostMode && BOOST_DURATION != 0) {
            boostStartTime = currentMicros;
            #if DEBUG
            Serial.println("Boost Extended");
            #endif
        }
        inputStateChanged = false;
    }
    if (isBlinking) {
        uint32_t timeSinceLastInput = currentMicros - lastChangeTime;
        
        // ペダル蹴り検出(ペダルが少し動いて止まったのを検知しブーストを素早く立ち上がらせる)
        if (!isInBoostMode && inputChangeCount >= BOOST_TRIGGER_COUNT && BOOST_DURATION > 0 && 
            timeSinceLastInput >= 50000 && timeSinceLastInput <= 200000) {
            uint32_t totalInputTime = lastChangeTime - firstInputTime;
            if (totalInputTime <= QUICK_BOOST_DETECTION_TIME) {
                isInBoostMode = true;
                boostStartTime = currentMicros;
                #if DEBUG
                Serial.println("Kick Pedal Boost Mode Activation");
                #endif
            }
        }
        
        if (timeSinceLastInput >= CHANGE_TIMEOUT) {
            // 必要回数以上の入力があった状態でペダルが止まるとブースト
            if (!isInBoostMode && inputChangeCount >= BOOST_TRIGGER_COUNT && BOOST_DURATION > 0) {
                isInBoostMode = true;
                boostStartTime = currentMicros;
                #if DEBUG
                Serial.println("Transitioning to Boost Mode");
                #endif
            }
            else {
                // 通常タイムアウト処理
                isBlinking = false;
                inputChangeCount = 0;
                firstInputTime = 0;
                #if DEBUG
                Serial.println("Normal Timeout End");
                #endif
            }
        }
    }
    
    // 0.3秒経ったら非ブースト＆非点滅時のカウントリセット
    if (!isBlinking && !isInBoostMode && 
        (currentMicros - lastChangeTime > 300000)) {
        if (inputChangeCount > 0) {
            #if DEBUG
            Serial.printf("Reset Input Count from %d\n", inputChangeCount);
            #endif
            inputChangeCount = 0;
            firstInputTime = 0;
        }
    }
}

void IRAM_ATTR handleOutputControl(uint32_t currentMicros) {
    if (isSystemLocked) {
        updateOutputs(HIGH);
        return;
    }

    if (offMode) {
        return;
    }

    processInputCount(currentMicros);
    if (isBlinking || isInBoostMode) {
        uint32_t blinkElapsed = currentMicros - lastBlinkTime;
        if (blinkElapsed >= BLINK_INTERVAL) {
            bool newState = !OutputState;
            OutputState = newState;
            updateOutputs(OutputState);
            lastBlinkTime = currentMicros;
        }
        
        // ブースト処理（BOOST_DURATIONが設定されている場合のみ）
        if (isInBoostMode && BOOST_DURATION > 0) {
            uint32_t boostElapsed = currentMicros - boostStartTime;
            
            // 指定時間経過でブースト終了
            if (boostElapsed >= BOOST_DURATION) {
                isInBoostMode = false;
                isBlinking = false;
                #if DEBUG
                Serial.println("Boost End");
                #endif
                inputChangeCount = 0;
                firstInputTime = 0;
            }
        }
    }
}

inline void updateOutputs(bool state) {
  digitalWrite(OUTPUT_PIN_GREEN, state);
  digitalWrite(OUTPUT_PIN_BLUE, !state);
}

void manageNotifyLED() {
  uint32_t currentMillis = millis();

  if (isSystemLocked) {
    // システムがロックされている場合、点滅
    if (currentMillis - notifyLastBlinkTime >= SYSTEM_LOCKED_BLINK_INTERVAL) {
      notifyLEDState = !notifyLEDState;
      digitalWrite(NOTIFY_LED_PIN, notifyLEDState);
      notifyLastBlinkTime = currentMillis;
    }
  }
  else {
    // システムがロックされていない場合
    if (!notifyBlinkActive) {
      digitalWrite(NOTIFY_LED_PIN, HIGH);
      return;
    }

    if (currentMillis - notifyLastBlinkTime >= NOTIFY_BLINK_INTERVAL) {
      notifyLEDState = !notifyLEDState;
      digitalWrite(NOTIFY_LED_PIN, notifyLEDState);
      notifyLastBlinkTime = currentMillis;

      if (!notifyLEDState) {
        notifyBlinkCounter++;
      }
    }

    if (notifyBlinkCounter >= BLINK_COUNT) {
      notifyBlinkActive = false;
      notifyBlinkCounter = 0;
      digitalWrite(NOTIFY_LED_PIN, HIGH);
    }
  }
}

void updateTimingSettings() {
  if (currentPresetIndex >= 0 && currentPresetIndex < sizeof(presets) / sizeof(presets[0])) {
    BLINK_INTERVAL = presets[currentPresetIndex].blinkInterval;
    DEBOUNCE_DELAY = presets[currentPresetIndex].debounceDelay;
    CHANGE_TIMEOUT = presets[currentPresetIndex].changeTimeout;
    BOOST_DURATION = presets[currentPresetIndex].boostDuration;
    #if DEBUG
    Serial.printf("Preset Updated - Preset: %d, Blink: %d, Debounce: %d, Timeout: %d, BoostDuration: %d\n",
                  currentPresetIndex, BLINK_INTERVAL, DEBOUNCE_DELAY, CHANGE_TIMEOUT, BOOST_DURATION);
    #endif
  }
}

void initializeTimingSettings() {
  if (currentPresetIndex >= 0 && currentPresetIndex < sizeof(presets) / sizeof(presets[0])) {
    updateTimingSettings();
  }
  else {
    currentPresetIndex = 0;
    updateTimingSettings();
  }
}

void initializeControlMode() {
  if (offMode) {
    isBlinking = false;
    inputChangedDuringTimeout = false;
  }
  else {
    updateOutputs(HIGH);
  }
}

void manageBLEConnection() {
    uint32_t currentMillis = millis();

    if (deviceConnected != wasConnected) {
        if (deviceConnected) {
            isAdvertising = false;
            #if DEBUG
            Serial.println("Device Connected");
            #endif
            notifyBlinkActive = true;
            notifyBlinkCounter = 0;
            notifyLastBlinkTime = currentMillis;
            notifyLEDState = false;
            digitalWrite(NOTIFY_LED_PIN, HIGH);
        }
        else {
            #if DEBUG
            Serial.println("Device Disconnected");
            #endif
            // 切断時は再アドバタイズ
            if (BLEDevice::getInitialized()) {
                BLEDevice::startAdvertising();
                advertiseStartTime = currentMillis;
                isAdvertising = true;
                notifyBlinkActive = true;
                notifyBlinkCounter = 0;
                notifyLastBlinkTime = currentMillis;
                notifyLEDState = false;
                digitalWrite(NOTIFY_LED_PIN, HIGH);
            }
        }
        wasConnected = deviceConnected;
    }

    // 未接続状態でアドバタイズしていない場合の処理
    if (!deviceConnected && !isAdvertising && !BLEDevice::getInitialized()) {
        #if DEBUG
        Serial.println("Starting Advertising");
        #endif
        String deviceName = getDeviceName();
        BLEDevice::init(deviceName.c_str());
        BLEDevice::startAdvertising();
        advertiseStartTime = currentMillis;
        isAdvertising = true;
        return;
    }

    // アドバタイズのタイムアウト処理
    // (処理が重くてアシストが遅れることがあるため将来的に変更が必要)
    if (isAdvertising && !deviceConnected && (currentMillis - advertiseStartTime >= ADVERTISE_TIMEOUT)) {
        #if DEBUG
        Serial.println("Advertising Timeout - Stopping BLE");
        #endif

        setCpuFrequencyMhz(ULTRA_HIGH_CLOCK_FREQ);

        pBLEScan->stop();

        // BLE無効化
        BLEDevice::deinit(true);
        esp_bt_controller_disable();
        esp_bt_controller_deinit();

        // Wi-Fi無効化
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        esp_wifi_stop();
        esp_wifi_deinit();
        
        isAdvertising = false;
        isScanning = false;
        return;
    }
}

void setupServer() {

  // ロックされているときはアップデートを禁止する
  // (物理的なアクセスがあると書き換えリスクあり)
  if (isSystemLocked) {
    Serial.println("System Locked!!!");
    return;
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  esp_wifi_set_max_tx_power(40);
  esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

  #if DEBUG
  Serial.println("Setting Up Update System...");
  Serial.printf("SSID: %s\nIP Address: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  int8_t new_power;
  esp_wifi_get_max_tx_power(&new_power);
  Serial.printf("New Wi-Fi TX Power: %d\n", new_power);
  #endif

  static mbedtls_cipher_context_t cipher_ctx;
  mbedtls_cipher_init(&cipher_ctx);
  const mbedtls_cipher_info_t *cipher_info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_256_CBC);
  
  if (mbedtls_cipher_setup(&cipher_ctx, cipher_info) != 0 ||
      mbedtls_cipher_setkey(&cipher_ctx, aesKey, 256, MBEDTLS_DECRYPT) != 0 ||
      mbedtls_cipher_set_iv(&cipher_ctx, aesIv, 16) != 0) {
    #if DEBUG
    Serial.println("Cipher initialization failed");
    #endif
    return;
  }

  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });

  server.on("/update", HTTP_POST, 
    []() {
      server.sendHeader("Connection", "close");
      server.send(Update.hasError() ? 500 : 200, "text/plain", 
        Update.hasError() ? "UPDATE FAILED: " + String(Update.getError()) : "OK");
      if (!Update.hasError()) {
        delay(200);
        ESP.restart();
      }
    },
    []() {
      HTTPUpload& upload = server.upload();
      static uint8_t* decryptBuffer = nullptr;
      static const size_t DECRYPTION_BUFFER_SIZE = 8192;
      static const size_t BATCH_SIZE = 8192;

      if (upload.status == UPLOAD_FILE_START) {
        #if DEBUG
        Serial.printf("Update: %s\n", upload.filename.c_str());
        #endif
        
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
          #if DEBUG
          Serial.printf("Update begin failed! Error: %d\n", Update.getError());
          #endif
          return;
        }

        decryptBuffer = (uint8_t*)heap_caps_malloc(DECRYPTION_BUFFER_SIZE, MALLOC_CAP_DMA);
        if (!decryptBuffer) {
          #if DEBUG
          Serial.println("Failed to allocate decryption buffer");
          #endif
          return;
        }

        mbedtls_cipher_reset(&cipher_ctx);
        mbedtls_cipher_set_iv(&cipher_ctx, aesIv, 16);
      }
      else if (upload.status == UPLOAD_FILE_WRITE && decryptBuffer) {
        if (upload.currentSize > 0) {
          size_t remainingSize = upload.currentSize;
          size_t processedSize = 0;
          
          while (remainingSize > 0) {
            size_t currentBatchSize = min(remainingSize, BATCH_SIZE);
            size_t decryptedSize = 0;
            
            int ret = mbedtls_cipher_update(&cipher_ctx,
                                          upload.buf + processedSize,
                                          currentBatchSize,
                                          decryptBuffer,
                                          &decryptedSize);
            
            if (ret == 0 && decryptedSize > 0) {
              if (!Update.write(decryptBuffer, decryptedSize)) {
                #if DEBUG
                Serial.printf("Update Write Failed! Error: %d\n", Update.getError());
                #endif
                return;
              }
            }
            else if (ret != 0) {
              #if DEBUG
              Serial.printf("Decryption Failed! Error: %d\n", ret);
              #endif
              return;
            }
            
            processedSize += currentBatchSize;
            remainingSize -= currentBatchSize;
          }
        }
      }
      else if (upload.status == UPLOAD_FILE_END && decryptBuffer) {
        size_t finalSize = 0;
        uint8_t finalBlock[32];
        
        if (mbedtls_cipher_finish(&cipher_ctx, finalBlock, &finalSize) == 0 && finalSize > 0) {
          uint8_t paddingLength = finalBlock[finalSize - 1];
          if (paddingLength <= 16 && finalSize > paddingLength) {
            if (!Update.write(finalBlock, finalSize - paddingLength)) {
              #if DEBUG
              Serial.printf("Final block write failed! Error: %d\n", Update.getError());
              #endif
            }
          }
        }

        heap_caps_free(decryptBuffer);
        decryptBuffer = nullptr;
        
        bool updateSuccess = Update.end(true);
        
        #if DEBUG
        Serial.printf(updateSuccess ? "Update Success: %u bytes\n" : "Update Failed! Error: %d\n",
                     Update.hasError() ? Update.getError() : upload.totalSize);
        #endif
      }
      else if (upload.status == UPLOAD_FILE_ABORTED) {
        #if DEBUG
        Serial.println("Update aborted");
        #endif
        if (decryptBuffer) {
          heap_caps_free(decryptBuffer);
          decryptBuffer = nullptr;
        }
        Update.abort();
      }
    }
  );

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.enableCORS(true);
  server.begin();
  #if DEBUG
  Serial.println("HTTP Server Started");
  #endif
}

void cleanupCipher() {
  mbedtls_cipher_free(&cipher_ctx);
}

void updateCpuFrequency() {
    ClockMode requiredMode = LOW_CLOCK;
    uint32_t currentmillis = millis();

    if (updateMode) {
        requiredMode = ULTRA_HIGH_CLOCK;
    } 
    else if (deviceConnected || isAdvertising || isScanning || isSetupblestep || isTurboMode) {
        requiredMode = HIGH_CLOCK;
    }
    else if (inputStateChanged || isBlinking || isInBoostMode) {
        requiredMode = MEDIUM_CLOCK;
    }

    if (requiredMode != currentClockMode) {
        bool shouldChangeNow = false;

        if (requiredMode < currentClockMode) {
            shouldChangeNow = true;
        }
        else if (currentmillis - lastClockChangeTime >= CLOCK_REDUCTION_DELAY) {
            shouldChangeNow = true;
        }

        if (shouldChangeNow) {
            switch (requiredMode) {
                case ULTRA_HIGH_CLOCK:
                    setCpuFrequencyMhz(ULTRA_HIGH_CLOCK_FREQ);
                    #if DEBUG
                    Serial.println("Switching to Ultra High Performance mode");
                    #endif
                    break;

                case HIGH_CLOCK:
                    setCpuFrequencyMhz(HIGH_CLOCK_FREQ);
                    #if DEBUG
                    Serial.println("Switching to High Performance mode");
                    #endif
                    break;

                case MEDIUM_CLOCK:
                    setCpuFrequencyMhz(MEDIUM_CLOCK_FREQ);
                    #if DEBUG
                    Serial.println("Switching to Medium Performance mode");
                    #endif
                    break;

                case LOW_CLOCK:
                    // BLEが動作していないかつクランクが止まっている時にクロックを落とします。
                    setCpuFrequencyMhz(LOW_CLOCK_FREQ);
                    #if DEBUG
                    Serial.println("Switching to Power Saving mode");
                    #endif
                    break;
            }
            #if DEBUG
            Serial.print("Current CPU Clock: ");
            Serial.println(getCpuFrequencyMhz());
            #endif
            currentClockMode = requiredMode;
            lastClockChangeTime = currentmillis;
        }
    }
}

void initTempSensor() {
#if !CONFIG_IDF_TARGET_ESP32  // ESP32以外で温度センサーを有効化
  temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
  ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));
  ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
#else
  temp_sensor = NULL;  // ESP32では無効化
#endif
}

void checkTemperature() {
#if !CONFIG_IDF_TARGET_ESP32  // ESP32以外でのみ温度チェック
  if (temp_sensor == NULL) return;
  
  uint32_t currentTime = millis();
  if (currentTime - lastCheckTime >= TEMP_CHECK_INTERVAL) {
    float temperature;
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &temperature));

    if (temperature > TEMPERATURE_THRESHOLD) {
      #if DEBUG
      Serial.println("[WARNING] Temperature Threshold Exceeded!");
      #endif
      enterPermanentSleep();
    }
    lastCheckTime = currentTime;
  }
#endif
}

void enterPermanentSleep() {
  #if DEBUG
  Serial.println("Entering Sleep mode...");
  #endif
  delay(100);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_deep_sleep_start();
}



class UpdateModeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() > 0) {
      updateMode = (value.charAt(0) == '1');
      pCharacteristic->setValue(String(updateMode ? 1 : 0));
      pCharacteristic->notify();

      if (updateMode) {
        static bool firstrun = true;
        if (firstrun){
          updateCpuFrequency(); 
          firstrun = false;
        }
        setupServer();
      }
      else {
        server.stop();
        WiFi.softAPdisconnect(true);
      }
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class PresetCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() > 0) {
      int newPresetIndex = value.toInt();

      if (newPresetIndex >= 0 && newPresetIndex < sizeof(presets) / sizeof(presets[0])) {
        if (currentPresetIndex != newPresetIndex) {
          currentPresetIndex = newPresetIndex;
          updateTimingSettings();

          if (modesInitialized) {
            EEPROM.write(CONTROL_MODE_ADDR, currentPresetIndex);
            EEPROM.commit();
          }
        }
      }
      pCharacteristic->setValue(String(currentPresetIndex));
      pCharacteristic->notify();
    }
  }
};

class ModeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() > 0) {
      if (value.charAt(0) == '1' && !offMode) {
        offMode = true;
        
        if (modesInitialized) {
          EEPROM.write(OPERATION_MODE_ADDR, 1);
          EEPROM.commit();
        }

        isBlinking = false;
        inputChangedDuringTimeout = false;
        
        #if DEBUG
        Serial.println("Off Mode Enabled");
        #endif
      }
      else if (value.charAt(0) == '0' && offMode) {
        if (value.length() >= 6 && value.charAt(1) == ':') {
          String inputPin = value.substring(2);

          if (inputPin == PIN_CODE && !isSystemLocked) {
            offMode = false;
            
            if (modesInitialized) {
              EEPROM.write(OPERATION_MODE_ADDR, 0);
              EEPROM.commit();
            }

            updateOutputs(HIGH);
            #if DEBUG
            Serial.println("Off Mode Disabled - PIN Accepted");
            #endif
          }
          else {
            #if DEBUG
            Serial.println("Off Mode Disable Failed - Invalid PIN");
            #endif
          }
        }
        else {
          #if DEBUG
          Serial.println("Off Mode Disable Failed - Invalid Format");
          #endif
        }
      }
      
      pCharacteristic->setValue(String(offMode ? 1 : 0));
      pCharacteristic->notify();
    }
  }
};

class LockCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    pCharacteristic->setValue(String(isSystemLocked ? 1 : 0));
    pCharacteristic->notify();
  }
};

class SmartKeyScanner: public BLEAdvertisedDeviceCallbacks {
    private:
        const int8_t RSSI_THRESHOLD = -80;
        const BLEUUID UNIVERSAL_KEY_UUID = BLEUUID(KEY_ID);

    public:
        void onResult(BLEAdvertisedDevice advertisedDevice) {
            if (!isSystemLocked) return;

            int rssi = advertisedDevice.getRSSI();
            if (rssi < RSSI_THRESHOLD) return;

            #if DEBUG
            Serial.printf("Device found: %s, RSSI: %d\n", 
                          advertisedDevice.toString().c_str(), rssi);
            #endif
            
            if (advertisedDevice.haveServiceUUID()) {
                BLEUUID uuid = advertisedDevice.getServiceUUID();
                #if DEBUG
                Serial.printf("UUID: %s\n", uuid.toString().c_str());
                #endif
                
                if (uuid.equals(UNIVERSAL_KEY_UUID)) {
                    #if DEBUG
                    Serial.println("Universal key detected!");
                    #endif
                    unlockSystem();
                    return;
                }
            }
        }

    private:
        void unlockSystem() {
            isSystemLocked = false;
            if (pLockCharacteristic) {
                pLockCharacteristic->setValue(String(isSystemLocked ? 1 : 0));
                pLockCharacteristic->notify();
            }
            isScanning = false;
            pBLEScan->stop();
            #if DEBUG
            Serial.println("System Unlocked");
            #endif
        }
};

String getDeviceName() {
  if (!EEPROM.begin(EEPROM_SIZE)) {
    #if DEBUG
    Serial.println("Failed to initialize EEPROM");
    #endif
    return "DD_Tuning Ver" + String(FIRMWARE_VERSION) + "_255";
  }

  uint8_t mfgNum = EEPROM.read(MFG_NUMBER_ADDR + 1);
  char deviceName[32];
  sprintf(deviceName, "DD_Tuning Ver%s_%02d", FIRMWARE_VERSION, mfgNum);
  return String(deviceName);
}

// データパケット構造体を事前に定義
struct __attribute__((packed)) DataPacket {
    uint8_t currentPresetIndex;   // 1byte
    uint8_t inputChangeCount;     // 1byte
    uint8_t isBlinking;           // 1byte (bool -> uint8_t)
    uint8_t isInBoostMode;        // 1byte (bool -> uint8_t)
    uint8_t offMode;              // 1byte (bool -> uint8_t)
    uint8_t OutputState;          // 1byte (bool -> uint8_t)
    int32_t inputStateGreen;      // 4bytes (int -> int32_t)
    int32_t inputStateBlue;       // 4bytes (int -> int32_t)
    float currentRPM;             // 4bytes
    uint8_t temperature;          // 1byte (温度データ追加)
    // 合計: 19bytes
};

void diagDataSend() {

  if (!deviceConnected) {
    return;
  }

  uint32_t currentMillis = millis();

  if ((currentMillis - lastsendMillis >= interval)) {
    lastsendMillis = currentMillis;
    
    // データの構造体
    DataPacket dataPacket;

    // データのパッキング
    dataPacket.currentPresetIndex = currentPresetIndex;
    dataPacket.inputChangeCount = inputChangeCount;
    dataPacket.isBlinking = isBlinking ? 1 : 0;        // bool -> uint8_t
    dataPacket.isInBoostMode = isInBoostMode ? 1 : 0;  // bool -> uint8_t
    dataPacket.offMode = offMode ? 1 : 0;              // bool -> uint8_t
    dataPacket.OutputState = OutputState ? 1 : 0;     // bool -> uint8_t
    dataPacket.inputStateGreen = inputStateGreen;
    dataPacket.inputStateBlue = inputStateBlue;
    dataPacket.currentRPM = currentRPMForDiag; // ケイデンス値をセット
    
    // 温度データ（摂氏+40でエンコード、エラー時は255）
    if (currentTemperatureForDiag >= -40 && currentTemperatureForDiag <= 215) {
        dataPacket.temperature = (uint8_t)(currentTemperatureForDiag + 40);
    }
    else {
        dataPacket.temperature = 255; // エラー値
    }

    // データの送信
    pDiagCharacteristic->setValue((uint8_t*)&dataPacket, sizeof(DataPacket));
    pDiagCharacteristic->notify(); // 通知を送信
    
    #if DEBUG
    static uint32_t debugCounter = 0;
    if (++debugCounter % 20 == 0) { // 1秒ごとにログ出力
        Serial.printf("DiagData sent: preset=%d, count=%d, RPM=%.1f, temp=%.1f, size=%d bytes\n",
                      dataPacket.currentPresetIndex, dataPacket.inputChangeCount, 
                      dataPacket.currentRPM, currentTemperatureForDiag, sizeof(DataPacket));
    }
    #endif
  }
}

void setup() {
  Serial.begin(921600);
  delay(800);
  Serial.print("DD_Tuning Ver");
  Serial.println(FIRMWARE_VERSION);
  Serial.println("Smart Bicycle Tuning System");
  Serial.println("");
  // DD_Hackはこのスケッチの使用に対して保証を行いません。自己責任で利用してください。
  Serial.println("[CAUTION] DD_Hack assumes no liability for product use.");
  Serial.println("");
  Serial.println("Setup now...");
  #if LOCK_MODE
  isSystemLocked = true;
  #endif

  pinMode(INPUT_PIN_GREEN, INPUT_PULLUP);
  pinMode(INPUT_PIN_BLUE, INPUT_PULLUP);
  pinMode(OUTPUT_PIN_GREEN, OUTPUT);
  pinMode(OUTPUT_PIN_BLUE, OUTPUT);
  pinMode(NOTIFY_LED_PIN, OUTPUT);

  digitalWrite(OUTPUT_PIN_GREEN, HIGH);
  digitalWrite(OUTPUT_PIN_BLUE, LOW);
  digitalWrite(NOTIFY_LED_PIN, HIGH);

  initTempSensor();

  if (!EEPROM.begin(EEPROM_SIZE)) {
    #if DEBUG
    Serial.println("[ERROR] Failed to initialize EEPROM");
    #endif
    Serial.println("[INFO] An error has been detected. Restarting...");
    delay(200);
    ESP.restart();
  }


  uint8_t initialized = EEPROM.read(MFG_NUMBER_ADDR);
  if (initialized != MFG_NUMBER_INITIALIZED_FLAG) {
    EEPROM.write(MFG_NUMBER_ADDR, MFG_NUMBER_INITIALIZED_FLAG);
    EEPROM.write(MFG_NUMBER_ADDR + 1, MFG_NUMBER);
    EEPROM.commit();
    #if DEBUG
    Serial.println("Initialized Manufacturing Number");
    #endif
  }

  modesInitialized = EEPROM.read(CONTROL_MODE_ADDR - 1) == MODE_INITIALIZED_FLAG;
  if (!modesInitialized) {
    #if DEBUG
    Serial.println("Initializing Mode Settings");
    #endif
    EEPROM.write(CONTROL_MODE_ADDR - 1, MODE_INITIALIZED_FLAG);
    EEPROM.write(CONTROL_MODE_ADDR, 0);
    EEPROM.write(OPERATION_MODE_ADDR, 0);
    EEPROM.commit();
    modesInitialized = true;
    currentPresetIndex = 0;
    offMode = false;
  }
  else {
    currentPresetIndex = EEPROM.read(CONTROL_MODE_ADDR);
    offMode = EEPROM.read(OPERATION_MODE_ADDR) == 1;
    #if DEBUG
    Serial.printf("Loaded settings - Control Mode: %d, Operation Mode: %d\n",
                  currentPresetIndex, offMode ? 1 : 0, isSystemLocked ? 1 : 0);
    #endif
  }

  initializeTimingSettings();
  initializeControlMode();

  String deviceName = getDeviceName();
  BLEDevice::init(deviceName.c_str());

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pPresetCharacteristic = pService->createCharacteristic(
    PRESET_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pPresetCharacteristic->setCallbacks(new PresetCallbacks());
  pPresetCharacteristic->setValue(String(currentPresetIndex));

  pModeCharacteristic = pService->createCharacteristic(
    MODE_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pModeCharacteristic->setCallbacks(new ModeCallbacks());
  pModeCharacteristic->setValue(String(offMode ? 1 : 0));

  pUpdateModeCharacteristic = pService->createCharacteristic(
    UPDATE_MODE_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pUpdateModeCharacteristic->setCallbacks(new UpdateModeCallbacks());
  pUpdateModeCharacteristic->setValue("0");

  pLockCharacteristic = pService->createCharacteristic(
      LOCK_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
  );

  pDiagCharacteristic = pService->createCharacteristic(
                      DIAG_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new SmartKeyScanner());
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(1000);
  pBLEScan->setWindow(1000);
  pLockCharacteristic->setValue(String(isSystemLocked ? 1 : 0));

  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  BLEAdvertisementData advert;
  advert.setName(deviceName.c_str());
  advert.setCompleteServices(BLEUUID(SERVICE_UUID));
  advert.setAppearance(BLE_APPEARANCE);
  BLEAdvertisementData scanResponse;
  scanResponse.setName(deviceName.c_str());

  pAdvertising->setAdvertisementData(advert);
  pAdvertising->setScanResponseData(scanResponse);

  pAdvertising->setScanResponse(false);
  pAdvertising->setMinInterval(200);
  pAdvertising->setMaxInterval(400);

  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_N9);

  isScanning = true;
  isAdvertising = true;
  pService->start();
  pAdvertising->start();
  pBLEScan->start(0, nullptr, false);
  advertiseStartTime = millis();
  wasConnected = false;

  Serial.println("Setup Complete!");
}

void loop() {

  if (updateMode) {
    server.handleClient();
    checkTemperature();
    return;
  }

  if (offMode) {
    digitalWrite(OUTPUT_PIN_GREEN, digitalRead(INPUT_PIN_GREEN));
    digitalWrite(OUTPUT_PIN_BLUE, digitalRead(INPUT_PIN_BLUE));
  }
  else {
    uint32_t currentMicros = micros();
    processInput(currentMicros);
    handleOutputControl(currentMicros);
  }

  manageBLEConnection();

  manageNotifyLED();

  updateCpuFrequency();

  checkTemperature();

  diagDataSend();

}