/**
 * ============================================================
 *  CATPAW FIRMWARE v1.0  |  ESP32-S3 Dev Module
 *  Platform: PlatformIO + Arduino Framework
 *
 *  State Machine:
 *    0 - BOOT          : ตรวจสอบ Hardware
 *    1 - IDLE          : รอรับเงิน
 *    2 - PAYMENT_CHECK : รอกดปุ่ม Start
 *    3 - READY         : นับถอยหลัง
 *    4 - OPERATION     : รัน Sequential Process 0→7
 *    5 - SUMMARY       : สรุปผล
 *    7 - LOCKED        : Hardware Error
 *
 *  Hardware:
 *    0x18 LOAD  (OUTPUT) LD0-LD7  — ใช้ lookup table LD_PIN[]
 *    0x1C LED   (OUTPUT) LED0-LED7 — ใช้ lookup table LED_PIN[]
 *    0x1E BTN   (INPUT)  BTN0 = Start (ปุ่มเดียว)
 *    0x1F WL    (INPUT)  Water Level / Switch (อ่านเฉยๆ)
 *
 *  OPERATION Logic:
 *    กด Start → รัน Process 0 (LED0+LD0 ON ตาม proc_durationSec[0])
 *             → Process 1 (LED1+LD1 ON ตาม proc_durationSec[1])
 *             → ... → Process 7 → SUMMARY
 *    เงินหักต่อวินาทีตลอดช่วง OPERATION
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_mac.h>
#include "TM1637Display.h"
#include "ExpanderManager.h"
#include "MoneyReader.h"
#include "LightSensor.h"

// ============================================================
//  [SECTION A] USER PREFERENCES
//  บันทึกใน NVS ไม่หายหลัง Reset
//  เปลี่ยน runtime: Serial SET / MQTT set-params
// ============================================================

// ── เงิน ──
#define DEFAULT_MIN_START_MONEY   30      // [บาท] เงินขั้นต่ำ

// ── Deduct ──
#define DEFAULT_DEDUCT_RATE       1       // [บาท/วิ] หักต่อวินาที
#define DEFAULT_READY_COUNTDOWN   5       // [วิ] นับถอยหลัง
#define DEFAULT_BOOT_DELAY_MS     3000    // [ms]

// ── Process Duration (วินาที) ──
// แต่ละ Process รัน LED[n]+LD[n] นานกี่วิ ก่อนเปลี่ยนไป Process ถัดไป
// ตั้งเป็น 0 = ข้าม Process นั้น
#define DEFAULT_PROC_0_SEC   10   // Process 0
#define DEFAULT_PROC_1_SEC   15   // Process 1
#define DEFAULT_PROC_2_SEC   20   // Process 2
#define DEFAULT_PROC_3_SEC   10   // Process 3
#define DEFAULT_PROC_4_SEC   15   // Process 4
#define DEFAULT_PROC_5_SEC   20   // Process 5
#define DEFAULT_PROC_6_SEC   10   // Process 6
#define DEFAULT_PROC_7_SEC   5    // Process 7

// ── Pulse Money Acceptors ──
#define DEFAULT_BANK_PULSE_VALUE  10.0f
#define DEFAULT_COIN_PULSE_VALUE  1.0f
#define BANK_MIN_PULSE_MS    20
#define BANK_MAX_PULSE_MS   200
#define BANK_PULSE_TIMEOUT_MS 600
#define COIN_MIN_PULSE_MS    15
#define COIN_MAX_PULSE_MS   150
#define COIN_PULSE_TIMEOUT_MS 400

// ── Telemetry ──
#define DEFAULT_TELEMETRY_SEC  30

// ============================================================
//  [SECTION B] PIN CONFIGURATION
// ============================================================
#define PIN_BANK_PULSE   4
#define PIN_COIN_PULSE   12
#define I2C_SDA          8
#define I2C_SCL          9
#define DISP_CLK         15
#define DISP_DIO         16
#define PIN_LIGHT_SENSOR     1
#define LIGHT_THRESHOLD_ADC  2000
#define LIGHT_DEBOUNCE_MS    200

// ============================================================
//  [SECTION C] EXPANDER MAPPING
//
//  0x18 LOAD  (OUTPUT) — LD0-LD7   — lookup LD_PIN[]
//  0x1C LED   (OUTPUT) — LED0-LED7 — lookup LED_PIN[]
//  0x1E BTN   (INPUT)  — BTN0 = Start button เท่านั้น
//  0x1F WL    (INPUT)  — Water Level + Switch (monitor เฉยๆ)
//
//  Physical wiring (logical→physical IO):
//    LD:   LD0=IO0 LD1=IO1 LD2=IO3 LD3=IO2 LD4=IO5 LD5=IO4 LD6=IO7 LD7=IO6
//    LED:  LED0=IO7 LED1=IO6 LED2=IO5 LED3=IO4 LED4=IO3 LED5=IO2 LED6=IO1 LED7=IO0
//    BTN:  BTN0=IO7 (Start) — ใช้แค่ BTN0
//    WL:   WL0=IO5 WL1=IO4 WL2=IO3 WL3=IO2 WL4=IO0 WL5=IO1 SW0=IO6 SW1=IO7
// ============================================================
#define NUM_PROCESSES   8
#define NUM_WL_LEVELS   6
#define NUM_SWITCHES    2

static const uint8_t LD_PIN[8]  = {0, 1, 3, 2, 5, 4, 7, 6};
static const uint8_t LED_PIN[8] = {7, 6, 5, 4, 3, 2, 1, 0};
static const uint8_t WL_PIN[6]  = {5, 4, 3, 2, 0, 1};
#define SW0_PIN  6
#define SW1_PIN  7

// BTN0 = IO7 บน EXP3 (0x1E) — Active LOW
#define BTN0_PHYSICAL_IO  7

// ============================================================
//  [SECTION D] NETWORK CONFIG
// ============================================================
#define WIFI_SSID        "SR803_5G"
#define WIFI_PASS        "80323SM5F"
#define MQTT_SERVER      "147.50.255.120"
#define MQTT_PORT        9359
#define MQTT_USER        "esp32"
#define MQTT_PASS        "Taweesak@5050"
#define MQTT_PREFIX      "machine/catpaw"
#define BLE_SERVER_NAME  "ESP32S3-QR"
#define BLE_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ============================================================
//  [SECTION E] CONSTANTS
// ============================================================
#define MAX_LOG_ENTRIES   64
#define BTN_POLL_MS       50
#define BTN_DEBOUNCE_MS   120

// ============================================================
//  STATE MACHINE
// ============================================================
enum SystemState : uint8_t {
    STATE_BOOT          = 0,
    STATE_IDLE          = 1,
    STATE_PAYMENT_CHECK = 2,
    STATE_READY         = 3,
    STATE_OPERATION     = 4,
    STATE_SUMMARY       = 5,
    STATE_LOCKED        = 7
};
const char* stateNames[] = {
    "BOOT","IDLE","PAYMENT_CHECK","READY","OPERATION","SUMMARY","??","LOCKED"
};

// ============================================================
//  LOG STRUCTURE
// ============================================================
struct LogEntry {
    char     event[24];
    uint32_t timestamp_ms;
    int16_t  money;
    uint32_t duration_ms;
};

// ============================================================
//  GLOBAL OBJECTS
// ============================================================
TM1637Display   display(DISP_CLK, DISP_DIO);
ExpanderManager expander1(0x18);   // LOAD
ExpanderManager expander2(0x1C);   // LED
ExpanderManager expander3(0x1E);   // BTN
ExpanderManager expander4(0x1F);   // WL+SW
MoneyReader     bankReader(PIN_BANK_PULSE, DEFAULT_BANK_PULSE_VALUE,
                           BANK_MIN_PULSE_MS, BANK_MAX_PULSE_MS, BANK_PULSE_TIMEOUT_MS);
MoneyReader     coinReader(PIN_COIN_PULSE, DEFAULT_COIN_PULSE_VALUE,
                           COIN_MIN_PULSE_MS, COIN_MAX_PULSE_MS, COIN_PULSE_TIMEOUT_MS);
LightSensor     lightSensor(PIN_LIGHT_SENSOR, LIGHT_THRESHOLD_ADC, LIGHT_DEBOUNCE_MS);
Preferences     prefs;
WiFiClient      espClient;
PubSubClient    mqttClient(espClient);
BLEServer*         bleServer          = nullptr;
BLECharacteristic* bleChar            = nullptr;
bool               bleClientConnected = false;

// ============================================================
//  RUNTIME STATE
// ============================================================
SystemState   currentState    = STATE_BOOT;
bool          stateChanged    = true;
unsigned long stateEnterTime  = 0;
float         totalMoney      = 0.0f;

// ── Operation state ──
int           currentProcess     = 0;       // Process ที่กำลังรันอยู่ (0-7)
unsigned long processStartTime   = 0;       // เวลาเริ่ม Process ปัจจุบัน
unsigned long lastDeductTime     = 0;       // หักเงินล่าสุด
bool          operationRunning   = false;   // กำลัง run process อยู่

// ── Process log ──
uint32_t      procActualMs[NUM_PROCESSES] = {};  // เวลาจริงที่รันแต่ละ Process

// ── Countdown ──
int           countdownRemaining  = 0;
unsigned long lastCountdownTick   = 0;

// ── Money check ──
unsigned long lastMoneyCheck    = 0;
unsigned long lastDisplayRefresh = 0;

// ── Water Level ──
bool     wlState[NUM_WL_LEVELS] = {};
bool     swState[NUM_SWITCHES]  = {};
int      waterLevel             = 0;
int      prevWaterLevel         = -1;

// ── Log ──
LogEntry sessionLog[MAX_LOG_ENTRIES];
int      logCount = 0;

// ── NVS ──
int pref_minStartMoney;
int pref_deductRate;
int pref_readyCountdown;
int pref_bootDelayMs;
int pref_telemetrySec;
int pref_procSec[NUM_PROCESSES];

String DEVICE_ID;

// ── Serial debug ──
bool  dbg_addMoney   = false;
float dbg_addAmount  = 0.0f;
bool  dbg_pressStart = false;
bool  dbg_verbose    = false;

// ============================================================
//  ISR
// ============================================================
void IRAM_ATTR bankISR() { bankReader.handleISR(); }
void IRAM_ATTR coinISR() { coinReader.handleISR(); }

// ============================================================
//  BLE
// ============================================================
class BLEServerCB : public BLEServerCallbacks {
    void onConnect(BLEServer*)    override {
        bleClientConnected = true;
        Serial.println(F("[BLE] Client connected"));
    }
    void onDisconnect(BLEServer*) override {
        bleClientConnected = false;
        Serial.println(F("[BLE] Disconnected — re-advertising"));
        BLEDevice::startAdvertising();
    }
};

void initBLE() {
    BLEDevice::init(BLE_SERVER_NAME);
    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new BLEServerCB());
    BLEService* svc = bleServer->createService(BLEUUID(BLE_SERVICE_UUID));
    bleChar = svc->createCharacteristic(
        BLEUUID(BLE_CHAR_UUID),
        BLECharacteristic::PROPERTY_READ  |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    bleChar->addDescriptor(new BLE2902());
    bleChar->setValue("Ready");
    svc->start();
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLEUUID(BLE_SERVICE_UUID));
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println(F("[BLE] Server started: " BLE_SERVER_NAME));
}

void sendBLE(const String& text) {
    if (!bleClientConnected) return;
    bleChar->setValue(text.c_str());
    bleChar->notify();
    Serial.printf("[BLE] Sent: %s\n", text.c_str());
}

// ============================================================
//  DEVICE ID
// ============================================================
void createDeviceID() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char id[13];
    snprintf(id, sizeof(id), "%02X%02X%02X%02X%02X%02X",
             mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    DEVICE_ID = String("catpaw-") + id;
    Serial.println("[NET] Device ID: " + DEVICE_ID);
}

// ============================================================
//  WIFI
// ============================================================
void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.println(F("[WiFi] Connecting..."));
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(true);   // required สำหรับ BLE+WiFi coexistence บน ESP32-S3
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 50 && WiFi.status() != WL_CONNECTED; i++) {
        delay(300); Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED)
        Serial.println("\n[WiFi] OK: " + WiFi.localIP().toString());
    else
        Serial.println(F("\n[WiFi] Timeout — will retry"));
}

// ============================================================
//  LOGGING
// ============================================================
void addLog(const char* event, uint32_t duration_ms = 0) {
    uint32_t ts = (uint32_t)millis();
    if (logCount < MAX_LOG_ENTRIES) {
        strncpy(sessionLog[logCount].event, event, sizeof(sessionLog[0].event)-1);
        sessionLog[logCount].event[sizeof(sessionLog[0].event)-1] = 0;
        sessionLog[logCount].timestamp_ms = ts;
        sessionLog[logCount].money        = (int16_t)totalMoney;
        sessionLog[logCount].duration_ms  = duration_ms;
        logCount++;
    }
    Serial.printf("[LOG][%8lu ms] %-22s | ฿%4d | %lu ms\n",
        (unsigned long)ts, event, (int)totalMoney, (unsigned long)duration_ms);
}

void printSummary() {
    Serial.println(F("╔══════════════════════════════════════════════════════╗"));
    Serial.println(F("║              CATPAW SESSION SUMMARY                  ║"));
    Serial.println(F("╠══════════════════════════════════════════════════════╣"));
    Serial.printf( "║  Entries : %-42d║\n", logCount);
    Serial.println(F("╠══════════════════════════════════════════════════════╣"));
    for (int i = 0; i < logCount; i++) {
        Serial.printf("║ [%02d] [%8lu ms] %-18s ฿%-4d %lu ms\n",
            i+1,
            (unsigned long)sessionLog[i].timestamp_ms,
            sessionLog[i].event,
            (int)sessionLog[i].money,
            (unsigned long)sessionLog[i].duration_ms);
    }
    Serial.println(F("╠══════════════════════════════════════════════════════╣"));
    Serial.println(F("║  Process summary:                                    ║"));
    for (int p = 0; p < NUM_PROCESSES; p++) {
        Serial.printf("║  PROC%d: config=%ds | actual=%lums\n",
            p, pref_procSec[p], (unsigned long)procActualMs[p]);
    }
    Serial.println(F("╚══════════════════════════════════════════════════════╝"));
}

void clearLog() {
    logCount = 0;
    memset(sessionLog, 0, sizeof(sessionLog));
    memset(procActualMs, 0, sizeof(procActualMs));
}

// ============================================================
//  NVS PREFERENCES
// ============================================================
void loadPreferences() {
    const int defProc[NUM_PROCESSES] = {
        DEFAULT_PROC_0_SEC, DEFAULT_PROC_1_SEC,
        DEFAULT_PROC_2_SEC, DEFAULT_PROC_3_SEC,
        DEFAULT_PROC_4_SEC, DEFAULT_PROC_5_SEC,
        DEFAULT_PROC_6_SEC, DEFAULT_PROC_7_SEC
    };
    prefs.begin("catpaw", false); prefs.end();  // ensure namespace exists
    prefs.begin("catpaw", true);
    pref_minStartMoney  = prefs.getInt("minMoney",     DEFAULT_MIN_START_MONEY);
    pref_deductRate     = prefs.getInt("deductRate",   DEFAULT_DEDUCT_RATE);
    pref_readyCountdown = prefs.getInt("countdown",    DEFAULT_READY_COUNTDOWN);
    pref_bootDelayMs    = prefs.getInt("bootDelay",    DEFAULT_BOOT_DELAY_MS);
    pref_telemetrySec   = prefs.getInt("telemetrySec", DEFAULT_TELEMETRY_SEC);
    for (int p = 0; p < NUM_PROCESSES; p++) {
        char key[8]; snprintf(key, sizeof(key), "proc%d", p);
        pref_procSec[p] = prefs.getInt(key, defProc[p]);
    }
    prefs.end();

    Serial.println(F("[PREFS] Loaded:"));
    Serial.printf("  minStartMoney  = %d bath\n",  pref_minStartMoney);
    Serial.printf("  deductRate     = %d bath/s\n", pref_deductRate);
    Serial.printf("  readyCountdown = %d s\n",      pref_readyCountdown);
    Serial.printf("  telemetrySec   = %d s\n",      pref_telemetrySec);
    Serial.print( "  processDurations = [");
    for (int p = 0; p < NUM_PROCESSES; p++)
        Serial.printf("%d%s", pref_procSec[p], p<7?",":"]");
    Serial.println(F(" sec"));
}

void savePreferences() {
    prefs.begin("catpaw", false);
    prefs.putInt("minMoney",     pref_minStartMoney);
    prefs.putInt("deductRate",   pref_deductRate);
    prefs.putInt("countdown",    pref_readyCountdown);
    prefs.putInt("bootDelay",    pref_bootDelayMs);
    prefs.putInt("telemetrySec", pref_telemetrySec);
    for (int p = 0; p < NUM_PROCESSES; p++) {
        char key[8]; snprintf(key, sizeof(key), "proc%d", p);
        prefs.putInt(key, pref_procSec[p]);
    }
    prefs.end();
    Serial.println(F("[PREFS] Saved"));
}

void resetPreferences() {
    prefs.begin("catpaw", false); prefs.clear(); prefs.end();
    pref_minStartMoney  = DEFAULT_MIN_START_MONEY;
    pref_deductRate     = DEFAULT_DEDUCT_RATE;
    pref_readyCountdown = DEFAULT_READY_COUNTDOWN;
    pref_bootDelayMs    = DEFAULT_BOOT_DELAY_MS;
    pref_telemetrySec   = DEFAULT_TELEMETRY_SEC;
    const int def[NUM_PROCESSES] = {
        DEFAULT_PROC_0_SEC, DEFAULT_PROC_1_SEC,
        DEFAULT_PROC_2_SEC, DEFAULT_PROC_3_SEC,
        DEFAULT_PROC_4_SEC, DEFAULT_PROC_5_SEC,
        DEFAULT_PROC_6_SEC, DEFAULT_PROC_7_SEC
    };
    for (int p = 0; p < NUM_PROCESSES; p++) pref_procSec[p] = def[p];
    Serial.println(F("[PREFS] Reset to defaults"));
}

// ============================================================
//  STATE TRANSITION
// ============================================================
void transitionTo(SystemState next) {
    Serial.printf("[STATE] %s -> %s\n", stateNames[currentState], stateNames[next]);
    addLog(stateNames[next]);
    currentState   = next;
    stateChanged   = true;
    stateEnterTime = millis();
}

// ============================================================
//  HARDWARE HELPERS
// ============================================================
void setProcess(int proc, bool on) {
    if (proc < 0 || proc >= NUM_PROCESSES) return;
    expander1.digitalWrite(LD_PIN[proc],  on ? HIGH : LOW);
    expander2.digitalWrite(LED_PIN[proc], on ? HIGH : LOW);
    Serial.printf("[PROC] PROC%d (LD%d IO%d + LED%d IO%d) %s\n",
        proc, proc, LD_PIN[proc], proc, LED_PIN[proc], on ? "ON" : "OFF");
}

void allOff() {
    for (uint8_t p = 0; p < 8; p++) {
        expander1.digitalWrite(LD_PIN[p],  LOW);
        expander2.digitalWrite(LED_PIN[p], LOW);
    }
    Serial.println(F("[HW] All LOAD+LED OFF"));
}

// ============================================================
//  WATER LEVEL READER
// ============================================================
void updateWaterLevel() {
    Wire.beginTransmission(0x1F);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x1F, (uint8_t)1);
    uint8_t raw = Wire.available() ? Wire.read() : 0xFF;
    for (int i = 0; i < NUM_WL_LEVELS; i++)
        wlState[i] = !(raw & (1 << WL_PIN[i]));
    swState[0] = !(raw & (1 << SW0_PIN));
    swState[1] = !(raw & (1 << SW1_PIN));
    int level = 0;
    for (int i = 0; i < NUM_WL_LEVELS; i++) if (wlState[i]) level = i + 1;
    waterLevel = level;
    if (waterLevel != prevWaterLevel) {
        prevWaterLevel = waterLevel;
        Serial.printf("[WL] Level=%d/6 | WL:", waterLevel);
        for (int i = 5; i >= 0; i--) Serial.print(wlState[i] ? "1" : "0");
        Serial.printf(" | SW0=%d SW1=%d\n", swState[0], swState[1]);
    }
}

// ============================================================
//  MQTT
// ============================================================
String makeTopic(const String& sfx) {
    return String(MQTT_PREFIX) + "/" + DEVICE_ID + "/" + sfx;
}

void mqttPublishStatus(const String& s) {
    if (!mqttClient.connected()) return;
    JsonDocument doc;
    doc["deviceId"] = DEVICE_ID; doc["status"] = s;
    String out; serializeJson(doc, out);
    mqttClient.publish(makeTopic("status").c_str(), out.c_str(), false);
}

void mqttPublishConfig(const char* action) {
    if (!mqttClient.connected()) return;
    JsonDocument doc;
    doc["deviceId"]   = DEVICE_ID;
    doc["action"]     = action;
    doc["minMoney"]   = pref_minStartMoney;
    doc["deductRate"] = pref_deductRate;
    JsonArray procs = doc["processDurations"].to<JsonArray>();
    for (int p = 0; p < NUM_PROCESSES; p++) procs.add(pref_procSec[p]);
    String out; serializeJson(doc, out);
    mqttClient.publish(makeTopic("config").c_str(), out.c_str(), false);
    Serial.println("[MQTT] Config: " + out);
}

void mqttPublishAck(const String& key, int val) {
    if (!mqttClient.connected()) return;
    JsonDocument doc;
    doc["deviceId"] = DEVICE_ID; doc[key] = val;
    String out; serializeJson(doc, out);
    mqttClient.publish(makeTopic("ack").c_str(), out.c_str(), false);
}

// ── Snapshot & Rollback ──
void snapshotPreferences() {
    prefs.begin("catpaw_bk", false);
    prefs.putInt("minMoney",   pref_minStartMoney);
    prefs.putInt("deductRate", pref_deductRate);
    for (int p = 0; p < NUM_PROCESSES; p++) {
        char key[8]; snprintf(key, sizeof(key), "proc%d", p);
        prefs.putInt(key, pref_procSec[p]);
    }
    prefs.putULong("snapTime", (unsigned long)(millis()/1000));
    prefs.end();
    Serial.println(F("[SNAP] Snapshot saved"));
}

bool rollbackPreferences() {
    prefs.begin("catpaw_bk", false); prefs.end();
    prefs.begin("catpaw_bk", true);
    bool has = prefs.isKey("minMoney");
    if (has) {
        pref_minStartMoney = prefs.getInt("minMoney",   pref_minStartMoney);
        pref_deductRate    = prefs.getInt("deductRate", pref_deductRate);
        for (int p = 0; p < NUM_PROCESSES; p++) {
            char key[8]; snprintf(key, sizeof(key), "proc%d", p);
            pref_procSec[p] = prefs.getInt(key, pref_procSec[p]);
        }
        prefs.end();
        savePreferences();
        Serial.println(F("[SNAP] Rollback OK"));
    } else {
        prefs.end();
        Serial.println(F("[SNAP] No snapshot"));
    }
    return has;
}

// ── MQTT Callback ──
// Commands:
//   {"cmd":"set-params","minMoney":30,"deductRate":1,"durations":[10,15,20,10,15,20,10,5]}
//   {"cmd":"set-min-money","value":30}
//   {"cmd":"set-deduct-rate","value":1}
//   {"cmd":"set-proc-duration","proc":0,"value":15}
//   {"cmd":"rollback"}
//   {"cmd":"get-config"}
//   {"cmd":"ble-send","text":"https://..."}
//   {"cmd":"reboot"}
void mqttCallback(char* topic, byte* payload, unsigned int len) {
    String msg;
    for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
    Serial.println("[MQTT] << " + msg);

    JsonDocument doc;
    if (deserializeJson(doc, msg)) { Serial.println(F("[MQTT] JSON error")); return; }
    String cmd = doc["cmd"].as<String>();

    if (cmd == "reboot") {
        mqttPublishStatus("rebooting");
        delay(500); ESP.restart();
    }
    // ── Group update ──
    else if (cmd == "set-params") {
        snapshotPreferences();
        mqttPublishConfig("snapshot-before-update");
        bool changed = false;
        String errors = "";

        if (!doc["minMoney"].isNull()) {
            int v = doc["minMoney"].as<int>();
            if (v >= 1 && v <= 9999) { pref_minStartMoney = v; changed = true; }
            else errors += "minMoney out of range; ";
        }
        if (!doc["deductRate"].isNull()) {
            int v = doc["deductRate"].as<int>();
            if (v >= 1 && v <= 1000) { pref_deductRate = v; changed = true; }
            else errors += "deductRate out of range; ";
        }
        if (doc["durations"].is<JsonArray>()) {
            JsonArray arr = doc["durations"].as<JsonArray>();
            int idx = 0;
            for (JsonVariant v : arr) {
                if (idx >= NUM_PROCESSES) break;
                int sec = v.as<int>();
                if (sec >= 0 && sec <= 3600) { pref_procSec[idx] = sec; changed = true; }
                else { char e[32]; snprintf(e,sizeof(e),"proc%d out of range; ",idx); errors += e; }
                idx++;
            }
        }
        if (changed) { savePreferences(); mqttPublishConfig("applied"); }
        if (errors.length() > 0) {
            JsonDocument err; err["deviceId"] = DEVICE_ID; err["errors"] = errors;
            String out; serializeJson(err, out);
            mqttClient.publish(makeTopic("error").c_str(), out.c_str(), false);
        }
    }
    // ── Single params ──
    else if (cmd == "set-min-money") {
        int v = doc["value"].as<int>();
        if (v >= 1 && v <= 9999) {
            snapshotPreferences();
            pref_minStartMoney = v; savePreferences();
            mqttPublishAck("minStartMoney", v);
        }
    }
    else if (cmd == "set-deduct-rate") {
        int v = doc["value"].as<int>();
        if (v >= 1 && v <= 1000) {
            snapshotPreferences();
            pref_deductRate = v; savePreferences();
            mqttPublishAck("deductRate", v);
        }
    }
    else if (cmd == "set-proc-duration") {
        int proc = doc["proc"].as<int>();
        int v    = doc["value"].as<int>();
        if (proc >= 0 && proc < NUM_PROCESSES && v >= 0 && v <= 3600) {
            snapshotPreferences();
            pref_procSec[proc] = v; savePreferences();
            Serial.printf("[MQTT] proc%d duration = %d s\n", proc, v);
            JsonDocument ack;
            ack["deviceId"] = DEVICE_ID;
            ack["proc"]     = proc;
            ack["duration"] = v;
            String out; serializeJson(ack, out);
            mqttClient.publish(makeTopic("ack").c_str(), out.c_str(), false);
        }
    }
    else if (cmd == "rollback") {
        if (rollbackPreferences()) mqttPublishConfig("rollback-applied");
        else {
            JsonDocument err; err["deviceId"] = DEVICE_ID; err["error"] = "no snapshot";
            String out; serializeJson(err, out);
            mqttClient.publish(makeTopic("error").c_str(), out.c_str(), false);
        }
    }
    else if (cmd == "get-config") { mqttPublishConfig("current-config"); }
    else if (cmd == "ble-send") {
        String text = doc["text"].as<String>();
        if (text.length() > 0) sendBLE(text);
    }
    else { Serial.println("[MQTT] Unknown: " + cmd); }
}

void connectMQTT() {
    if (mqttClient.connected()) return;
    static unsigned long last = 0;
    if (millis() - last < 5000) return;
    last = millis();
    Serial.print(F("[MQTT] Connecting..."));
    String wt = makeTopic("status");
    String wp = "{\"deviceId\":\"" + DEVICE_ID + "\",\"status\":\"offline\"}";
    if (mqttClient.connect(DEVICE_ID.c_str(), MQTT_USER, MQTT_PASS,
                            wt.c_str(), 1, false, wp.c_str())) {
        Serial.println(F(" OK"));
        mqttPublishStatus("online");
        mqttClient.subscribe(makeTopic("cmd").c_str());
        Serial.println("[MQTT] Sub: " + makeTopic("cmd"));
    } else {
        Serial.printf(" FAIL rc=%d\n", mqttClient.state());
    }
}

void handleTelemetry() {
    if (!mqttClient.connected()) return;
    static unsigned long lastSend = 0;
    if (millis() - lastSend < (unsigned long)pref_telemetrySec * 1000UL) return;
    lastSend = millis();
    JsonDocument doc;
    doc["deviceId"]    = DEVICE_ID;
    doc["state"]       = stateNames[currentState];
    doc["money"]       = (int)totalMoney;
    doc["uptime_s"]    = millis() / 1000;
    doc["ble"]         = bleClientConnected;
    doc["minMoney"]    = pref_minStartMoney;
    doc["deductRate"]  = pref_deductRate;
    doc["curProcess"]  = currentProcess;
    doc["waterLevel"]  = waterLevel;
    doc["sw0"]         = swState[0];
    doc["sw1"]         = swState[1];
    JsonArray wls = doc["wl"].to<JsonArray>();
    for (int i = 0; i < NUM_WL_LEVELS; i++) wls.add(wlState[i]);
    JsonArray procs = doc["processes"].to<JsonArray>();
    for (int p = 0; p < NUM_PROCESSES; p++) {
        JsonObject po = procs.add<JsonObject>();
        po["proc"]     = p;
        po["duration"] = pref_procSec[p];
        po["active"]   = (operationRunning && currentProcess == p);
    }
    String out; serializeJson(doc, out);
    mqttClient.publish(makeTopic("telemetry").c_str(), out.c_str(), false);
    Serial.println(F("[MQTT] Telemetry sent"));
}

// ============================================================
//  START BUTTON READER — BTN0 (IO7 บน EXP3 0x1E)
// ============================================================
struct StartButton {
    bool prevRaw       = false;
    bool isPressed     = false;
    bool pressEvent    = false;
    bool _consumed     = false;
    unsigned long lastChange  = 0;
    unsigned long debounceMs  = BTN_DEBOUNCE_MS;

    void update() {
        if (_consumed) { _consumed = false; return; }
        pressEvent = false;
        // อ่าน IO7 ของ EXP3 โดยตรง (BTN0 = IO7, Active-LOW)
        Wire.beginTransmission(0x1E);
        Wire.write(0x00);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)0x1E, (uint8_t)1);
        uint8_t raw = Wire.available() ? Wire.read() : 0xFF;
        bool pressed = !(raw & (1 << BTN0_PHYSICAL_IO));

        unsigned long now = millis();
        if (pressed != prevRaw) { lastChange = now; prevRaw = pressed; }
        if (now - lastChange < debounceMs) return;

        if (prevRaw && !isPressed) {
            isPressed = true; pressEvent = true;
            Serial.printf("[BTN0][%8lu ms] START PRESS\n", now);
        } else if (!prevRaw && isPressed) {
            isPressed = false;
            Serial.printf("[BTN0][%8lu ms] START RELEASE\n", now);
        }
    }

    void injectPress() {
        pressEvent = true; _consumed = true;
        Serial.printf("[BTN0][%8lu ms] START PRESS (sim)\n", (unsigned long)millis());
    }
} btnStart;

// ============================================================
//  UPDATE BUTTONS
// ============================================================
static unsigned long lastBtnPoll = 0;

void updateButtons() {
    if (dbg_pressStart) { btnStart.injectPress(); dbg_pressStart = false; return; }
    btnStart.pressEvent = false;
    unsigned long now = millis();
    if (now - lastBtnPoll < BTN_POLL_MS) return;
    lastBtnPoll = now;
    btnStart.update();
}

// ============================================================
//  MONEY INPUT
// ============================================================
void checkMoneyInput() {
    if (millis() - lastMoneyCheck < 50) return;
    lastMoneyCheck = millis();
    float bk = bankReader.checkAmount();
    float co = coinReader.checkAmount();
    if (dbg_addMoney) { bk += dbg_addAmount; dbg_addMoney=false; dbg_addAmount=0; }
    if (bk > 0) {
        totalMoney += bk;
        display.showNumberDec((int)totalMoney, false, 4, 0);
        addLog("INSERT_BANK", (uint32_t)bk);
    }
    if (co > 0) {
        totalMoney += co;
        display.showNumberDec((int)totalMoney, false, 4, 0);
        addLog("INSERT_COIN", (uint32_t)co);
    }
}

// ============================================================
//  SERIAL COMMAND PARSER
// ============================================================
void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    Serial.printf("[CMD] > %s\n", cmd.c_str());
    String up = cmd; up.toUpperCase();

    if (up.startsWith("ADD ")) {
        float v = cmd.substring(4).toFloat();
        if (v > 0) { dbg_addMoney=true; dbg_addAmount=v; Serial.printf("[CMD] +%.0f bath\n",v); }
    }
    else if (up == "START") { dbg_pressStart = true; }
    else if (up == "STATUS") {
        Serial.println(F("──────────────────────────────────────────"));
        Serial.printf("  State      : %s\n", stateNames[currentState]);
        Serial.printf("  Money      : %.0f bath\n", totalMoney);
        Serial.printf("  Process    : %d/%d %s\n",
            currentProcess, NUM_PROCESSES-1, operationRunning?"(running)":"(idle)");
        Serial.printf("  WiFi       : %s\n", WiFi.status()==WL_CONNECTED?WiFi.localIP().toString().c_str():"offline");
        Serial.printf("  MQTT       : %s\n", mqttClient.connected()?"OK":"offline");
        Serial.printf("  BLE        : %s\n", bleClientConnected?"connected":"no client");
        Serial.println(F("  ── Preferences ──────────────────────────────"));
        Serial.printf("  minStartMoney  = %d bath\n",   pref_minStartMoney);
        Serial.printf("  deductRate     = %d bath/s\n",  pref_deductRate);
        Serial.printf("  readyCountdown = %d s\n",       pref_readyCountdown);
        Serial.print( "  processDurations = [");
        for (int p = 0; p < NUM_PROCESSES; p++)
            Serial.printf("%d%s", pref_procSec[p], p<7?",":"]");
        Serial.println(F(" sec"));
        Serial.println(F("  ── Water Level ───────────────────────────────"));
        Serial.printf("  Level=%d/6 | SW0=%d SW1=%d\n", waterLevel, swState[0], swState[1]);
        Serial.println(F("──────────────────────────────────────────"));
    }
    else if (up == "LOG")  { printSummary(); }
    else if (up.startsWith("SET ")) {
        int s = cmd.indexOf(' ', 4); if (s<0) return;
        String key = cmd.substring(4,s); key.toLowerCase();
        int val = cmd.substring(s+1).toInt();
        if      (key == "minmoney")     pref_minStartMoney  = val;
        else if (key == "deductrate")   pref_deductRate     = val;
        else if (key == "countdown")    pref_readyCountdown = val;
        else if (key == "telemetrysec") pref_telemetrySec   = val;
        else if (key.startsWith("proc") && key.length()==5) {
            int p = key.charAt(4) - '0';
            if (p>=0 && p<NUM_PROCESSES) pref_procSec[p] = val;
            else { Serial.println(F("[CMD] Invalid proc index")); return; }
        }
        else { Serial.println(F("[CMD] Unknown key")); return; }
        savePreferences();
        Serial.printf("[SET] %s = %d\n", key.c_str(), val);
    }
    else if (up.startsWith("BLE ")) { sendBLE(cmd.substring(4)); }
    else if (up == "SCAN") {
        Serial.println(F("[SCAN] I2C..."));
        for (uint8_t a=1;a<127;a++) {
            Wire.beginTransmission(a);
            if (Wire.endTransmission()==0) Serial.printf("[SCAN] 0x%02X\n",a);
        }
    }
    else if (up == "PINREAD") {
        uint8_t addrs[4]={0x18,0x1C,0x1E,0x1F};
        const char* roles[4]={"LOAD","LED","BTN","WL+SW"};
        for(int e=0;e<4;e++){
            Wire.beginTransmission(addrs[e]);Wire.write(0x00);
            Wire.endTransmission(false);Wire.requestFrom(addrs[e],(uint8_t)1);
            uint8_t r=Wire.available()?Wire.read():0xFF;
            Serial.printf("[PIN] 0x%02X %-7s = 0b",addrs[e],roles[e]);
            for(int b=7;b>=0;b--)Serial.print((r>>b)&1);
            Serial.printf(" (0x%02X)\n",r);
        }
        Serial.printf("[PIN] BTN0=%s | WaterLevel=%d\n",
            btnStart.isPressed?"PRESSED":"released", waterLevel);
    }
    else if (up == "VERBOSE ON")  { dbg_verbose=true;  Serial.println(F("[VRB] ON")); }
    else if (up == "VERBOSE OFF") { dbg_verbose=false; Serial.println(F("[VRB] OFF")); }
    else if (up == "RESET PREFS") { resetPreferences(); }
    else if (up.startsWith("GOTO ")) {
        int st=cmd.substring(5).toInt();
        if (st>=0&&st<=7) transitionTo((SystemState)st);
    }
    else if (up == "HELP") {
        Serial.println(F("══════════════ CATPAW Serial Commands ══════════════"));
        Serial.println(F("  ADD <n>             เพิ่มเงิน n บาท"));
        Serial.println(F("  START               จำลองกดปุ่ม Start (BTN0)"));
        Serial.println(F("  STATUS              ดูสถานะทั้งหมด"));
        Serial.println(F("  LOG                 ดู session log"));
        Serial.println(F("  BLE <text>          ส่งข้อความไป CYD"));
        Serial.println(F("  SET minMoney <n>    เงินขั้นต่ำ (บาท)"));
        Serial.println(F("  SET deductRate <n>  อัตราหักเงิน (บาท/วิ)"));
        Serial.println(F("  SET proc0..7 <n>    ระยะเวลา Process (วิ)"));
        Serial.println(F("  SET countdown <n>   นับถอยหลัง (วิ)"));
        Serial.println(F("  SET telemetrySec <n>"));
        Serial.println(F("  SCAN / PINREAD / VERBOSE ON/OFF"));
        Serial.println(F("  RESET PREFS / GOTO <0-7>"));
        Serial.println(F("══════════════════════════════════════════════════"));
        Serial.println(F("  MQTT: set-params, set-min-money, set-deduct-rate,"));
        Serial.println(F("        set-proc-duration, rollback, get-config,"));
        Serial.println(F("        ble-send, reboot"));
        Serial.println(F("══════════════════════════════════════════════════"));
    }
    else { Serial.println(F("[CMD] Unknown. Type HELP")); }
}

// ============================================================
//  STATE HANDLERS
// ============================================================

void handleBoot() {
    if (!stateChanged) return;
    stateChanged = false;
    Serial.println(F("[BOOT] CATPAW hardware check..."));

    uint8_t seg8[] = {0x7F,0x7F,0x7F,0x7F};
    display.setSegments(seg8); delay(400);

    bool ok[4] = {
        expander1.begin(), expander2.begin(),
        expander3.begin(), expander4.begin()
    };
    uint8_t exAddr[4] = {0x18,0x1C,0x1E,0x1F};
    const char* exRole[4] = {"LOAD","LED","BTN","WL+SW"};
    for (int i=0;i<4;i++)
        Serial.printf("[BOOT] EXP 0x%02X %-6s : %s\n",
            exAddr[i], exRole[i], ok[i]?"OK":"FAIL");

    if (!ok[0]||!ok[1]||!ok[2]||!ok[3]) {
        transitionTo(STATE_LOCKED); return;
    }

    // Output: เขียน LOW ก่อน → OUTPUT
    for (uint8_t p=0;p<8;p++) {
        expander1.digitalWrite(p,LOW);
        expander2.digitalWrite(p,LOW);
    }
    for (uint8_t p=0;p<8;p++) {
        expander1.pinMode(p,OUTPUT);
        expander2.pinMode(p,OUTPUT);
    }
    // Input
    for (uint8_t p=0;p<8;p++) {
        expander3.pinMode(p,INPUT);
        expander4.pinMode(p,INPUT);
    }
    Serial.println(F("[BOOT] LOAD/LED=OUTPUT  BTN/WL=INPUT"));

    pinMode(PIN_BANK_PULSE,INPUT_PULLUP);
    pinMode(PIN_COIN_PULSE,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BANK_PULSE), bankISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_COIN_PULSE), coinISR, CHANGE);
    lightSensor.begin();

    display.showNumberDec(100, false, 4, 0);
    Serial.printf("[BOOT] OK — delay %d ms\n", pref_bootDelayMs);
    delay(pref_bootDelayMs);
    transitionTo(STATE_IDLE);
}

void handleIdle() {
    if (stateChanged) {
        stateChanged = false;
        totalMoney   = 0.0f;
        display.showNumberDec(0, false, 4, 0);
        bankReader.enable();
        coinReader.enable();
        Serial.printf("[IDLE] Waiting money >= %d bath\n", pref_minStartMoney);
    }

    lightSensor.update();
    static bool prevLight = false;
    bool nowLight = lightSensor.detected();
    if (nowLight != prevLight) {
        prevLight = nowLight;
        Serial.printf("[LIGHT] %s | ADC=%d brightness=%d%%\n",
            nowLight?"DETECTED":"not detected",
            lightSensor.rawADC(), lightSensor.brightness());
    }

    static unsigned long lastWL = 0;
    if (millis()-lastWL >= 200) { lastWL = millis(); updateWaterLevel(); }

    checkMoneyInput();
    if (totalMoney >= (float)pref_minStartMoney)
        transitionTo(STATE_PAYMENT_CHECK);
}

void handlePaymentCheck() {
    if (stateChanged) {
        stateChanged = false;
        Serial.printf("[PAY] %.0f >= %d bath — press BTN0 (START)\n",
            totalMoney, pref_minStartMoney);
        String qr = "catpaw://" + DEVICE_ID + "?amt=" + String((int)totalMoney);
        sendBLE(qr);
    }
    checkMoneyInput();
    if (btnStart.pressEvent) {
        bankReader.disable();
        coinReader.disable();
        addLog("INITIALIZE");
        transitionTo(STATE_READY);
    }
}

void handleReady() {
    if (stateChanged) {
        stateChanged       = false;
        countdownRemaining = pref_readyCountdown;
        lastCountdownTick  = millis();
        display.showNumberDec(countdownRemaining, false, 4, 0);
        Serial.printf("[READY] Countdown: %d\n", countdownRemaining);
        addLog("COUNTDOWN_START");
        return;
    }
    if (countdownRemaining > 0 && millis()-lastCountdownTick >= 1000) {
        lastCountdownTick = millis();
        countdownRemaining--;
        if (countdownRemaining > 0) {
            display.showNumberDec(countdownRemaining, false, 4, 0);
            Serial.printf("[READY] %d...\n", countdownRemaining);
        } else {
            display.showNumberDec((int)totalMoney, false, 4, 0);
            addLog("COUNTDOWN_END");
            Serial.println(F("[READY] GO! Starting process sequence..."));
            transitionTo(STATE_OPERATION);
        }
    }
}

// ── OPERATION: Sequential Process ──────────────────────────
// ไล่ Process 0→7
// แต่ละ Process: LED[n]+LD[n] ON นาน pref_procSec[n] วิ
// ข้าม Process ที่ duration=0
// หักเงิน pref_deductRate บาทต่อวิ ตลอดเวลา
void handleOperation() {
    if (stateChanged) {
        stateChanged     = false;
        currentProcess   = 0;
        operationRunning = false;
        lastDeductTime   = millis();
        lastDisplayRefresh = millis();
        memset(procActualMs, 0, sizeof(procActualMs));
        // หา process แรกที่ duration > 0
        while (currentProcess < NUM_PROCESSES && pref_procSec[currentProcess] == 0)
            currentProcess++;
        if (currentProcess >= NUM_PROCESSES) {
            Serial.println(F("[OP] All processes skipped (duration=0) → SUMMARY"));
            transitionTo(STATE_SUMMARY);
            return;
        }
        setProcess(currentProcess, true);
        processStartTime = millis();
        operationRunning = true;
        char label[24]; snprintf(label,sizeof(label),"PROC%d_START",currentProcess);
        addLog(label);
        Serial.printf("[OP] PROC%d started | duration=%ds | Money:%.0f\n",
            currentProcess, pref_procSec[currentProcess], totalMoney);
    }

    unsigned long now = millis();

    // ── หักเงิน 1 บาท/วิ ──
    if (now - lastDeductTime >= 1000) {
        lastDeductTime = now;
        totalMoney -= (float)pref_deductRate;
        if (totalMoney < 0.0f) totalMoney = 0.0f;
        display.showNumberDec((int)totalMoney, false, 4, 0);
        lastDisplayRefresh = now;
        Serial.printf("[OP] -1 bath | Left: %.0f bath\n", totalMoney);

        if (totalMoney <= 0.0f) {
            // เงินหมดกลางคัน
            uint32_t elapsed = (uint32_t)(now - processStartTime);
            procActualMs[currentProcess] += elapsed;
            char label[24]; snprintf(label,sizeof(label),"PROC%d_MONEY_EMPTY",currentProcess);
            addLog(label, elapsed);
            setProcess(currentProcess, false);
            allOff();
            operationRunning = false;
            display.showNumberDec(0, false, 4, 0);
            Serial.println(F("[OP] Money depleted → SUMMARY"));
            transitionTo(STATE_SUMMARY);
            return;
        }
    }

    // ── ตรวจว่า Process ปัจจุบันครบเวลาหรือยัง ──
    unsigned long procElapsed = now - processStartTime;
    if (procElapsed >= (unsigned long)pref_procSec[currentProcess] * 1000UL) {
        procActualMs[currentProcess] = procElapsed;
        char label[24]; snprintf(label,sizeof(label),"PROC%d_DONE",currentProcess);
        addLog(label, procElapsed);
        setProcess(currentProcess, false);
        Serial.printf("[OP] PROC%d done | actual=%lums\n",
            currentProcess, (unsigned long)procElapsed);

        // ── หา Process ถัดไป (ข้ามที่ duration=0) ──
        currentProcess++;
        while (currentProcess < NUM_PROCESSES && pref_procSec[currentProcess] == 0)
            currentProcess++;

        if (currentProcess >= NUM_PROCESSES) {
            // รันครบทุก Process แล้ว
            operationRunning = false;
            allOff();
            Serial.println(F("[OP] All processes complete → SUMMARY"));
            transitionTo(STATE_SUMMARY);
            return;
        }

        // เริ่ม Process ถัดไป
        setProcess(currentProcess, true);
        processStartTime = millis();
        snprintf(label, sizeof(label), "PROC%d_START", currentProcess);
        addLog(label);
        Serial.printf("[OP] PROC%d started | duration=%ds | Money:%.0f\n",
            currentProcess, pref_procSec[currentProcess], totalMoney);
    }

    // ── Display refresh ทุก 1 วิ ──
    if (now - lastDisplayRefresh >= 1000) {
        lastDisplayRefresh = now;
        display.showNumberDec((int)totalMoney, false, 4, 0);
    }

    // ── Top-up debug ──
    if (dbg_addMoney) {
        totalMoney += dbg_addAmount;
        addLog("TOPUP", (uint32_t)dbg_addAmount);
        Serial.printf("[OP] +%.0f bath | Total: %.0f\n", dbg_addAmount, totalMoney);
        dbg_addMoney = false; dbg_addAmount = 0.0f;
        display.showNumberDec((int)totalMoney, false, 4, 0);
    }
}

void handleSummary() {
    if (stateChanged) {
        stateChanged = false;
        operationRunning = false;
        allOff();
        display.showNumberDec(0, false, 4, 0);
        // สรุป process
        Serial.println(F("[SUMMARY] Process results:"));
        for (int p = 0; p < NUM_PROCESSES; p++) {
            if (pref_procSec[p] == 0) continue;
            Serial.printf("  PROC%d: config=%ds actual=%lums\n",
                p, pref_procSec[p], (unsigned long)procActualMs[p]);
        }
        Serial.printf("[SUMMARY] WaterLevel=%d/6 | Money left=%.0f\n",
            waterLevel, totalMoney);
        String qr = "catpaw://" + DEVICE_ID + "/done";
        sendBLE(qr);
        Serial.println(F("[SUMMARY] Press BTN0 to view log & reset"));
    }
    if (btnStart.pressEvent) {
        addLog("SUMMARY_VIEW");
        printSummary();
        // Publish session summary
        if (mqttClient.connected()) {
            JsonDocument doc;
            doc["deviceId"]   = DEVICE_ID;
            doc["event"]      = "session_end";
            doc["entries"]    = logCount;
            doc["money_left"] = (int)totalMoney;
            doc["waterLevel"] = waterLevel;
            doc["sw0"]        = swState[0];
            doc["sw1"]        = swState[1];
            JsonArray procs = doc["procActualMs"].to<JsonArray>();
            for (int p = 0; p < NUM_PROCESSES; p++) procs.add(procActualMs[p]);
            String out; serializeJson(doc, out);
            mqttClient.publish(makeTopic("session").c_str(), out.c_str(), false);
        }
        totalMoney = 0.0f;
        clearLog();
        transitionTo(STATE_IDLE);
    }
}

void handleLocked() {
    if (stateChanged) {
        stateChanged = false;
        allOff();
        display.showNumberDec(1337, false, 4, 0);
        Serial.println(F("[LOCKED] Hardware error — reset required."));
    }
    static unsigned long lastWarn = 0;
    if (millis()-lastWarn > 5000) {
        lastWarn = millis();
        Serial.println(F("[LOCKED] Halted."));
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(600);
    Serial.println(F("\n╔═══════════════════════════════════════════════╗"));
    Serial.println(F("║  CATPAW FIRMWARE v1.0  |  ESP32-S3           ║"));
    Serial.println(F("║  Sequential Process | MQTT | BLE->QR         ║"));
    Serial.println(F("╚═══════════════════════════════════════════════╝"));
    Serial.println(F("Type HELP for commands\n"));

    loadPreferences();

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000UL);

    display.setBrightness(7);
    display.showNumberDec(8888, false, 4, 0);

    createDeviceID();

    connectWiFi();
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setKeepAlive(60);
    mqttClient.setSocketTimeout(10);
    mqttClient.setBufferSize(4096);
    connectMQTT();

    initBLE();

    currentState   = STATE_BOOT;
    stateChanged   = true;
    stateEnterTime = millis();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (!mqttClient.connected())        connectMQTT();
    mqttClient.loop();

    handleSerial();
    updateButtons();
    handleTelemetry();

    if (dbg_verbose) {
        static unsigned long lv = 0;
        if (millis()-lv >= 500) {
            lv = millis();
            Wire.beginTransmission(0x1E); Wire.write(0x00);
            Wire.endTransmission(false);
            Wire.requestFrom((uint8_t)0x1E, (uint8_t)1);
            uint8_t r3 = Wire.available() ? Wire.read() : 0xFF;
            Wire.beginTransmission(0x1F); Wire.write(0x00);
            Wire.endTransmission(false);
            Wire.requestFrom((uint8_t)0x1F, (uint8_t)1);
            uint8_t r4 = Wire.available() ? Wire.read() : 0xFF;
            Serial.printf("[VRB] BTN(0x1E)=0b");
            for(int b=7;b>=0;b--) Serial.print((r3>>b)&1);
            Serial.printf("  WL(0x1F)=0b");
            for(int b=7;b>=0;b--) Serial.print((r4>>b)&1);
            Serial.printf("  BTN0=%s PROC=%d WL=%d\n",
                btnStart.isPressed?"ON":"off", currentProcess, waterLevel);
        }
    }

    switch (currentState) {
        case STATE_BOOT:          handleBoot();          break;
        case STATE_IDLE:          handleIdle();          break;
        case STATE_PAYMENT_CHECK: handlePaymentCheck();  break;
        case STATE_READY:         handleReady();         break;
        case STATE_OPERATION:     handleOperation();     break;
        case STATE_SUMMARY:       handleSummary();       break;
        case STATE_LOCKED:        handleLocked();        break;
        default:
            Serial.printf("[ERROR] Unknown state %d\n", currentState);
            transitionTo(STATE_LOCKED);
    }
}