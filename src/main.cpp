/**
 * ============================================================
 *  MACHINE FIRMWARE v3.0  |  ESP32-S3 Dev Module
 *  Platform: PlatformIO + Arduino Framework
 *
 *  State Machine:
 *    0 - BOOT          : ตรวจสอบ Hardware
 *    1 - IDLE          : รอรับเงิน (cash / QR)
 *    2 - PAYMENT_CHECK : รอกดปุ่ม Start (เงินครบแล้ว)
 *    3 - READY         : นับถอยหลัง
 *    4 - OPERATION     : รัน 6 Step อัตโนมัติ
 *    5 - SUMMARY       : สรุปผล / รอกด START เพื่อ reset
 *    7 - LOCKED        : Hardware Error
 *
 *  ราคา: pref_price (default 20 บาท)
 *    - Fix ไว้ 1 ค่า (แทน minStartMoney)
 *    - ปรับได้ผ่าน MQTT: {"cmd":"set-price","value":20}
 *    - รับเงินเหมือนเดิมทั้ง coin, bank, QR
 *    - *** ไม่มี topup ระหว่าง OPERATION ***
 *
 *  Operation Mode (Fixed-Step):
 *    - 6 Step อัตโนมัติ: Step N → เปิด LD(N-1) ตัวเดียว
 *    - แต่ละ step มี duration_ms ของตัวเอง ปรับผ่าน MQTT ได้
 *    - Display แสดง step ปัจจุบัน (1-6)
 *    - ไม่มีการเลือก channel โดยผู้ใช้
 *
 *  Connectivity:
 *    - WiFi + MQTT  : ปรับ parameter ผ่าน server
 *    - BLE Server   : ส่ง QR Code ไปแสดงบน CYD display
 *
 *  Channel Architecture:
 *    EXP3 (0x1E) P0-P7 = ปุ่ม (Active-LOW) — Start trigger
 *    EXP1 (0x18) P0-P7 = Relay LD0-LD7
 *    Step 1→LD0, Step 2→LD1, Step 3→LD2, Step 4→LD3, Step 5→LD4, Step 6→LD5
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
//  [SECTION A] USER PREFERENCES — แก้ไขค่าเริ่มต้นได้ที่นี่
//  บันทึกใน NVS (Flash) ไม่หายหลัง Reset
//  เปลี่ยน runtime ได้ผ่าน Serial: SET <key> <val>
//                    หรือ MQTT:   {"cmd":"set-price","value":20}
// ============================================================

// ── ราคา Fix ──
#define DEFAULT_PRICE             20      // [บาท] ราคาต่อ session (แทน minStartMoney)

// ── Timing ──
#define DEFAULT_READY_COUNTDOWN   5       // [วิ] นับถอยหลังก่อนเข้า OPERATION
#define DEFAULT_BOOT_DELAY_MS     3000    // [ms] pause หลัง Boot OK

// ── Step Duration (ms) ──
// ปรับผ่าน MQTT: {"cmd":"set-step-duration","step":0,"value":5000}
// หรือ Serial:   SET step0 5000
#define NUM_STEPS           6
#define DEFAULT_STEP_0_MS   5000    // Step 1 → LD0
#define DEFAULT_STEP_1_MS   8000    // Step 2 → LD1
#define DEFAULT_STEP_2_MS   6000    // Step 3 → LD2
#define DEFAULT_STEP_3_MS   7000    // Step 4 → LD3
#define DEFAULT_STEP_4_MS   10000   // Step 5 → LD4
#define DEFAULT_STEP_5_MS   5000    // Step 6 → LD5

// ── Pulse Money Acceptors ──
#define DEFAULT_BANK_PULSE_VALUE   10.0f  // [บาท/pulse] แบงค์
#define DEFAULT_COIN_PULSE_VALUE   1.0f   // [บาท/pulse] เหรียญ

// ── Pulse Width Timing ──
#define BANK_MIN_PULSE_MS     20
#define BANK_MAX_PULSE_MS    200
#define BANK_PULSE_TIMEOUT_MS 600
#define COIN_MIN_PULSE_MS     15
#define COIN_MAX_PULSE_MS    150
#define COIN_PULSE_TIMEOUT_MS 400

// ── Telemetry ──
#define DEFAULT_TELEMETRY_SEC     30      // [วิ] ส่ง telemetry ทุกกี่วิ

// ============================================================
//  [SECTION B] PIN CONFIGURATION
// ============================================================
#define PIN_BANK_PULSE   5
#define PIN_COIN_PULSE   12
#define PI_UART_TX      13
#define PI_UART_RX      14
#define I2C_SDA          8
#define I2C_SCL          9
#define DISP_CLK         15
#define DISP_DIO         16

// ── Light Sensor ────────────────────────────────────────────
#define PIN_LIGHT_SENSOR     1
#define LIGHT_THRESHOLD_ADC  2000
#define LIGHT_DEBOUNCE_MS    200

// ============================================================
//  [SECTION C] EXPANDER MAPPING
//
//  0x18  LOAD  (OUTPUT) — Relay LD0-LD7
//  0x1C  LED   (OUTPUT) — LED0-LED7
//  0x1E  BTN   (INPUT)  — ปุ่ม BTN0-BTN7 (Active-LOW) → Start trigger
//  0x1F  WL    (INPUT)  — Water Level + Switch (Active-LOW)
//
//  LOAD (0x18):  LD0=IO0  LD1=IO1  LD2=IO3  LD3=IO2
//                LD4=IO5  LD5=IO4  LD6=IO7  LD7=IO6
//  LED  (0x1C):  LED0=IO7 LED1=IO6 LED2=IO5 LED3=IO4
//                LED4=IO3 LED5=IO2 LED6=IO1 LED7=IO0
//  BTN  (0x1E):  BTN0=IO7 BTN1=IO6 BTN2=IO5 BTN3=IO4
//                BTN4=IO3 BTN5=IO2 BTN6=IO1 BTN7=IO0
//  WL   (0x1F):  WL0=IO5  WL1=IO4  WL2=IO3  WL3=IO2
//                WL4=IO0  WL5=IO1  IO6=SW0  IO7=SW1
// ============================================================
#define NUM_CHANNELS    8
#define NUM_WL_LEVELS   6
#define NUM_SWITCHES    2

static const uint8_t LD_PIN[8]  = {1, 0, 2, 3, 4, 5, 7, 6};
static const uint8_t LED_PIN[8] = {7, 6, 5, 4, 3, 2, 0, 1};
static const uint8_t BTN_LOGICAL[8] = {6, 7, 0, 1, 2, 3, 4, 5};
static const uint8_t WL_PIN[6] = {5, 4, 3, 2, 0, 1};

#define SW0_PIN  6
#define SW1_PIN  7
#define EXP5_ADDR  0x27

// ============================================================
//  [SECTION D] NETWORK CONFIG
// ============================================================
#define WIFI_SSID           "SR803_5G"
#define WIFI_PASS           "80323SM5F"

#define MQTT_SERVER         "147.50.255.120"
#define MQTT_PORT           9359
#define MQTT_USER           "esp32"
#define MQTT_PASS           "Taweesak@5050"
#define MQTT_PREFIX         "machine/vend"

#define BLE_SERVER_NAME     "ESP32S3-QR"
#define BLE_SERVICE_UUID    "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ============================================================
//  [SECTION E] CONSTANTS
// ============================================================
#define MAX_LOG_ENTRIES   64
#define BTN_POLL_MS       20
#define BTN_DEBOUNCE_MS   60

// ============================================================
//  MQTT COMMAND QUEUE
// ============================================================
#define MQTT_DEFER_IDLE_MS   (5UL * 60UL * 1000UL)
#define MQTT_QUEUE_SLOTS     4
#define MQTT_JSON_MAX        512

struct MqttDeferredCmd { bool used; char json[MQTT_JSON_MAX]; };
MqttDeferredCmd mqttQueue[MQTT_QUEUE_SLOTS] = {};
bool            _applyingQueue = false;

void mqttEnqueue(const String& json) {
    for (int i = 0; i < MQTT_QUEUE_SLOTS; i++) {
        if (!mqttQueue[i].used) {
            strncpy(mqttQueue[i].json, json.c_str(), MQTT_JSON_MAX - 1);
            mqttQueue[i].used = true;
            Serial.println(F("[MQTT] Queued (session active)"));
            return;
        }
    }
    memmove(&mqttQueue[0], &mqttQueue[1],
            sizeof(MqttDeferredCmd) * (MQTT_QUEUE_SLOTS - 1));
    strncpy(mqttQueue[MQTT_QUEUE_SLOTS-1].json, json.c_str(), MQTT_JSON_MAX - 1);
    mqttQueue[MQTT_QUEUE_SLOTS-1].used = true;
    Serial.println(F("[MQTT] Queue full — oldest dropped"));
}

void mqttCallback(char* topic, byte* payload, unsigned int len);  // forward

void mqttProcessQueue() {
    bool hasCmd = false;
    for (int i = 0; i < MQTT_QUEUE_SLOTS; i++) if (mqttQueue[i].used) { hasCmd = true; break; }
    if (!hasCmd) return;
    Serial.println(F("[MQTT] Applying deferred commands..."));
    _applyingQueue = true;
    for (int i = 0; i < MQTT_QUEUE_SLOTS; i++) {
        if (!mqttQueue[i].used) continue;
        Serial.println("[MQTT] Deferred: " + String(mqttQueue[i].json));
        mqttCallback(nullptr, (byte*)mqttQueue[i].json, strlen(mqttQueue[i].json));
        mqttQueue[i].used = false;
    }
    _applyingQueue = false;
}

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
//  LOG
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
ExpanderManager expander1(0x18);
ExpanderManager expander2(0x1C);
ExpanderManager expander3(0x1E);
ExpanderManager expander4(0x1F);
ExpanderManager expander5(EXP5_ADDR);
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

// ── Water Level & Switch ──
bool     wlState[NUM_WL_LEVELS] = {};
bool     swState[NUM_SWITCHES]  = {};
int      waterLevel             = 0;
int      prevWaterLevel         = -1;

unsigned long lastMoneyCheck     = 0;
int           countdownRemaining = 0;
unsigned long lastCountdownTick  = 0;

// ── Operation: Step tracking ──
int           currentStep    = 0;   // 0-based
unsigned long stepStartTime  = 0;

LogEntry      sessionLog[MAX_LOG_ENTRIES];
int           logCount = 0;

// ── NVS loaded values ──
int pref_price;               // ราคา fix (แทน minStartMoney)
int pref_readyCountdown;
int pref_bootDelayMs;
int pref_telemetrySec;
int pref_stepDurationMs[NUM_STEPS];

String DEVICE_ID;

// ── Serial debug ──
bool  dbg_addMoney    = false;
float dbg_addAmount   = 0.0f;
bool  dbg_pressStart  = false;
bool  dbg_verbose     = false;

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
    if (!bleClientConnected) {
        Serial.printf("[BLE] No client — skipped: %s\n", text.c_str());
        return;
    }
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
    DEVICE_ID = String("vend-") + id;
    Serial.println("[NET] Device ID: " + DEVICE_ID);
}

// ============================================================
//  WIFI
// ============================================================
void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.println(F("[WiFi] Connecting..."));
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(true);
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
    Serial.println(F("╔══════════════════════════════════════════════════╗"));
    Serial.println(F("║                  SESSION SUMMARY                 ║"));
    Serial.println(F("╠══════════════════════════════════════════════════╣"));
    Serial.printf( "║  Price   : %-38d║\n", pref_price);
    Serial.printf( "║  Entries : %-38d║\n", logCount);
    Serial.println(F("╠══════════════════════════════════════════════════╣"));
    for (int i = 0; i < logCount; i++) {
        Serial.printf("║ [%02d] [%8lu ms] %-18s ฿%-4d %lu ms\n",
            i+1,
            (unsigned long)sessionLog[i].timestamp_ms,
            sessionLog[i].event,
            (int)sessionLog[i].money,
            (unsigned long)sessionLog[i].duration_ms);
    }
    Serial.println(F("╚══════════════════════════════════════════════════╝"));
}

void clearLog() {
    logCount = 0;
    memset(sessionLog, 0, sizeof(sessionLog));
}

// ============================================================
//  NVS PREFERENCES
// ============================================================
void loadPreferences() {
    const int defSteps[NUM_STEPS] = {
        DEFAULT_STEP_0_MS, DEFAULT_STEP_1_MS,
        DEFAULT_STEP_2_MS, DEFAULT_STEP_3_MS,
        DEFAULT_STEP_4_MS, DEFAULT_STEP_5_MS
    };
    prefs.begin("machine", false);
    prefs.end();
    prefs.begin("machine", true);
    pref_price          = prefs.getInt("price",        DEFAULT_PRICE);
    pref_readyCountdown = prefs.getInt("countdown",    DEFAULT_READY_COUNTDOWN);
    pref_bootDelayMs    = prefs.getInt("bootDelay",    DEFAULT_BOOT_DELAY_MS);
    pref_telemetrySec   = prefs.getInt("telemetrySec", DEFAULT_TELEMETRY_SEC);
    for (int s = 0; s < NUM_STEPS; s++) {
        char key[8]; snprintf(key, sizeof(key), "step%d", s);
        pref_stepDurationMs[s] = prefs.getInt(key, defSteps[s]);
    }
    prefs.end();

    Serial.println(F("[PREFS] Loaded:"));
    Serial.printf("  price          = %d bath\n",    pref_price);
    Serial.printf("  readyCountdown = %d s\n",       pref_readyCountdown);
    Serial.printf("  telemetrySec   = %d s\n",       pref_telemetrySec);
    Serial.print( "  stepDurations  = [");
    for (int s = 0; s < NUM_STEPS; s++)
        Serial.printf("%d%s", pref_stepDurationMs[s], s < NUM_STEPS-1 ? "," : "] ms\n");
}

void savePreferences() {
    prefs.begin("machine", false);
    prefs.putInt("price",        pref_price);
    prefs.putInt("countdown",    pref_readyCountdown);
    prefs.putInt("bootDelay",    pref_bootDelayMs);
    prefs.putInt("telemetrySec", pref_telemetrySec);
    for (int s = 0; s < NUM_STEPS; s++) {
        char key[8]; snprintf(key, sizeof(key), "step%d", s);
        prefs.putInt(key, pref_stepDurationMs[s]);
    }
    prefs.end();
    Serial.println(F("[PREFS] Saved"));
}

void resetPreferences() {
    prefs.begin("machine", false); prefs.clear(); prefs.end();
    pref_price          = DEFAULT_PRICE;
    pref_readyCountdown = DEFAULT_READY_COUNTDOWN;
    pref_bootDelayMs    = DEFAULT_BOOT_DELAY_MS;
    pref_telemetrySec   = DEFAULT_TELEMETRY_SEC;
    const int defSteps[NUM_STEPS] = {
        DEFAULT_STEP_0_MS, DEFAULT_STEP_1_MS,
        DEFAULT_STEP_2_MS, DEFAULT_STEP_3_MS,
        DEFAULT_STEP_4_MS, DEFAULT_STEP_5_MS
    };
    for (int s = 0; s < NUM_STEPS; s++) pref_stepDurationMs[s] = defSteps[s];
    Serial.println(F("[PREFS] Reset to defaults"));
}

// ============================================================
//  CONFIG SNAPSHOT & ROLLBACK
// ============================================================
String makeTopic(const String& sfx) {
    return String(MQTT_PREFIX) + "/" + DEVICE_ID + "/" + sfx;
}

void snapshotPreferences() {
    prefs.begin("machine_bk", false);
    prefs.putInt("price",        pref_price);
    prefs.putInt("countdown",    pref_readyCountdown);
    prefs.putInt("telemetrySec", pref_telemetrySec);
    for (int s = 0; s < NUM_STEPS; s++) {
        char key[8]; snprintf(key, sizeof(key), "step%d", s);
        prefs.putInt(key, pref_stepDurationMs[s]);
    }
    prefs.putULong("snapTime", (unsigned long)(millis() / 1000));
    prefs.end();
    Serial.println(F("[SNAP] Config snapshot saved"));
}

bool rollbackPreferences() {
    prefs.begin("machine_bk", false);
    prefs.end();
    prefs.begin("machine_bk", true);
    bool hasSnap = prefs.isKey("price");
    if (hasSnap) {
        pref_price          = prefs.getInt("price",        pref_price);
        pref_readyCountdown = prefs.getInt("countdown",    pref_readyCountdown);
        pref_telemetrySec   = prefs.getInt("telemetrySec", pref_telemetrySec);
        for (int s = 0; s < NUM_STEPS; s++) {
            char key[8]; snprintf(key, sizeof(key), "step%d", s);
            pref_stepDurationMs[s] = prefs.getInt(key, pref_stepDurationMs[s]);
        }
        unsigned long snapTime = prefs.getULong("snapTime", 0);
        prefs.end();
        savePreferences();
        Serial.printf("[SNAP] Rollback OK (snapshot age: %lu s)\n",
            (unsigned long)(millis()/1000) - snapTime);
    } else {
        prefs.end();
        Serial.println(F("[SNAP] No snapshot found — rollback cancelled"));
    }
    return hasSnap;
}

void mqttPublishSnapshot(const char* action) {
    if (!mqttClient.connected()) return;
    JsonDocument doc;
    doc["deviceId"]  = DEVICE_ID;
    doc["action"]    = action;
    doc["price"]     = pref_price;
    doc["countdown"] = pref_readyCountdown;
    JsonArray steps = doc["stepDurations"].to<JsonArray>();
    for (int s = 0; s < NUM_STEPS; s++) steps.add(pref_stepDurationMs[s]);
    String out; serializeJson(doc, out);
    mqttClient.publish(makeTopic("config").c_str(), out.c_str(), false);
    Serial.println("[MQTT] Config published: " + out);
}

// ============================================================
//  Pi UART COMM
// ============================================================
HardwareSerial PiSerial(2);

void sendPi(JsonDocument& doc) {
    String out; serializeJson(doc, out); out += "\n";
    PiSerial.print(out);
    Serial.println("[PI<<] " + out);
}
void sendPiState(const char* state) {
    JsonDocument doc; doc["evt"] = "state"; doc["state"] = state; sendPi(doc);
}
void sendPiMoney() {
    JsonDocument doc;
    doc["evt"]     = "money";
    doc["balance"] = (int)totalMoney;
    doc["price"]   = pref_price;   // ส่ง price แทน min
    sendPi(doc);
}
void sendPiStep(int step1based) {
    JsonDocument doc;
    doc["evt"]      = "step";
    doc["step"]     = step1based;
    doc["total"]    = NUM_STEPS;
    doc["duration"] = pref_stepDurationMs[step1based - 1];
    sendPi(doc);
}

// ============================================================
//  STATE TRANSITION
// ============================================================
void transitionTo(SystemState next) {
    Serial.printf("[STATE] %s -> %s\n",
        stateNames[currentState], stateNames[next]);
    if (next != STATE_READY && next != STATE_BOOT) {
        addLog(stateNames[next]);
    }
    currentState   = next;
    stateChanged   = true;
    stateEnterTime = millis();
    sendPiState(stateNames[next]);
}

// ============================================================
//  RELAY / LED HELPERS
// ============================================================
void setChannelRelay(uint8_t ch, bool on) {
    if (ch >= NUM_CHANNELS) return;
    uint8_t physPin = LD_PIN[ch];
    expander1.digitalWrite(physPin, on ? HIGH : LOW);
    Serial.printf("[LOAD] LD%d (IO%d) %s\n", ch, physPin, on ? "ON" : "OFF");
}

void allRelaysOff() {
    for (uint8_t c = 0; c < NUM_CHANNELS; c++)
        expander1.digitalWrite(LD_PIN[c], LOW);
    Serial.println(F("[LOAD] All OFF"));
}

void setLED(uint8_t ch, bool on) {
    if (ch >= NUM_CHANNELS) return;
    expander2.digitalWrite(LED_PIN[ch], on ? HIGH : LOW);
}

void allLEDsOff() {
    for (uint8_t c = 0; c < NUM_CHANNELS; c++)
        expander2.digitalWrite(LED_PIN[c], LOW);
}

// ============================================================
//  WATER LEVEL READER — EXP4 (0x1F)
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
    for (int i = 0; i < NUM_WL_LEVELS; i++)
        if (wlState[i]) level = i + 1;
    waterLevel = level;

    if (waterLevel != prevWaterLevel) {
        prevWaterLevel = waterLevel;
        Serial.printf("[WL] Level=%d/6 | WL:", waterLevel);
        for (int i = 5; i >= 0; i--) Serial.print(wlState[i] ? "1" : "0");
        Serial.printf(" | SW0=%d SW1=%d\n", swState[0], swState[1]);
    }
}

// ============================================================
//  MQTT CALLBACK
// ============================================================
// Commands:
//   {"cmd":"set-params","price":20,"stepDurations":[5000,...]}
//   {"cmd":"rollback"}
//   {"cmd":"get-config"}
//   {"cmd":"set-price","value":20}
//   {"cmd":"set-step-duration","step":0,"value":5000}  (step: 0-based)
//   {"cmd":"ble-send","text":"https://..."}
//   {"cmd":"reboot"}
void mqttPublishStatus(const String& s) {
    if (!mqttClient.connected()) return;
    JsonDocument doc;
    doc["deviceId"] = DEVICE_ID; doc["status"] = s;
    String out; serializeJson(doc, out);
    mqttClient.publish(makeTopic("status").c_str(), out.c_str(), false);
}

void mqttPublishAck(const String& key, int val) {
    if (!mqttClient.connected()) return;
    JsonDocument doc;
    doc["deviceId"] = DEVICE_ID; doc[key] = val;
    String out; serializeJson(doc, out);
    mqttClient.publish(makeTopic("ack").c_str(), out.c_str(), false);
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
    String msg;
    for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
    Serial.println("[MQTT] << " + msg);

    JsonDocument doc;
    if (deserializeJson(doc, msg)) {
        Serial.println(F("[MQTT] JSON parse error"));
        return;
    }
    String cmd = doc["cmd"].as<String>();

    // Deferred: queue ถ้า session กำลังดำเนินอยู่
    bool isDeferrable = (cmd == "set-params" || cmd == "set-price" ||
                         cmd == "set-step-duration" || cmd == "rollback");
    bool sessionActive = (currentState != STATE_IDLE && currentState != STATE_BOOT);
    if (isDeferrable && sessionActive && !_applyingQueue) {
        mqttEnqueue(msg);
        return;
    }

    if (cmd == "reboot") {
        mqttPublishStatus("rebooting");
        delay(500); ESP.restart();
    }
    else if (cmd == "set-params") {
        snapshotPreferences();
        mqttPublishSnapshot("snapshot-before-update");

        bool changed = false;
        String errors = "";

        if (!doc["price"].isNull()) {
            int v = doc["price"].as<int>();
            if (v >= 1 && v <= 9999) {
                pref_price = v; changed = true;
                Serial.printf("[SET] price = %d\n", v);
            } else { errors += "price out of range(1-9999); "; }
        }
        if (doc["stepDurations"].is<JsonArray>()) {
            JsonArray arr = doc["stepDurations"].as<JsonArray>();
            int idx = 0;
            for (JsonVariant v : arr) {
                if (idx >= NUM_STEPS) break;
                int ms = v.as<int>();
                if (ms >= 500 && ms <= 600000) {
                    pref_stepDurationMs[idx] = ms; changed = true;
                    Serial.printf("[SET] step[%d] = %d ms\n", idx, ms);
                } else {
                    char e[40]; snprintf(e, sizeof(e), "step[%d] out of range(500-600000); ", idx);
                    errors += e;
                }
                idx++;
            }
        }

        if (changed) {
            savePreferences();
            mqttPublishSnapshot("applied");
        }
        if (errors.length() > 0) {
            Serial.println("[MQTT] Errors: " + errors);
            JsonDocument errDoc;
            errDoc["deviceId"] = DEVICE_ID; errDoc["errors"] = errors;
            String out; serializeJson(errDoc, out);
            mqttClient.publish(makeTopic("error").c_str(), out.c_str(), false);
        }
    }
    else if (cmd == "rollback") {
        if (rollbackPreferences()) mqttPublishSnapshot("rollback-applied");
        else {
            JsonDocument err;
            err["deviceId"] = DEVICE_ID; err["error"] = "no snapshot available";
            String out; serializeJson(err, out);
            mqttClient.publish(makeTopic("error").c_str(), out.c_str(), false);
        }
    }
    else if (cmd == "get-config") {
        mqttPublishSnapshot("current-config");
    }
    else if (cmd == "set-price") {
        int v = doc["value"].as<int>();
        if (v >= 1 && v <= 9999) {
            snapshotPreferences();
            pref_price = v; savePreferences();
            Serial.printf("[MQTT] price = %d bath\n", v);
            mqttPublishAck("price", v);
        }
    }
    else if (cmd == "set-step-duration") {
        int s = doc["step"].as<int>();
        int v = doc["value"].as<int>();
        if (s >= 0 && s < NUM_STEPS && v >= 500 && v <= 600000) {
            snapshotPreferences();
            pref_stepDurationMs[s] = v; savePreferences();
            Serial.printf("[MQTT] step[%d] = %d ms\n", s, v);
            JsonDocument ack;
            ack["deviceId"] = DEVICE_ID; ack["step"] = s; ack["duration"] = v;
            String out; serializeJson(ack, out);
            mqttClient.publish(makeTopic("ack").c_str(), out.c_str(), false);
        }
    }
    else if (cmd == "ble-send") {
        String text = doc["text"].as<String>();
        if (text.length() > 0) sendBLE(text);
    }
    else {
        Serial.println("[MQTT] Unknown cmd: " + cmd);
    }
}

void connectMQTT() {
    if (mqttClient.connected()) return;
    static unsigned long lastAttempt = 0;
    if (millis() - lastAttempt < 5000) return;
    lastAttempt = millis();
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
    doc["deviceId"]   = DEVICE_ID;
    doc["state"]      = stateNames[currentState];
    doc["money"]      = (int)totalMoney;
    doc["price"]      = pref_price;
    doc["uptime_s"]   = millis() / 1000;
    doc["ble"]        = bleClientConnected;
    doc["waterLevel"] = waterLevel;
    doc["sw0"]        = swState[0];
    doc["sw1"]        = swState[1];
    if (currentState == STATE_OPERATION) {
        doc["currentStep"]   = currentStep + 1;
        doc["stepRemain_ms"] = (long)pref_stepDurationMs[currentStep]
                               - (long)(millis() - stepStartTime);
    }
    JsonArray wls = doc["wl"].to<JsonArray>();
    for (int i = 0; i < NUM_WL_LEVELS; i++) wls.add(wlState[i]);
    JsonArray steps = doc["stepDurations"].to<JsonArray>();
    for (int s = 0; s < NUM_STEPS; s++) steps.add(pref_stepDurationMs[s]);
    String out; serializeJson(doc, out);
    mqttClient.publish(makeTopic("telemetry").c_str(), out.c_str(), false);
    Serial.println("[MQTT] Telemetry sent");
}

// ============================================================
//  BUTTON READER — Single pin (Start)
// ============================================================
struct ButtonReader {
    ButtonReader(const char* n, bool (*fn)())
        : name(n), readRaw(fn),
          prevPhysical(false), isPressed(false),
          pressEvent(false), releaseEvent(false),
          _consumed(false),
          holdMs(0), pressedAt(0), lastChange(0),
          debounceMs(BTN_DEBOUNCE_MS) {}

    const char*   name;
    bool (*readRaw)();
    bool          prevPhysical, isPressed;
    bool          pressEvent, releaseEvent, _consumed;
    uint32_t      holdMs;
    unsigned long pressedAt, lastChange, debounceMs;

    void update() {
        if (_consumed) { _consumed = false; return; }
        bool raw = readRaw ? readRaw() : false;
        unsigned long now = millis();
        if (raw != prevPhysical) { lastChange = now; prevPhysical = raw; }
        if (now - lastChange < debounceMs) return;
        if (prevPhysical && !isPressed) {
            isPressed = true; pressedAt = now; pressEvent = true;
            Serial.printf("[BTN][%8lu ms] %s PRESS\n", now, name);
        } else if (!prevPhysical && isPressed) {
            holdMs = (uint32_t)(now - pressedAt);
            isPressed = false; releaseEvent = true;
            Serial.printf("[BTN][%8lu ms] %s RELEASE held=%lu ms\n",
                now, name, (unsigned long)holdMs);
        }
    }
    void injectPress() {
        pressEvent = true; _consumed = true;
        Serial.printf("[BTN][%8lu ms] %s PRESS (sim)\n",
            (unsigned long)millis(), name);
    }
};

// ============================================================
//  MULTI-PIN BUTTON READER — EXP3 (0x1E)
//  กดปุ่มไหนก็ได้ → anyPressEvent=true
// ============================================================
struct MultiPinButtonReader {
    uint8_t  prevRaw, stableRaw, pressedPins;
    bool     anyPressEvent;
    uint8_t  pressedPin;
    bool     _consumed;
    unsigned long lastChangeTime, debounceMs;

    MultiPinButtonReader()
        : prevRaw(0xFF), stableRaw(0xFF), pressedPins(0),
          anyPressEvent(false), pressedPin(0), _consumed(false),
          lastChangeTime(0), debounceMs(BTN_DEBOUNCE_MS) {}

    uint8_t readByte() {
        Wire.beginTransmission(0x1E);
        Wire.write(0x00);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)0x1E, (uint8_t)1);
        return Wire.available() ? Wire.read() : 0xFF;
    }

    void update() {
        if (_consumed) { _consumed = false; return; }
        uint8_t raw = readByte();
        unsigned long now = millis();
        if (raw != prevRaw) { lastChangeTime = now; prevRaw = raw; }
        if (now - lastChangeTime < debounceMs) return;
        uint8_t changed = stableRaw ^ raw;
        stableRaw = raw;
        if (!changed) return;

        for (uint8_t p = 0; p < 8; p++) {
            if (!(changed & (1<<p))) continue;
            bool gnd = !(raw & (1<<p));
            uint8_t logicalCh = BTN_LOGICAL[p];
            if (gnd && !(pressedPins & (1<<p))) {
                pressedPins  |= (1<<p);
                anyPressEvent = true;
                pressedPin    = logicalCh;
                Serial.printf("[BTN][%8lu ms] BTN%d PRESS (IO%d, EXP3 0x1E)\n",
                    now, logicalCh, p);
            } else if (!gnd && (pressedPins & (1<<p))) {
                pressedPins &= ~(1<<p);
                Serial.printf("[BTN][%8lu ms] BTN%d RELEASE (IO%d, EXP3 0x1E)\n",
                    now, logicalCh, p);
            }
        }
    }

    void injectPress(uint8_t logicalCh = 0) {
        anyPressEvent = true; pressedPin = logicalCh; _consumed = true;
        Serial.printf("[BTN][%8lu ms] BTN%d PRESS (sim)\n",
            (unsigned long)millis(), logicalCh);
    }
};

bool rawStart() { return false; }
ButtonReader         btnStart("START", rawStart);
MultiPinButtonReader btnAny;

const char* waterLevelDesc(int level) {
    switch (level) {
        case 1: return "Tank1 >50%"; case 2: return "Tank1 >25%";
        case 3: return "Tank2 >50%"; case 4: return "Tank2 >25%";
        case 5: return "Tank3 >50%"; case 6: return "Tank3 >25%";
        default: return "All <25%";
    }
}

// Forward declaration
void doSummaryAndReset();

// ============================================================
//  Pi UART READER
// ============================================================
void readPi() {
    while (PiSerial.available()) {
        String line = PiSerial.readStringUntil('\n');
        line.trim();
        if (line.isEmpty()) continue;
        Serial.println("[PI>>] " + line);
        JsonDocument doc;
        if (deserializeJson(doc, line)) continue;
        String cmd = doc["cmd"].as<String>();

        if (cmd == "start") {
            // Pi confirm payment (เช่น QR สำเร็จ) → กด Start แทนผู้ใช้
            if (currentState == STATE_PAYMENT_CHECK) {
                bankReader.disable(); coinReader.disable();
                addLog("INITIALIZE_PI"); transitionTo(STATE_READY);
            }
        }
        else if (cmd == "finish") {
            if (currentState == STATE_OPERATION) {
                allRelaysOff(); allLEDsOff();
                addLog("FINISH_PI");
                transitionTo(STATE_SUMMARY);
            } else if (currentState == STATE_SUMMARY) {
                doSummaryAndReset();
            }
        }
        else if (cmd == "pay_qr") {
            int amt = doc["amount"].as<int>();
            if (amt <= 0) amt = pref_price;
            String link = "https://catcarwash.example.com/pay?id=" + DEVICE_ID + "&amt=" + String(amt);
            JsonDocument _qd; _qd["evt"] = "qr_link"; _qd["url"] = link; sendPi(_qd);
            Serial.println("[PI] QR: " + link);
        }
        else if (cmd == "get_status") {
            sendPiState(stateNames[currentState]);
            sendPiMoney();
            if (currentState == STATE_OPERATION) sendPiStep(currentStep + 1);
        }
        else if (cmd == "ping") {
            JsonDocument _d;
            _d["evt"]     = "pong";
            _d["state"]   = stateNames[currentState];
            _d["balance"] = (int)totalMoney;
            _d["price"]   = pref_price;
            if (currentState == STATE_OPERATION) _d["step"] = currentStep + 1;
            sendPi(_d);
        }
    }
}

// ============================================================
//  MONEY INPUT
// ============================================================
void checkMoneyInput() {
    if (millis() - lastMoneyCheck < 50) return;
    lastMoneyCheck = millis();
    float bk = bankReader.checkAmount();
    float co = coinReader.checkAmount();
    if (dbg_addMoney) { bk += dbg_addAmount; dbg_addMoney = false; dbg_addAmount = 0; }
    if (bk > 0) {
        totalMoney += bk;
        display.showNumberDec((int)totalMoney, false, 4, 0);
        addLog("INSERT_BANK", (uint32_t)bk);
        sendPiMoney();
    }
    if (co > 0) {
        totalMoney += co;
        display.showNumberDec((int)totalMoney, false, 4, 0);
        addLog("INSERT_COIN", (uint32_t)co);
        sendPiMoney();
    }
    // Real-time preview ขณะ pulse ยังไม่ timeout
    static int lastSentPreview = -1;
    float pending = bankReader.getPendingAmount() + coinReader.getPendingAmount();
    if (pending > 0) {
        int previewTotal = (int)(totalMoney + pending);
        display.showNumberDec(previewTotal, false, 4, 0);
        if (previewTotal != lastSentPreview) {
            lastSentPreview = previewTotal;
            JsonDocument _pd;
            _pd["evt"] = "money"; _pd["balance"] = previewTotal; _pd["price"] = pref_price;
            sendPi(_pd);
        }
    } else {
        lastSentPreview = -1;
    }
}

// ============================================================
//  UPDATE BUTTONS
// ============================================================
static unsigned long lastBtnPoll = 0;

void updateButtons() {
    if (dbg_pressStart) { btnStart.injectPress(); dbg_pressStart = false; return; }

    btnStart.pressEvent   = false;
    btnStart.releaseEvent = false;
    btnAny.anyPressEvent  = false;

    unsigned long now = millis();
    if (now - lastBtnPoll < BTN_POLL_MS) return;
    lastBtnPoll = now;

    btnStart.update();
    btnAny.update();
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
        if (v > 0) { dbg_addMoney = true; dbg_addAmount = v;
                     Serial.printf("[CMD] +%.0f bath\n", v); }
    }
    else if (up == "START") { dbg_pressStart = true; }
    else if (up == "STATUS") {
        Serial.println(F("─────────────────────────────────────────────"));
        Serial.printf("  State     : %s\n", stateNames[currentState]);
        Serial.printf("  Money     : %.0f / %d bath\n", totalMoney, pref_price);
        if (currentState == STATE_OPERATION)
            Serial.printf("  Step      : %d/%d | Remain: %ld ms\n",
                currentStep+1, NUM_STEPS,
                (long)pref_stepDurationMs[currentStep] - (long)(millis() - stepStartTime));
        Serial.printf("  WiFi      : %s\n",
            WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString().c_str() : "offline");
        Serial.printf("  MQTT      : %s\n", mqttClient.connected()?"OK":"offline");
        Serial.printf("  BLE       : %s\n", bleClientConnected?"connected":"no client");
        Serial.println(F("  ── Money Readers ─────────────────────────────"));
        Serial.printf("  Bank: min=%dms max=%dms timeout=%dms | pending=%d pulses\n",
            BANK_MIN_PULSE_MS, BANK_MAX_PULSE_MS, BANK_PULSE_TIMEOUT_MS,
            bankReader.getPulseCount());
        Serial.printf("  Coin: min=%dms max=%dms timeout=%dms | pending=%d pulses\n",
            COIN_MIN_PULSE_MS, COIN_MAX_PULSE_MS, COIN_PULSE_TIMEOUT_MS,
            coinReader.getPulseCount());
        Serial.println(F("  ── Water Level (EXP4 0x1F) ───────────────────"));
        Serial.printf("  Level=%d/6 | ", waterLevel);
        for (int i = 5; i >= 0; i--)
            Serial.printf("WL%d=%s ", i, wlState[i] ? "ON" : "--");
        Serial.println();
        Serial.printf("  SW0=%s  SW1=%s\n",
            swState[0] ? "ON" : "OFF", swState[1] ? "ON" : "OFF");
        Serial.println(F("  ── Light Sensor ──────────────────────────────"));
        Serial.printf("  Pin=%d | ADC=%d | brightness=%d%% | detected=%s\n",
            PIN_LIGHT_SENSOR,
            lightSensor.rawADC(),
            lightSensor.brightness(),
            lightSensor.detected() ? "YES (light ON)" : "NO (dark)");
        Serial.println(F("  ── Preferences ───────────────────────────────"));
        Serial.printf("  price          = %d bath\n", pref_price);
        Serial.printf("  readyCountdown = %d s\n",    pref_readyCountdown);
        Serial.printf("  telemetrySec   = %d s\n",    pref_telemetrySec);
        Serial.println(F("  ── Step Durations ────────────────────────────"));
        for (int s = 0; s < NUM_STEPS; s++)
            Serial.printf("  Step%d → LD%d : %d ms\n", s+1, s, pref_stepDurationMs[s]);
        Serial.println(F("─────────────────────────────────────────────"));
    }
    else if (up == "LOG")  { printSummary(); }
    else if (up.startsWith("SET ")) {
        int sp = cmd.indexOf(' ', 4); if (sp < 0) return;
        String key = cmd.substring(4, sp); key.toLowerCase();
        int val = cmd.substring(sp+1).toInt();
        bool saved = true;
        if      (key == "price")        pref_price          = val;
        else if (key == "countdown")    pref_readyCountdown = val;
        else if (key == "telemetrysec") pref_telemetrySec   = val;
        else if (key.startsWith("step") && key.length() == 5) {
            int s = key.charAt(4) - '0';
            if (s >= 0 && s < NUM_STEPS) pref_stepDurationMs[s] = val;
            else { Serial.println(F("[CMD] Invalid step index (0-5)")); saved = false; }
        }
        else { Serial.println(F("[CMD] Unknown key")); saved = false; }
        if (saved) { savePreferences();
                     Serial.printf("[SET] %s = %d\n", key.c_str(), val); }
    }
    else if (up.startsWith("BLE ")) { sendBLE(cmd.substring(4)); }
    else if (up == "SCAN") {
        Serial.println(F("[SCAN] I2C bus..."));
        uint8_t found = 0;
        for (uint8_t a = 1; a < 127; a++) {
            Wire.beginTransmission(a);
            if (Wire.endTransmission() == 0) { Serial.printf("[SCAN] 0x%02X\n", a); found++; }
        }
        Serial.printf("[SCAN] %d device(s)\n", found);
    }
    else if (up == "PINREAD") {
        uint8_t addrs[4] = {0x18,0x1C,0x1E,0x1F};
        const char* roles[4] = {"LOAD(LD0-7)","LED(LED0-7)","BTN(BTN0-7)","WL+SW"};
        for (int e = 0; e < 4; e++) {
            Wire.beginTransmission(addrs[e]); Wire.write(0x00);
            Wire.endTransmission(false); Wire.requestFrom(addrs[e],(uint8_t)1);
            uint8_t r = Wire.available() ? Wire.read() : 0xFF;
            Serial.printf("[PIN] 0x%02X %-14s = 0b", addrs[e], roles[e]);
            for (int b = 7; b >= 0; b--) Serial.print((r>>b)&1);
            Serial.printf(" (0x%02X)\n", r);
        }
        Serial.printf("[PIN] WaterLevel=%d/6 | SW0=%d SW1=%d\n",
            waterLevel, swState[0], swState[1]);
    }
    else if (up == "VERBOSE ON")  { dbg_verbose = true;  Serial.println(F("[VRB] ON")); }
    else if (up == "VERBOSE OFF") { dbg_verbose = false; Serial.println(F("[VRB] OFF")); }
    else if (up == "RESET PREFS") { resetPreferences(); }
    else if (up.startsWith("GOTO ")) {
        int st = cmd.substring(5).toInt();
        if (st >= 0 && st <= 7) transitionTo((SystemState)st);
    }
    else if (up == "HELP") {
        Serial.println(F("══════════════════════ Serial Commands ═══════════════════════"));
        Serial.println(F("  ADD <n>              เพิ่มเงิน n บาท (sim)"));
        Serial.println(F("  START                จำลองกดปุ่ม Start"));
        Serial.println(F("  STATUS               ดูสถานะครบ"));
        Serial.println(F("  LOG                  ดู session log"));
        Serial.println(F("  BLE <text>           ส่งข้อความไปแสดง QR บน CYD"));
        Serial.println(F("  ── Preferences (บันทึก NVS ทันที) ──────────────────────────"));
        Serial.println(F("  SET price <n>        ราคา fix (บาท)"));
        Serial.println(F("  SET step0..5 <ms>    ระยะเวลาแต่ละ step (ms)"));
        Serial.println(F("  SET countdown <n>    นับถอยหลัง (วิ)"));
        Serial.println(F("  SET telemetrySec <n> Telemetry interval (วิ)"));
        Serial.println(F("  RESET PREFS          คืนค่า default ทั้งหมด"));
        Serial.println(F("  ── Debug ─────────────────────────────────────────────────"));
        Serial.println(F("  SCAN                 สแกน I2C bus"));
        Serial.println(F("  PINREAD              อ่านค่า pin register ทุก expander"));
        Serial.println(F("  VERBOSE ON/OFF       raw EXP3 byte ทุก 500ms"));
        Serial.println(F("  GOTO <0-7>           บังคับเปลี่ยน state"));
        Serial.println(F("═════════════════════════════════════════════════════════════"));
        Serial.println(F("  ── MQTT Commands ──────────────────────────────────────────"));
        Serial.println(F("  {\"cmd\":\"set-price\",\"value\":20}"));
        Serial.println(F("  {\"cmd\":\"set-step-duration\",\"step\":0,\"value\":5000}"));
        Serial.println(F("  {\"cmd\":\"set-params\",\"price\":20,\"stepDurations\":[5000,8000,6000,7000,10000,5000]}"));
        Serial.println(F("  {\"cmd\":\"get-config\"}"));
        Serial.println(F("  {\"cmd\":\"rollback\"}"));
        Serial.println(F("  {\"cmd\":\"reboot\"}"));
        Serial.println(F("  {\"cmd\":\"ble-send\",\"text\":\"https://...\"}"));
        Serial.println(F("═════════════════════════════════════════════════════════════"));
    }
    else { Serial.println(F("[CMD] Unknown. Type HELP")); }
}

// ============================================================
//  STATE HANDLERS
// ============================================================

void handleBoot() {
    if (!stateChanged) return;
    stateChanged = false;
    Serial.println(F("[BOOT] Hardware check..."));

    uint8_t seg8[] = {0x7F,0x7F,0x7F,0x7F};
    display.setSegments(seg8);
    delay(400);

    bool ok[4] = {
        expander1.begin(), expander2.begin(),
        expander3.begin(), expander4.begin()
    };
    uint8_t exAddr[4] = {0x18,0x1C,0x1E,0x1F};
    const char* exRole[4] = {"OUT-Relay","OUT-LED","IN-BTN","IN-WL"};
    for (int i = 0; i < 4; i++)
        Serial.printf("[BOOT] EXP 0x%02X %-12s : %s\n",
            exAddr[i], exRole[i], ok[i] ? "OK" : "FAIL");

    if (!ok[0]||!ok[1]||!ok[2]||!ok[3]) {
        transitionTo(STATE_LOCKED); return;
    }

    bool exp5ok = expander5.begin();
    Serial.printf("[BOOT] EXP 0x%02X %-12s : %s\n", EXP5_ADDR, "IN-Extra",
        exp5ok ? "OK" : "FAIL (optional)");
    if (exp5ok) {
        for (uint8_t p = 0; p < 6; p++) expander5.pinMode(p, INPUT);
    }

    for (uint8_t p = 0; p < 8; p++) {
        expander1.digitalWrite(p, LOW);
        expander2.digitalWrite(p, LOW);
    }
    for (uint8_t p = 0; p < 8; p++) {
        expander1.pinMode(p, OUTPUT);
        expander2.pinMode(p, OUTPUT);
    }
    for (uint8_t p = 0; p < 8; p++) {
        expander3.pinMode(p, INPUT);
        expander4.pinMode(p, INPUT);
    }
    Serial.println(F("[BOOT] LOAD/LED=OUTPUT(LOW)  BTN/WL=INPUT"));

    pinMode(PIN_BANK_PULSE, INPUT_PULLUP);
    pinMode(PIN_COIN_PULSE, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BANK_PULSE), bankISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_COIN_PULSE), coinISR, CHANGE);
    Serial.println(F("[BOOT] ISR attached (CHANGE mode — pulse width validation)"));

    lightSensor.begin();

    display.showNumberDec(100, false, 4, 0);
    Serial.printf("[BOOT] OK — delay %d ms\n", pref_bootDelayMs);
    delay(pref_bootDelayMs);
    transitionTo(STATE_IDLE);
}

// ── IDLE: รอรับเงินจนครบ pref_price ──
void handleIdle() {
    if (stateChanged) {
        stateChanged = false;
        totalMoney   = 0.0f;
        display.showNumberDec(0, false, 4, 0);
        bankReader.enable();
        coinReader.enable();
        Serial.printf("[IDLE] Waiting money >= %d bath (price)\n", pref_price);
        sendPiMoney();
    }

    // Process MQTT deferred commands หลัง IDLE นาน 5 นาที
    if (millis() - stateEnterTime > MQTT_DEFER_IDLE_MS) {
        mqttProcessQueue();
    }

    // Light Sensor
    lightSensor.update();
    static bool prevLightState = false;
    bool nowLight = lightSensor.detected();
    if (nowLight != prevLightState) {
        prevLightState = nowLight;
        Serial.printf("[LIGHT] %s | ADC=%d brightness=%d%%\n",
            nowLight ? "DETECTED (light ON)" : "NOT detected (dark)",
            lightSensor.rawADC(), lightSensor.brightness());
    }

    // Water Level
    static unsigned long lastWLCheck = 0;
    if (millis() - lastWLCheck >= 200) {
        lastWLCheck = millis();
        updateWaterLevel();
    }

    checkMoneyInput();

    // เงินครบราคา → ไป PAYMENT_CHECK
    if (totalMoney >= (float)pref_price)
        transitionTo(STATE_PAYMENT_CHECK);
}

// ── PAYMENT_CHECK: เงินครบแล้ว รอกด Start ──
void handlePaymentCheck() {
    if (stateChanged) {
        stateChanged = false;
        Serial.printf("[PAY] %.0f >= %d bath — press START\n",
            totalMoney, pref_price);
        // ส่ง QR แสดงยอดและ device ไป CYD
        String qr = "vend://" + DEVICE_ID + "?amt=" + String((int)totalMoney);
        sendBLE(qr);
    }
    // ยังรับเงินเพิ่มได้ (เกินราคาก็ไม่เป็นไร)
    checkMoneyInput();

    // กด BTN ไหนก็ได้ หรือ Serial START → เริ่ม
    if (btnAny.anyPressEvent || btnStart.pressEvent) {
        bankReader.disable();
        coinReader.disable();
        addLog("INITIALIZE");
        transitionTo(STATE_READY);
    }
}

// ── READY: นับถอยหลังก่อน OPERATION ──
void handleReady() {
    if (stateChanged) {
        stateChanged       = false;
        countdownRemaining = pref_readyCountdown;
        lastCountdownTick  = millis();
        display.showNumberDec(countdownRemaining, false, 4, 0);
        Serial.printf("[READY] Countdown %d s\n", countdownRemaining);
        String qr = "vend://" + DEVICE_ID + "/start?price=" + String(pref_price);
        sendBLE(qr);
    }

    unsigned long now = millis();
    if (now - lastCountdownTick >= 1000) {
        lastCountdownTick = now;
        countdownRemaining--;
        if (countdownRemaining > 0) {
            display.showNumberDec(countdownRemaining, false, 4, 0);
            Serial.printf("[READY] %d...\n", countdownRemaining);
        } else {
            display.showNumberDec(0, false, 4, 0);
            Serial.println(F("[READY] GO!"));
            transitionTo(STATE_OPERATION);
        }
    }
}

// ── OPERATION: รัน 6 Step อัตโนมัติ (ไม่มี topup) ──
void handleOperation() {
    if (stateChanged) {
        stateChanged  = false;
        currentStep   = 0;
        stepStartTime = millis();

        allRelaysOff(); allLEDsOff();

        // เริ่ม Step 1 → LD0
        setChannelRelay(0, true);
        setLED(0, true);
        display.showNumberDec(1, false, 4, 0);

        Serial.println(F("[OP] OPERATION started — 6 fixed steps"));
        Serial.printf( "[OP] Step 1/%d → LD0 ON | duration=%d ms\n",
            NUM_STEPS, pref_stepDurationMs[0]);
        addLog("STEP_1_START");
        sendPiStep(1);

        // Publish session_start ไป MQTT
        if (mqttClient.connected()) {
            JsonDocument doc;
            doc["deviceId"] = DEVICE_ID;
            doc["event"]    = "session_start";
            doc["price"]    = pref_price;
            doc["steps"]    = NUM_STEPS;
            JsonArray durs = doc["stepDurations"].to<JsonArray>();
            for (int s = 0; s < NUM_STEPS; s++) durs.add(pref_stepDurationMs[s]);
            String out; serializeJson(doc, out);
            mqttClient.publish(makeTopic("session").c_str(), out.c_str(), false);
        }
        return;
    }

    unsigned long elapsed = millis() - stepStartTime;

    // Step ปัจจุบันครบเวลา?
    if (elapsed >= (unsigned long)pref_stepDurationMs[currentStep]) {
        char logEv[24];
        snprintf(logEv, sizeof(logEv), "STEP_%d_DONE", currentStep + 1);
        addLog(logEv, (uint32_t)elapsed);

        setChannelRelay(currentStep, false);
        setLED(currentStep, false);

        currentStep++;

        if (currentStep >= NUM_STEPS) {
            // ครบ 6 step → SUMMARY
            allRelaysOff(); allLEDsOff();
            display.showNumberDec(0, false, 4, 0);
            Serial.println(F("[OP] All steps complete → SUMMARY"));
            transitionTo(STATE_SUMMARY);
            return;
        }

        // เริ่ม step ถัดไป
        stepStartTime = millis();
        setChannelRelay(currentStep, true);
        setLED(currentStep, true);
        display.showNumberDec(currentStep + 1, false, 4, 0);

        Serial.printf("[OP] Step %d/%d → LD%d ON | duration=%d ms\n",
            currentStep + 1, NUM_STEPS, currentStep, pref_stepDurationMs[currentStep]);
        snprintf(logEv, sizeof(logEv), "STEP_%d_START", currentStep + 1);
        addLog(logEv);
        sendPiStep(currentStep + 1);
    }
}

void doSummaryAndReset() {
    addLog("SESSION_END");
    printSummary();
    if (mqttClient.connected()) {
        JsonDocument doc;
        doc["deviceId"]  = DEVICE_ID;
        doc["event"]     = "session_end";
        doc["entries"]   = logCount;
        doc["price"]     = pref_price;
        doc["waterLevel"]= waterLevel;
        doc["waterDesc"] = waterLevelDesc(waterLevel);
        doc["sw0"]       = swState[0];
        doc["sw1"]       = swState[1];
        JsonArray wls = doc["wl"].to<JsonArray>();
        for (int i = 0; i < NUM_WL_LEVELS; i++) wls.add(wlState[i]);
        String out; serializeJson(doc, out);
        mqttClient.publish(makeTopic("session").c_str(), out.c_str(), false);
    }
    totalMoney = 0.0f;
    clearLog();
    transitionTo(STATE_IDLE);
}

// ── SUMMARY: รอกด START เพื่อ reset กลับ IDLE ──
void handleSummary() {
    if (stateChanged) {
        stateChanged = false;
        allRelaysOff();
        allLEDsOff();
        display.showNumberDec(0, false, 4, 0);
        Serial.printf("[SUMMARY] WaterLevel=%d/6 | SW0=%d SW1=%d\n",
            waterLevel, swState[0], swState[1]);
        String qr = "vend://" + DEVICE_ID + "/done";
        sendBLE(qr);
        Serial.println(F("[SUMMARY] Press START (or any BTN) to view log & reset"));
    }

    if (btnStart.pressEvent || btnAny.anyPressEvent) {
        addLog("SUMMARY_VIEW");
        doSummaryAndReset();
    }
}

void handleLocked() {
    if (stateChanged) {
        stateChanged = false;
        display.showNumberDec(1337, false, 4, 0);
        Serial.println(F("[LOCKED] Hardware error — check I2C wiring. Reset required."));
    }
    static unsigned long lastWarn = 0;
    if (millis()-lastWarn > 5000) {
        lastWarn = millis();
        Serial.println(F("[LOCKED] System halted."));
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    PiSerial.begin(115200, SERIAL_8N1, PI_UART_RX, PI_UART_TX);
    delay(600);
    Serial.println(F("\n╔═══════════════════════════════════════════════╗"));
    Serial.println(F("║  MACHINE FIRMWARE v3.0  |  ESP32-S3          ║"));
    Serial.println(F("║  Fixed-Price 6-Step | MQTT | BLE->QR (CYD)   ║"));
    Serial.println(F("╚═══════════════════════════════════════════════╝"));
    Serial.println(F("Type HELP for all commands\n"));

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
    readPi();

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
            for (int b = 7; b >= 0; b--) Serial.print((r3>>b)&1);
            Serial.printf("  WL(0x1F)=0b");
            for (int b = 7; b >= 0; b--) Serial.print((r4>>b)&1);
            Serial.printf("  WaterLv=%d | Step=%d/%d\n",
                waterLevel,
                currentState == STATE_OPERATION ? currentStep+1 : 0,
                NUM_STEPS);
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