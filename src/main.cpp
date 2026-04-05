/**
 * ============================================================
 *  CAT PAW FIRMWARE v1.0  |  ESP32-S3 Dev Module
 *  Platform: PlatformIO + Arduino Framework
 *
 *  State Machine:
 *    0 - BOOT       : Hardware check
 *    1 - IDLE       : Wait for payment
 *    2 - PAYMENT_CHECK : Money received, wait START
 *    3 - READY      : Blink → enter OPERATION
 *    4 - OPERATION  : Run 6 sequential processes
 *    5 - SUMMARY    : Publish MQTT, reset
 *    7 - LOCKED     : Hardware error
 *
 *  Process order (CH 0-5):
 *    0 Dust Removal       1 Eliminate Bacteria
 *    2 Sterilize with UV  3 Ozone Purification
 *    4 Hot Air Drying     5 Perfume Treatment
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_mac.h>
#include "TM1637Display.h"
#include "ExpanderManager.h"
#include "MoneyReader.h"
#include "LightSensor.h"

// ============================================================
//  [SECTION A] USER DEFAULTS
// ============================================================
#define DEFAULT_PRICE            30     // [บาท] ราคาคงที่
#define DEFAULT_BOOT_DELAY_MS   1000

// ── ระยะเวลาแต่ละ process (วินาที) ──
#define DEFAULT_STEP_DUR_0   5   // Dust Removal
#define DEFAULT_STEP_DUR_1   5   // Eliminate Bacteria
#define DEFAULT_STEP_DUR_2   5   // Sterilize with UV
#define DEFAULT_STEP_DUR_3   5   // Ozone Purification
#define DEFAULT_STEP_DUR_4   5   // Hot Air Drying
#define DEFAULT_STEP_DUR_5   5   // Perfume Treatment

// ── Money readers ──
#define DEFAULT_BANK_PULSE_VALUE   10.0f
#define DEFAULT_COIN_PULSE_VALUE    1.0f
#define BANK_MIN_PULSE_MS     20
#define BANK_MAX_PULSE_MS    200
#define BANK_PULSE_TIMEOUT_MS 600
#define COIN_MIN_PULSE_MS     15
#define COIN_MAX_PULSE_MS    150
#define COIN_PULSE_TIMEOUT_MS 400

#define DEFAULT_TELEMETRY_SEC  30

// ============================================================
//  [SECTION B] PIN CONFIG
// ============================================================
#define PIN_BANK_PULSE   5
#define PIN_COIN_PULSE   12
#define PI_UART_TX      13
#define PI_UART_RX      14
#define I2C_SDA          8
#define I2C_SCL          9
#define DISP_CLK        15
#define DISP_DIO        16
#define PIN_LIGHT_SENSOR  1
#define LIGHT_THRESHOLD_ADC 2000
#define LIGHT_DEBOUNCE_MS   200

// ============================================================
//  [SECTION C] EXPANDER MAPPING
//  0x18 LOAD (OUTPUT) — Relay  CH0-CH5
//  0x1C LED  (OUTPUT) — LED    CH0-CH5
//  0x1E BTN  (INPUT)  — ปุ่ม  (ไม่ใช้ใน PAW แต่ไว้ debug)
//  0x1F WL   (INPUT)  — Water Level
// ============================================================
#define NUM_STEPS      6
#define NUM_WL_LEVELS  6
#define NUM_SWITCHES   2

static const uint8_t LD_PIN[6]  = {1, 0, 2, 3, 4, 5};
static const uint8_t LED_PIN[6] = {7, 6, 5, 4, 3, 2};
static const uint8_t WL_PIN[6]  = {5, 4, 3, 2, 0, 1};
#define SW0_PIN  6
#define SW1_PIN  7

// ============================================================
//  [SECTION D] NETWORK
// ============================================================
#define WIFI_SSID_DEFAULT  "SR803_5G"
#define WIFI_PASS_DEFAULT  "80323SM5F"
#define MQTT_SERVER        "147.50.255.120"
#define MQTT_PORT          9359
#define MQTT_USER          "esp32"
#define MQTT_PASS          "Taweesak@5050"
#define MQTT_PREFIX        "machine/paw"
#define BLE_NOT_USED       // PAW ไม่ใช้ BLE

// ============================================================
//  [SECTION E] CONSTANTS
// ============================================================
#define MAX_LOG_ENTRIES  32
#define BTN_POLL_MS      20
#define BTN_DEBOUNCE_MS  60
#define MQTT_DEFER_IDLE_MS (30UL * 1000UL)

// ── Process names ──
const char* STEP_NAMES[NUM_STEPS] = {
    "Dust Removal",
    "Eliminate Bacteria",
    "Sterilize with UV",
    "Ozone Purification",
    "Hot Air Drying",
    "Perfume Treatment"
};

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
//  GLOBALS
// ============================================================
TM1637Display   display(DISP_CLK, DISP_DIO);
ExpanderManager expander1(0x18);
ExpanderManager expander2(0x1C);
ExpanderManager expander3(0x1E);
ExpanderManager expander4(0x1F);
MoneyReader     bankReader(PIN_BANK_PULSE, DEFAULT_BANK_PULSE_VALUE,
                           BANK_MIN_PULSE_MS, BANK_MAX_PULSE_MS, BANK_PULSE_TIMEOUT_MS);
MoneyReader     coinReader(PIN_COIN_PULSE, DEFAULT_COIN_PULSE_VALUE,
                           COIN_MIN_PULSE_MS, COIN_MAX_PULSE_MS, COIN_PULSE_TIMEOUT_MS);
LightSensor     lightSensor(PIN_LIGHT_SENSOR, LIGHT_THRESHOLD_ADC, LIGHT_DEBOUNCE_MS);
Preferences     prefs;
WiFiClient      espClient;
PubSubClient    mqttClient(espClient);
HardwareSerial  PiSerial(2);

SystemState   currentState    = STATE_BOOT;
bool          stateChanged    = true;
unsigned long stateEnterTime  = 0;

float         totalMoney      = 0.0f;
unsigned long lastMoneyCheck  = 0;

// ── Operation state ──
int           currentStep     = 0;       // 0-5
unsigned long stepStartTime   = 0;
bool          operationDone   = false;
unsigned long lastStepTick    = 0;       // ส่ง paw_step ทุก 1 วิ

// ── Water level ──
bool     wlState[NUM_WL_LEVELS] = {};
bool     swState[NUM_SWITCHES]  = {};
int      waterLevel             = 0;
int      prevWaterLevel         = -1;

// ── Log ──
LogEntry sessionLog[MAX_LOG_ENTRIES];
int      logCount = 0;

// ── NVS ──
int     pref_price;
int     pref_stepDur[NUM_STEPS];
int     pref_bootDelayMs;
int     pref_telemetrySec;
String  wifi_ssid;
String  wifi_pass;
String  DEVICE_ID;

// ── Debug ──
bool    dbg_pressStart = false;
bool    dbg_verbose    = false;
bool    _mqttQueueReady = false;

// ── MQTT deferred queue ──
#define MQTT_QUEUE_SLOTS  4
#define MQTT_JSON_MAX    512
struct MqttDeferredCmd { bool used; char json[MQTT_JSON_MAX]; };
MqttDeferredCmd mqttQueue[MQTT_QUEUE_SLOTS] = {};
bool            _applyingQueue = false;

// ============================================================
//  ISR
// ============================================================
void IRAM_ATTR bankISR() { bankReader.handleISR(); }
void IRAM_ATTR coinISR() { coinReader.handleISR(); }

// ============================================================
//  MQTT QUEUE
// ============================================================
void mqttEnqueue(const String& json) {
    for (int i = 0; i < MQTT_QUEUE_SLOTS; i++) {
        if (!mqttQueue[i].used) {
            strncpy(mqttQueue[i].json, json.c_str(), MQTT_JSON_MAX-1);
            mqttQueue[i].used = true;
            Serial.println(F("[MQTT] Queued"));
            return;
        }
    }
    memmove(&mqttQueue[0], &mqttQueue[1], sizeof(MqttDeferredCmd)*(MQTT_QUEUE_SLOTS-1));
    strncpy(mqttQueue[MQTT_QUEUE_SLOTS-1].json, json.c_str(), MQTT_JSON_MAX-1);
    mqttQueue[MQTT_QUEUE_SLOTS-1].used = true;
}

void mqttCallback(char* topic, byte* payload, unsigned int len);  // forward

void mqttProcessQueue() {
    bool hasCmd = false;
    for (int i = 0; i < MQTT_QUEUE_SLOTS; i++) if (mqttQueue[i].used) { hasCmd=true; break; }
    if (!hasCmd) return;
    Serial.println(F("[MQTT] Applying deferred..."));
    _applyingQueue = true;
    for (int i = 0; i < MQTT_QUEUE_SLOTS; i++) {
        if (!mqttQueue[i].used) continue;
        mqttCallback(nullptr, (byte*)mqttQueue[i].json, strlen(mqttQueue[i].json));
        mqttQueue[i].used = false;
    }
    _applyingQueue = false;
}

// ============================================================
//  RELAY / LED
// ============================================================
void setStepRelay(uint8_t step, bool on) {
    if (step >= NUM_STEPS) return;
    expander1.digitalWrite(LD_PIN[step], on ? HIGH : LOW);
    Serial.printf("[LOAD] Step%d (IO%d) %s\n", step, LD_PIN[step], on?"ON":"OFF");
}
void allRelaysOff() {
    for (uint8_t i = 0; i < NUM_STEPS; i++) {
        expander1.digitalWrite(LD_PIN[i], LOW);
    }
    Serial.println(F("[LOAD] All OFF"));
}
void setStepLED(uint8_t step, bool on) {
    if (step >= NUM_STEPS) return;
    expander2.digitalWrite(LED_PIN[step], on ? HIGH : LOW);
}
void allLEDsOff() {
    for (uint8_t i = 0; i < NUM_STEPS; i++) expander2.digitalWrite(LED_PIN[i], LOW);
}

// ============================================================
//  WATER LEVEL
// ============================================================
void updateWaterLevel() {
    Wire.beginTransmission(0x1F); Wire.write(0x00); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x1F, (uint8_t)1);
    uint8_t raw = Wire.available() ? Wire.read() : 0xFF;
    for (int i = 0; i < NUM_WL_LEVELS; i++) wlState[i] = !(raw & (1 << WL_PIN[i]));
    swState[0] = !(raw & (1 << SW0_PIN));
    swState[1] = !(raw & (1 << SW1_PIN));
    int level = 0;
    for (int i = 0; i < NUM_WL_LEVELS; i++) if (wlState[i]) level = i+1;
    waterLevel = level;
    if (waterLevel != prevWaterLevel) {
        prevWaterLevel = waterLevel;
        Serial.printf("[WL] Level=%d/6\n", waterLevel);
    }
}

const char* tankDesc(int hi, int lo) {
    if (wlState[hi]) return ">50%";
    if (wlState[lo]) return ">25%";
    return "<25%";
}

// ============================================================
//  LOG
// ============================================================
void addLog(const char* event, uint32_t dur = 0) {
    if (logCount < MAX_LOG_ENTRIES) {
        strncpy(sessionLog[logCount].event, event, sizeof(sessionLog[0].event)-1);
        sessionLog[logCount].timestamp_ms = millis();
        sessionLog[logCount].money        = (int16_t)totalMoney;
        sessionLog[logCount].duration_ms  = dur;
        logCount++;
    }
    Serial.printf("[LOG][%8lu] %-22s ฿%d %lums\n",
        millis(), event, (int)totalMoney, (unsigned long)dur);
}
void clearLog() { logCount=0; memset(sessionLog,0,sizeof(sessionLog)); }
void printSummary() {
    Serial.println(F("╔══════════════════════════════════════════╗"));
    Serial.println(F("║          CAT PAW SESSION SUMMARY         ║"));
    Serial.println(F("╠══════════════════════════════════════════╣"));
    Serial.printf( "║  Entries : %-29d║\n", logCount);
    Serial.println(F("╠══════════════════════════════════════════╣"));
    for (int i=0;i<logCount;i++)
        Serial.printf("║[%02d][%8lu] %-18s ฿%-4d %lums\n",
            i+1, (unsigned long)sessionLog[i].timestamp_ms,
            sessionLog[i].event, (int)sessionLog[i].money,
            (unsigned long)sessionLog[i].duration_ms);
    Serial.println(F("╠══════════════════════════════════════════╣"));
    Serial.printf( "║  Water  T1:%-5s T2:%-5s T3:%-5s     ║\n",
        tankDesc(0,1), tankDesc(2,3), tankDesc(4,5));
    Serial.println(F("╚══════════════════════════════════════════╝"));
}

// ============================================================
//  NVS
// ============================================================
void loadWiFiCredentials() {
    prefs.begin("catpaw", true);
    wifi_ssid = prefs.getString("wifiSsid", WIFI_SSID_DEFAULT);
    wifi_pass = prefs.getString("wifiPass", WIFI_PASS_DEFAULT);
    prefs.end();
    Serial.printf("[WiFi] SSID=%s\n", wifi_ssid.c_str());
}
void saveWiFiCredentials(const String& s, const String& p) {
    prefs.begin("catpaw", false);
    prefs.putString("wifiSsid", s); prefs.putString("wifiPass", p);
    prefs.end();
    wifi_ssid=s; wifi_pass=p;
    Serial.printf("[WiFi] Saved SSID=%s\n", s.c_str());
}

void loadPreferences() {
    const int defDur[NUM_STEPS] = {
        DEFAULT_STEP_DUR_0, DEFAULT_STEP_DUR_1, DEFAULT_STEP_DUR_2,
        DEFAULT_STEP_DUR_3, DEFAULT_STEP_DUR_4, DEFAULT_STEP_DUR_5
    };
    prefs.begin("catpaw", false); prefs.end();
    prefs.begin("catpaw", true);
    pref_price        = prefs.getInt("price",       DEFAULT_PRICE);
    pref_bootDelayMs  = prefs.getInt("bootDelay",   DEFAULT_BOOT_DELAY_MS);
    pref_telemetrySec = prefs.getInt("telemetrySec",DEFAULT_TELEMETRY_SEC);
    for (int i=0;i<NUM_STEPS;i++) {
        char key[8]; snprintf(key,sizeof(key),"dur%d",i);
        pref_stepDur[i] = prefs.getInt(key, defDur[i]);
    }
    prefs.end();
    Serial.printf("[PREFS] price=%d  steps=[", pref_price);
    for (int i=0;i<NUM_STEPS;i++) Serial.printf("%d%s", pref_stepDur[i], i<5?",":"");
    Serial.println("]s");
}

void savePreferences() {
    prefs.begin("catpaw", false);
    prefs.putInt("price",       pref_price);
    prefs.putInt("bootDelay",   pref_bootDelayMs);
    prefs.putInt("telemetrySec",pref_telemetrySec);
    for (int i=0;i<NUM_STEPS;i++) {
        char key[8]; snprintf(key,sizeof(key),"dur%d",i);
        prefs.putInt(key, pref_stepDur[i]);
    }
    prefs.end();
}

// ============================================================
//  DEVICE ID + WiFi
// ============================================================
void createDeviceID() {
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char id[13];
    snprintf(id,sizeof(id),"%02X%02X%02X%02X%02X%02X",
             mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    DEVICE_ID = String("paw-") + id;
    Serial.println("[NET] Device: " + DEVICE_ID);
}

void connectWiFi() {
    if (WiFi.status()==WL_CONNECTED) return;
    Serial.printf("[WiFi] Connecting %s...\n", wifi_ssid.c_str());
    WiFi.disconnect(true); WiFi.mode(WIFI_STA);
    WiFi.setSleep(true); WiFi.setAutoReconnect(true);
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
    for (int i=0; i<15 && WiFi.status()!=WL_CONNECTED; i++) {
        delay(200); Serial.print(".");
    }
    if (WiFi.status()==WL_CONNECTED)
        Serial.println("\n[WiFi] OK: " + WiFi.localIP().toString());
    else
        Serial.println(F("\n[WiFi] Timeout"));
}

// ============================================================
//  MQTT
// ============================================================
String makeTopic(const String& sfx) {
    return String(MQTT_PREFIX) + "/" + DEVICE_ID + "/" + sfx;
}
void mqttPublishStatus(const String& s) {
    if (!mqttClient.connected()) return;
    JsonDocument d; d["deviceId"]=DEVICE_ID; d["status"]=s;
    String out; serializeJson(d,out);
    mqttClient.publish(makeTopic("status").c_str(), out.c_str(), false);
}
void mqttPublishConfig(const char* action) {
    if (!mqttClient.connected()) return;
    JsonDocument d;
    d["deviceId"]=DEVICE_ID; d["action"]=action; d["price"]=pref_price;
    JsonArray arr = d["stepDur"].to<JsonArray>();
    for (int i=0;i<NUM_STEPS;i++) arr.add(pref_stepDur[i]);
    String out; serializeJson(d,out);
    mqttClient.publish(makeTopic("config").c_str(), out.c_str(), false);
    Serial.println("[MQTT] Config: " + out);
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
    String msg;
    for (unsigned int i=0;i<len;i++) msg+=(char)payload[i];
    Serial.println("[MQTT] << " + msg);
    JsonDocument doc;
    if (deserializeJson(doc, msg)) return;
    String cmd = doc["cmd"].as<String>();

    bool isDeferrable = (cmd=="set-price"||cmd=="set-step-dur"||cmd=="set-params"||cmd=="rollback");
    bool sessionActive = (currentState!=STATE_IDLE && currentState!=STATE_BOOT);
    if (isDeferrable && sessionActive && !_applyingQueue) {
        mqttEnqueue(msg); return;
    }

    if (cmd == "reboot") {
        mqttPublishStatus("rebooting"); delay(500); ESP.restart();
    }
    else if (cmd == "set-price") {
        int v = doc["value"].as<int>();
        if (v>0 && v<=9999) {
            pref_price=v; savePreferences();
            Serial.printf("[SET] price=%d\n", v);
            mqttPublishConfig("applied");
        }
    }
    else if (cmd == "set-step-dur") {
        int step = doc["step"].as<int>();
        int secs = doc["value"].as<int>();
        if (step>=0 && step<NUM_STEPS && secs>0 && secs<=3600) {
            pref_stepDur[step]=secs; savePreferences();
            Serial.printf("[SET] dur%d=%ds\n", step, secs);
            mqttPublishConfig("applied");
        }
    }
    else if (cmd == "set-params") {
        bool changed = false;
        if (!doc["price"].isNull()) {
            int v=doc["price"].as<int>();
            if (v>0&&v<=9999) { pref_price=v; changed=true; }
        }
        if (doc["stepDur"].is<JsonArray>()) {
            JsonArray arr=doc["stepDur"].as<JsonArray>(); int idx=0;
            for (JsonVariant v:arr) {
                if (idx>=NUM_STEPS) break;
                int s=v.as<int>();
                if (s>0&&s<=3600) { pref_stepDur[idx]=s; changed=true; }
                idx++;
            }
        }
        if (changed) { savePreferences(); mqttPublishConfig("applied"); }
    }
    else if (cmd == "get-config") { mqttPublishConfig("current"); }
    else { Serial.println("[MQTT] Unknown: " + cmd); }
}

void connectMQTT() {
    if (mqttClient.connected()) return;
    static unsigned long lastAttempt = 0;
    if (millis()-lastAttempt < 5000) return;
    lastAttempt = millis();
    Serial.print(F("[MQTT] Connecting..."));
    String wt=makeTopic("status");
    String wp="{\"deviceId\":\""+DEVICE_ID+"\",\"status\":\"offline\"}";
    if (mqttClient.connect(DEVICE_ID.c_str(), MQTT_USER, MQTT_PASS,
                           wt.c_str(), 1, false, wp.c_str())) {
        Serial.println(F(" OK"));
        mqttPublishStatus("online");
        mqttClient.subscribe(makeTopic("cmd").c_str());
    } else {
        Serial.printf(" FAIL rc=%d\n", mqttClient.state());
    }
}

void handleTelemetry() {
    if (!mqttClient.connected()) return;
    static unsigned long lastSend = 0;
    if (millis()-lastSend < (unsigned long)pref_telemetrySec*1000UL) return;
    lastSend = millis();
    JsonDocument d;
    d["deviceId"]=DEVICE_ID; d["state"]=stateNames[currentState];
    d["money"]=(int)totalMoney; d["price"]=pref_price;
    d["uptime_s"]=millis()/1000;
    d["waterLevel"]=waterLevel; d["sw0"]=swState[0]; d["sw1"]=swState[1];
    if (currentState==STATE_OPERATION) {
        d["step"]=currentStep+1;
        d["stepName"]=STEP_NAMES[currentStep];
    }
    JsonArray wls=d["wl"].to<JsonArray>();
    for (int i=0;i<NUM_WL_LEVELS;i++) wls.add(wlState[i]);
    String out; serializeJson(d,out);
    mqttClient.publish(makeTopic("telemetry").c_str(), out.c_str(), false);
}

// ============================================================
//  Pi UART
// ============================================================
void sendPi(JsonDocument& doc) {
    String out; serializeJson(doc,out); out+="\n";
    PiSerial.print(out);
    Serial.println("[PI<<] " + out);
}
void sendPiState(const char* state) {
    JsonDocument d; d["evt"]="state"; d["state"]=state; sendPi(d);
}
void sendPiMoney() {
    JsonDocument d; d["evt"]="money"; d["balance"]=(int)totalMoney; d["min"]=pref_price; sendPi(d);
}
void sendPiPrice() {
    JsonDocument d; d["evt"]="paw_price"; d["price"]=pref_price; sendPi(d);
}
void sendPiStep(int step, int stepLeftSec, int totalLeftSec) {
    JsonDocument d;
    d["evt"]       = "paw_step";
    d["step"]      = step+1;
    d["total"]     = NUM_STEPS;
    d["name"]      = STEP_NAMES[step];
    d["step_left"] = stepLeftSec;
    d["total_left"]= totalLeftSec;
    sendPi(d);
}

// ── forward decls ──
void doSummaryAndReset();
void transitionTo(SystemState next);  // forward declaration

void readPi() {
    while (PiSerial.available()) {
        String line = PiSerial.readStringUntil('\n');
        line.trim(); if (line.isEmpty()) continue;
        Serial.println("[PI>>] " + line);
        JsonDocument doc;
        if (deserializeJson(doc, line)) continue;
        String cmd = doc["cmd"].as<String>();

        if (cmd == "start") {
            if (currentState == STATE_PAYMENT_CHECK) {
                bankReader.disable(); coinReader.disable();
                addLog("INITIALIZE_PI");
                transitionTo(STATE_READY);
            }
        }
        else if (cmd == "finish") {
            if (currentState == STATE_OPERATION || currentState == STATE_SUMMARY) {
                allRelaysOff(); allLEDsOff(); doSummaryAndReset();
            }
        }
        else if (cmd == "wifi_config") {
            String ssid=doc["ssid"].as<String>(); String pass=doc["pass"].as<String>();
            if (ssid.length()>0) {
                saveWiFiCredentials(ssid, pass);
                WiFi.disconnect(true); delay(100); WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
                JsonDocument r;
                r["evt"]="wifi_result"; r["ok"]=(WiFi.status()==WL_CONNECTED);
                r["ip"]=WiFi.localIP().toString(); r["ssid"]=ssid;
                sendPi(r);
            }
        }
        else if (cmd == "pay_qr") {
            // ส่ง QR link สำหรับราคาคงที่
            int amt = doc["amount"].as<int>();
            if (amt <= 0) amt = pref_price;
            String link = "https://catpaw.example.com/pay?id="+DEVICE_ID+"&amt="+String(amt);
            JsonDocument q; q["evt"]="qr_link"; q["url"]=link; sendPi(q);
        }
        else if (cmd == "get_status") {
            sendPiState(stateNames[currentState]);
            sendPiMoney(); sendPiPrice();
        }
        else if (cmd == "ping") {
            JsonDocument p; p["evt"]="pong"; p["state"]=stateNames[currentState];
            p["balance"]=(int)totalMoney; sendPi(p);
        }
    }
}

// ============================================================
//  TRANSITION
// ============================================================
void transitionTo(SystemState next) {
    Serial.printf("[STATE] %s -> %s\n", stateNames[currentState], stateNames[next]);
    if (next!=STATE_READY && next!=STATE_BOOT) addLog(stateNames[next]);
    currentState=next; stateChanged=true; stateEnterTime=millis();
    sendPiState(stateNames[next]);
}

// ============================================================
//  MONEY INPUT
// ============================================================
void checkMoneyInput() {
    if (millis()-lastMoneyCheck < 50) return;
    lastMoneyCheck = millis();
    float bk=bankReader.checkAmount(), co=coinReader.checkAmount();
    if (bk>0) {
        totalMoney+=bk;
        display.showNumberDec((int)totalMoney,false,4,0);
        addLog("INSERT_BANK",(uint32_t)bk); sendPiMoney();
    }
    if (co>0) {
        totalMoney+=co;
        display.showNumberDec((int)totalMoney,false,4,0);
        addLog("INSERT_COIN",(uint32_t)co); sendPiMoney();
    }
    // pending preview
    static int lastPreview=-1;
    float pending=bankReader.getPendingAmount()+coinReader.getPendingAmount();
    if (pending>0) {
        int preview=(int)(totalMoney+pending);
        display.showNumberDec(preview,false,4,0);
        if (preview!=lastPreview) {
            lastPreview=preview;
            JsonDocument p; p["evt"]="money"; p["balance"]=preview; p["min"]=pref_price;
            sendPi(p);
        }
    } else { lastPreview=-1; }
}

// ============================================================
//  SUMMARY
// ============================================================
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
        doc["sw0"]       = swState[0]; doc["sw1"]=swState[1];
        doc["T1"]        = tankDesc(0,1); doc["T2"]=tankDesc(2,3); doc["T3"]=tankDesc(4,5);
        JsonArray wls=doc["wl"].to<JsonArray>();
        for (int i=0;i<NUM_WL_LEVELS;i++) wls.add(wlState[i]);
        String out; serializeJson(doc,out);
        mqttClient.publish(makeTopic("session").c_str(), out.c_str(), false);
    }
    totalMoney=0.0f; clearLog(); transitionTo(STATE_IDLE);
}

// ============================================================
//  STATE HANDLERS
// ============================================================
void handleBoot() {
    if (!stateChanged) return;
    stateChanged = false;
    Serial.println(F("[BOOT] CAT PAW Hardware check..."));
    uint8_t seg8[]={0x7F,0x7F,0x7F,0x7F}; display.setSegments(seg8); delay(400);

    bool ok[4]={expander1.begin(),expander2.begin(),expander3.begin(),expander4.begin()};
    uint8_t addr[4]={0x18,0x1C,0x1E,0x1F};
    for (int i=0;i<4;i++)
        Serial.printf("[BOOT] EXP 0x%02X : %s\n", addr[i], ok[i]?"OK":"FAIL");
    if (!ok[0]||!ok[1]||!ok[2]||!ok[3]) { transitionTo(STATE_LOCKED); return; }

    for (uint8_t p=0;p<8;p++) { expander1.digitalWrite(p,LOW); expander2.digitalWrite(p,LOW); }
    for (uint8_t p=0;p<8;p++) {
        expander1.pinMode(p,OUTPUT); expander2.pinMode(p,OUTPUT);
        expander3.pinMode(p,INPUT);  expander4.pinMode(p,INPUT);
    }

    pinMode(PIN_BANK_PULSE,INPUT_PULLUP); pinMode(PIN_COIN_PULSE,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_BANK_PULSE), bankISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_COIN_PULSE), coinISR, CHANGE);
    lightSensor.begin();

    display.showNumberDec(pref_price,false,4,0);
    Serial.printf("[BOOT] OK — price=%d delay=%dms\n", pref_price, pref_bootDelayMs);
    delay(pref_bootDelayMs);
    transitionTo(STATE_IDLE);
}

void handleIdle() {
    if (stateChanged) {
        stateChanged=false; totalMoney=0.0f;
        display.showNumberDec(0,false,4,0);
        bankReader.enable(); coinReader.enable();
        Serial.printf("[IDLE] Waiting payment >= %d\n", pref_price);
        sendPiMoney(); sendPiPrice();
        _mqttQueueReady=false;
    }
    if (!_mqttQueueReady && millis()-stateEnterTime > MQTT_DEFER_IDLE_MS) {
        _mqttQueueReady=true; mqttProcessQueue();
    }
    // Light sensor
    lightSensor.update();
    // Water level
    static unsigned long lastWL=0;
    if (millis()-lastWL>=200) { lastWL=millis(); updateWaterLevel(); }

    checkMoneyInput();
    if (totalMoney >= (float)pref_price) transitionTo(STATE_PAYMENT_CHECK);
}

void handlePaymentCheck() {
    if (stateChanged) {
        stateChanged=false;
        Serial.printf("[PAY] %.0f >= %d — press START\n", totalMoney, pref_price);
        display.showNumberDec((int)totalMoney,false,4,0);
    }
    checkMoneyInput();
    if (dbg_pressStart) { dbg_pressStart=false;
        bankReader.disable(); coinReader.disable();
        addLog("INITIALIZE"); transitionTo(STATE_READY); }
}

void handleReady() {
    if (stateChanged) {
        stateChanged=false;
        display.clear(); delay(200);
        display.showNumberDec((int)totalMoney,false,4,0); delay(200);
        transitionTo(STATE_OPERATION);
    }
}

void handleOperation() {
    if (stateChanged) {
        stateChanged=false;
        currentStep=0; stepStartTime=millis(); lastStepTick=0; operationDone=false;
        allLEDsOff(); allRelaysOff();
        setStepRelay(0,true); setStepLED(0,true);
        Serial.printf("[OP] Step 1/%d: %s (%ds)\n", NUM_STEPS, STEP_NAMES[0], pref_stepDur[0]);
        addLog(STEP_NAMES[0]);
    }

    unsigned long now = millis();

    // ── ส่ง paw_step ไป Pi ทุก 1 วิ ──
    if (now-lastStepTick >= 1000) {
        lastStepTick = now;
        unsigned long stepElapsed = (now-stepStartTime)/1000;
        int stepLeft  = max(0, pref_stepDur[currentStep] - (int)stepElapsed);
        // คำนวณ totalLeft
        int totalLeft = stepLeft;
        for (int i=currentStep+1; i<NUM_STEPS; i++) totalLeft+=pref_stepDur[i];
        sendPiStep(currentStep, stepLeft, totalLeft);
        display.showNumberDec(totalLeft, false, 4, 0);
    }

    // ── ตรวจว่า step ปัจจุบันหมดเวลาหรือยัง ──
    unsigned long stepElapsedMs = now - stepStartTime;
    if (stepElapsedMs >= (unsigned long)pref_stepDur[currentStep] * 1000UL) {
        // step เสร็จ
        char ev[28]; snprintf(ev,sizeof(ev),"%s_DONE",STEP_NAMES[currentStep]);
        addLog(ev, stepElapsedMs);
        setStepRelay(currentStep,false); setStepLED(currentStep,false);

        currentStep++;
        if (currentStep >= NUM_STEPS) {
            // ทุก step เสร็จ
            Serial.println(F("[OP] All steps complete → SUMMARY"));
            allRelaysOff(); allLEDsOff();
            display.showNumberDec(0,false,4,0);
            doSummaryAndReset();
            return;
        }
        // step ถัดไป
        stepStartTime = now;
        setStepRelay(currentStep,true); setStepLED(currentStep,true);
        Serial.printf("[OP] Step %d/%d: %s (%ds)\n",
            currentStep+1, NUM_STEPS, STEP_NAMES[currentStep], pref_stepDur[currentStep]);
        addLog(STEP_NAMES[currentStep]);
        // ส่ง step ใหม่ทันที
        int totalLeft=0; for (int i=currentStep;i<NUM_STEPS;i++) totalLeft+=pref_stepDur[i];
        sendPiStep(currentStep, pref_stepDur[currentStep], totalLeft);
    }
}

void handleSummary() {
    if (stateChanged) {
        stateChanged=false;
        allRelaysOff(); allLEDsOff();
        display.showNumberDec(0,false,4,0);
        Serial.println(F("[SUMMARY] Waiting finish cmd from Pi"));
    }
}

void handleLocked() {
    if (stateChanged) { stateChanged=false; display.showNumberDec(1337,false,4,0); }
    static unsigned long lw=0;
    if (millis()-lw>5000) { lw=millis(); Serial.println(F("[LOCKED]")); }
}

// ============================================================
//  SERIAL DEBUG
// ============================================================
void handleSerial() {
    if (!Serial.available()) return;
    String cmd=Serial.readStringUntil('\n'); cmd.trim();
    String up=cmd; up.toUpperCase();
    if      (up=="START")        { dbg_pressStart=true; }
    else if (up.startsWith("ADD ")) {
        float v=cmd.substring(4).toFloat();
        if (v>0) { totalMoney+=v; display.showNumberDec((int)totalMoney,false,4,0);
                   Serial.printf("[CMD] +%.0f\n",v); }
    }
    else if (up=="STATUS") {
        Serial.printf("  State=%s Money=%.0f Price=%d WiFi=%s MQTT=%s\n",
            stateNames[currentState], totalMoney, pref_price,
            WiFi.status()==WL_CONNECTED?WiFi.localIP().toString().c_str():"offline",
            mqttClient.connected()?"OK":"offline");
        Serial.printf("  WaterLevel=%d/6  Step=%d/%d\n",
            waterLevel, currentStep+1, NUM_STEPS);
        Serial.print("  StepDurs=[");
        for (int i=0;i<NUM_STEPS;i++) Serial.printf("%d%s",pref_stepDur[i],i<5?",":"");
        Serial.println("]s");
    }
    else if (up=="LOG")          { printSummary(); }
    else if (up.startsWith("SET PRICE ")) {
        int v=cmd.substring(10).toInt();
        if (v>0) { pref_price=v; savePreferences(); Serial.printf("[SET] price=%d\n",v); }
    }
    else if (up=="VERBOSE ON")  { dbg_verbose=true; }
    else if (up=="VERBOSE OFF") { dbg_verbose=false; }
    else if (up.startsWith("GOTO ")) {
        int st=cmd.substring(5).toInt();
        if (st>=0&&st<=7) transitionTo((SystemState)st);
    }
    else if (up=="HELP") {
        Serial.println(F("START / ADD <n> / SET PRICE <n> / STATUS / LOG / GOTO <n> / VERBOSE ON/OFF"));
    }
    else { Serial.println(F("[CMD] Type HELP")); }
}

// ============================================================
//  BUTTON READER (minimal — just for physical START override)
// ============================================================
void updateButtons() {
    // PAW ใช้ Pi START command เป็นหลัก
    // physical button เพิ่มเติมทีหลังถ้าต้องการ
}

// ============================================================
//  SETUP + LOOP
// ============================================================
void setup() {
    Serial.begin(115200);
    PiSerial.begin(115200, SERIAL_8N1, PI_UART_RX, PI_UART_TX);
    delay(600);
    Serial.println(F("\n╔═══════════════════════════════════════╗"));
    Serial.println(F("║  CAT PAW FIRMWARE v1.0 | ESP32-S3    ║"));
    Serial.println(F("╚═══════════════════════════════════════╝"));

    loadPreferences();
    loadWiFiCredentials();

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000UL);  // 4x faster I2C
    display.setBrightness(7);
    display.showNumberDec(8888,false,4,0);

    createDeviceID();
    // WiFi: begin async ไม่ block boot
    WiFi.mode(WIFI_STA); WiFi.setSleep(true); WiFi.setAutoReconnect(true);
    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
    Serial.printf("[WiFi] Begin async: %s\n", wifi_ssid.c_str());

    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setKeepAlive(60);
    mqttClient.setSocketTimeout(10);
    mqttClient.setBufferSize(2048);
    connectMQTT();

    currentState=STATE_BOOT; stateChanged=true; stateEnterTime=millis();
}

void loop() {
    // ── Network (non-blocking, ตรวจทุก 30s) ──────────────────
    static unsigned long lastNetCheck = 0;
    unsigned long now = millis();
    if (now - lastNetCheck >= 30000UL) {
        lastNetCheck = now;
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
        }
    }
    if (!mqttClient.connected()) connectMQTT();
    mqttClient.loop();

    handleSerial();
    updateButtons();
    handleTelemetry();
    readPi();

    switch (currentState) {
        case STATE_BOOT:          handleBoot();          break;
        case STATE_IDLE:          handleIdle();          break;
        case STATE_PAYMENT_CHECK: handlePaymentCheck();  break;
        case STATE_READY:         handleReady();         break;
        case STATE_OPERATION:     handleOperation();     break;
        case STATE_SUMMARY:       handleSummary();       break;
        case STATE_LOCKED:        handleLocked();        break;
        default: transitionTo(STATE_LOCKED);
    }
}