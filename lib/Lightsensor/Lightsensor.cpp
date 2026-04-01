#include "LightSensor.h"

LightSensor::LightSensor(uint8_t pin, uint16_t thresholdADC, unsigned long debounceMs)
    : _pin(pin),
      _threshold(thresholdADC),
      _debounceMs(debounceMs),
      _rawADC(4095),
      _lightDetected(false),
      _prevRaw(false),
      _lastChangeTime(0)
{}

void LightSensor::begin() {
    // ใช้ internal pull-up ของ ESP32
    // ถ้าต่อ pull-up ภายนอกแล้ว ให้เปลี่ยนเป็น INPUT
    pinMode(_pin, INPUT_PULLUP);
    Serial.printf("[LIGHT] Init pin=%d threshold=%d debounce=%lums\n",
        _pin, _threshold, _debounceMs);
}

void LightSensor::update() {
    // อ่าน ADC (0 = สว่างจ้า, 4095 = มืดสนิท เพราะ pull-up)
    _rawADC = (uint16_t)analogRead(_pin);

    // ตีความ: raw < threshold = มีแสง
    bool rawLight = (_rawADC < _threshold);

    // debounce: รอให้ state นิ่งก่อนยืนยัน
    unsigned long now = millis();
    if (rawLight != _prevRaw) {
        _lastChangeTime = now;
        _prevRaw = rawLight;
    }
    if ((now - _lastChangeTime) >= _debounceMs) {
        _lightDetected = _prevRaw;
    }
}