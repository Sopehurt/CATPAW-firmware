#include "MoneyReader.h"

MoneyReader::MoneyReader(int pin, float pulseValue,
                         unsigned long minPulseMs,
                         unsigned long maxPulseMs,
                         unsigned long timeoutMs)
{
    _pin           = pin;
    _pulseValue    = pulseValue;
    _minPulseMs    = minPulseMs;
    _maxPulseMs    = maxPulseMs;
    _timeoutMs     = timeoutMs;
    _fallingTime   = 0;
    _pulseCount    = 0;
    _lastValidTime = 0;
    _enabled       = false;
}

void MoneyReader::enable() {
    _enabled = true;
}

void MoneyReader::disable() {
    _enabled       = false;
    _pulseCount    = 0;
    _fallingTime   = 0;
    _lastValidTime = 0;
}

// ── ISR: เรียกทั้งขาลงและขาขึ้น (CHANGE) ──
//
//  ขาลง (FALLING): บันทึกเวลา
//  ขาขึ้น (RISING): วัด pulse width
//    - กว้างน้อยกว่า _minPulseMs → noise/vibration → ทิ้ง
//    - กว้างมากกว่า _maxPulseMs  → stuck / error   → ทิ้ง
//    - อยู่ในช่วง min-max        → pulse valid      → นับ
void IRAM_ATTR MoneyReader::handleISR() {
    if (!_enabled) return;

    unsigned long now = millis();

    if (digitalRead(_pin) == LOW) {
        // ── Falling edge: pin เพิ่งลง LOW ──
        _fallingTime = now;
    } else {
        // ── Rising edge: pin เพิ่งขึ้น HIGH ──
        if (_fallingTime == 0) return; // ไม่มี falling ก่อนหน้า → ข้าม

        unsigned long width = now - _fallingTime;
        _fallingTime = 0; // reset

        if (width >= _minPulseMs && width <= _maxPulseMs) {
            // pulse กว้างพอดี → valid
            _pulseCount++;
            _lastValidTime = now;
        }
        // ถ้ากว้างน้อยกว่า min = noise → ไม่นับ (ทิ้ง)
        // ถ้ากว้างมากกว่า max = stuck  → ไม่นับ (ทิ้ง)
    }
}

// ── checkAmount: commit เมื่อ pulse หยุดนิ่ง _timeoutMs ──
float MoneyReader::checkAmount() {
    if (!_enabled)       return 0;
    if (_pulseCount == 0) return 0;
    if (millis() - _lastValidTime < _timeoutMs) return 0;

    int count   = _pulseCount;
    _pulseCount = 0;
    return (float)count * _pulseValue;
}