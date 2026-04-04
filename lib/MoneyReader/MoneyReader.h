#ifndef MONEY_READER_H
#define MONEY_READER_H

#include <Arduino.h>

// ============================================================
//  MoneyReader v2 — Pulse-width validation
//
//  หลักการ: วัดความกว้างของ pulse (เวลาที่ pin อยู่ LOW)
//    - Noise/vibration spike สั้น < 5ms  → ทิ้ง
//    - Pulse จริงจากเครื่องรับ: LOW 20-80ms  → นับ
//
//  ต้อง attachInterrupt ด้วย CHANGE (ไม่ใช่ FALLING)
//  เพื่อให้วัด rising edge ได้ด้วย
//
//  ค่าที่ปรับได้ใน main.cpp SECTION A:
//    BANK_MIN_PULSE_MS   : กว้างต่ำสุดที่ถือว่า valid  (แนะนำ 20)
//    BANK_MAX_PULSE_MS   : กว้างสูงสุด กัน stuck pin  (แนะนำ 200)
//    BANK_PULSE_TIMEOUT_MS : รอหลัง pulse สุดท้าย     (แนะนำ 600)
// ============================================================
class MoneyReader {
private:
    volatile unsigned long _fallingTime;   // timestamp ตอน pin ลง LOW
    volatile int           _pulseCount;    // pulse ที่ผ่านเกณฑ์แล้ว
    volatile unsigned long _lastValidTime; // timestamp pulse สุดท้ายที่ valid
    int                    _pin;
    float                  _pulseValue;
    unsigned long          _minPulseMs;
    unsigned long          _maxPulseMs;
    unsigned long          _timeoutMs;
    bool                   _enabled;

public:
    MoneyReader(int pin, float pulseValue,
                unsigned long minPulseMs  = 20,
                unsigned long maxPulseMs  = 200,
                unsigned long timeoutMs   = 600);

    // เรียกใน ISR — ต้อง attach ด้วย CHANGE
    void IRAM_ATTR handleISR();

    void enable();
    void disable();

    float checkAmount();
    int   getPulseCount()    const { return _pulseCount; }
    // คืนยอดเงินที่นับได้แล้วแต่ยัง timeout ไม่ครบ (ใช้แสดง display real-time)
    float getPendingAmount() const { return (float)_pulseCount * _pulseValue; }
};

#endif