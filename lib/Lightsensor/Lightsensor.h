#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include <Arduino.h>

// ============================================================
//  LightSensor — Phototransistor แบบ Pull-up
//
//  วงจร:
//    GPIO → Phototransistor (Collector) → GND
//    GPIO ต่อ Pull-up (internal หรือ external)
//
//  Logic:
//    มีแสง  → Transistor ON  → pin = LOW
//    ไม่มีแสง → Transistor OFF → pin = HIGH (pull-up)
//
//  Features:
//    - อ่านค่า analogRead เพื่อวัด intensity (0-4095 บน ESP32)
//    - debounce ป้องกัน flicker
//    - threshold ปรับได้ว่า "สว่างพอ" คือเท่าไร
//    - detect() คืน true ถ้ามีแสงเกิน threshold
// ============================================================
class LightSensor {
public:
    // pin          : GPIO ที่ต่อ phototransistor
    // thresholdADC : ค่า ADC ที่ถือว่า "มีแสง" (0-4095)
    //                Pullup → มีแสง = ค่าต่ำ
    //                ค่า default 2000 = สว่างเกินครึ่ง scale
    // debounceMs   : รอให้ state นิ่งก่อนเปลี่ยน (ms)
    LightSensor(uint8_t pin,
                uint16_t thresholdADC = 2000,
                unsigned long debounceMs = 100);

    // เรียกใน setup() หลัง Wire.begin()
    void begin();

    // เรียกทุก loop — อัปเดต state
    void update();

    // คืน true ถ้ามีแสงเกิน threshold (หลัง debounce)
    bool detected() const { return _lightDetected; }

    // คืนค่า ADC raw ล่าสุด (0-4095)
    // ค่าต่ำ = สว่าง, ค่าสูง = มืด (เพราะ pull-up)
    uint16_t rawADC() const { return _rawADC; }

    // คืน % ความสว่าง (0-100)
    // 0 = มืดสนิท, 100 = สว่างจ้า
    uint8_t brightness() const {
        // กลับด้านเพราะ pullup (4095=มืด, 0=สว่าง)
        return (uint8_t)(100 - (_rawADC * 100UL / 4095));
    }

    // เปลี่ยน threshold runtime
    void setThreshold(uint16_t adc) { _threshold = adc; }

private:
    uint8_t       _pin;
    uint16_t      _threshold;
    unsigned long _debounceMs;

    uint16_t      _rawADC;
    bool          _lightDetected;   // state หลัง debounce
    bool          _prevRaw;         // raw state รอบก่อน
    unsigned long _lastChangeTime;
};

#endif