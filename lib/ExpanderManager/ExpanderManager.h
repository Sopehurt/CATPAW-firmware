#ifndef EXPANDER_MANAGER_H
#define EXPANDER_MANAGER_H

#include <Arduino.h>
#include <Wire.h>

// PCA9557 Register Addresses
#define PCA9557_REG_INPUT     0x00
#define PCA9557_REG_OUTPUT    0x01
#define PCA9557_REG_POLARITY  0x02
#define PCA9557_REG_CONFIG    0x03

class ExpanderManager {
  private:
    uint8_t _address;
    uint8_t _configReg; // เก็บสถานะทิศทาง (0=Output, 1=Input)
    uint8_t _outputReg; // เก็บสถานะ Output ปัจจุบัน
    TwoWire* _i2c;

    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);

  public:
    // Constructor กำหนด Address (Default มักจะเป็น 0x18 - 0x1F)
    ExpanderManager(uint8_t address, TwoWire* w = &Wire);

    bool begin();
    
    // กำหนดโหมด: INPUT หรือ OUTPUT
    void pinMode(uint8_t pin, uint8_t mode);
    
    // สั่งงาน Output
    void digitalWrite(uint8_t pin, uint8_t level);
    
    // อ่านค่า Input
    uint8_t digitalRead(uint8_t pin);
};

#endif