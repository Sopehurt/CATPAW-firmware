#include "ExpanderManager.h"

ExpanderManager::ExpanderManager(uint8_t address, TwoWire* w) {
  _address = address;
  _i2c = w;
  _configReg = 0xFF; // เริ่มต้นให้เป็น Input ทั้งหมดเพื่อความปลอดภัย
  _outputReg = 0x00;
}

bool ExpanderManager::begin() {
  _i2c->beginTransmission(_address);
  uint8_t error = _i2c->endTransmission();
  if (error == 0) {
    writeRegister(PCA9557_REG_POLARITY, 0x00); // เพิ่มบรรทัดนี้: บังคับให้ Logic ไม่กลับด้าน
    return true;
  }
  return false;
}

void ExpanderManager::pinMode(uint8_t pin, uint8_t mode) {
  if (pin > 7) return;
  
  if (mode == INPUT) {
    _configReg |= (1 << pin); // Set bit to 1 for Input
  } else {
    _configReg &= ~(1 << pin); // Set bit to 0 for Output
  }
  writeRegister(PCA9557_REG_CONFIG, _configReg);
}

void ExpanderManager::digitalWrite(uint8_t pin, uint8_t level) {
  if (pin > 7) return;

  if (level == HIGH) {
    _outputReg |= (1 << pin);
  } else {
    _outputReg &= ~(1 << pin);
  }
  writeRegister(PCA9557_REG_OUTPUT, _outputReg);
}

uint8_t ExpanderManager::digitalRead(uint8_t pin) {
  if (pin > 7) return LOW;
  uint8_t currentInput = readRegister(PCA9557_REG_INPUT);
  return (currentInput & (1 << pin)) ? HIGH : LOW;
}

bool ExpanderManager::writeRegister(uint8_t reg, uint8_t value) {
  _i2c->beginTransmission(_address);
  _i2c->write(reg);
  _i2c->write(value);
  return (_i2c->endTransmission() == 0);
}

uint8_t ExpanderManager::readRegister(uint8_t reg) {
  _i2c->beginTransmission(_address);
  _i2c->write(reg);
  _i2c->endTransmission(false);
  _i2c->requestFrom(_address, (uint8_t)1);
  return _i2c->available() ? _i2c->read() : 0;
}