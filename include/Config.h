#ifndef CONFIG_H
#define CONFIG_H

// --- Pulse Money Acceptors ---
#define PIN_BANK_PULSE 4     
#define PIN_COIN_PULSE 12    

// --- I2C & GPIO Expander (PCF8575) ---
#define I2C_SDA 8
#define I2C_SCL 9
#define PCF_ADDR 0x20
#define PCF_P00_RELAY 0      // Output: Relay
#define PCF_P11_TOGGLE 9     // Input: ปุ่ม Toggle On/Off (ขา P11 ใน Lib คือเลข 9)
#define PCF_P17_START 15     // Input: ปุ่ม Start (ขา P17 ใน Lib คือเลข 15)

// --- Display 4-Digit ---
#define DISP_CLK 15
#define DISP_DIO 16

// --- UART Communications ---
#define UART1_TX 17          
#define UART1_RX 18
#define UART2_TX 13          
#define UART2_RX 14

#endif