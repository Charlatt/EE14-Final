# STM32 I2C LCD 1602 Project

This is a PlatformIO port of the STM32 I2C LCD 1602 project.

## Hardware Requirements
- STM32F411RE Nucleo board
- 1602 LCD display with I2C interface (typically using PCF8574 I2C expander)
- Connecting wires

## Connections
- Connect LCD's SDA to I2C1 SDA (PB9)
- Connect LCD's SCL to I2C1 SCL (PB8)
- Connect LCD's VCC to 5V
- Connect LCD's GND to GND

## Features
- I2C bus scanning
- Display initialization
- Text display on LCD
- UART communication for debugging

## Building and Flashing
This project uses PlatformIO for building and flashing.
To build and upload to the board:
```
pio run -t upload
```

To monitor serial output:
```
pio device monitor -b 9600
```

## Original Project
This project is based on the [stm32-i2c-lcd-1602](https://github.com/afiskon/stm32-i2c-lcd-1602) project by Aleksander Alekseev. 