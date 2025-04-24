# STM32 I2C LCD 1602 Project

This is a PlatformIO port of the STM32 I2C LCD 1602 project.

## Hardware Requirements
- STM32L432KC Nucleo board (tested)
- 1602 LCD display with I2C interface (typically using PCF8574 I2C expander)
- Connecting wires

## Connections
- Connect LCD's SDA to I2C1 SDA (PB9)
- Connect LCD's SCL to I2C1 SCL (PB8)
- Connect potentiometer wiper to A1 (PA1) for analog reading

## Features
- Pure CMSIS (register-level) implementation – no HAL
- LCD initialisation & 4-bit data transfer over I2C (PCF8574 expander)
- Reads potentiometer (ADC on PA1) and shows raw value on second LCD line
- Blinks onboard LED (PB3) so you know firmware is alive

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

This repo is now configured for the `cmsis` framework and the **Nucleo-L432KC** board.

## Original Project
This project's usage of i2c display is based on the [stm32-i2c-lcd-1602](https://github.com/afiskon/stm32-i2c-lcd-1602) project by Aleksander Alekseev. 