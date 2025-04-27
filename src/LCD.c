/*
 * Drives the LCD display
 */
#include "stm32l432xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "LCD.h"
#include "ee14lib.h"

// LCD I2C Address (7-bit << 1). For PCF8574 modules this is typically 0x27.
#define LCD_ADDR (0x27 << 1) // Confirmed by scanner

// Simple delay function globals
volatile uint32_t ticks = 0;

/**
  * @brief System Clock Configuration
  * Configures the system clock to run from the MSI (4MHz default)
  */
void SystemClock_Config(void)
{
  // Enable MSI oscillator
  RCC->CR |= RCC_CR_MSION;
  // Wait until MSI is ready
  while (!(RCC->CR & RCC_CR_MSIRDY));

  // Select MSI as system clock source
  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_MSI;
  // Wait until MSI is used as system clock source
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI);

  // Update SystemCoreClock variable (optional but good practice)
  SystemCoreClockUpdate();
}


/**
  * @brief SysTick Interrupt Handler
  * Increments the ticks counter every 1ms.
  */
void SysTick_Handler(void)
{
  ticks++;
}

/**
  * @brief Simple Blocking Delay Function
  * @param ms: Delay time in milliseconds
  */
void delay_ms(uint32_t ms)
{
  uint32_t start_ticks = ticks;
  while ((ticks - start_ticks) < ms);
}

/**
  * @brief Error Handler Function (required by some CMSIS parts potentially)
  */
void _Error_Handler(char * file, int line)
{
  // Simple error handler: Infinite loop
  while(1) {}
}

/**
  * @brief Assert Failed Function (required by some CMSIS parts potentially)
  */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/**
 * @brief ADC1 Initialization Function
 */
void ADC1_Init(void)
{
  // Enable ADC Clock
  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

  // Disable ADC if already enabled
  if (ADC1->CR & ADC_CR_ADEN) {
      ADC1->CR |= ADC_CR_ADDIS;
      while (ADC1->CR & ADC_CR_ADEN);
  }

  // Enable ADC Voltage Regulator
  ADC1->CR |= ADC_CR_ADVREGEN; // Enable ADC voltage regulator

  delay_ms(2); // Regulator start-up time

  // Perform ADC Calibration
  ADC1->CR |= ADC_CR_ADCAL;
  while (ADC1->CR & ADC_CR_ADCAL);

  // Set prescaler to synchronous clock /2
  ADC1_COMMON->CCR &= ~ADC_CCR_PRESC;
  ADC1_COMMON->CCR |= 0x1 << ADC_CCR_PRESC_Pos; // DIV2

  // 12-bit resolution, right alignment, single conversion
  ADC1->CFGR = 0; // Single conversion, software trigger

  // Sequence: 2 conversions (L = 1) -> SQ1=6 (PA1), SQ2=8 (PA3)
  ADC1->SQR1 = (1 << ADC_SQR1_L_Pos) | (6 << ADC_SQR1_SQ1_Pos) | (8 << ADC_SQR1_SQ2_Pos);

  // Sampling time for channel 6 and 8 -> 12.5 cycles (010)
  ADC1->SMPR1 &= ~(ADC_SMPR1_SMP6 | ADC_SMPR1_SMP8);
  ADC1->SMPR1 |= (0x1 << ADC_SMPR1_SMP6_Pos) | (0x1 << ADC_SMPR1_SMP8_Pos);

  // Enable ADC
  ADC1->ISR |= ADC_ISR_ADRDY; // Clear ADRDY
  ADC1->CR |= ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY));
}

//rewritten ~ Char
void I2C1_Init(void)
{
    i2c_init(I2C1, D5, D4, I2C_TIMING_10KHZ); // SCL = D5 (PB6), SDA = D4 (PB7)
}

// --- I2C Helper Function ---
void I2C1_WaitTXIS() {
    while (!(I2C1->ISR & I2C_ISR_TXIS));
}

void I2C1_WaitTC() {
    while (!(I2C1->ISR & I2C_ISR_TC));
}

//somehow can't get this one to work with EE14 funcs - TODO
// Simplified I2C Write function (blocking)
void I2C1_Write(uint8_t slave_addr, uint8_t *data, uint32_t size) {
    // Configure slave address and number of bytes
    I2C1->CR2 = (slave_addr & I2C_CR2_SADD) | ((size << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | I2C_CR2_AUTOEND;

    // Set START condition
    I2C1->CR2 |= I2C_CR2_START;

    for (uint32_t i = 0; i < size; i++) {
        I2C1_WaitTXIS(); // Wait until TXDR is empty
        I2C1->TXDR = data[i]; // Send data byte
    }

    // Wait for Transfer Complete (or NACKF/STOPF implicitly handled by AUTOEND)
    // For robustness, checking NACKF might be needed.
    // Note: AUTOEND generates STOP automatically after NBYTES are transferred.
    while (!(I2C1->ISR & I2C_ISR_STOPF)); // Wait for stop condition
    I2C1->ICR |= I2C_ICR_STOPCF; // Clear stop flag
}
  
/*
void I2C1_Write(uint8_t target, uint8_t* data, uint32_t size)
{
    i2c_write(I2C1, target, data, size);
}
*/

// --- LCD Functions (using I2C1_Write) ---
void LCD_SendCommand(uint8_t cmd) {
    uint8_t data_t[4];
    uint8_t up = cmd & 0xF0; // Upper nibble
    uint8_t lo = (cmd & 0x0F) << 4; // Lower nibble shifted left by 4
    
    // Send upper nibble
    data_t[0] = up | 0x0C; // en=1, rs=0, backlight=1
    data_t[1] = up | 0x08; // en=0, rs=0, backlight=1
    I2C1_Write(LCD_ADDR, data_t, 2);
    delay_ms(2);
    
    // Send lower nibble
    data_t[0] = lo | 0x0C; // en=1, rs=0, backlight=1
    data_t[1] = lo | 0x08; // en=0, rs=0, backlight=1
    I2C1_Write(LCD_ADDR, data_t, 2);
    delay_ms(2);
}

void LCD_SendData(uint8_t data) {
    uint8_t data_t[4];
    uint8_t up = data & 0xF0; // Upper nibble
    uint8_t lo = (data & 0x0F) << 4; // Lower nibble shifted left by 4
    
    // Send upper nibble
    data_t[0] = up | 0x0D; // en=1, rs=1, backlight=1
    data_t[1] = up | 0x09; // en=0, rs=1, backlight=1
    I2C1_Write(LCD_ADDR, data_t, 2);
    delay_ms(2);
    
    // Send lower nibble
    data_t[0] = lo | 0x0D; // en=1, rs=1, backlight=1
    data_t[1] = lo | 0x09; // en=0, rs=1, backlight=1
    I2C1_Write(LCD_ADDR, data_t, 2);
    delay_ms(2);
}

void LCD_Init() {
    delay_ms(50); // Wait for LCD power up
    
    // Initialization sequence
    LCD_SendCommand(0x33); // Initialize
    delay_ms(5);
    LCD_SendCommand(0x32); // Initialize
    delay_ms(5);
    LCD_SendCommand(0x28); // 4-bit mode, 2 lines, 5x8 dots
    delay_ms(5);
    
    // Configure display
    LCD_SendCommand(0x0C); // Display on, cursor off, blink off
    delay_ms(5);
    LCD_SendCommand(0x01); // Clear display
    delay_ms(5);
    LCD_SendCommand(0x06); // Entry mode set: increment, no shift
    delay_ms(5);
}

void LCD_SendString(char *str) {
    while (*str) LCD_SendData(*str++);
}

void LCD_Clear() {
    LCD_SendCommand(0x01); // Clear display command
    delay_ms(2); // Needs > 1.53ms
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address;
    switch (row) {
        case 1: address = 0x00; break;
        case 2: address = 0x40; break;
        // Add cases for 4-line displays if needed
        default: address = 0x00;
    }
    address += (col - 1);
    LCD_SendCommand(0x80 | address); // Set DDRAM address command
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 