/*
 * Basic CMSIS example for Nucleo-L432KC
 * Blinks the onboard LED (PB3)
 */
#include "stm32l432xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// LCD I2C Address (7-bit << 1). For PCF8574 modules this is typically 0x27.
#define LCD_ADDR (0x27 << 1) // Confirmed by scanner

// Simple delay function globals
volatile uint32_t ticks = 0;

// Function Prototypes
void SystemClock_Config(void);
void GPIO_Init(void);
void ADC1_Init(void);
void I2C1_Init(void);
void delay_ms(uint32_t ms);
void I2C1_Write(uint8_t slave_addr, uint8_t *data, uint32_t size);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_Init(void);
void LCD_SendString(char *str);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);

int main(void)
{
  // Configure System Clock
  SystemClock_Config();

  // Configure SysTick timer for 1ms interrupts
  SysTick_Config(SystemCoreClock / 1000);

  // Initialize GPIO including I2C pins
  GPIO_Init();

  // Initialize I2C with SLOW timing
  I2C1_Init();

  // Initialize LCD
  LCD_Init(); 

  // Send the desired string
  LCD_SendString("EE14 Project");
  // Set cursor position to row 2, column 1
  LCD_SetCursor(2, 1);
  // Display 'TEST' at the cursor position
  LCD_SendData('T');
  LCD_SendData('E');
  LCD_SendData('S');
  LCD_SendData('T');

  // Blink LED to confirm code finished init and is running
  while (1)
  {
    // Blink user LED on PB3 (LD3)
    GPIOB->ODR ^= GPIO_ODR_OD3;
    delay_ms(500);
  }
}

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
  * @brief GPIO Initialization Function
  * Configures the onboard LED pin (PB3) as output push-pull
  */
void GPIO_Init(void)
{
  // Enable GPIOA and GPIOB Clocks
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

  // Configure PB3 (User LED LD3)
  GPIOB->MODER &= ~GPIO_MODER_MODE3;
  GPIOB->MODER |= GPIO_MODER_MODE3_0;
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT3;
  GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED3;
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD3;

  // Keep Analog configs commented out unless needed
  // // Configure PA1 (Potentiometer A1) - Analog ...
  // // Configure PA3 (Potentiometer A2) - Analog ...

  // Configure PB7 (I2C1_SDA / D11) and PB6 (I2C1_SCL / D12)
  GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
  GPIOB->MODER |= (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
  GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
  GPIOB->AFR[0] |= (4 << GPIO_AFRL_AFSEL6_Pos) | (4 << GPIO_AFRL_AFSEL7_Pos);
  GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
  GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
  GPIOB->PUPDR |= (GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD7_0);
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

/**
 * @brief I2C1 Initialization Function
 * Configures I2C1 for ~10kHz @ 4MHz MSI clock (Very Slow)
 */
void I2C1_Init(void)
{
  // Enable I2C1 Clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

  // Disable I2C1 peripheral
  I2C1->CR1 &= ~I2C_CR1_PE;

  // Configure I2C1 Timing Register for ~10kHz @ 4MHz MSI clock (Very Slow)
  // PRESC = 3, SCLDEL = 15, SDADEL = 15, SCLH = 25, SCLL = 30 (Approx values)
  I2C1->TIMINGR = (3 << I2C_TIMINGR_PRESC_Pos) |
                  (15 << I2C_TIMINGR_SCLDEL_Pos) |
                  (15 << I2C_TIMINGR_SDADEL_Pos) |
                  (25 << I2C_TIMINGR_SCLH_Pos)  |
                  (30 << I2C_TIMINGR_SCLL_Pos);

  // Enable I2C1 peripheral
  I2C1->CR1 |= I2C_CR1_PE;
}

// --- I2C Helper Function ---
void I2C1_WaitTXIS() {
    while (!(I2C1->ISR & I2C_ISR_TXIS));
}

void I2C1_WaitTC() {
    while (!(I2C1->ISR & I2C_ISR_TC));
}

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