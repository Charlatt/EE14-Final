#include "LCD.h"
#include "stm32l432xx.h"
#include <stdint.h>
#include "stdio.h"
#include <string.h>
#include "ee14lib.h"

void GPIO_Init(void);
void adc();
void pitch_shift_simple(uint16_t *input, uint16_t *output, int length, float pitch_factor);
void setup_dac(void);

int _write(int file, char *data, int len) {
    serial_write(USART2, data, len);
    return len;
}

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
  LCD_SendData('4');

  //volatile int size = 1024;

  //uint16_t input_samples[size];
  //uint16_t output_samples[size]; 
  //char buffer[100];
  //char buffer2[100];

    adc_config_single(A1);

    //this can be done with EE14LIB

    gpio_config_mode(A3, ANALOG);

    //GPIOA->MODER |=  (0b11 << (4 * 2)); // Set to analog mode

    gpio_config_pullup(A3, PULL_OFF);


    setup_dac();
while(1){

    uint16_t sample = adc_read_single();
    DAC->DHR12R1 = sample;
}


  // Blink LED to confirm code finished init and is running
  while (1)
  {
    // Blink user LED on PB3 (LD3)
    GPIOB->ODR ^= GPIO_ODR_OD3;
    delay_ms(500);
  }
}

void setup_dac(void)
{
    // Enable DAC clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;

    // Enable DAC Channel 1
    DAC->CR |= DAC_CR_EN1;
}

void GPIO_Init(void)
{
    /* 
    // Configure PB3 (User LED LD3)
    gpio_config_mode(D13, OUTPUT); // D13 = PB3
    gpio_config_otype(D13, PUSH_PULL); // push-pull
    gpio_config_ospeed(D13, LOW_SPD); // low speed
    gpio_config_pullup(D13, PULL_OFF); // no pull-up/pull-down
    */

    // Configure PB6 (I2C1_SCL) and PB7 (I2C1_SDA)
    gpio_config_mode(D5, ALTERNATE_FUNCTION); // D5 = PB6
    gpio_config_mode(D4, ALTERNATE_FUNCTION); // D4 = PB7

    gpio_config_alternate_function(D5, 4); // AF4 for I2C1_SCL
    gpio_config_alternate_function(D4, 4); // AF4 for I2C1_SDA

    gpio_config_otype(D5, OPEN_DRAIN); // open-drain
    gpio_config_otype(D4, OPEN_DRAIN);

    gpio_config_ospeed(D5, HI_SPD); // high speed
    gpio_config_ospeed(D4, HI_SPD);

    gpio_config_pullup(D5, PULL_UP); // pull-up enabled
    gpio_config_pullup(D4, PULL_UP);
    
    // configure the microphone
    //gpio_config_mode(A1, INPUT);
    
    
}

