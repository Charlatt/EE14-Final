#include "LCD.h"
#include "stm32l432xx.h"
#include <stdint.h>
#include "stdio.h"
#include <string.h>
#include "ee14lib.h"

void GPIO_Init(void);
void adc();
void pitch_shift_simple(uint16_t *input, uint16_t *output, int length, float pitch_factor);

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

  volatile int size = 1024;

  uint16_t input_samples[size];
  uint16_t output_samples[size]; 
  char buffer[100];
  char buffer2[100];
while(1){
  adc(input_samples, size);

  //pitch_shift_simple(input_samples, output_samples, size, 2.0f);  // pitch up

  for(int i = 0; i < size; i++){
    sprintf(buffer, "%d", output_samples[i]);
    sprintf(buffer2, "%d\n", input_samples[i]);
    printf("%s    ", buffer);
    printf("%s", buffer2);
  }
}


  // Blink LED to confirm code finished init and is running
  while (1)
  {
    // Blink user LED on PB3 (LD3)
    GPIOB->ODR ^= GPIO_ODR_OD3;
    delay_ms(500);
  }
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

void adc(uint16_t input_samples[], volatile int input_samples_size) {
    // initialize everything
    host_serial_init();
    unsigned int output;

    // set all my pins as outputs for the joystick
    adc_config_single(A1);

    // row of LEDs for the joystick
    for(int i = 0; i < input_samples_size; i++){
        //delay_ms(100);
        input_samples[i] = adc_read_single();
        //sprintf(buffer, "%d\n", output);
        //printf("%s", buffer);
    }
}

/* Doesn't really work
void pitch_shift_simple(uint16_t *input, uint16_t *output, int length, float pitch_factor) {
  for (int i = 0; i < length; i++) {
      float src_index = i / pitch_factor;  // where in input to grab
      int index_base = (int)src_index;      // integer part
      float fraction = src_index - index_base; // decimal part

      if (index_base + 1 < length) {
          // Simple linear interpolation between two samples
          output[i] = (1.0f - fraction) * input[index_base] + fraction * input[index_base + 1];
      } else {
          // If we're at the very end, just copy the last sample
          output[i] = input[length - 1];
      }
  }
}
*/