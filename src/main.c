#include "LCD.h"
#include "stm32l432xx.h"
#include <stdint.h>
#include "stdio.h"
#include <string.h>
#include "ee14lib.h"

void GPIO_Init(void);
void setup_dac(void);
void timer2_init_for_8khz(void);
void setup();


#define NUM_SAMPLES 16000  // 1 second at 8 kHz
uint16_t buffer[NUM_SAMPLES];


//override printf
int _write(int file, char *data, int len) {
    serial_write(USART2, data, len);
    return len;
}

int main(void)
{
  
    setup();

  // Send the desired string
  LCD_SendString("EE14 Project");
  // Set cursor position to row 2, column 1
  LCD_SetCursor(2, 1);
  // Display 'TEST' at the cursor position
  LCD_SendData('T');
  LCD_SendData('E');
  LCD_SendData('S');
  LCD_SendData('T');
  LCD_SendData('6');

  for (volatile int a = 0; a < 1000000; a++); 
  
  //printf("hi!\n");

  //adc_config_single(A0);          // Configure ADC on D12
  adc_config_single(A1);
    
    while(1){
        //adc_config_single(A0); 
        //int val = adc_read_single();  // Read value from ADC
        //printf("Value: %u\n", val);        // Print actual numeric value
        //printf("loop\n");
        // --- Sampling phase ---
        for (int i = 0; i < NUM_SAMPLES; i++) {
            while (!(TIM2->SR & TIM_SR_UIF));  // wait for timer overflow
            TIM2->SR &= ~TIM_SR_UIF;           // clear update flag

            buffer[i] = adc_read_single();     // sample from ADC
        }

        //printf("done sample\n");

        delay_ms(500);  // Optional delay before playback

        // --- Playback phase ---
        for (int i = 0; i < NUM_SAMPLES; i++) {
            while (!(TIM2->SR & TIM_SR_UIF));  // wait for timer
            TIM2->SR &= ~TIM_SR_UIF;           // clear flag

            DAC->DHR12R1 = (buffer[i] / 4);          // play one sample
        }
}

}

void setup() {
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

    host_serial_init();

    setup_dac();

    timer2_init_for_8khz();

    


}

void setup_dac(void)
{
    // Enable DAC clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;

    // Enable DAC Channel 1
    DAC->CR |= DAC_CR_EN1;
}

void timer2_init_for_8khz(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;  // Enable TIM2 clock

    // Calculate the prescaler and ARR based on the actual system clock
    // Assuming SystemCoreClock holds the current system clock (updated via SystemCoreClockUpdate())
    
    uint32_t sysclk = SystemCoreClock; // This will give the current system clock frequency
    uint32_t prescaler = sysclk / 1000000 - 1;  // Convert to 1 MHz timer
    uint32_t arr = 124; // For 8 kHz, we need 125 µs per cycle, so ARR = 124

    TIM2->PSC = prescaler;    // Set prescaler
    TIM2->ARR = arr;          // Set auto-reload value

    TIM2->CNT = 0;            // Reset counter
    TIM2->SR = 0;             // Clear status register
    TIM2->CR1 = TIM_CR1_CEN;  // Enable the timer
}

void GPIO_Init(void)
{

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
    
    //DAC out
    gpio_config_mode(A3, ANALOG);
    gpio_config_pullup(A3, PULL_OFF);
    
    
}

