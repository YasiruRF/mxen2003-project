#include "Controller.h"

#define PWM_TOP 2000
#define DEADZONE 20

uint8_t databyte1 = 0;
uint8_t databyte2 = 0;
uint8_t recievedData[2];      // received data array
char serial_string[60] = {0}; // String used for printing to terminal

int main(void)
{
  serial0_init();
  serial2_init();
  milliseconds_init();
  adc_init();
  _delay_ms(20);

  // Set PB5 (OC1A) and PB6 (OC1B) as output for PWM
  DDRB |= (1 << PB5) | (1 << PB6);

  // Set PORTA (PA0–PA3) as output for motor direction
  DDRA |= (1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3);
  PORTA &= ~((1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3)); // initialize to low

  // Timer1 PWM setup (Mode 8: Phase and Frequency Correct PWM using ICR1)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1); // Non-inverting mode
  TCCR1B = (1 << WGM13) | (1 << CS11);    // Mode 8 with prescaler 8

  ICR1 = PWM_TOP; // Set TOP value for 500Hz

  while (1)
  {
    uint16_t x_val = adc_read(0); // Left/right joystick
    uint16_t y_val = adc_read(1); // Forward/backward joystick

    // Debugging: print analog values
    char serialString[50];
    sprintf(serialString, "X: %u, Y: %u\n", x_val, y_val);
    serial0_print_string(serialString);

    // Convert joystick readings to signed values centered at 0
    int16_t x = (int16_t)x_val - 512;
    int16_t y = (int16_t)y_val - 512;

    int16_t lm = y + x;
    int16_t rm = y - x;

    int16_t lmDuty = 0;
    int16_t rmDuty = 0;

    // Apply deadzone
    if (abs(x) > DEADZONE || abs(y) > DEADZONE)
    {
      lmDuty = abs((int32_t)lm * PWM_TOP / 512);
      rmDuty = abs((int32_t)rm * PWM_TOP / 512);
    }

    // Set PWM duty cycle
    OCR1A = (lmDuty > PWM_TOP) ? PWM_TOP : lmDuty;
    OCR1B = (rmDuty > PWM_TOP) ? PWM_TOP : rmDuty;

    // Control left motor direction
    if (lm > DEADZONE)
    {
      PORTA |= (1 << PA0);
      PORTA &= ~(1 << PA1);
    }
    else if (lm < -DEADZONE)
    {
      PORTA &= ~(1 << PA0);
      PORTA |= (1 << PA1);
    }
    else
    {
      PORTA &= ~(1 << PA0);
      PORTA &= ~(1 << PA1);
    }

    // Control right motor direction
    if (rm > DEADZONE)
    {
      PORTA |= (1 << PA2);
      PORTA &= ~(1 << PA3);
    }
    else if (rm < -DEADZONE)
    {
      PORTA &= ~(1 << PA2);
      PORTA |= (1 << PA3);
    }
    else
    {
      PORTA &= ~(1 << PA2);
      PORTA &= ~(1 << PA3);
    }
  }

  return 0;
}
