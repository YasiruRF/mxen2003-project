#include "Robot.h"

#define PWM_TOP 2000
#define DEADZONE 20

uint8_t serial_byte_in = 0;
uint8_t serial_fsm_state = 0;

uint8_t temp_recv_databytes[3];

uint8_t constant_byte_received = 0;
uint8_t x_val_received = 128;
uint8_t y_val_received = 128;

char serial_string[60] = {0};

int main(void)
{
  serial0_init();
  serial1_init();
  milliseconds_init();
  _delay_ms(20);

  DDRB |= (1 << PB5) | (1 << PB6);
  DDRA |= (1 << PA1) | (1 << PA3) | (1 << PA5) | (1 << PA7);
  PORTA &= ~((1 << PA1) | (1 << PA3) | (1 << PA5) | (1 << PA7));

  TCCR1A = (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << WGM13) | (1 << CS11);
  ICR1 = PWM_TOP;

  UCSR1B |= (1 << RXCIE1);
  sei();

  while (1)
  {
    int16_t x = (int16_t)x_val_received - 128;
    int16_t y = (int16_t)y_val_received - 128;

    int16_t lm_speed_cmd = y + x;
    int16_t rm_speed_cmd = y - x;

    int16_t lm_duty_cycle = 0;
    int16_t rm_duty_cycle = 0;

    if (abs(x) > DEADZONE || abs(y) > DEADZONE)
    {
      lm_duty_cycle = abs((int32_t)lm_speed_cmd * PWM_TOP / 128);
      rm_duty_cycle = abs((int32_t)rm_speed_cmd * PWM_TOP / 128);

      lm_duty_cycle = (lm_duty_cycle > PWM_TOP) ? PWM_TOP : lm_duty_cycle;
      rm_duty_cycle = (rm_duty_cycle > PWM_TOP) ? PWM_TOP : rm_duty_cycle;
    }

    OCR1A = lm_duty_cycle;
    OCR1B = rm_duty_cycle;

    if (lm_speed_cmd > DEADZONE)
    {
      PORTA |= (1 << PA1);
      PORTA &= ~(1 << PA3);
    }
    else if (lm_speed_cmd < -DEADZONE)
    {
      PORTA &= ~(1 << PA1);
      PORTA |= (1 << PA3);
    }
    else
    {
      PORTA &= ~((1 << PA1) | (1 << PA3));
    }

    if (rm_speed_cmd > DEADZONE)
    {
      PORTA |= (1 << PA5);
      PORTA &= ~(1 << PA7);
    }
    else if (rm_speed_cmd < -DEADZONE)
    {
      PORTA &= ~(1 << PA5);
      PORTA |= (1 << PA7);
    }
    else
    {
      PORTA &= ~((1 << PA5) | (1 << PA7));
    }

    sprintf(serial_string, "Rx (Robot): Const=%u, X=%d, Y=%d\n",
            constant_byte_received, (int)x, (int)y);
    serial0_print_string(serial_string);
  }
  return 0;
}

ISR(USART1_RX_vect)
{
  serial_byte_in = UDR1;

  switch (serial_fsm_state)
  {
  case 0:
    if (serial_byte_in == 254)
    {
      serial_fsm_state = 1;
    }
    break;

  case 1:
    temp_recv_databytes[0] = serial_byte_in;
    serial_fsm_state = 2;
    break;

  case 2:
    temp_recv_databytes[1] = serial_byte_in;
    serial_fsm_state = 3;
    break;

  case 3:
    temp_recv_databytes[2] = serial_byte_in;
    serial_fsm_state = 4;
    break;

  case 4:
    if (serial_byte_in == 253)
    {
      constant_byte_received = temp_recv_databytes[0];
      x_val_received = temp_recv_databytes[1];
      y_val_received = temp_recv_databytes[2];
    }
    serial_fsm_state = 0;
    break;

  default:
    serial_fsm_state = 0;
    break;
  }
}