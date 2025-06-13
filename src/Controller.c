#include "Controller.h"

char serial_string[60] = {0};

unsigned long current_ms = 0;
unsigned long last_send_ms = 0;

int main(void)
{
  serial0_init();
  serial1_init();
  milliseconds_init();
  adc_init();
  _delay_ms(20);

  const uint8_t constant_control_byte = 1;

  while (1)
  {
    current_ms = milliseconds;
    uint16_t x_adc_val = adc_read(0);
    uint16_t y_adc_val = adc_read(1);

    uint8_t x_byte = (uint8_t)(x_adc_val >> 2);
    uint8_t y_byte = (uint8_t)(y_adc_val >> 2);

    uint8_t databytes[3];

    databytes[0] = constant_control_byte;
    databytes[1] = x_byte;
    databytes[2] = y_byte;

    sprintf(serial_string, "Tx (Controller): Const:%u, X:%u, Y:%u\r\n",
            databytes[0], databytes[1], databytes[2]);
    serial0_print_string(serial_string);

    if ((current_ms - last_send_ms) >= 100)
    {
      last_send_ms = current_ms;
      serial1_write_byte(254);

      for (int i = 0; i < sizeof(databytes); i++)
      {
        serial1_write_byte(databytes[i]);
      }

      serial1_write_byte(253);
    }
  }

  return 0;
}