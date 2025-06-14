#include "Controller.h"

char serial_string[60] = {0};
volatile uint8_t auto_mode = 0;
unsigned long current_ms = 0;
unsigned long last_send_ms = 0;

void update_lcd_mode()
{
  lcd_clrscr();
  if (auto_mode)
  {
    lcd_goto(0);
    lcd_puts("Auto mode");
  }
  else
  {
    lcd_goto(0);
    lcd_puts("All Yameen's");
    lcd_goto(0x40);
    lcd_puts("Fault");
  }
}

ISR(INT5_vect)
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = milliseconds;

  if (interrupt_time - last_interrupt_time > 200)
  {
    auto_mode ^= 1;
    update_lcd_mode();
  }

  last_interrupt_time = interrupt_time;
}

int main(void)
{
  serial0_init();
  serial1_init();
  milliseconds_init();
  adc_init();
  _delay_ms(20);

  lcd_init();
  _delay_ms(50);
  lcd_clrscr();
  _delay_ms(10);

  DDRE &= ~(1 << PE5);
  PORTE |= (1 << PE5);

  EICRB |= (1 << ISC51) | (1 << ISC50);
  EIFR |= (1 << INTF5);
  EIMSK |= (1 << INT5);

  sei();

  update_lcd_mode();

  const uint8_t constant_control_byte = 1;

  while (1)
  {
    current_ms = milliseconds;

    // --- Joystick ADC reads ---
    uint16_t x_adc_val_raw = adc_read(0);
    uint16_t y_adc_val_raw = adc_read(1);
    uint16_t servo_adc_val_raw = adc_read(2);

    const uint16_t threshold = 30;
    const uint16_t center = 512;

    uint8_t x_byte = 128;
    uint8_t y_byte = 128;
    uint8_t servo_byte = 128;

    uint8_t xy_active = (abs(x_adc_val_raw - center) > threshold) ||
                        (abs(y_adc_val_raw - center) > threshold);
    uint8_t servo_active = abs(servo_adc_val_raw - center) > threshold;

    if (xy_active)
    {
      x_byte = (uint8_t)(x_adc_val_raw >> 2);
      y_byte = (uint8_t)(y_adc_val_raw >> 2);
    }
    else if (servo_active)
    {
      servo_byte = (uint8_t)(servo_adc_val_raw >> 2);
    }

    uint8_t databytes[4] = {
        constant_control_byte,
        x_byte,
        y_byte,
        servo_byte};

    sprintf(serial_string, "Tx: Const:%u, X:%u, Y:%u, Servo:%u\r\n",
            databytes[0], databytes[1], databytes[2], databytes[3]);
    serial0_print_string(serial_string);

    if ((current_ms - last_send_ms) >= 100)
    {
      last_send_ms = current_ms;

      serial1_write_byte(254); // Start byte

      for (int i = 0; i < sizeof(databytes); i++)
      {
        serial1_write_byte(databytes[i]);
      }

      serial1_write_byte(253); // End byte
    }
  }

  return 0;
}
