#include "Controller.h"

char serial_string[60] = {0};
volatile uint8_t auto_mode = 0; // 0 = manual, 1 = auto
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
  static uint8_t last_state = 0;
  uint8_t current_state = (PINE & (1 << PE5)) ? 1 : 0;
  // Simple debounce: only toggle on rising edge
  if (current_state && !last_state)
  {
    auto_mode ^= 1; // Toggle mode
    update_lcd_mode();
  }
  last_state = current_state;
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

  // --- Configure PE5 as input with pull-up ---
  DDRE &= ~(1 << PE5); // Input
  PORTE |= (1 << PE5); // Enable pull-up

  // --- Enable INT5 on any edge ---
  EICRB |= (1 << ISC50); // ISC51=0, ISC50=1: any logical change
  EICRB &= ~(1 << ISC51);
  EIFR |= (1 << INTF5); // Clear any pending
  EIMSK |= (1 << INT5); // Enable INT5

  sei(); // Enable global interrupts

  update_lcd_mode(); // Show initial mode

  const uint8_t constant_control_byte = 1;

  while (1)
  {
    current_ms = milliseconds;

    uint16_t x_adc_val = adc_read(0);
    uint16_t y_adc_val = adc_read(1);
    uint16_t servo_val = adc_read(2);

    uint8_t x_byte = (uint8_t)(x_adc_val >> 2);
    uint8_t y_byte = (uint8_t)(y_adc_val >> 2);
    uint8_t servo_byte = (uint8_t)(servo_val >> 2);

    uint8_t databytes[4] = {
        constant_control_byte,
        x_byte,
        y_byte, servo_byte};

    sprintf(serial_string, "Tx (Controller): Const:%u, X:%u, Y:%u, Servo:%u\r\n",
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