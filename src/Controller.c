#include "Controller.h"  // serial0, serial1, milliseconds, adc, lcd

#define START_BYTE 0xFE
#define STOP_BYTE  0xFD

//////////////////////////////////////////////////////////////////////////
//                          GLOBALS
//////////////////////////////////////////////////////////////////////////

char serial_string[60] = {0};
char lcd_string[33]    = {0};

unsigned long current_ms     = 0;
unsigned long last_send_ms   = 0;
unsigned long last_debounce  = 0;
const uint32_t debounce_delay = 50;

volatile bool Auto = false;

volatile uint8_t data_from_robot[5] = {0};
volatile bool new_msg_flag = false;

uint16_t servo_position_us = 1500; // µs (1000–2000)

//////////////////////////////////////////////////////////////////////////
//                          INIT FUNCTIONS
//////////////////////////////////////////////////////////////////////////

// Servo on PB7 (OC2A) using Timer2 fast PWM
void servo_init(void) {
    DDRB |= (1 << PB7); // PB7 output (OC2A)

    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20); // Fast PWM, clear OC2A on compare
    TCCR2B = (1 << CS21); // Prescaler 8
    OCR2A = (servo_position_us - 1000) * 255UL / 1000; // Initial pulse width
}

void servo_set(uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    servo_position_us = us;
    OCR2A = (us - 1000) * 255UL / 1000;
}

void button_init(void) {
    DDRE  &= ~(1 << PE3);
    PORTE |=  (1 << PE3); // Enable pull-up on PE3 (button)
}

//////////////////////////////////////////////////////////////////////////
//                          RX ISR
//////////////////////////////////////////////////////////////////////////

ISR(USART1_RX_vect) {
    static enum { WAIT_START, BYTE1, BYTE2, BYTE3, BYTE4, BYTE5, WAIT_STOP } rx_state = WAIT_START;
    static uint8_t buffer[5];
    uint8_t byte = UDR1;

    switch (rx_state) {
        case WAIT_START:
            if (byte == START_BYTE) rx_state = BYTE1;
            break;
        case BYTE1: case BYTE2: case BYTE3: case BYTE4: case BYTE5:
            buffer[rx_state - BYTE1] = byte;
            rx_state++;
            break;
        case WAIT_STOP:
            if (byte == STOP_BYTE) {
                for (int i = 0; i < 5; i++) data_from_robot[i] = buffer[i];
                new_msg_flag = true;
            }
            rx_state = WAIT_START;
            break;
    }
}

//////////////////////////////////////////////////////////////////////////
//                          MAIN LOOP
//////////////////////////////////////////////////////////////////////////

int main(void) {
    cli();
    serial0_init();
    serial1_init();
    milliseconds_init();
    adc_init();
    lcd_init();
    servo_init();
    button_init();

    UCSR1B |= (1 << RXCIE1); // Enable RX interrupt
    sei();

    _delay_ms(20);

    lcd_home();
    lcd_puts("Controller Ready");
    _delay_ms(1000);

    bool last_button = 1;

    while (1) {
        current_ms = milliseconds_now();

        // === JOYSTICK + SERVO READ ===
        uint16_t x_raw = adc_read(0);
        uint16_t y_raw = adc_read(1);
        uint16_t s_raw = adc_read(2);

        uint8_t x_val = x_raw >> 2;
        uint8_t y_val = 255 - (y_raw >> 2); // Inverted for up=forward

        uint16_t servo_us = 1000 + ((uint32_t)s_raw * 1000) / 1023;
        servo_set(servo_us);

        uint8_t servo_byte = (uint8_t)(((servo_us - 1000) * 255UL) / 1000);

        // === BUTTON DEBOUNCE ===
        bool btn = (PINE & (1 << PE3));
        if (btn != last_button) {
            last_debounce = current_ms;
            last_button = btn;
        }

        if ((current_ms - last_debounce) > debounce_delay && btn == 0) {
            Auto = !Auto;
            while (!(PINE & (1 << PE3))); // Wait until release
            _delay_ms(10);
        }

        // === SEND DATA TO ROBOT ===
        if ((current_ms - last_send_ms) >= 100) {
            last_send_ms = current_ms;
            uint8_t packet[4] = {
                Auto ? 255 : 1,
                x_val,
                y_val,
                servo_byte
            };

            serial1_write_byte(START_BYTE);
            for (int i = 0; i < 4; i++)
                serial1_write_byte(packet[i]);
            serial1_write_byte(STOP_BYTE);
        }

        // === DEBUG SERIAL OUT ===
        sprintf(serial_string, "Tx: %s | X:%u Y:%u Servo:%u\r\n",
                Auto ? "AUTO" : "MAN", x_val, y_val, servo_us);
        serial0_print_string(serial_string);

        // === LCD DISPLAY RECEIVED ===
        if (new_msg_flag) {
            lcd_goto(0);
            sprintf(lcd_string, "Range:%3u Freq:%3u", data_from_robot[0], data_from_robot[1]);
            lcd_puts(lcd_string);

            lcd_goto(0x40);
            sprintf(lcd_string, "L:%2u R:%2u Mode:%s",
                    data_from_robot[2], data_from_robot[3],
                    Auto ? "AUTO" : "MAN");
            lcd_puts(lcd_string);

            sprintf(serial_string, "Rx: R:%u F:%u L:%u R:%u S:%u\r\n",
                    data_from_robot[0], data_from_robot[1],
                    data_from_robot[2], data_from_robot[3], data_from_robot[4]);
            serial0_print_string(serial_string);

            new_msg_flag = false;
        }
    }

    return 0;
}
