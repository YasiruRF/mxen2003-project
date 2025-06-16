#include "Controller.h"

#define START_BYTE 0xFE
#define STOP_BYTE  0xFD

char serial_string[60] = {0};
char lcd_string[33]    = {0};

unsigned long current_ms     = 0;
unsigned long last_send_ms   = 0;
unsigned long last_debounce  = 0;
const uint32_t debounce_delay = 50;

volatile bool Auto = false;

volatile uint8_t data_from_robot[4] = {0};
volatile bool new_msg_flag = false;

static bool previous_debounced_button_state = false;

void button_init(void) {
    DDRE  &= ~(1 << PE5);
    PORTE |=  (1 << PE5);
}

ISR(USART1_RX_vect) {
    static enum { WAIT_START, BYTE1, BYTE2, BYTE3, BYTE4, WAIT_STOP } rx_state = WAIT_START;
    static uint8_t buffer[4];
    uint8_t byte = UDR1;

    switch (rx_state) {
        case WAIT_START:
            if (byte == START_BYTE) rx_state = BYTE1;
            break;
        case BYTE1:
        case BYTE2:
        case BYTE3:
        case BYTE4:
            buffer[rx_state - BYTE1] = byte;
            rx_state++;
            break;
        case WAIT_STOP:
            if (byte == STOP_BYTE) {
                for (int i = 0; i < 4; i++) data_from_robot[i] = buffer[i];
                new_msg_flag = true;
            }
            rx_state = WAIT_START;
            break;
    }
}

int main(void) {
    cli();
    serial0_init();
    serial1_init();
    milliseconds_init();
    adc_init();
    lcd_init();
    button_init();

    UCSR1B |= (1 << RXCIE1);
    sei();

    _delay_ms(20);

    lcd_home();
    lcd_puts("Controller Ready");
    _delay_ms(1000);

    bool last_button_raw_state = !(PINE & (1 << PE5));

    while (1) {
        current_ms = milliseconds_now();

        uint16_t x_raw = adc_read(0);
        uint16_t y_raw = adc_read(1);
        uint16_t s_raw = adc_read(2);

        uint8_t x_val = x_raw >> 2;
        uint8_t y_val = 255 - (y_raw >> 2);

        uint8_t servo_byte = (uint8_t)((((uint32_t)s_raw) * 255) / 1023);

        bool current_button_raw_state = !(PINE & (1 << PE5));

        if (current_button_raw_state != last_button_raw_state) {
            last_debounce = current_ms;
        }

        if ((current_ms - last_debounce) > debounce_delay) {
            if (current_button_raw_state != previous_debounced_button_state) {
                previous_debounced_button_state = current_button_raw_state;

                if (current_button_raw_state == 1) {
                    Auto = !Auto;
                }
            }
        }
        last_button_raw_state = current_button_raw_state;

        if ((current_ms - last_send_ms) >= 100) {
            last_send_ms = current_ms;

            uint8_t packet[4] = {
                Auto ? 255 : 1,
                x_val,
                y_val,
                servo_byte
            };

            serial1_write_byte(START_BYTE);
            for (int i = 0; i < 4; i++) {
                serial1_write_byte(packet[i]);
            }
            serial1_write_byte(STOP_BYTE);
        }

        sprintf(serial_string, "Tx: %s | X:%3u Y:%3u Servo:%3u\r\n",
                Auto ? "AUTO" : "MAN", x_val, y_val, servo_byte);
        serial0_print_string(serial_string);

        if (new_msg_flag) {
            lcd_goto(0);
            sprintf(lcd_string, "Range:%3u L:%2u R:%2u",
                    data_from_robot[0],
                    data_from_robot[1],
                    data_from_robot[2]
            );
            lcd_puts(lcd_string);

            lcd_goto(0x40);
            sprintf(lcd_string, "Robot Mode: %s",
                    data_from_robot[3] == 1 ? "AUTO" : "MAN");
            lcd_puts(lcd_string);

            sprintf(serial_string, "Rx: R:%u L:%u R:%u S:%u\r\n", 
                    data_from_robot[0],
                    data_from_robot[1],
                    data_from_robot[2],
                    data_from_robot[3]
            );
            serial0_print_string(serial_string);

            new_msg_flag = false;
        }
    }

    return 0;
}