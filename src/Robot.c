    #include "Robot.h" // Assuming this includes necessary AVR headers and function prototypes

    #define START_BYTE 0xFE
    #define STOP_BYTE  0xFD
    #define PWM_TOP 2000
    #define DEADZONE 20

    // Global variables for serial communication
    volatile uint8_t data_from_controller[4] = {0};
    volatile bool new_msg_flag = false;

    // Motor control variables
    volatile bool Auto = false;
    uint8_t x_val = 128;
    uint8_t y_val = 128;
    uint8_t servo_val = 128;

    // Sensor data to send back
    uint8_t range_sensor = 0;        // front long range sensor (distance in cm)
    uint8_t frequency_sensor = 0;    // averaged short range sensors (distance in cm) - **Now specifically for general avg/beacon**
    uint8_t left_motor_feedback = 0;
    uint8_t right_motor_feedback = 0;
    uint8_t status_byte = 0;

    // **NEW GLOBAL SENSOR VARIABLES FOR INDIVIDUAL SHORT-RANGE READINGS**
    uint8_t short_range_left_cm = 0;
    uint8_t short_range_right_cm = 0;

    char serial_string[100] = {0};

    //////////////////////////////////////////////////////////////////////////
    //                          INIT FUNCTIONS
    //////////////////////////////////////////////////////////////////////////

    void motors_init(void) {
        // Timer1 for motor PWM (PB5=OC1A, PB6=OC1B)
        DDRB |= (1 << PB5) | (1 << PB6);

        // Direction pins
        DDRA |= (1 << PA1) | (1 << PA3) | (1 << PA5) | (1 << PA7);
        PORTA &= ~((1 << PA1) | (1 << PA3) | (1 << PA5) | (1 << PA7));

        // Timer1 configuration: Mode 14, prescaler 8
        TCCR1A = (1 << COM1A1) | (1 << COM1B1);
        TCCR1B = (1 << WGM13) | (1 << CS11);
        ICR1 = PWM_TOP;
        OCR1A = 0;
        OCR1B = 0;
    }



    //////////////////////////////////////////////////////////////////////////
    //                          MOTOR CONTROL
    //////////////////////////////////////////////////////////////////////////

    void set_motor_speeds(int16_t left_speed, int16_t right_speed) {
        uint16_t left_pwm = 0;
        uint16_t right_pwm = 0;

        if (abs(left_speed) > DEADZONE) {
            left_pwm = (abs(left_speed) * PWM_TOP) / 128;
            if (left_pwm > PWM_TOP) left_pwm = PWM_TOP;
        }

        if (abs(right_speed) > DEADZONE) {
            right_pwm = (abs(right_speed) * PWM_TOP) / 128;
            if (right_pwm > PWM_TOP) right_pwm = PWM_TOP;
        }

        OCR1A = left_pwm;
        OCR1B = right_pwm;

        // Left motor direction pins: PA1 (forward), PA3 (backward)
        if (left_speed > DEADZONE) {
            PORTA |= (1 << PA1);
            PORTA &= ~(1 << PA3);
        } else if (left_speed < -DEADZONE) {
            PORTA &= ~(1 << PA1);
            PORTA |= (1 << PA3);
        } else {
            PORTA &= ~((1 << PA1) | (1 << PA3));
        }

        // Right motor direction pins: PA5 (forward), PA7 (backward)
        if (right_speed > DEADZONE) {
            PORTA |= (1 << PA5);
            PORTA &= ~(1 << PA7);
        } else if (right_speed < -DEADZONE) {
            PORTA &= ~(1 << PA5);
            PORTA |= (1 << PA7);
        } else {
            PORTA &= ~((1 << PA5) | (1 << PA7));
        }
    }

    void manual_control(void) {
        int16_t x = (int16_t)x_val - 128;
        int16_t y = (int16_t)y_val - 128;

        int16_t left_motor = y + x;
        int16_t right_motor = y - x;

        // Clamp motor speeds
        if (left_motor > 127) left_motor = 127;
        if (left_motor < -128) left_motor = -128;
        if (right_motor > 127) right_motor = 127;
        if (right_motor < -128) right_motor = -128;

        set_motor_speeds(left_motor, right_motor);

        left_motor_feedback = abs(left_motor);
        right_motor_feedback = abs(right_motor);
    }

    //////////////////////////////////////////////////////////////////////////
    //                          SENSOR READING & CONVERSION
    //////////////////////////////////////////////////////////////////////////

    void read_sensors(void) {
        // Read raw ADC values
        uint16_t short_range_raw_left = adc_read(1);   // Short range sensor ADC1
        uint16_t short_range_raw_right = adc_read(2);  // Short range sensor ADC2
        uint16_t long_range_raw = adc_read(0);         // Long range sensor ADC0

        // Convert ADC values to distance (cm) using given formulas
        float short_range_dist_left_f = 0.0f;
        if ((float)short_range_raw_left > 16.568f) {
            short_range_dist_left_f = 942.71f / ((float)short_range_raw_left - 16.568f);
        }

        float short_range_dist_right_f = 0.0f;
        if ((float)short_range_raw_right > 16.568f) {
            short_range_dist_right_f = 942.71f / ((float)short_range_raw_right - 16.568f);
        }

        float long_range_dist_f = 0.0f;
        if ((float)long_range_raw > 25.079f) {
            long_range_dist_f = 4531.8f / ((float)long_range_raw - 25.079f);
        }

        // Clamp distances to max 255 cm for 8-bit uint
        if (short_range_dist_left_f > 255.0f) short_range_dist_left_f = 255.0f;
        if (short_range_dist_right_f > 255.0f) short_range_dist_right_f = 255.0f;
        if (long_range_dist_f > 255.0f) long_range_dist_f = 255.0f;

        // Assign converted values to global sensor variables
        range_sensor = (uint8_t)long_range_dist_f;
        // **UPDATE INDIVIDUAL SHORT-RANGE GLOBAL VARIABLES**
        short_range_left_cm = (uint8_t)short_range_dist_left_f;
        short_range_right_cm = (uint8_t)short_range_dist_right_f;

        // Average short range distances for 'frequency_sensor' (if still used as a general average or for beacon sim)
        float short_range_avg = (short_range_dist_left_f + short_range_dist_right_f) / 2.0f;
        frequency_sensor = (uint8_t)short_range_avg; // Renamed to clarify its purpose for beacon/general average

        status_byte = Auto ? 1 : 0;
    }

    //////////////////////////////////////////////////////////////////////////
    //                          SERIAL COMMUNICATION
    //////////////////////////////////////////////////////////////////////////

    ISR(USART1_RX_vect) {
        static enum { WAIT_START, BYTE1, BYTE2, BYTE3, BYTE4, WAIT_STOP } rx_state = WAIT_START;
        static uint8_t buffer[4];
        uint8_t byte = UDR1;

        switch (rx_state) {
            case WAIT_START:
                if (byte == START_BYTE) rx_state = BYTE1;
                break;
            case BYTE1: case BYTE2: case BYTE3: case BYTE4:
                buffer[rx_state - BYTE1] = byte;
                rx_state++;
                break;
            case WAIT_STOP:
                if (byte == STOP_BYTE) {
                    for (int i = 0; i < 4; i++) {
                        data_from_controller[i] = buffer[i];
                    }
                    new_msg_flag = true;
                }
                rx_state = WAIT_START;
                break;
        }
    }

    void send_data_to_controller(void) {
        serial1_write_byte(START_BYTE);
        serial1_write_byte(range_sensor);
        serial1_write_byte(frequency_sensor); // Still sending the averaged value for general purpose/beacon feedback
        serial1_write_byte(left_motor_feedback);
        serial1_write_byte(right_motor_feedback);
        serial1_write_byte(status_byte);
        serial1_write_byte(STOP_BYTE);
    }

    //////////////////////////////////////////////////////////////////////////
    //              AUTONOMOUS CONTROL MODULE
    //////////////////////////////////////////////////////////////////////////

    // Autonomous control parameters
    #define OBSTACLE_THRESHOLD 30    // Distance threshold for obstacle detection
    #define TURN_SPEED 60           // Speed for turning maneuvers
    #define FORWARD_SPEED 80        // Speed for forward movement
    #define SEARCH_SPEED 40         // Speed for searching/exploration

    // **NEW: Define turn/reverse durations as constants**
    #define TURN_DURATION_MS 600    // Increased slightly for more effective turns
    #define REVERSE_DURATION_MS 400

    // Autonomous states
    typedef enum {
        AUTO_FORWARD,
        AUTO_TURN_LEFT,
        AUTO_TURN_RIGHT,
        AUTO_REVERSE,
        AUTO_SEARCH,
        AUTO_BEACON_TRACK
    } auto_state_t;

    static auto_state_t current_state = AUTO_FORWARD;
    static unsigned long state_start_time = 0;

    // External variables (from main robot code) - These are already global in this file
    // extern uint8_t range_sensor;
    // extern uint8_t frequency_sensor;
    // extern uint8_t left_motor_feedback;
    // extern uint8_t right_motor_feedback;

    //////////////////////////////////////////////////////////////////////////
    //                    SENSOR READING FUNCTIONS (FOR AUTONOMOUS MODULE)
    //////////////////////////////////////////////////////////////////////////

    uint8_t read_front_distance(void) {
        // Use global sensor variable already updated
        return range_sensor;
    }

    uint8_t read_left_distance(void) {
        // **FIXED: Use individual left short range sensor reading**
        return short_range_left_cm;
    }

    uint8_t read_right_distance(void) {
        // **FIXED: Use individual right short range sensor reading**
        return short_range_right_cm;
    }

    uint8_t read_beacon_frequency(void) {
        return frequency_sensor;  // This can remain the averaged value for beacon simulation
    }

    uint8_t read_light_level(void) {
        // Not used here, return 0
        return 0;
    }

    //////////////////////////////////////////////////////////////////////////
    //              AUTONOMOUS BEHAVIORS
    //////////////////////////////////////////////////////////////////////////

    void obstacle_avoidance(int16_t *left_speed, int16_t *right_speed) {
        uint8_t front_dist = read_front_distance();
        uint8_t left_dist = read_left_distance();
        uint8_t right_dist = read_right_distance();
        unsigned long current_time = milliseconds_now();

        switch (current_state) {
            case AUTO_FORWARD:
                if (front_dist < OBSTACLE_THRESHOLD) {
                    // Obstacle ahead, decide which way to turn
                    if (left_dist > right_dist) {
                        current_state = AUTO_TURN_LEFT;
                    } else {
                        current_state = AUTO_TURN_RIGHT;
                    }
                    state_start_time = current_time;
                } else if (left_dist < OBSTACLE_THRESHOLD) {
                    // Obstacle on left, turn right
                    current_state = AUTO_TURN_RIGHT;
                    state_start_time = current_time;
                } else if (right_dist < OBSTACLE_THRESHOLD) {
                    // Obstacle on right, turn left
                    current_state = AUTO_TURN_LEFT;
                    state_start_time = current_time;
                } else {
                    // Clear path, move forward
                    *left_speed = FORWARD_SPEED;
                    *right_speed = FORWARD_SPEED;
                }
                break;

            case AUTO_TURN_LEFT:
                *left_speed = -TURN_SPEED;
                *right_speed = TURN_SPEED;

                // Turn for a fixed duration or until front is clear
                if ((current_time - state_start_time > TURN_DURATION_MS) || (front_dist > OBSTACLE_THRESHOLD + 5)) { // Added buffer
                    current_state = AUTO_FORWARD;
                }
                break;

            case AUTO_TURN_RIGHT:
                *left_speed = TURN_SPEED;
                *right_speed = -TURN_SPEED;

                // Turn for a fixed duration or until front is clear
                if ((current_time - state_start_time > TURN_DURATION_MS) || (front_dist > OBSTACLE_THRESHOLD + 5)) { // Added buffer
                    current_state = AUTO_FORWARD;
                }
                break;

            case AUTO_REVERSE: // This state is not currently being entered in the obstacle_avoidance logic
                *left_speed = -FORWARD_SPEED;
                *right_speed = -FORWARD_SPEED;

                // Reverse for a short duration
                if (current_time - state_start_time > REVERSE_DURATION_MS) {
                    current_state = (left_dist > right_dist) ? AUTO_TURN_LEFT : AUTO_TURN_RIGHT;
                    state_start_time = current_time;
                }
                break;

            default:
                current_state = AUTO_FORWARD;
                break;
        }
    }

    void beacon_tracking(int16_t *left_speed, int16_t *right_speed) {
        uint8_t beacon_freq = read_beacon_frequency();

        if (beacon_freq > 0) {  // Beacon detected (assuming non-zero frequency means detection)
            // For simplicity, just move forward at search speed
            *left_speed = SEARCH_SPEED;
            *right_speed = SEARCH_SPEED;
        } else {
            // No beacon, simple search pattern (turn left)
            *left_speed = -SEARCH_SPEED;
            *right_speed = SEARCH_SPEED;
        }
    }

    void wall_following(int16_t *left_speed, int16_t *right_speed) {
        uint8_t front_dist = read_front_distance();
        uint8_t right_dist = read_right_distance(); // Following the right wall

        const uint8_t WALL_DISTANCE = 15;  // Desired distance from wall (cm)
        const int8_t KP = 2;               // Proportional gain

        if (front_dist < OBSTACLE_THRESHOLD) {
            // Wall ahead, turn left to avoid head-on collision
            *left_speed = -TURN_SPEED;
            *right_speed = TURN_SPEED;
        } else {
            // Follow right wall using a proportional control loop
            int16_t error = right_dist - WALL_DISTANCE;
            int16_t correction = error * KP;

            // Apply correction: if too far from wall (positive error), turn right (left motor faster).
            // If too close (negative error), turn left (right motor faster).
            *left_speed = FORWARD_SPEED + correction;
            *right_speed = FORWARD_SPEED - correction;

            // Limit speeds to prevent runaway or reverse in wall following
            if (*left_speed > 100) *left_speed = 100;
            if (*left_speed < -100) *left_speed = -100; // Allow slight reverse if error is large
            if (*right_speed > 100) *right_speed = 100;
            if (*right_speed < -100) *right_speed = -100; // Allow slight reverse if error is large
        }
    }

    //////////////////////////////////////////////////////////////////////////
    //                    MAIN AUTONOMOUS CONTROL FUNCTION
    //////////////////////////////////////////////////////////////////////////

    void autonomous_control(void) {
        int16_t left_speed = 0;
        int16_t right_speed = 0;

        // Choose behavior based on conditions
        uint8_t beacon_freq = read_beacon_frequency();

        static bool use_wall_following = false;
        static unsigned long strategy_timer = 0;
        unsigned long now = milliseconds_now();

        if (beacon_freq > 0) {
            // Beacon detected, track it
            beacon_tracking(&left_speed, &right_speed);
        } else {
            // No beacon, do obstacle avoidance or wall following
            if (use_wall_following) {
                wall_following(&left_speed, &right_speed);
            } else {
                obstacle_avoidance(&left_speed, &right_speed);
            }

            // Switch strategies periodically
            // **NEW: Define strategy switch interval as a constant**
            #define STRATEGY_SWITCH_INTERVAL_MS 15000 // Switch every 15 seconds
            if (now - strategy_timer > STRATEGY_SWITCH_INTERVAL_MS) {
                use_wall_following = !use_wall_following;
                strategy_timer = now;
            }
        }

        // Apply the calculated speeds
        set_motor_speeds(left_speed, right_speed);

        // Update feedback values
        left_motor_feedback = abs(left_speed);
        right_motor_feedback = abs(right_speed);
    }

    //////////////////////////////////////////////////////////////////////////
    //                    UTILITY FUNCTIONS
    //////////////////////////////////////////////////////////////////////////

    void reset_autonomous_state(void) {
        current_state = AUTO_FORWARD;
        state_start_time = milliseconds_now();
    }

    auto_state_t get_autonomous_state(void) {
        return current_state;
    }

    //////////////////////////////////////////////////////////////////////////
    //                          MAIN LOOP
    //////////////////////////////////////////////////////////////////////////

    int main(void) {
        cli(); // Disable global interrupts
        serial0_init();
        serial1_init();
        milliseconds_init();
        adc_init();
        motors_init();

        UCSR1B |= (1 << RXCIE1); // Enable RX interrupt for USART1
        sei(); // Enable global interrupts

        _delay_ms(20); // Small delay for stabilization

        unsigned long current_ms = 0;
        unsigned long last_send_ms = 0;

        while (1) {
            current_ms = milliseconds_now();

            if (new_msg_flag) {
                uint8_t mode_byte = data_from_controller[0];
                x_val = data_from_controller[1];
                y_val = data_from_controller[2];
                servo_val = data_from_controller[3];

                Auto = (mode_byte == 255);
                new_msg_flag = false;
            }

            // Always read sensors to keep global sensor variables updated
            read_sensors();

            if (Auto) {
                autonomous_control();
            } else {
                manual_control();
            }

            // Send data to controller periodically
            if ((current_ms - last_send_ms) >= 100) { // Every 100 ms
                send_data_to_controller();
                last_send_ms = current_ms;
            }

            // Debug output to Serial0 (PC)
            sprintf(serial_string, "Mode:%s X:%u Y:%u Servo:%u Range:%u LeftSR:%u RightSR:%u\r\n",
                    Auto ? "AUTO" : "MAN", x_val, y_val, servo_val, range_sensor, short_range_left_cm, short_range_right_cm);
            serial0_print_string(serial_string);

            _delay_ms(10); // Small delay to prevent busy-waiting too intensely and allow other tasks
        }

        return 0;
    }