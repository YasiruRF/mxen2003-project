#include "Robot.h" // Assuming this includes necessary AVR headers and function prototypes (like milliseconds_init(), adc_init(), serial0_init(), serial1_init(), _delay_ms, etc.)
#include <stdio.h>
#include <math.h>   // For expf, fabsf

#define START_BYTE 0xFE
#define STOP_BYTE  0xFD
#define PWM_TOP    2000
#define DEADZONE   20
#define MOTOR_SPEED (PWM_TOP * 4 / 8)

volatile uint8_t data_from_controller[4] = {0};
volatile bool new_msg_flag = false;

volatile bool Auto = false;
uint8_t x_val = 128;
uint8_t y_val = 128;
uint8_t servo_val = 128;

uint8_t range_sensor_feedback = 0; // Renamed to avoid conflict with `range_sensor` in NN code
uint8_t left_motor_feedback = 0;
uint8_t right_motor_feedback = 0;
uint8_t status_byte = 0;
uint8_t short_range_left_cm_feedback = 0; // Renamed
uint8_t short_range_right_cm_feedback = 0; // Renamed

char serial_string[100] = {0};

//////////////////////////////////////////////////////////////////////////
//                          SERVO PWM (PB7/OC2A)
//////////////////////////////////////////////////////////////////////////

volatile uint16_t servo_us = 1500; // microseconds (1000–2000)
volatile bool servo_pulse = false;

void servo_init(void) {
    DDRB |= (1 << PB7); // PB7 output (OC2A)
    TCCR2A = (1 << WGM21); // CTC mode
    TCCR2B = (1 << CS21);  // prescaler 8, 16 MHz / 8 = 2 MHz
    OCR2A = 19;            // 10 us: 2 MHz * 10 us = 20, OCR2A counts from 0, so 19
    TIMSK2 = (1 << OCIE2A);
}

volatile uint16_t servo_tick = 0;
volatile uint16_t servo_high_ticks = 150; // 150 * 10 us = 1500 us
volatile uint16_t servo_period_ticks = 2000; // 2000 * 10 us = 20 ms

ISR(TIMER2_COMPA_vect) {
    servo_tick++;
    if (servo_pulse) {
        if (servo_tick >= servo_high_ticks) {
            PORTB &= ~(1 << PB7); // Set low
            servo_pulse = false;
        }
    } else {
        if (servo_tick >= servo_period_ticks) {
            servo_tick = 0;
            PORTB |= (1 << PB7); // Set high
            servo_pulse = true;
        }
    }
}

void servo_set(uint16_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    servo_us = us;
    servo_high_ticks = us / 10; // Each tick = 10 us
}

//////////////////////////////////////////////////////////////////////////
//                          MOTOR CONTROL
//////////////////////////////////////////////////////////////////////////

// Motor control functions declarations (now in global scope)
void move_forward(void);
void move_backward(void);
void turn_left(void);
void turn_right(void);
void stay_idle(void);


void motors_init(void) {
    DDRB |= (1 << PB5) | (1 << PB6);
    DDRA |= (1 << PA1) | (1 << PA3) | (1 << PA5) | (1 << PA7);
    PORTA &= ~((1 << PA1) | (1 << PA3) | (1 << PA5) | (1 << PA7));
    TCCR1A = (1 << COM1A1) | (1 << COM1B1);
    TCCR1B = (1 << WGM13) | (1 << CS11);
    ICR1 = PWM_TOP;
    OCR1A = 0;
    OCR1B = 0;
}

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

    if (left_speed > DEADZONE) {
        PORTA |= (1 << PA1);
        PORTA &= ~(1 << PA3);
    } else if (left_speed < -DEADZONE) {
        PORTA &= ~(1 << PA1);
        PORTA |= (1 << PA3);
    } else {
        PORTA &= ~((1 << PA1) | (1 << PA3));
    }

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
    if (left_motor > 127) left_motor = 127;
    if (left_motor < -128) left_motor = -128;
    if (right_motor > 127) right_motor = 127;
    if (right_motor < -128) right_motor = -128;
    set_motor_speeds(left_motor, right_motor);
    left_motor_feedback = abs(left_motor);
    right_motor_feedback = abs(right_motor);
}

// Motor control functions definitions
void move_backward(void)
{
    OCR1A = MOTOR_SPEED;
    OCR1B = MOTOR_SPEED;
    PORTA |= (1 << PA1) | (1 << PA5);    // Both motors forward
    PORTA &= ~((1 << PA3) | (1 << PA7)); // Reverse pins low
}

void move_forward(void)
{
    OCR1A = MOTOR_SPEED;
    OCR1B = MOTOR_SPEED;
    PORTA |= ((1 << PA3) | (1 << PA7)); // Both motors backward
    PORTA &= ~((1 << PA1) | (1 << PA5));
}

void turn_left(void)
{
    OCR1A = MOTOR_SPEED;  // Left motor
    OCR1B = MOTOR_SPEED;  // Right motor
    PORTA &= ~(1 << PA1); // Left motor reverse
    PORTA |= (1 << PA3);
    PORTA |= (1 << PA5); // Right motor forward
    PORTA &= ~(1 << PA7);
}

void turn_right(void)
{
    OCR1A = MOTOR_SPEED; // Left motor
    OCR1B = MOTOR_SPEED; // Right motor
    PORTA |= (1 << PA1); // Left motor forward
    PORTA &= ~(1 << PA3);
    PORTA &= ~(1 << PA5); // Right motor reverse
    PORTA |= (1 << PA7);
}

void stay_idle(void)
{
    OCR1A = 0;
    OCR1B = 0;
    PORTA &= ~((1 << PA1) | (1 << PA3) | (1 << PA5) | (1 << PA7));
}

//////////////////////////////////////////////////////////////////////////
//                          SENSOR READING & CONVERSION
//////////////////////////////////////////////////////////////////////////

// ADC channel definitions (re-defined from your Robot.c for clarity)
#define L_Range_Sensor 0 // Long range sensor on A0
#define Left_Sensor 1    // Left sensor on A1
#define Right_Sensor 2   // Right sensor on A2

uint16_t long_range_value;
uint16_t left_sensor_value;
uint16_t right_sensor_value;

void read_sensors(void) {
    uint16_t short_range_raw_left = adc_read(Left_Sensor);
    uint16_t short_range_raw_right = adc_read(Right_Sensor);
    uint16_t long_range_raw = adc_read(L_Range_Sensor);

    float short_left = 0.0f, short_right = 0.0f, long_front = 0.0f;
    if ((float)short_range_raw_left > 16.568f)
        short_left = 942.71f / ((float)short_range_raw_left - 16.568f);
    if ((float)short_range_raw_right > 16.568f)
        short_right = 942.71f / ((float)short_range_raw_right - 16.568f);
    if ((float)long_range_raw > 25.079f)
        long_front = 4531.8f / ((float)long_range_raw - 25.079f);

    // Clamp values to a reasonable range if necessary, but keep original for NN input
    // The NN will use the actual calculated float values
    // These are for the feedback to the controller (capped at 255)
    short_range_left_cm_feedback = (uint8_t)(short_left > 255.0f ? 255 : (short_left < 0.0f ? 0 : short_left));
    short_range_right_cm_feedback = (uint8_t)(short_right > 255.0f ? 255 : (short_right < 0.0f ? 0 : short_right));
    range_sensor_feedback = (uint8_t)(long_front > 255.0f ? 255 : (long_front < 0.0f ? 0 : long_front));

    // Store the actual calculated values for the neural network
    long_range_value = (uint16_t)long_front;
    left_sensor_value = (uint16_t)short_left;
    right_sensor_value = (uint16_t)short_right;

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
                for (int i = 0; i < 4; i++) data_from_controller[i] = buffer[i];
                new_msg_flag = true;
            }
            rx_state = WAIT_START;
            break;
    }
}

void send_data_to_controller(void) {
    serial1_write_byte(START_BYTE);
    serial1_write_byte(range_sensor_feedback);
    serial1_write_byte(left_motor_feedback); // No frequency_sensor
    serial1_write_byte(right_motor_feedback);
    serial1_write_byte(status_byte);
    serial1_write_byte(STOP_BYTE);
}

// Neural Network Parameters (copied from your Robot.c)
#define DETECTION_THRESHOLD 10        // 10cm threshold
#define INPUT_SIZE 6        // 3 sensors + 3 normalized sensors
#define HIDDEN_SIZE 8       // Hidden layer neurons
#define OUTPUT_SIZE 4       // 4 actions: forward, backward, left, right
#define LEARNING_RATE 0.01f
#define MAZE_WIDTH 30       // Average maze width in cm

// Action definitions (copied from your Robot.c)
typedef enum {
    ACTION_FORWARD = 0,
    ACTION_BACKWARD = 1,
    ACTION_LEFT = 2,
    ACTION_RIGHT = 3
} robot_action_t;

// Neural Network Structure (copied from your Robot.c)
typedef struct {
    float input[INPUT_SIZE];
    float hidden[HIDDEN_SIZE];
    float output[OUTPUT_SIZE];
    
    // Weights (simplified fixed-point for embedded systems)
    float weights_input_hidden[INPUT_SIZE][HIDDEN_SIZE];
    float weights_hidden_output[HIDDEN_SIZE][OUTPUT_SIZE];
    float bias_hidden[HIDDEN_SIZE];
    float bias_output[OUTPUT_SIZE];
} neural_network_t;

// Experience buffer for learning (copied from your Robot.c)
typedef struct {
    float state[INPUT_SIZE];
    int action;
    float reward;
    float next_state[INPUT_SIZE];
    int valid;
} experience_t;

#define BUFFER_SIZE 50
experience_t experience_buffer[BUFFER_SIZE];
int buffer_index = 0;

// Global neural network instance and state variables
neural_network_t nn;
uint32_t last_action_time = 0;
int last_action = ACTION_FORWARD;
float last_state[INPUT_SIZE];
int stuck_counter = 0;
uint16_t last_positions[3] = {0, 0, 0}; // For detecting if robot is stuck

// Activation function (ReLU for hidden, softmax for output) (copied from your Robot.c)
float relu(float x) {
    return (x > 0.0f) ? x : 0.0f;
}

float sigmoid(float x) { // Not used in your provided NN, but kept for completeness
    return 1.0f / (1.0f + expf(-x));
}

// Initialize neural network with small random weights (copied from your Robot.c)
void init_neural_network(void) {
    // Initialize weights with small random values
    // Using simple LFSR for pseudo-random numbers on embedded system
    uint16_t seed = 12345;
    
    for (int i = 0; i < INPUT_SIZE; i++) {
        for (int j = 0; j < HIDDEN_SIZE; j++) {
            seed = seed * 1103515245 + 12345; // Simple PRNG
            nn.weights_input_hidden[i][j] = ((float)(seed % 200) - 100) / 1000.0f;
        }
    }
    
    for (int i = 0; i < HIDDEN_SIZE; i++) {
        for (int j = 0; j < OUTPUT_SIZE; j++) {
            seed = seed * 1103515245 + 12345;
            nn.weights_hidden_output[i][j] = ((float)(seed % 200) - 100) / 1000.0f;
        }
        seed = seed * 1103515245 + 12345;
        nn.bias_hidden[i] = ((float)(seed % 100) - 50) / 1000.0f;
    }
    
    for (int i = 0; i < OUTPUT_SIZE; i++) {
        seed = seed * 1103515245 + 12345;
        nn.bias_output[i] = ((float)(seed % 100) - 50) / 1000.0f;
    }
}

// Prepare input features from sensor readings (copied from your Robot.c)
void prepare_input(uint16_t long_range, uint16_t left_sensor, uint16_t right_sensor) {
    // Raw sensor values (normalized)
    nn.input[0] = (float)long_range / 50.0f;  // Normalize to 0-1 range
    nn.input[1] = (float)left_sensor / 30.0f;
    nn.input[2] = (float)right_sensor / 30.0f;
    
    // Derived features
    nn.input[3] = (nn.input[1] + nn.input[2]) / 2.0f;   // Average side distance
    nn.input[4] = fabsf(nn.input[1] - nn.input[2]);     // Side difference
    nn.input[5] = (nn.input[0] > 0.2f) ? 1.0f : 0.0f;   // Front clear indicator
}

// Forward pass through neural network (copied from your Robot.c)
void forward_pass(void) {
    // Input to hidden layer
    for (int j = 0; j < HIDDEN_SIZE; j++) {
        nn.hidden[j] = nn.bias_hidden[j];
        for (int i = 0; i < INPUT_SIZE; i++) {
            nn.hidden[j] += nn.input[i] * nn.weights_input_hidden[i][j];
        }
        nn.hidden[j] = relu(nn.hidden[j]);
    }
    
    // Hidden to output layer
    float max_output = -1000.0f;
    for (int j = 0; j < OUTPUT_SIZE; j++) {
        nn.output[j] = nn.bias_output[j];
        for (int i = 0; i < HIDDEN_SIZE; i++) {
            nn.output[j] += nn.hidden[i] * nn.weights_hidden_output[i][j];
        }
        if (nn.output[j] > max_output) {
            max_output = nn.output[j];
        }
    }
    
    // Softmax normalization
    float sum_exp = 0.0f;
    for (int j = 0; j < OUTPUT_SIZE; j++) {
        nn.output[j] = expf(nn.output[j] - max_output);
        sum_exp += nn.output[j];
    }
    for (int j = 0; j < OUTPUT_SIZE; j++) {
        nn.output[j] /= sum_exp;
    }
}

// Calculate reward based on sensor readings and action taken (copied from your Robot.c)
float calculate_reward(uint16_t long_range, uint16_t left_sensor, uint16_t right_sensor, int action) {
    float reward = 0.0f;
    
    // Base reward for forward movement (encourages exploration)
    if (action == ACTION_FORWARD) {
        reward += 0.1f;
    }
    
    // Penalty for getting too close to walls
    if (long_range < DETECTION_THRESHOLD) {
        reward -= 0.5f;
        if (action == ACTION_FORWARD) reward -= 0.3f; // Extra penalty for moving toward obstacle
    }
    
    if (left_sensor < DETECTION_THRESHOLD || right_sensor < DETECTION_THRESHOLD) {
        reward -= 0.2f;
    }
    
    // Reward for maintaining good distance from walls
    if (long_range > 15 && left_sensor > 8 && right_sensor > 8) {
        reward += 0.2f;
    }
    
    // Penalty for being stuck (similar sensor readings for too long)
    if (stuck_counter > 5) {
        reward -= 0.4f;
    }
    
    // Reward for balanced side distances (center of corridor)
    float side_balance = 1.0f - (fabsf(left_sensor - right_sensor) / MAZE_WIDTH);
    reward += side_balance * 0.1f;
    
    return reward;
}

// Store experience for learning (copied from your Robot.c)
void store_experience(float* state, int action, float reward, float* next_state) {
    experience_t* exp = &experience_buffer[buffer_index];
    
    for (int i = 0; i < INPUT_SIZE; i++) {
        exp->state[i] = state[i];
        exp->next_state[i] = next_state[i];
    }
    exp->action = action;
    exp->reward = reward;
    exp->valid = 1;
    
    buffer_index = (buffer_index + 1) % BUFFER_SIZE;
}

// Simple Q-learning update (simplified for embedded system) (copied from your Robot.c)
void update_weights(void) {
    static int update_counter = 0;
    update_counter++;
    
    // Only update every few steps to save computation
    if (update_counter % 10 != 0) return;
    
    for (int exp_idx = 0; exp_idx < BUFFER_SIZE; exp_idx++) {
        experience_t* exp = &experience_buffer[exp_idx];
        if (!exp->valid) continue;
        
        // Simple gradient update based on reward
        float error = exp->reward;
        
        // Update output layer weights
        for (int i = 0; i < HIDDEN_SIZE; i++) {
            nn.weights_hidden_output[i][exp->action] += LEARNING_RATE * error * nn.hidden[i];
        }
        nn.bias_output[exp->action] += LEARNING_RATE * error;
        
        // Decay the experience
        exp->reward *= 0.9f;
        if (fabsf(exp->reward) < 0.01f) {
            exp->valid = 0;
        }
    }
}

// Select action using epsilon-greedy with neural network (copied from your Robot.c)
int select_action(uint16_t long_range, uint16_t left_sensor, uint16_t right_sensor) {
    static uint16_t epsilon_seed = 54321;
    
    prepare_input(long_range, left_sensor, right_sensor);
    forward_pass();
    
    // Epsilon-greedy action selection (90% exploitation, 10% exploration)
    epsilon_seed = epsilon_seed * 1103515245 + 12345;
    if ((epsilon_seed % 100) < 10) { // 10% exploration
        return epsilon_seed % OUTPUT_SIZE;
    }
    
    // Select action with highest probability
    int best_action = 0;
    float best_prob = nn.output[0];
    for (int i = 1; i < OUTPUT_SIZE; i++) {
        if (nn.output[i] > best_prob) {
            best_prob = nn.output[i];
            best_action = i;
        }
    }
    
    // Safety overrides for critical situations
    if (long_range <= DETECTION_THRESHOLD && best_action == ACTION_FORWARD) {
        return ACTION_BACKWARD;
    }
    
    return best_action;
}

// Check if robot is stuck (copied from your Robot.c)
int is_stuck(uint16_t long_range, uint16_t left_sensor, uint16_t right_sensor) {
    uint16_t current_pos = long_range + left_sensor + right_sensor;
    
    // Check if position hasn't changed much
    int similar_count = 0;
    for (int i = 0; i < 3; i++) {
        if (abs(current_pos - last_positions[i]) < 5) {
            similar_count++;
        }
    }
    
    // Shift position history
    last_positions[2] = last_positions[1];
    last_positions[1] = last_positions[0];
    last_positions[0] = current_pos;
    
    return similar_count >= 2;
}

// Execute the selected action (copied from your Robot.c)
void execute_action(int action) {
    switch (action) {
        case ACTION_FORWARD:
            move_forward();
            break;
        case ACTION_BACKWARD:
            move_backward();
            break;
        case ACTION_LEFT:
            turn_left();
            break;
        case ACTION_RIGHT:
            turn_right();
            break;
        default:
            stay_idle();
            break;
    }
}

// This function encapsulates your entire neural network autonomous logic
void neural_network_autonomous_control(void) {
    uint32_t current_time;
    int selected_action;
    float current_reward;

    // Fix: Access milliseconds as a variable, not a function call
    current_time = milliseconds; 
    
    // The sensor values (long_range_value, left_sensor_value, right_sensor_value)
    // are updated in the main loop's read_sensors() function,
    // so they are already available here.

    // Check if robot is stuck
    if (is_stuck(long_range_value, left_sensor_value, right_sensor_value)) {
        stuck_counter++;
    } else {
        stuck_counter = 0;
    }

    // Prepare current state
    prepare_input(long_range_value, left_sensor_value, right_sensor_value);
    
    // Calculate reward for previous action
    if (last_action_time > 0) {
        current_reward = calculate_reward(long_range_value, left_sensor_value, right_sensor_value, last_action);
        store_experience(last_state, last_action, current_reward, nn.input);
    }
    
    // Select next action using ML model
    selected_action = select_action(long_range_value, left_sensor_value, right_sensor_value);
    
    // Execute the action
    execute_action(selected_action);
    
    // Print comprehensive status (to serial0, assumed connected to a debug terminal)
    sprintf(serial_string, "NN: L:%u L:%u R:%u | Act:%d Rew:%.2f Stuck:%d | Out:[%.2f %.2f %.2f %.2f]\r\n",
            long_range_value, left_sensor_value, right_sensor_value, 
            selected_action, current_reward, stuck_counter,
            nn.output[0], nn.output[1], nn.output[2], nn.output[3]);
    serial0_print_string(serial_string);
    
    // Store current state for next iteration
    for (int i = 0; i < INPUT_SIZE; i++) {
        last_state[i] = nn.input[i];
    }
    last_action = selected_action;
    last_action_time = current_time;
    
    // Update neural network weights
    update_weights();

    // The _delay_ms(100) from your original NN code should be managed by the main loop's delay,
    // or adjusted as needed within the `main` while loop.
}

int main(void) {
    // Disable global interrupts initially
    cli();

    // Initialize core robot systems
    serial0_init();      // Debug serial
    serial1_init();      // Controller communication serial
    milliseconds_init(); // Millisecond timer
    adc_init();          // Analog-to-Digital Converter
    motors_init();       // Motor PWM and direction control
    servo_init();        // Servo motor control

    // Enable USART1 Receive Complete Interrupt
    UCSR1B |= (1 << RXCIE1); 
    
    // Enable global interrupts
    sei();

    _delay_ms(20); // Small delay after initialization

    // Initialize the neural network for autonomous mode
    init_neural_network(); // IMPORTANT: Call this once at startup

    serial0_print_string("Robot Ready\r\n");

    while (1) {
        // Read all sensor values
        read_sensors();

        // Check for new messages from the controller
        if (new_msg_flag) {
            // Data received: [Auto_Mode_Byte, X_Joystick, Y_Joystick, Servo_Value]
            Auto      = (data_from_controller[0] == 255); // Auto mode if first byte is 255
            x_val     = data_from_controller[1];
            y_val     = data_from_controller[2];
            servo_val = data_from_controller[3];
            
            // Map servo_val (0-255) to servo_us (1000-2000)
            uint16_t servo_us_mapped = 1000 + ((uint32_t)servo_val * 1000) / 255;
            servo_set(servo_us_mapped);
            
            new_msg_flag = false; // Clear the flag
        }

        // Control logic: Autonomous or Manual
        if (Auto) {
            // Engage the neural network for autonomous control
            neural_network_autonomous_control();
        } else {
            // Manual control based on joystick values
            manual_control();
        }

        // Send feedback data back to the controller
        send_data_to_controller();

        // Print status to debug serial
        sprintf(serial_string, "C:A:%u X:%u Y:%u S:%u Lf:%u Rf:%u St:%u | Sen:LR:%u SR_L:%u SR_R:%u\r\n",
            Auto, x_val, y_val, servo_val, left_motor_feedback, right_motor_feedback, status_byte,
            long_range_value, left_sensor_value, right_sensor_value); // Added actual sensor values
        serial0_print_string(serial_string);

        _delay_ms(20); // Main loop delay
    }

    return 0;
}