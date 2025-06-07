#include "Robot.h" // Includes necessary libraries like serial0.h, serial1.h, milliseconds.h, math.h

#define PWM_TOP 2000    // Timer's TOP value for motor PWM frequency
#define DEADZONE 20     // Joystick deadzone to prevent drift (approx. +/- 20 from 128 center)

// Global variables for serial communication FSM
uint8_t serial_byte_in = 0;   // Stores the currently received byte
uint8_t serial_fsm_state = 0; // State of the serial communication FSM

// Array to hold the data bytes temporarily during reception
// Corresponds to: [Constant, X-axis, Y-axis]
uint8_t temp_recv_databytes[3];

// Confirmed variables after a successful packet reception
uint8_t constant_byte_received = 0;
uint8_t x_val_received = 128;
uint8_t y_val_received = 128;

char serial_string[60] = {0}; // Debug output string for PC serial monitor

int main(void)
{
  // --- Initialization ---
  serial0_init();     // For debugging on PC (e.g., via USB-to-serial adapter)
  serial1_init();     // For Bluetooth communication with the Controller
  milliseconds_init(); // Initialize millisecond timer
  _delay_ms(20);      // Small delay for peripherals to stabilize

  // --- Motor PWM and Direction Pins Setup ---
  DDRB |= (1 << PB5) | (1 << PB6); // PB5 (OC1A), PB6 (OC1B)
  DDRA |= (1 << PA1) | (1 << PA3) | (1 << PA5) | (1 << PA7);
  PORTA &= ~((1 << PA1) | (1 << PA3) | (1 << PA5) | (1 << PA7)); // Initialize direction pins LOW

  // --- Timer1 Configuration for Motor PWM ---
  TCCR1A = (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << WGM13) | (1 << CS11);
  ICR1 = PWM_TOP;

  // --- Enable Serial1 Receive Interrupt ---
  UCSR1B |= (1 << RXCIE1); // Enable USART Receive Complete Interrupt for serial1
  sei();                   // Globally enable interrupts

  // --- Main Program Loop ---
  while (1)
  {
    // The serial reception happens in the ISR (USART1_RX_vect).
    // The main loop continuously checks the received joystick values and controls motors.

    // === Control Motors based on Received Data ===
    // Convert 8-bit joystick values (0-255) to signed integer (-128 to 127).
    int16_t x = (int16_t)x_val_received - 128; // X-axis (-128 to 127)
    int16_t y = (int16_t)y_val_received - 128; // Y-axis (-128 to 127)

    // Differential Drive Calculation
    int16_t lm_speed_cmd = y + x; // Left Motor speed command
    int16_t rm_speed_cmd = y - x; // Right Motor speed command

    // Calculate PWM Duty Cycles
    int16_t lm_duty_cycle = 0;
    int16_t rm_duty_cycle = 0;

    // Apply DEADZONE
    if (abs(x) > DEADZONE || abs(y) > DEADZONE)
    {
      lm_duty_cycle = abs((int32_t)lm_speed_cmd * PWM_TOP / 128);
      rm_duty_cycle = abs((int32_t)rm_speed_cmd * PWM_TOP / 128);

      // Clamp duty cycles to PWM_TOP
      lm_duty_cycle = (lm_duty_cycle > PWM_TOP) ? PWM_TOP : lm_duty_cycle;
      rm_duty_cycle = (rm_duty_cycle > PWM_TOP) ? PWM_TOP : rm_duty_cycle;
    }

    // Set PWM values for motors
    OCR1A = lm_duty_cycle; // PWM for Left Motor
    OCR1B = rm_duty_cycle; // PWM for Right Motor

    // Set Motor Directions
    // Left Motor (PA1/PA3)
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

    // Right Motor (PA5/PA7)
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

    // --- Debug Print to PC Serial Monitor (serial0) ---
    sprintf(serial_string, "Rx (Robot): Const=%u, X=%d, Y=%d\n", 
            constant_byte_received, (int)x, (int)y);
    serial0_print_string(serial_string);
  }

  return 0; // Should not be reached
}

// --- Interrupt Service Routine for Serial1 Reception ---
ISR(USART1_RX_vect)
{
  serial_byte_in = UDR1; // Read the incoming byte

  switch (serial_fsm_state)
  {
    case 0: // Waiting for START_BYTE (254)
      if (serial_byte_in == 254) {
        serial_fsm_state = 1; // Move to receive first data byte
      }
      break;

    case 1: // Expecting DATABYTE0 (Constant Byte)
      temp_recv_databytes[0] = serial_byte_in;
      serial_fsm_state = 2; // Move to receive DATABYTE1
      break;

    case 2: // Expecting DATABYTE1 (X-axis Byte)
      temp_recv_databytes[1] = serial_byte_in;
      serial_fsm_state = 3; // Move to receive DATABYTE2
      break;

    case 3: // Expecting DATABYTE2 (Y-axis Byte)
      temp_recv_databytes[2] = serial_byte_in;
      serial_fsm_state = 4; // Move to receive END_BYTE
      break;

    case 4: // Expecting END_BYTE (253)
      if (serial_byte_in == 253) {
        // Full, valid packet received! Transfer temporary data to active variables.
        constant_byte_received = temp_recv_databytes[0];
        x_val_received = temp_recv_databytes[1];
        y_val_received = temp_recv_databytes[2];
      }
      serial_fsm_state = 0; // Reset FSM to wait for next START_BYTE
      break;

    default: // Error state, reset FSM
      serial_fsm_state = 0;
      break;
  }
}