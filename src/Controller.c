#include "Controller.h" // Includes necessary libraries like serial0.h, serial1.h, milliseconds.h, adc.h

char serial_string[60] = {0}; // Debug output string

unsigned long current_ms = 0;
unsigned long last_send_ms = 0;

int main(void)
{
  // --- Initialization ---
  serial0_init();     // For debugging on PC (e.g., via USB-to-serial adapter)
  serial1_init();     // For Bluetooth communication with the Robot
  milliseconds_init(); // Initialize millisecond timer for delays
  adc_init();         // Initialize Analog-to-Digital Converter for joystick
  _delay_ms(20);      // Small delay for peripherals to stabilize

  // This constant byte will be sent to the robot.
  // You can tie this to a button or switch on your controller if needed.
  const uint8_t constant_control_byte = 1; // Example: 1 for enabled, 0 for disabled

  // --- Main Program Loop ---
  while (1)
  {
    current_ms = milliseconds; // Get current time in milliseconds

    // === Read Joystick Values ===
    // Assuming ADC channel 0 for X-axis and ADC channel 1 for Y-axis.
    // ADC returns 10-bit values (0-1023).
    uint16_t x_adc_val = adc_read(0);
    uint16_t y_adc_val = adc_read(1);

    // === Convert 10-bit ADC to 8-bit byte ===
    // Shifting right by 2 effectively divides by 4, mapping 0-1023 to 0-255.
    uint8_t x_byte = (uint8_t)(x_adc_val >> 2);
    uint8_t y_byte = (uint8_t)(y_adc_val >> 2);

    // === Prepare Data Packet using databytes array ===
    // Define an array to hold the data bytes for the packet.
    // Order: [Constant, X-axis, Y-axis]
    uint8_t databytes[3]; // We have 3 data bytes to send

    databytes[0] = constant_control_byte;
    databytes[1] = x_byte;
    databytes[2] = y_byte;

    // === Debug Print to PC Serial Monitor (serial0) ===
    // This helps you verify what data is being read and prepared for sending.
    sprintf(serial_string, "Tx (Controller): Const:%u, X:%u, Y:%u\r\n", 
            databytes[0], databytes[1], databytes[2]);
    serial0_print_string(serial_string);

    // === Send Data via Bluetooth (serial1) ===
    // Only send data every 100 milliseconds to avoid overwhelming the receiver
    // and to provide a consistent update rate.
    if ((current_ms - last_send_ms) >= 100)
    {
      last_send_ms = current_ms; // Update last send time

      // Packet format: [START_BYTE, DATABYTE0, DATABYTE1, DATABYTE2, END_BYTE]
      // Using 254 (0xFE) as a distinct start byte and 253 (0xFD) as an end byte
      serial1_write_byte(254); // Start byte (0xFE)

      // Send all data bytes from the array
      for (int i = 0; i < sizeof(databytes); i++) {
          serial1_write_byte(databytes[i]);
      }
      
      serial1_write_byte(253); // End byte (0xFD)
    }
  }

  return 0; // Should not be reached in an infinite loop
}