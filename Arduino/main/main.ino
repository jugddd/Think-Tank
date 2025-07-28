/* =====================================================================
 *  Think Tank - Arduino Motor Controller (Robust Final Version)
 * =====================================================================
 *  This version uses a robust, industry-standard parsing method
 *  (C-style strings) to eliminate any potential errors from the
 *  Arduino String class. The control logic is based on the definitive
 *  hardware ground-truth test. This is the final version.
 *
 *  - Left Stick -> Left Motor (Channel 3, POSITIVE PWM for forward)
 *  - Right Stick -> Right Motor (Channel 1, NEGATIVE PWM for forward)
 * =====================================================================
 */
#include <Wire.h>

#define I2C_ADDR_MOTOR_DRIVER 0x34
#define MOTOR_FIXED_SPEED_ADDR 51
#define MOTOR_TYPE_ADDR 20
#define MOTOR_ENCODER_POLARITY_ADDR 21
#define MOTOR_TYPE_JGB37_520_12V_110RPM 3

// I2C communication function
bool writeDataArray(uint8_t reg, uint8_t *val, unsigned int len) {
    Wire.beginTransmission(I2C_ADDR_MOTOR_DRIVER);
    Wire.write(reg);
    for(unsigned int i = 0; i < len; i++) {
        Wire.write(val[i]);
    }
    return (Wire.endTransmission() == 0);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(200);
    
    // Initialize motor driver
    uint8_t motorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
    uint8_t motorEncoderPolarity = 0;
    writeDataArray(MOTOR_TYPE_ADDR, &motorType, 1);
    delay(5);
    writeDataArray(MOTOR_ENCODER_POLARITY_ADDR, &motorEncoderPolarity, 1);
    delay(5);
}

void loop() {
    static char serial_buffer[32];
    static byte buffer_pos = 0;

    while (Serial.available() > 0) {
        char inChar = Serial.read();

        if (inChar == '\n') {
            serial_buffer[buffer_pos] = '\0'; // Null-terminate the string
            if (buffer_pos > 0 && serial_buffer[0] == 'D') {
                process_drive_command(serial_buffer + 1); // Pass pointer to payload
            }
            buffer_pos = 0; // Reset for next command
        } else {
            if (buffer_pos < sizeof(serial_buffer) - 1) {
                serial_buffer[buffer_pos++] = inChar;
            }
        }
    }
}

void process_drive_command(char* payload) {
    // Use strtok to robustly parse the C-string
    char* left_str = strtok(payload, ",");
    char* right_str = strtok(NULL, ",");

    if (left_str == NULL || right_str == NULL) {
        return; // Parsing failed
    }

    // Use atof for reliable float conversion
    float left_stick = atof(left_str);
    float right_stick = atof(right_str);
    
    // --- GROUND TRUTH MAPPING ---
    int8_t left_motor_speed = (int8_t)(constrain(left_stick, -1.0, 1.0) * 50.0f);
    int8_t right_motor_speed = (int8_t)(constrain(right_stick, -1.0, 1.0) * -50.0f);
    
    // Build command array: [Ch0, Ch1 (Right), Ch2, Ch3 (Left)]
    int8_t motor_speeds[4] = {0, right_motor_speed, 0, left_motor_speed};
    
    writeDataArray(MOTOR_FIXED_SPEED_ADDR, (uint8_t*)motor_speeds, 4);
}
