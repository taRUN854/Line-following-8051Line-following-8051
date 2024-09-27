#include <reg51.h>

// motor control pins
#define MOTOR_LEFT_FORWARD P1_0
#define MOTOR_LEFT_BACKWARD P1_1
#define MOTOR_RIGHT_FORWARD P1_2
#define MOTOR_RIGHT_BACKWARD P1_3

// Define IR sensor pins
#define IR_SENSOR_LEFT P2_0
#define IR_SENSOR_RIGHT P2_1

// PID Parameters
#define KP 1.0
#define KI 0.0
#define KD 0.0

// PID variables
float setPoint = 0;  // Desired position (center of line)
float input, output;
float lastInput;
float integral = 0;

// Function prototypes
void init(void);
void moveForward(void);
void turnLeft(void);
void turnRight(void);
void stop(void);
void PIDControl(void);

void main(void) {
    init();
    
    while(1) {
        // Read IR sensor values
        unsigned char leftSensor = IR_SENSOR_LEFT;
        unsigned char rightSensor = IR_SENSOR_RIGHT;
        
        // Simple line-following logic
        if (leftSensor == 0 && rightSensor == 1) {
            turnLeft();
        } else if (leftSensor == 1 && rightSensor == 0) {
            turnRight();
        } else if (leftSensor == 0 && rightSensor == 0) {
            moveForward();
        } else {
            stop();
        }

        // PID control
        input = (float)(leftSensor - rightSensor);
        PIDControl();
        
        // Apply PID output to motor control
        if (output > 0) {
            // Adjust motor speed based on PID output
            MOTOR_LEFT_FORWARD = 1;
            MOTOR_LEFT_BACKWARD = 0;
            MOTOR_RIGHT_FORWARD = 0;
            MOTOR_RIGHT_BACKWARD = 0;
        } else {
            MOTOR_LEFT_FORWARD = 0;
            MOTOR_LEFT_BACKWARD = 1;
            MOTOR_RIGHT_FORWARD = 1;
            MOTOR_RIGHT_BACKWARD = 0;
        }
    }
}

void init(void) {
    // Initialize I/O pins
    P1 = 0x00;  // Set all P1 pins to low
    P2 = 0xFF;  // Set all P2 pins to input
}

void moveForward(void) {
    MOTOR_LEFT_FORWARD = 1;
    MOTOR_LEFT_BACKWARD = 0;
    MOTOR_RIGHT_FORWARD = 1;
    MOTOR_RIGHT_BACKWARD = 0;
}

void turnLeft(void) {
    MOTOR_LEFT_FORWARD = 0;
    MOTOR_LEFT_BACKWARD = 1;
    MOTOR_RIGHT_FORWARD = 1;
    MOTOR_RIGHT_BACKWARD = 0;
}

void turnRight(void) {
    MOTOR_LEFT_FORWARD = 1;
    MOTOR_LEFT_BACKWARD = 0;
    MOTOR_RIGHT_FORWARD = 0;
    MOTOR_RIGHT_BACKWARD = 1;
}

void stop(void) {
    MOTOR_LEFT_FORWARD = 0;
    MOTOR_LEFT_BACKWARD = 0;
    MOTOR_RIGHT_FORWARD = 0;
    MOTOR_RIGHT_BACKWARD = 0;
}

void PIDControl(void) {
    float error = setPoint - input;
    integral += error;
    float derivative = input - lastInput;
    
    output = KP * error + KI * integral + KD * derivative;
    
    lastInput = input;
}
