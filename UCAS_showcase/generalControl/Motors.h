#ifndef motors_h
#define motors_h

#include "Arduino.h"
#include <ARB.h>
#include <Wire.h>
#include <PID_v1.h>

// Define macros
#define PI 3.1415926535897932384626433832795

// MOTORS

#define REG_RECEIVE_DIR_MOTOR_A 30 // Motor A receive direction
#define REG_RECEIVE_PWM_MOTOR_A 31 // Motor A receive PWM
#define REG_SEND_DATA_MOTOR_A 32 // Motor A send data (current direction and speed)

#define REG_RECEIVE_DIR_MOTOR_B 33 // Motor B receive direction
#define REG_RECEIVE_PWM_MOTOR_B 34 // Motor B receive PWM
#define REG_SEND_DATA_MOTOR_B 35 // Motor B send data (current direction and speed)

#define REG_RECEIVE_SPEED_DATA 40 // Receive speed data
#define REG_RECEIVE_MSG_DRIVE 41 // Receive drive control data
#define REG_SEND_MSG_DRIVE 42 // Send drive control data (current state)

// ENCODERS

#define REG_SEND_DATA_ENCODER_A_1 36 // Encoder A send data
#define REG_SEND_DATA_ENCODER_A_2 37 // Encoder A send data
#define REG_SEND_DATA_ENCODER_B_1 38 // Encoder B send data
#define REG_SEND_DATA_ENCODER_B_2 39 // Encoder B send data

#define REG_SEND_DISTANCE_A 43 // Calculated distance in wheel A
#define REG_SEND_DISTANCE_A_DEC 44 // Calculated distance in wheel A (decimal part)

#define REG_SEND_DISTANCE_B 45 // Calculated distance in wheel B
#define REG_SEND_DISTANCE_B_DEC 46 // Calculated distance in wheel B (decimal part)

#define REG_SEND_SPEED_A 47 // Calculated speed of wheel A
#define REG_SEND_SPEED_A_DEC 48 // Calculated speed of wheel A (decimal part)

#define REG_SEND_SPEED_B 49 // Calculated speed of wheel B
#define REG_SEND_SPEED_B_DEC 50 // Calculated speed of wheel B (decimal part)

#define REG_RECEIVE_RESET_ENCODER_A 51 // Resets absolute steps of encoder A
#define REG_RECEIVE_RESET_ENCODER_B 52 // Resets absolute steps of encoder B

class Motors {
  public:
    Motors();
    void initialize();
    void runMotors();
    
  private:
    // Define some enums to make specifying motor and direction easier
    typedef enum {CW,CCW} m_Direction; // CW = 0, CCW = 1
    typedef enum {A,B} m_Motor; // A = 0, B = 1
    
    // Global variables for motor direction so they can be read by encoder ISR to determine direction
    static m_Direction m_dirA;
    static m_Direction m_dirB;
    
    m_Direction m_prevDirA = CW;
    m_Direction m_prevDirB = CCW;
    
    // To check if the value from the direction registers has changed
    m_Direction m_registerDirAprev = CW;
    m_Direction m_registerDirBprev = CCW;
    
    bool m_robotDirectionChanged = false;
    
    // Variables to store step count from motors, must be volatlie to update from within ISR
    volatile static int m_stepsA;
    volatile static int m_stepsB;
    
    // These values will not be reset unless a signal is received
    volatile static int m_absoluteStepsA;
    volatile static int m_absoluteStepsB;

    int m_absoluteStepsAprev = 0;
    int m_absoluteStepsBprev = 0;

    // Absolute steps converted to cm
    double m_absoluteStepsA_cm = 0;
    double m_absoluteStepsB_cm = 0;
    
    // Flags to check if steps have changed
    int m_stepsAprev = 0;
    int m_stepsBprev = 0;
    bool m_newMeasureA = false;
    bool m_newMeasureB = false;
    
    // Variables to store distance count from motors
    double m_distanceA = 0;
    double m_distanceB = 0;
    
    // Variables to store speed count from motors
    float m_speedA = 0;
    float m_speedB = 0;
    
    int m_speedAPWM_prev = 0;
    int m_speedBPWM_prev = 0;
    
    // Global time variables
    double m_previousTimeA = 0;
    double m_previousTimeB = 0;
    double m_currentTimeA = 0;
    double m_currentTimeB = 0;
    
    volatile int m_speedValuePrev = 0;
    volatile int m_inputPrev = 0;

    void m_setPinModes();
    void m_initializeComponents(); // Initializes values
    void m_attachInterrupts(); // Attaches interrupts
    void m_initializeSerialRegisters(); // Initialize serial

    // Wheels
    void m_readWheelDirections();
    void m_readPWMSignals();
  
    // General movement
    void m_readSpeedValue();
    void m_readDirectionInput();
  
    // Encoders
    void m_setAbsoluteEncoderSteps();
    void m_calculateCurrentStepsA();
    void m_calculateCurrentStepsB();

    void m_sendEncoderStepsToRegisters(int t_register1, int t_register2, int t_steps);
    void m_sendDecimalToRegisters(int t_register1, int t_register2, float t_number);
    void m_resetEncoders();
    double m_stepsToCentimetres(int t_steps);

    void m_moveForward();
    void m_moveBackward();
    void m_moveLeft();
    void m_moveRight();
    void m_stopRobot();

    void m_adjustSpeed(int t_speedLevel);
    void m_motorSetDir(m_Motor t_motor, m_Direction t_dir);

    static void m_ENCA_ISR();
    static void m_ENCB_ISR();
};

#endif
