#include "Motors.h"
#include <ARB.h>
#include <Wire.h>
#include <PID_v1.h>

// CONSTRUCTOR

Motors::Motors(){
}

// CLASS INITIALISATIONS

//PID motorA_PID(&Motors::m_absoluteStepsA_cm, &Motors::m_outputA, &Motors::m_setpointA, Motors::m_kpA, Motors::m_kiA, Motors::m_kdA, DIRECT);
//PID motorB_PID(&Motors::m_absoluteStepsB_cm, &Motors::m_outputB, &Motors::m_setpointB, Motors::m_kpB, Motors::m_kiB, Motors::m_kdB, DIRECT);

// STATIC MEMBER VARIABLE INITIALISATION

Motors::m_Direction Motors::m_dirA = CW;
Motors::m_Direction Motors::m_dirB = CCW;

// Variables to store step count from motors, must be volatlie to update from within ISR
volatile int Motors::m_stepsA = 0;
volatile int Motors::m_stepsB = 0;

// These values will not be reset unless a signal is received
volatile int Motors::m_absoluteStepsA = 0;
volatile int Motors::m_absoluteStepsB = 0;

// PUBLIC METHODS

void Motors::initialize(){
  m_setPinModes();
  m_initializeComponents(); // Initializes values
  m_attachInterrupts(); // Attaches interrupts
  m_initializeSerialRegisters(); // Initialize serial registers
}

void Motors::runMotors(){
  // Wheels
  m_readWheelDirections();
  m_readPWMSignals();

  // General movement
  m_readSpeedValue();
  m_readDirectionInput();

  // Encoders
  m_setAbsoluteEncoderSteps();
  m_calculateCurrentStepsA();
  m_calculateCurrentStepsB();
}

// PRIVATE MEMBERS

void Motors::m_setPinModes(){
  // Set relevant modes for pins
  pinMode(MOTOR_DIRA, OUTPUT);
  pinMode(MOTOR_DIRB, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_PWMB, OUTPUT);
  pinMode(MOTOR_ENCA, INPUT);
  pinMode(MOTOR_ENCB, INPUT);
}

void Motors::m_initializeComponents(){
  // Set motors off by default
  m_moveForward();
  m_stopRobot();

  m_resetEncoders();

  m_newMeasureA = false;
  m_newMeasureB = false;
}

void Motors::m_attachInterrupts(){
  // Attach interrupts to motor encoder inputs so we don't miss any steps
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), m_ENCA_ISR, CHANGE); // Pin number put be converted to interrupt number
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), m_ENCB_ISR, CHANGE); // CHANGE will activate the ISR on either edge, can be changed to either RISING or FALLING
}

void Motors::m_initializeSerialRegisters(){
  // Setup some dummy data in the registers to be read by the Raspberry Pi
  // Motors
  putRegister(REG_RECEIVE_DIR_MOTOR_A, 0); // Motor A receive direction
  putRegister(REG_RECEIVE_PWM_MOTOR_A, 0); // Motor A receive PWM
  putRegister(REG_SEND_DATA_MOTOR_A, 0); // Motor A send data (current direction and speed)
  
  putRegister(REG_RECEIVE_DIR_MOTOR_B, 1); // Motor B receive direction
  putRegister(REG_RECEIVE_PWM_MOTOR_B, 0); // Motor B receive PWM
  putRegister(REG_SEND_DATA_MOTOR_B, 0); // Motor B send data (current direction and speed)

  putRegister(REG_SEND_DATA_ENCODER_A_1, 0); // Encoder A send data
  putRegister(REG_SEND_DATA_ENCODER_A_2, 0); // Encoder A send data
  putRegister(REG_SEND_DATA_ENCODER_B_1, 0); // Encoder B send data
  putRegister(REG_SEND_DATA_ENCODER_B_2, 0); // Encoder B send data

  putRegister(REG_RECEIVE_SPEED_DATA, 0); // Receive speed data
  putRegister(REG_RECEIVE_MSG_DRIVE, 0);// Receive drive control data (forward, backward, left, right)
  putRegister(REG_SEND_MSG_DRIVE, 0); // Send drive control data (current state)

  putRegister(REG_SEND_DISTANCE_A, 0); // Calculated distance in wheel A
  putRegister(REG_SEND_DISTANCE_A_DEC, 0); // Calculated distance in wheel A (decimal part)
  
  putRegister(REG_SEND_DISTANCE_B, 0); // Calculated distance in wheel B
  putRegister(REG_SEND_DISTANCE_B_DEC, 0); // Calculated distance in wheel B (decimal part)
  
  putRegister(REG_SEND_SPEED_A, 0); // Calculated speed of wheel A
  putRegister(REG_SEND_SPEED_A_DEC, 0); // Calculated speed of wheel A (decimal part)
  
  putRegister(REG_SEND_SPEED_B, 0); // Calculated speed of wheel B
  putRegister(REG_SEND_SPEED_B_DEC, 0); // Calculated speed of wheel B (decimal part)

  putRegister(REG_RECEIVE_RESET_ENCODER_A, 0);
  putRegister(REG_RECEIVE_RESET_ENCODER_B, 0);
}

void Motors::m_readWheelDirections(){
  // Read wheel direction signals
  m_Direction registerDirA = (m_Direction)getRegister(REG_RECEIVE_DIR_MOTOR_A);
  m_Direction registerDirB = (m_Direction)getRegister(REG_RECEIVE_DIR_MOTOR_B);

  // If the direction signal for motor A has changed, change direction
  if(registerDirA != m_registerDirAprev){
    m_motorSetDir(A, registerDirA);
    m_registerDirAprev = registerDirA;
    putRegister(REG_SEND_DATA_MOTOR_A, m_dirA); // Send current wheel direction to register
  }

  // If the direction signal for motor B has changed, change direction
  if(registerDirB != m_registerDirBprev){
    m_motorSetDir(B, registerDirB);
    m_registerDirBprev = registerDirB;
    putRegister(REG_SEND_DATA_MOTOR_B, m_dirB); // Send current wheel direction to register
  }
}

void Motors::m_readPWMSignals(){
  // Read PWM signals
  int speedAPWM = (uint8_t)getRegister(REG_RECEIVE_PWM_MOTOR_A);
  int speedBPWM = (uint8_t)getRegister(REG_RECEIVE_PWM_MOTOR_B);

  // If the PWM signal for motor A has changed, change speed level
  if(speedAPWM != m_speedAPWM_prev){
    analogWrite(MOTOR_PWMA, speedAPWM);
    Serial.print("Adjusting motor A PWM speed to: ");
    Serial.println(speedAPWM);
    m_speedAPWM_prev = speedAPWM;
  }

  // If the PWM signal for motor B has changed, change speed level
  if(speedBPWM != m_speedBPWM_prev){
    analogWrite(MOTOR_PWMB, speedBPWM);
    Serial.print("Adjusting motor B PWM speed to: ");
    Serial.println(speedBPWM);
    m_speedBPWM_prev = speedBPWM;
  }
}

void Motors::m_readSpeedValue(){
  // Get data from general robot speed register
  int speedValue = getRegister(REG_RECEIVE_SPEED_DATA);

  // If the data has changed, change speed level
  if(speedValue != m_speedValuePrev){
    m_adjustSpeed(speedValue);
    m_speedValuePrev = speedValue;
  }
}

void Motors::m_readDirectionInput(){
  // Get input data from register (WASD)
  int input = getRegister(REG_RECEIVE_MSG_DRIVE);

  // If the input has changed
  if(input != m_inputPrev){
    // Forward input
    if(input == 1){
      m_moveForward();
      putRegister(REG_SEND_MSG_DRIVE, 1);
    }
    // Backward input
    else if (input == 2){
      m_moveBackward();
      putRegister(REG_SEND_MSG_DRIVE, 2);
    }
    // Left input
    else if (input == 3){
      m_moveLeft();
      putRegister(REG_SEND_MSG_DRIVE, 3);
    }
    // Right input
    else if (input == 4){
      m_moveRight();
      putRegister(REG_SEND_MSG_DRIVE, 4);
    }
    // Anything else stops the motors
    else {
      m_stopRobot();
      putRegister(REG_SEND_MSG_DRIVE, 5);
    }
    
    m_inputPrev = input;
  }
}

void Motors::m_setAbsoluteEncoderSteps(){
  // Absolute encoder steps
  m_sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_A_1, REG_SEND_DATA_ENCODER_A_2, m_absoluteStepsA); // Send data to the register
  m_sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_B_1, REG_SEND_DATA_ENCODER_B_2, m_absoluteStepsB); // Send data to the register

  int resetInputA = getRegister(REG_RECEIVE_RESET_ENCODER_A);
  int resetInputB = getRegister(REG_RECEIVE_RESET_ENCODER_B);

  if(resetInputA == 1){
    m_absoluteStepsA = 0;
    putRegister(REG_RECEIVE_RESET_ENCODER_A, 0);
  }

  if(resetInputB == 1){
    m_absoluteStepsB = 0;
    putRegister(REG_RECEIVE_RESET_ENCODER_B, 0);
  }
}

void Motors::m_calculateCurrentStepsA(){
  // If the steps value has changed (the wheel is not stationary)
  if(m_stepsA != m_stepsAprev) {
    if (m_newMeasureA == false){ // If the robot was not moving before
      m_newMeasureA = true; // Set new measurement
      m_previousTimeA = millis(); // Record initial time
    }

    // If the direction has changed otherwise (of any wheel), and the robot was already moving
    else if(((m_dirA != m_prevDirA) || (m_dirB != m_prevDirB)) && (m_newMeasureA == true)){
      m_robotDirectionChanged = true; // Setup flag to change wheel B as well
      m_newMeasureA = false; // Stop current measurement
      m_currentTimeA = millis(); // Record current time

      // Calculate distance travelled and speed
      m_distanceA = m_stepsToCentimetres(m_stepsA);
      double timeDifferenceInSeconds = (m_currentTimeA - m_previousTimeA)/1000;
      m_speedA = m_distanceA/timeDifferenceInSeconds; // Speed in cm/second
  
      // Print the current number of steps, distance and speed recorded
      Serial.print("A steps: ");
      Serial.println(m_stepsA);
      Serial.print("A distance: ");
      Serial.print(m_distanceA);
      Serial.println(" cm");
      Serial.print("A speed: ");
      Serial.print(m_speedA);
      Serial.println(" cm/s");

      // Send distance and speed data to the registers
      m_sendDecimalToRegisters(REG_SEND_DISTANCE_A, REG_SEND_DISTANCE_A_DEC, m_distanceA);
      m_sendDecimalToRegisters(REG_SEND_SPEED_A, REG_SEND_SPEED_A_DEC, m_speedA);
      // sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_A_1, REG_SEND_DATA_ENCODER_A_2, stepsA);

      // Reset step count value
      m_stepsA = 0;
      m_stepsAprev = 0;
      
      // Start new measurement
      m_newMeasureA = true;
      m_previousTimeA = millis();
    }
    m_prevDirA = m_dirA; // Update wheel direction
    m_stepsAprev = m_stepsA; // Update number of steps
    // sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_A_1, REG_SEND_DATA_ENCODER_A_2, stepsA); // Send data to the register
  }

  // If the wheel has stopped
  else if((m_stepsA == m_stepsAprev) && (m_newMeasureA == true)){
    m_newMeasureA = false; // Stop current measurement
    m_currentTimeA = millis(); // Record current time

    // Calculate distance travelled and speed
    m_distanceA = m_stepsToCentimetres(m_stepsA);
    double timeDifferenceInSeconds = (m_currentTimeA - m_previousTimeA)/1000;
    m_speedA = m_distanceA/timeDifferenceInSeconds; // Speed in cm/second

    // Print the current number of steps, distance and speed recorded
    Serial.print("A steps: ");
    Serial.println(m_stepsA);
    Serial.print("A distance: ");
    Serial.print(m_distanceA);
    Serial.println(" cm");
    Serial.print("A speed: ");
    Serial.print(m_speedA);
    Serial.println(" cm/s");

    // Send distance and speed data to the registers
    m_sendDecimalToRegisters(REG_SEND_DISTANCE_A, REG_SEND_DISTANCE_A_DEC, m_distanceA);
    m_sendDecimalToRegisters(REG_SEND_SPEED_A, REG_SEND_SPEED_A_DEC, m_speedA);
    // sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_A_1, REG_SEND_DATA_ENCODER_A_2, stepsA);

    // Reset step count value
    m_stepsA = 0;
    m_stepsAprev = 0;
  }
}

void Motors::m_calculateCurrentStepsB(){
  // If the steps value has changed (the wheel is not stationary)
  if(m_stepsB != m_stepsBprev) {
    if (m_newMeasureB == false){ // If the robot was not moving before
      m_newMeasureB = true; // Set new measurement
      m_previousTimeB = millis(); // Record initial time
    }

    // If the direction has changed otherwise (of any wheel), and the robot was already moving
    else if(((m_dirB != m_prevDirB) || m_robotDirectionChanged) && (m_newMeasureB == true)){
      m_robotDirectionChanged = false; // Remove flag to change wheel B
      m_newMeasureB = false; // Stop current measurement
      m_currentTimeB = millis(); // Record current time

      // Calculate distance travelled and speed
      m_distanceB = m_stepsToCentimetres(m_stepsB);
      double timeDifferenceInSeconds = (m_currentTimeB - m_previousTimeB)/1000;
      m_speedB = m_distanceB/timeDifferenceInSeconds; // Speed in cm/second
  
      // Print the current number of steps, distance and speed recorded
      Serial.print("B steps: ");
      Serial.println(m_stepsB);
      Serial.print("B distance: ");
      Serial.print(m_distanceB);
      Serial.println(" cm");
      Serial.print("B speed: ");
      Serial.print(m_speedB);
      Serial.println(" cm/s");

      // Send distance and speed data to the registers
      m_sendDecimalToRegisters(REG_SEND_DISTANCE_B, REG_SEND_DISTANCE_B_DEC, m_distanceB);
      m_sendDecimalToRegisters(REG_SEND_SPEED_B, REG_SEND_SPEED_B_DEC, m_speedB);
      // sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_B_1, REG_SEND_DATA_ENCODER_B_2, stepsB);

      // Reset step count value
      m_stepsB = 0;
      m_stepsBprev = 0;

      // Start new measurement
      m_newMeasureB = true;
      m_previousTimeB = millis();
    }
    m_prevDirB = m_dirB; // Update wheel direction
    m_stepsBprev = m_stepsB; // Update number of steps
    // sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_B_1, REG_SEND_DATA_ENCODER_B_2, stepsB); // Send data to the register
  }

  // If the wheel has stopped
  else if((m_stepsB == m_stepsBprev) && (m_newMeasureB == true)){
    m_newMeasureB = false; // Stop current measurement
    m_currentTimeB = millis(); // Record current time

    // Calculate distance travelled and speed
    m_distanceB = m_stepsToCentimetres(m_stepsB);
    double timeDifferenceInSeconds = (m_currentTimeB - m_previousTimeB)/1000;
    m_speedB = m_distanceB/timeDifferenceInSeconds; // Speed in cm/second

    // Print the current number of steps, distance and speed recorded
    Serial.print("B steps: ");
    Serial.println(m_stepsB);
    Serial.print("B distance: ");
    Serial.print(m_distanceB);
    Serial.println(" cm");
    Serial.print("B speed: ");
    Serial.print(m_speedB);
    Serial.println(" cm/s");

    // Send distance and speed data to the registers
    m_sendDecimalToRegisters(REG_SEND_DISTANCE_B, REG_SEND_DISTANCE_B_DEC, m_distanceB);
    m_sendDecimalToRegisters(REG_SEND_SPEED_B, REG_SEND_SPEED_B_DEC, m_speedB);
    // sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_B_1, REG_SEND_DATA_ENCODER_B_2, stepsB);

    // Reset step count value
    m_stepsB = 0;
    m_stepsBprev = 0;
  }
}

// Sends large numbers to registers (up to + or - 14bit numbers)
void Motors::m_sendEncoderStepsToRegisters(int t_register1, int t_register2, int t_steps){
  int firstPart = t_steps >> 7;
  int secondPart = t_steps - (firstPart << 7);

  putRegister(t_register1, secondPart);
  putRegister(t_register2, firstPart);
}

// Sends decimal numbers to registers by splitting the whole from the fractionary part
void Motors::m_sendDecimalToRegisters(int t_register1, int t_register2, float t_number){
    int wholePart = (int)t_number;
    int fractPart = t_number*100 - wholePart*100;

    putRegister(t_register1, wholePart);
    putRegister(t_register2, fractPart);
}

// Resets encoder values
void Motors::m_resetEncoders(){
  m_stepsA = 0;
  m_stepsB = 0;
}

// Converts encoder steps to centimetres
double Motors::m_stepsToCentimetres(int t_steps){
  double fullRotation = 298 * 6;
  double wheelRadius = 5.85 / 2;
  double perimeter = 2 * PI * wheelRadius;
  double distance = perimeter * (t_steps/fullRotation);
  return distance;
}

// Moves the robot forwards
void Motors::m_moveForward(){
  Serial.println("Moving forwards.");
  m_motorSetDir(A, CW);
  m_motorSetDir(B, CCW);
}

// Moves the robot backwards
void Motors::m_moveBackward(){
  Serial.println("Moving backwards.");
  m_motorSetDir(A, CCW);
  m_motorSetDir(B, CW);
}

// Moves the robot left
void Motors::m_moveLeft(){
  Serial.println("Moving left.");
  m_motorSetDir(A, CCW);
  m_motorSetDir(B, CCW);
}

// Moves the robot right
void Motors::m_moveRight(){
  Serial.println("Moving right.");
  m_motorSetDir(A, CW);
  m_motorSetDir(B, CW);
}

// Stops the robot
void Motors::m_stopRobot(){
  Serial.println("Stopping robot.");
  m_adjustSpeed(0);
}

// Sets speed of both motors to a desired speed level (0 - 9)
void Motors::m_adjustSpeed(int t_speedLevel){
  // Variables for motor speed. Speeds are measured in PWM pulses 0 - 255
  int speedAPWM = 0; // MotorA
  int speedBPWM = 0; // MotorB
  int baseSpeed = 255 / 9; // Divides maximum pwm into 9 speed levels

  // If the speed level is between 0 and 9
  if(t_speedLevel >= 0 && t_speedLevel <= 9){
    // Set the speed to the desired level
    speedAPWM = baseSpeed * t_speedLevel;
    speedBPWM = baseSpeed * t_speedLevel;
  }
  else {
    // Stop motors
    speedAPWM = 0;
    speedBPWM = 0;
  }

  Serial.print("Adjusting motor A PWM speed to: ");
  Serial.println(speedAPWM);
  Serial.print("Adjusting motor B PWM speed to: ");
  Serial.println(speedBPWM);

  // Send speed to motors
  analogWrite(MOTOR_PWMA, speedAPWM);
  analogWrite(MOTOR_PWMB, speedBPWM);
}

// Helper function to set direction output and update the global direction variable
void Motors::m_motorSetDir(m_Motor t_motor, m_Direction t_dir){
  if(t_motor == A){
    digitalWrite(MOTOR_DIRA, t_dir); // Write out the direction, 0 = CW, 1 = CCW
    m_prevDirA = m_dirA;
    m_dirA = t_dir; // Update the direction variable
  }
  else if(t_motor == B){
    digitalWrite(MOTOR_DIRB, t_dir);
    m_prevDirB = m_dirB;
    m_dirB = t_dir;
  }
}

/* 
  Since we know the direction the motors should be travelling in, we do not need
  the full quadrature output of the encoders, we only take one channel from each
  to save on pins.
  
  These ISRs will be called once per edge, either rising or falling, and will add
  to the number of steps if the motor is currently moving CW, and take away if CCW
*/

//ISR for reading MOTOR_ENCA
void Motors::m_ENCA_ISR(){
  if(m_dirA == CW){
    m_stepsA++;
    m_absoluteStepsA++;
  }
  else if(m_dirA == CCW){
    m_stepsA--;
    m_absoluteStepsA--;
  }
}

//ISR for reading MOTOR_ENCB
void Motors::m_ENCB_ISR(){
  if(m_dirB == CCW){
    m_stepsB++;
    m_absoluteStepsB++;
  }
  else if(m_dirB == CW){
    m_stepsB--;
    m_absoluteStepsB--;
  }
}
