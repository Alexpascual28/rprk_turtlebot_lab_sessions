/*
	encoderSpeed.ino - Arduino Robotics Board
	Encoder reading and odometry speed calculation.
	May 2023
	
*/

#include <ARB.h>

// Define macros
#define PI 3.1415926535897932384626433832795

// Serial registers
#define REG_SEND_MSG_MOTOR_A 30 // Motor A send messages
#define REG_RECEIVE_MSG_MOTOR_A 31 // Motor A & Encoder A receive messages
#define REG_SEND_DATA_MOTOR_A 32 // Motor A send data (current direction)

#define REG_SEND_MSG_MOTOR_B 33 // Motor B send messages
#define REG_RECEIVE_MSG_MOTOR_B 34 // Motor B & Encoder B receive messages
#define REG_SEND_DATA_MOTOR_B 35 // Motor B send data (current direction)

#define REG_SEND_MSG_ENCODER_A 36 // Encoder A send messages
#define REG_SEND_DATA_ENCODER_A 37 // Encoder A send data
#define REG_SEND_MSG_ENCODER_B 38 // Encoder B send messages
#define REG_SEND_DATA_ENCODER_B 39 // Encoder B send data

#define REG_RECEIVE_SPEED_DATA 40 // Receive speed data
#define REG_RECEIVE_MSG_DRIVE 41 // Receive drive control data
#define REG_SEND_MSG_DRIVE 42 // Send drive control data (current state)

#define REG_SEND_DISTANCE_A 43 // Calculated distance in wheel A
#define REG_SEND_DISTANCE_B 44 // Calculated distance in wheel B

#define REG_SEND_SPEED_A 45 // Calculated speed of wheel A
#define REG_SEND_SPEED_B 46 // Calculated speed of wheel B

#define REG_SEND_MSG_JOYSTICK 50 // Joystick send messages
#define REG_RECEIVE_MSG_JOYSTICK 51 // Joystic receive messages
#define REG_SEND_DATA_JOYSTICK 52 // Joystick send data (direction)

// Define some enums to make specifying motor and direction easier
typedef enum {CW,CCW} Direction; // CW = 0, CCW = 1
typedef enum {A,B} Motor;

// Global variables for motor direction so they can be read by encoder ISR to determine direction
Direction dirA = CW;
Direction dirB = CW;

Direction prevDirA = CW;
Direction prevDirB = CW;

// Variables to store step count from motors, must be volatlie to update from within ISR
volatile int stepsA = 0;
volatile int stepsB = 0;

// Flags to check if steps have changed
int stepsAprev = 0;
int stepsBprev = 0;
bool newMeasureA = false;
bool newMeasureB = false;

// Variables to store distance count from motors
float distanceA = 0;
float distanceB = 0;

// Variables to store speed count from motors
float speedA = 0;
float speedB = 0;

// Global time variables
double previousTimeA = 0;
double previousTimeB = 0;
double currentTimeA = 0;
double currentTimeB = 0;

// Define struct to hold button push flags
volatile struct ButtonState{
  bool left = false;
  bool right = false;
  bool up = false;
  bool down = false;
} buttons;

volatile int speedValuePrev = 0;
volatile int inputPrev = 0;

void setup() {
	ARBSetup(true); // Setup everything required by the board and enable serial comms

	setPinModes();
  initializeComponents(); // Initializes values
  attachInterrupts(); // Attaches interrupts
  initializeSerialRegisters(); // Initialize serial registers

	Serial.begin(9600); // Start serial for debugging
}

void setPinModes(){
  // Set relevant modes for pins
  pinMode(MOTOR_DIRA, OUTPUT);
  pinMode(MOTOR_DIRB, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_PWMB, OUTPUT);
  pinMode(MOTOR_ENCA, INPUT);
  pinMode(MOTOR_ENCB, INPUT);

  // Set pins to inputs
  pinMode(PB_LEFT, INPUT);
  pinMode(PB_RIGHT, INPUT);
  pinMode(PB_UP, INPUT);
  pinMode(PB_DOWN, INPUT);
}

void initializeComponents(){
  // Enable internal pull-ups
  digitalWrite(PB_LEFT, HIGH);
  digitalWrite(PB_RIGHT, HIGH);
  digitalWrite(PB_UP, HIGH);
  digitalWrite(PB_DOWN, HIGH);

  // Set motors off by default
  motorSetDir(A, CW);
  motorSetDir(B, CW);
  stopRobot();

  resetEncoders();

  newMeasureA = false;
  newMeasureB = false;
}

void attachInterrupts(){
  // Attach interrupts to motor encoder inputs so we don't miss any steps
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), ENCA_ISR, CHANGE); // Pin number put be converted to interrupt number
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), ENCB_ISR, CHANGE); // CHANGE will activate the ISR on either edge, can be changed to either RISING or FALLING

  /* 
    Attach interrupts to PB pins
    The internal pull-ups pul the pins high when the button is open.
    When the button is closed it grounds the pin, so we want to be 
    looking for falling edges to see when the button has been pressed.
  */
  attachInterrupt(digitalPinToInterrupt(PB_LEFT), LEFT_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_RIGHT), RIGHT_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_UP), UP_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PB_DOWN), DOWN_ISR, FALLING);
}

void initializeSerialRegisters(){
  // Setup some dummy data in the registers to be read by the Raspberry Pi

  // Motors
  putRegister(REG_RECEIVE_SPEED_DATA, 0); // Speed data
  
  putRegister(REG_SEND_MSG_MOTOR_A, 0); // Motor A send messages
  putRegister(REG_RECEIVE_MSG_MOTOR_A, 0); // Motor A & Encoder A receive messages
  putRegister(REG_SEND_DATA_MOTOR_A, 0); // Motor A send data (current state)
  
  putRegister(REG_SEND_MSG_MOTOR_B, 0); // Motor B send messages
  putRegister(REG_RECEIVE_MSG_MOTOR_B, 0); // Motor B & Encoder B receive messages
  putRegister(REG_SEND_DATA_MOTOR_B, 0); // Motor B send data (current state)

  putRegister(REG_SEND_MSG_ENCODER_A, 0); // Encoder A send messages
  putRegister(REG_SEND_DATA_ENCODER_A, 0); // Encoder A send data
  putRegister(REG_SEND_MSG_ENCODER_B, 0); // Encoder B send messages
  putRegister(REG_SEND_DATA_ENCODER_B, 0); // Encoder B send data

  putRegister(REG_RECEIVE_SPEED_DATA, 0); // Receive speed data
  putRegister(REG_RECEIVE_MSG_DRIVE, 0);// Receive drive control data (forward, backward, left, right)
  putRegister(REG_SEND_MSG_DRIVE, 0); // Send drive control data (current state)

  putRegister(REG_SEND_DISTANCE_A, 0); // Calculated distance in wheel A
  putRegister(REG_SEND_DISTANCE_B, 0); // Calculated distance in wheel B
  
  putRegister(REG_SEND_SPEED_A, 0); // Calculated speed of wheel A
  putRegister(REG_SEND_SPEED_B, 0); // Calculated speed of wheel B

  // Joystick
  putRegister(REG_SEND_MSG_JOYSTICK, 0); // Joystick send messages
  putRegister(REG_RECEIVE_MSG_JOYSTICK, 0); // Joystic receive messages
  putRegister(REG_SEND_DATA_JOYSTICK, 0); // Joystick send data (direction)
}

// This example loop moves the robot forwards, the backwards, then left and then right at varying speeds
void loop() {
  if(stepsA != stepsAprev) {
    if (newMeasureA == false){
      newMeasureA = true;
      previousTimeA = millis();
    }
    else if((dirA != prevDirA) && (newMeasureA == true)){
      newMeasureA = false;
      currentTimeA = millis();
      distanceA = stepsToCentimetres(stepsA);
      double timeDifferenceInSeconds = (currentTimeA - previousTimeA)/1000;
      speedA = distanceA/timeDifferenceInSeconds; // Speed in cm/second
  
      // Print the current number of steps recorded
      Serial.print("A steps: ");
      Serial.println(stepsA);
      Serial.print("A distance: ");
      Serial.print(distanceA);
      Serial.println(" cm");
      Serial.print("A speed: ");
      Serial.print(speedA);
      Serial.println(" cm/s");
  
      putRegister(REG_SEND_DISTANCE_A, distanceA);
      putRegister(REG_SEND_SPEED_A, speedA);
      
      stepsA = 0;
      stepsAprev = 0;
      putRegister(REG_SEND_DATA_ENCODER_A, stepsA);
      
      newMeasureA = true;
      previousTimeA = millis();
    }
    prevDirA = dirA;
    stepsAprev = stepsA;
    putRegister(REG_SEND_DATA_ENCODER_A, stepsA);
  }
  else if((stepsA == stepsAprev) && (newMeasureA == true)){
    newMeasureA = false;
    currentTimeA = millis();
    distanceA = stepsToCentimetres(stepsA);
    double timeDifferenceInSeconds = (currentTimeA - previousTimeA)/1000;
    speedA = distanceA/timeDifferenceInSeconds; // Speed in cm/second

    // Print the current number of steps recorded
    Serial.print("A steps: ");
    Serial.println(stepsA);
    Serial.print("A distance: ");
    Serial.print(distanceA);
    Serial.println(" cm");
    Serial.print("A speed: ");
    Serial.print(speedA);
    Serial.println(" cm/s");

    putRegister(REG_SEND_DISTANCE_A, distanceA);
    putRegister(REG_SEND_SPEED_A, speedA);
    
    stepsA = 0;
    stepsAprev = 0;
    putRegister(REG_SEND_DATA_ENCODER_A, stepsA);
  }

  if(stepsB != stepsBprev) {
    if (newMeasureB == false){
      newMeasureB = true;
      previousTimeB = millis();
    }
    else if((dirB != prevDirB) && (newMeasureB == true)){
      newMeasureB = false;
      currentTimeB = millis();
      distanceB = stepsToCentimetres(stepsB);
      double timeDifferenceInSeconds = (currentTimeB - previousTimeB)/1000;
      speedB = distanceB/timeDifferenceInSeconds; // Speed in cm/second
  
      // Print the current number of steps recorded
      Serial.print("B steps: ");
      Serial.println(stepsB);
      Serial.print("B distance: ");
      Serial.print(distanceB);
      Serial.println(" cm");
      Serial.print("B speed: ");
      Serial.print(speedB);
      Serial.println(" cm/s");
  
      putRegister(REG_SEND_DISTANCE_B, distanceB);
      putRegister(REG_SEND_SPEED_B, speedB);
  
      stepsB = 0;
      stepsBprev = 0;
      putRegister(REG_SEND_DATA_ENCODER_B, stepsB);
      
      newMeasureB = true;
      previousTimeB = millis();
    }
    prevDirB = dirB;
    stepsBprev = stepsB;
    putRegister(REG_SEND_DATA_ENCODER_B, stepsB);
  }
  else if((stepsB == stepsBprev) && (newMeasureB == true)){
    newMeasureB = false;
    currentTimeB = millis();
    distanceB = stepsToCentimetres(stepsB);
    double timeDifferenceInSeconds = (currentTimeB - previousTimeB)/1000;
    speedB = distanceB/timeDifferenceInSeconds; // Speed in cm/second

    // Print the current number of steps recorded
    Serial.print("B steps: ");
    Serial.println(stepsB);
    Serial.print("B distance: ");
    Serial.print(distanceB);
    Serial.println(" cm");
    Serial.print("B speed: ");
    Serial.print(speedB);
    Serial.println(" cm/s");

    putRegister(REG_SEND_DISTANCE_B, distanceB);
    putRegister(REG_SEND_SPEED_B, speedB);

    stepsB = 0;
    stepsBprev = 0;
    putRegister(REG_SEND_DATA_ENCODER_B, stepsB);
  }

  // Call the serialUpdate function at least once per loop
  serialUpdate();
  
  delay(40);
}

// Resets encoder values
void resetEncoders(){
  stepsA = 0;
  stepsB = 0;
}

// Converts encoder steps to centimetres
float stepsToCentimetres(int steps){
  float fullRotation = 298 * 6;
  float wheelRadius = 4.97 / 2;
  float perimeter = 2 * PI * wheelRadius;
  float distance = perimeter * (steps/fullRotation);
  return distance;
}

// Moves the robot forwards
void moveForward(){
  Serial.println("Moving forwards.");
  motorSetDir(A, CW);
  motorSetDir(B, CCW);
}

// Moves the robot backwards
void moveBackward(){
  Serial.println("Moving backwards.");
  motorSetDir(A, CCW);
  motorSetDir(B, CW);
}

// Moves the robot left
void moveLeft(){
  Serial.println("Moving left.");
  motorSetDir(A, CCW);
  motorSetDir(B, CCW);
}

// Moves the robot right
void moveRight(){
  Serial.println("Moving right.");
  motorSetDir(A, CW);
  motorSetDir(B, CW);
}

// Stops the robot
void stopRobot(){
  Serial.println("Stopping robot.");
  adjustSpeed(0);
}

// Sets speed of both motors to a desired speed level (0 - 9)
void adjustSpeed(int speedLevel){
  // Variables for motor speed. Speeds are measured in PWM pulses 0 - 255
  int speedAPWM = 0; // MotorA
  int speedBPWM = 0; // MotorB
  int baseSpeed = 255 / 9; // Divides maximum pwm into 9 speed levels

  // If the speed level is between 0 and 9
  if(speedLevel >= 0 && speedLevel <= 9){
    // Set the speed to the desired level
    speedAPWM = baseSpeed * speedLevel;
    speedBPWM = baseSpeed * speedLevel;
  }
  else {
    // Stop motors
    speedAPWM = 0;
    speedBPWM = 0;
  }

  Serial.print("Adjusting motor A speed to: ");
  Serial.println(speedAPWM);
  Serial.print("Adjusting motor B speed to: ");
  Serial.println(speedBPWM);

  // Send speed to motors
  analogWrite(MOTOR_PWMA, speedAPWM);
  analogWrite(MOTOR_PWMB, speedBPWM);
}

// Helper function to set direction output and update the global direction variable
void motorSetDir(Motor motor, Direction dir){
	if(motor == A){
		digitalWrite(MOTOR_DIRA, dir); // Write out the direction, 0 = CW, 1 = CCW
    prevDirA = dirA;
		dirA = dir; // Update the direction variable
	}
	else if(motor == B){
		digitalWrite(MOTOR_DIRB, dir);
    prevDirB = dirB;
		dirB = dir;
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
void ENCA_ISR(){
	if(dirA == CW){
		stepsA++;
	}
	else if(dirA == CCW){
		stepsA--;
	}
}

//ISR for reading MOTOR_ENCB
void ENCB_ISR(){
	if(dirB == CW){
		stepsB++;
	}
	else if(dirB == CCW){
		stepsB--;
	}
}

/*
  It's best practice to keep ISRs as short as possible so these ISRs
  just set a flag if a button has been pressed so it can be acted upon
  outside of the ISR
*/
void LEFT_ISR(){
  buttons.left = true;
}

void RIGHT_ISR(){
  buttons.right = true;
}

void UP_ISR(){
  buttons.up = true;
}

void DOWN_ISR(){
  buttons.down = true;
}