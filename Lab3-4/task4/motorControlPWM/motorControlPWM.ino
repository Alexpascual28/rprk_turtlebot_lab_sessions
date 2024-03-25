/*
	motorControlPWM.ino - Arduino Robotics Board
	Laboratory Session 4 Task 4: SET UP PULSE-WIDTH MODULATION (PWM)
	May 2023
	
*/

#include <ARB.h>

// Define some enums to make specifying motor and direction easier
typedef enum {CW,CCW} Direction; // CW = 0, CCW = 1
typedef enum {A,B} Motor;

// Global variables for motor direction so they can be read by encoder ISR to determine direction
Direction dirA = CW;
Direction dirB = CW;

// Variables to store step count from motors, must be volatlie to update from within ISR
volatile int stepsA = 0;
volatile int stepsB = 0;

void setup() {
	ARBSetup(); // Setup ARB functionallity

	// Set relevant modes for pins
	pinMode(MOTOR_DIRA, OUTPUT);
	pinMode(MOTOR_DIRB, OUTPUT);
	pinMode(MOTOR_PWMA, OUTPUT);
	pinMode(MOTOR_PWMB, OUTPUT);
	pinMode(MOTOR_ENCA, INPUT);
	pinMode(MOTOR_ENCB, INPUT);

	// Set motors off by default
	motorSetDir(A, CW);
	motorSetDir(B, CW);
	analogWrite(MOTOR_PWMA, 0);
	analogWrite(MOTOR_PWMB, 0);

	// Attach interrupts to motor encoder inputs so we don't miss any steps
	attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), ENCA_ISR, CHANGE); // Pin number put be converted to interrupt number
	attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), ENCB_ISR, CHANGE); // CHANGE will activate the ISR on either edge, can be changed to either RISING or FALLING

	Serial.begin(9600); // Start serial for debugging
}

// This example loop moves the robot forwards, the backwards, then left and then right at varying speeds
void loop() {
  for(int s = 1; s <= 9; s++){
    adjustSpeed(s);

    for(int i = 0; i <= 4; i++){
      if(i == 0){
        moveForward();
      }
      else if(i == 1) {
        moveBackward();
      }
      else if(i == 2) {
        moveRight();
      }
      else if(i == 3) {
        moveLeft();
      }
      else if(i == 4) {
        stopRobot();
      }
  
      delay(2000);
  
      // Print the current number of steps recorded
      Serial.print("A steps: ");
      Serial.println(stepsA);
      Serial.print("B steps: ");
      Serial.println(stepsB);
    }
  }
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
  int speedA = 0; // MotorA
  int speedB = 0; // MotorB
  int baseSpeed = 255 / 9; // Divides maximum pwm into 9 speed levels

  // If the speed level is between 0 and 9
  if(speedLevel >= 0 && speedLevel <= 9){
    // Set the speed to the desired level
    speedA = baseSpeed * speedLevel;
    speedB = baseSpeed * speedLevel;
  }
  else {
    // Stop motors
    speedA = 0;
    speedB = 0;
  }

  Serial.print("Adjusting motor A speed to: ");
  Serial.println(speedA);
  Serial.print("Adjusting motor B speed to: ");
  Serial.println(speedB);

  // Send speed to motors
  analogWrite(MOTOR_PWMA, speedA);
  analogWrite(MOTOR_PWMB, speedB);
}

// Helper function to set direction output and update the global direction variable
void motorSetDir(Motor motor, Direction dir){
	if(motor == A){
		digitalWrite(MOTOR_DIRA, dir); // Write out the direction, 0 = CW, 1 = CCW
		dirA = dir; // Update the direction variable
	}
	else if(motor == B){
		digitalWrite(MOTOR_DIRB, dir);
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
