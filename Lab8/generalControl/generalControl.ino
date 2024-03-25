/*
	odometryControl.ino - Arduino Robotics Board
	Odometry speed calculation.
	May 2023
	
*/

#include <ARB.h>
#include <Wire.h>

// Define macros
#define PI 3.1415926535897932384626433832795

// I2C MUX communication
#define IR_SENSOR_ADDRESS      0x80 >> 1  // 0xEE >> 1  // 0x80 >> 1 // Address for IR sensor, shifted to provide 7-bit version
#define IR_SENSOR_DISTANCE_REG 0x5E // Register address to read distance from
#define IR_SENSOR_SHIFT_REG    0x35 // Register address to read shift value from
#define IR_BUS_NUMBER 0 // I2C bus number to read from. For the IR sensor.

// Serial registers
#define REG_SEND_IR 10 // Serial register to send the IR data to

#define REG_SEND_DATA_ULTRASOUND_1 20 // Ultrasound 1 send data
#define REG_SEND_DATA_ULTRASOUND_2 21 // Ultrasound 2 send data

#define REG_RECEIVE_DIR_MOTOR_A 30 // Motor A receive direction
#define REG_RECEIVE_PWM_MOTOR_A 31 // Motor A receive PWM
#define REG_SEND_DATA_MOTOR_A 32 // Motor A send data (current direction and speed)

#define REG_RECEIVE_DIR_MOTOR_B 33 // Motor B receive direction
#define REG_RECEIVE_PWM_MOTOR_B 34 // Motor B receive PWM
#define REG_SEND_DATA_MOTOR_B 35 // Motor B send data (current direction and speed)

#define REG_SEND_DATA_ENCODER_A_1 36 // Encoder A send data
#define REG_SEND_DATA_ENCODER_A_2 37 // Encoder A send data
#define REG_SEND_DATA_ENCODER_B_1 38 // Encoder B send data
#define REG_SEND_DATA_ENCODER_B_2 39 // Encoder B send data

#define REG_RECEIVE_SPEED_DATA 40 // Receive speed data
#define REG_RECEIVE_MSG_DRIVE 41 // Receive drive control data
#define REG_SEND_MSG_DRIVE 42 // Send drive control data (current state)

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

#define REG_SEND_MSG_JOYSTICK 60 // Joystick send messages
#define REG_RECEIVE_MSG_JOYSTICK 61 // Joystic receive messages
#define REG_SEND_DATA_JOYSTICK 62 // Joystick send data (direction)

// Ir sensor variables
int shift; //Stores shift value from IR sensor
int infraredDistancePrev = 0;

// Ultrasound variables
int distances[2] = {0,0}; // Setup variable for results
int distancesPrev[2] = {0,0};

// Define some enums to make specifying motor and direction easier
typedef enum {CW,CCW} Direction; // CW = 0, CCW = 1
typedef enum {A,B} Motor; // A = 0, B = 1

// Global variables for motor direction so they can be read by encoder ISR to determine direction
Direction dirA = CW;
Direction dirB = CCW;

Direction prevDirA = CW;
Direction prevDirB = CCW;

// To check if the value from the direction registers has changed
Direction registerDirAprev = CW;
Direction registerDirBprev = CCW;

bool robotDirectionChanged = false;

// Variables to store step count from motors, must be volatlie to update from within ISR
volatile int stepsA = 0;
volatile int stepsB = 0;

// These values will not be reset unless a signal is received
volatile int absoluteStepsA = 0;
volatile int absoluteStepsB = 0;

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

int speedAPWM_prev = 0;
int speedBPWM_prev = 0;

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

  setupI2C(IR_BUS_NUMBER);

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
  moveForward();
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

  // IR sensor
  putRegister(REG_SEND_IR, 0);

  // Ultrasound
  putRegister(REG_SEND_DATA_ULTRASOUND_1, 0);
  putRegister(REG_SEND_DATA_ULTRASOUND_2, 0);

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

  // Joystick
  putRegister(REG_SEND_MSG_JOYSTICK, 0); // Joystick send messages
  putRegister(REG_RECEIVE_MSG_JOYSTICK, 0); // Joystic receive messages
  putRegister(REG_SEND_DATA_JOYSTICK, 0); // Joystick send data (direction)
}

// This example loop moves the robot based on user input
void loop() {
  // Read the IR sensor
  int infraredDistance = readI2CSensor(IR_BUS_NUMBER);

  if (infraredDistance != infraredDistancePrev){
    putRegister(REG_SEND_IR, infraredDistance);
    infraredDistancePrev = infraredDistance;
  }

  // Read ultrasonic sensors
  getUltrasoundDistances();

  if(distances[0] != distancesPrev[0]){
    putRegister(REG_SEND_DATA_ULTRASOUND_1, distances[0]);
    distancesPrev[0] = distances[0];
  }

  if(distances[1] != distancesPrev[1]){
    putRegister(REG_SEND_DATA_ULTRASOUND_2, distances[1]);
    distancesPrev[1] = distances[1];
  }

  // Read Joystick (orientations are inverted, if the point of reference is forward=up)
  if(buttons.left == true){
    Serial.println("Joystick right");
    putRegister(REG_SEND_DATA_JOYSTICK, 4);
    buttons.left = false;
  }
  
  if(buttons.right == true){
    Serial.println("Joystick left");
    putRegister(REG_SEND_DATA_JOYSTICK, 3);
    buttons.right = false;
  }
  
  if(buttons.up == true){
    Serial.println("Joystick down");
    putRegister(REG_SEND_DATA_JOYSTICK, 2);
    buttons.up = false;
  }
  
  if(buttons.down == true){
    Serial.println("Joystick up");
    putRegister(REG_SEND_DATA_JOYSTICK, 1);
    buttons.down = false;
  }
  
  // Read wheel direction signals
  Direction registerDirA = (Direction)getRegister(REG_RECEIVE_DIR_MOTOR_A);
  Direction registerDirB = (Direction)getRegister(REG_RECEIVE_DIR_MOTOR_B);

  // If the direction signal for motor A has changed, change direction
  if(registerDirA != registerDirAprev){
    motorSetDir(A, registerDirA);
    registerDirAprev = registerDirA;
    putRegister(REG_SEND_DATA_MOTOR_A, dirA); // Send current wheel direction to register
  }

  // If the direction signal for motor B has changed, change direction
  if(registerDirB != registerDirBprev){
    motorSetDir(B, registerDirB);
    registerDirBprev = registerDirB;
    putRegister(REG_SEND_DATA_MOTOR_B, dirB); // Send current wheel direction to register
  }
  
  // Read PWM signals
  int speedAPWM = (uint8_t)getRegister(REG_RECEIVE_PWM_MOTOR_A);
  int speedBPWM = (uint8_t)getRegister(REG_RECEIVE_PWM_MOTOR_B);

  // If the PWM signal for motor A has changed, change speed level
  if(speedAPWM != speedAPWM_prev){
    analogWrite(MOTOR_PWMA, speedAPWM);
    Serial.print("Adjusting motor A PWM speed to: ");
    Serial.println(speedAPWM);
    speedAPWM_prev = speedAPWM;
  }

  // If the PWM signal for motor B has changed, change speed level
  if(speedBPWM != speedBPWM_prev){
    analogWrite(MOTOR_PWMB, speedBPWM);
    Serial.print("Adjusting motor B PWM speed to: ");
    Serial.println(speedBPWM);
    speedBPWM_prev = speedBPWM;
  }
  
  // Get data from general robot speed register
  int speedValue = getRegister(REG_RECEIVE_SPEED_DATA);

  // If the data has changed, change speed level
  if(speedValue != speedValuePrev){
    adjustSpeed(speedValue);
    speedValuePrev = speedValue;
  }

  // Get input data from register (WASD)
  int input = getRegister(REG_RECEIVE_MSG_DRIVE);

  // If the input has changed
  if(input != inputPrev){
    // Forward input
    if(input == 1){
      moveForward();
      putRegister(REG_SEND_MSG_DRIVE, 1);
    }
    // Backward input
    else if (input == 2){
      moveBackward();
      putRegister(REG_SEND_MSG_DRIVE, 2);
    }
    // Left input
    else if (input == 3){
      moveLeft();
      putRegister(REG_SEND_MSG_DRIVE, 3);
    }
    // Right input
    else if (input == 4){
      moveRight();
      putRegister(REG_SEND_MSG_DRIVE, 4);
    }
    // Anything else stops the motors
    else {
      stopRobot();
      putRegister(REG_SEND_MSG_DRIVE, 5);
    }
    
    inputPrev = input;
  }

  // Absolute encoder steps
  sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_A_1, REG_SEND_DATA_ENCODER_A_2, absoluteStepsA); // Send data to the register
  sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_B_1, REG_SEND_DATA_ENCODER_B_2, absoluteStepsB); // Send data to the register

  int resetInputA = getRegister(REG_RECEIVE_RESET_ENCODER_A);
  int resetInputB = getRegister(REG_RECEIVE_RESET_ENCODER_B);

  if(resetInputA == 1){
    absoluteStepsA = 0;
    putRegister(REG_RECEIVE_RESET_ENCODER_A, 0);
  }

  if(resetInputB == 1){
    absoluteStepsB = 0;
    putRegister(REG_RECEIVE_RESET_ENCODER_B, 0);
  }

  // If the steps value has changed (the wheel is not stationary)
  if(stepsA != stepsAprev) {
    if (newMeasureA == false){ // If the robot was not moving before
      newMeasureA = true; // Set new measurement
      previousTimeA = millis(); // Record initial time
    }

    // If the direction has changed otherwise (of any wheel), and the robot was already moving
    else if(((dirA != prevDirA) || (dirB != prevDirB)) && (newMeasureA == true)){
      robotDirectionChanged = true; // Setup flag to change wheel B as well
      newMeasureA = false; // Stop current measurement
      currentTimeA = millis(); // Record current time

      // Calculate distance travelled and speed
      distanceA = stepsToCentimetres(stepsA);
      double timeDifferenceInSeconds = (currentTimeA - previousTimeA)/1000;
      speedA = distanceA/timeDifferenceInSeconds; // Speed in cm/second
  
      // Print the current number of steps, distance and speed recorded
      Serial.print("A steps: ");
      Serial.println(stepsA);
      Serial.print("A distance: ");
      Serial.print(distanceA);
      Serial.println(" cm");
      Serial.print("A speed: ");
      Serial.print(speedA);
      Serial.println(" cm/s");

      // Send distance and speed data to the registers
      sendDecimalToRegisters(REG_SEND_DISTANCE_A, REG_SEND_DISTANCE_A_DEC, distanceA);
      sendDecimalToRegisters(REG_SEND_SPEED_A, REG_SEND_SPEED_A_DEC, speedA);
      // sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_A_1, REG_SEND_DATA_ENCODER_A_2, stepsA);

      // Reset step count value
      stepsA = 0;
      stepsAprev = 0;
      
      // Start new measurement
      newMeasureA = true;
      previousTimeA = millis();
    }
    prevDirA = dirA; // Update wheel direction
    stepsAprev = stepsA; // Update number of steps
    // sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_A_1, REG_SEND_DATA_ENCODER_A_2, stepsA); // Send data to the register
  }

  // If the wheel has stopped
  else if((stepsA == stepsAprev) && (newMeasureA == true)){
    newMeasureA = false; // Stop current measurement
    currentTimeA = millis(); // Record current time

    // Calculate distance travelled and speed
    distanceA = stepsToCentimetres(stepsA);
    double timeDifferenceInSeconds = (currentTimeA - previousTimeA)/1000;
    speedA = distanceA/timeDifferenceInSeconds; // Speed in cm/second

    // Print the current number of steps, distance and speed recorded
    Serial.print("A steps: ");
    Serial.println(stepsA);
    Serial.print("A distance: ");
    Serial.print(distanceA);
    Serial.println(" cm");
    Serial.print("A speed: ");
    Serial.print(speedA);
    Serial.println(" cm/s");

    // Send distance and speed data to the registers
    sendDecimalToRegisters(REG_SEND_DISTANCE_A, REG_SEND_DISTANCE_A_DEC, distanceA);
    sendDecimalToRegisters(REG_SEND_SPEED_A, REG_SEND_SPEED_A_DEC, speedA);
    // sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_A_1, REG_SEND_DATA_ENCODER_A_2, stepsA);

    // Reset step count value
    stepsA = 0;
    stepsAprev = 0;
  }

  // If the steps value has changed (the wheel is not stationary)
  if(stepsB != stepsBprev) {
    if (newMeasureB == false){ // If the robot was not moving before
      newMeasureB = true; // Set new measurement
      previousTimeB = millis(); // Record initial time
    }

    // If the direction has changed otherwise (of any wheel), and the robot was already moving
    else if(((dirB != prevDirB) || robotDirectionChanged) && (newMeasureB == true)){
      robotDirectionChanged = false; // Remove flag to change wheel B
      newMeasureB = false; // Stop current measurement
      currentTimeB = millis(); // Record current time

      // Calculate distance travelled and speed
      distanceB = stepsToCentimetres(stepsB);
      double timeDifferenceInSeconds = (currentTimeB - previousTimeB)/1000;
      speedB = distanceB/timeDifferenceInSeconds; // Speed in cm/second
  
      // Print the current number of steps, distance and speed recorded
      Serial.print("B steps: ");
      Serial.println(stepsB);
      Serial.print("B distance: ");
      Serial.print(distanceB);
      Serial.println(" cm");
      Serial.print("B speed: ");
      Serial.print(speedB);
      Serial.println(" cm/s");

      // Send distance and speed data to the registers
      sendDecimalToRegisters(REG_SEND_DISTANCE_B, REG_SEND_DISTANCE_B_DEC, distanceB);
      sendDecimalToRegisters(REG_SEND_SPEED_B, REG_SEND_SPEED_B_DEC, speedB);
      // sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_B_1, REG_SEND_DATA_ENCODER_B_2, stepsB);

      // Reset step count value
      stepsB = 0;
      stepsBprev = 0;

      // Start new measurement
      newMeasureB = true;
      previousTimeB = millis();
    }
    prevDirB = dirB; // Update wheel direction
    stepsBprev = stepsB; // Update number of steps
    // sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_B_1, REG_SEND_DATA_ENCODER_B_2, stepsB); // Send data to the register
  }

  // If the wheel has stopped
  else if((stepsB == stepsBprev) && (newMeasureB == true)){
    newMeasureB = false; // Stop current measurement
    currentTimeB = millis(); // Record current time

    // Calculate distance travelled and speed
    distanceB = stepsToCentimetres(stepsB);
    double timeDifferenceInSeconds = (currentTimeB - previousTimeB)/1000;
    speedB = distanceB/timeDifferenceInSeconds; // Speed in cm/second

    // Print the current number of steps, distance and speed recorded
    Serial.print("B steps: ");
    Serial.println(stepsB);
    Serial.print("B distance: ");
    Serial.print(distanceB);
    Serial.println(" cm");
    Serial.print("B speed: ");
    Serial.print(speedB);
    Serial.println(" cm/s");

    // Send distance and speed data to the registers
    sendDecimalToRegisters(REG_SEND_DISTANCE_B, REG_SEND_DISTANCE_B_DEC, distanceB);
    sendDecimalToRegisters(REG_SEND_SPEED_B, REG_SEND_SPEED_B_DEC, speedB);
    // sendEncoderStepsToRegisters(REG_SEND_DATA_ENCODER_B_1, REG_SEND_DATA_ENCODER_B_2, stepsB);

    // Reset step count value
    stepsB = 0;
    stepsBprev = 0;
  }

  // Call the serialUpdate function at least once per loop
  serialUpdate();
  
  delay(0.1);
}

// Sends large numbers to registers (up to + or - 14bit numbers)
void sendEncoderStepsToRegisters(int register1, int register2, int steps){
  int firstPart = steps >> 7;
  int secondPart = steps - (firstPart << 7);

  putRegister(register1, secondPart);
  putRegister(register2, firstPart);
}

// Sends decimal numbers to registers by splitting the whole from the fractionary part
void sendDecimalToRegisters(int register1, int register2, float number){
    int wholePart = (int)number;
    int fractPart = number*100 - wholePart*100;

    putRegister(register1, wholePart);
    putRegister(register2, fractPart);
}

// Resets encoder values
void resetEncoders(){
  stepsA = 0;
  stepsB = 0;
}

// Converts encoder steps to centimetres
float stepsToCentimetres(int steps){
  float fullRotation = 298 * 6;
  float wheelRadius = 5.85 / 2;
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

  Serial.print("Adjusting motor A PWM speed to: ");
  Serial.println(speedAPWM);
  Serial.print("Adjusting motor B PWM speed to: ");
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

// I2C
void setupI2C(int bus_number) {
  setI2CBus(bus_number); // Set which bus we are reading from

  // Write to the sensor to tell it we are reading from the shift register
  Wire.beginTransmission(IR_SENSOR_ADDRESS);
  Wire.write(IR_SENSOR_SHIFT_REG);
  Wire.endTransmission();

  // Request 1 byte of data from the sensor to read the shift register
  Wire.requestFrom(IR_SENSOR_ADDRESS, 1);

  while(Wire.available() == 0){
    // Tells the user if the sketch is waiting for a particular sensor
    // If sensor does not reply, it may be a sign of a faulty or disconnected sensor
    Serial.print("Waiting for sensor ");
    Serial.print(bus_number);
    Serial.println();
  }

  // Once the data become available in the Wire bufer, put it into the shift array
  shift = Wire.read();
}

int readI2CSensor(int bus_number){
  setI2CBus(bus_number); // Set bus we are accessing

  // Write to sensor to tell it we are reading from the distance register
  Wire.beginTransmission(IR_SENSOR_ADDRESS);
  Wire.write(IR_SENSOR_DISTANCE_REG);
  Wire.endTransmission();

  // Request two bytes of data from the sensor
  Wire.requestFrom(IR_SENSOR_ADDRESS, 2);

  // Wait until bytes are received in the buffer
  while(Wire.available() <2);

  // Temporarily store the bytes read. Stores high and low byte read
  byte high = Wire.read();
  byte low = Wire.read();

  // Calculate the distance in cm
  int distance = (high * 16 + low)/16/(int)pow(2,shift); // Stores calculated distance

  // Print out values over serial
  Serial.print("IR sensor distance is ");
  Serial.print(distance);
  Serial.print("cm.\n");

  return distance;
}

void getUltrasoundDistances(){
  int duration[2], cm[2]; // Setup variables for results

  // Set the pin to output, bring it low, then high, then low to generate pulse
  pinMode(USONIC1, OUTPUT);
  digitalWrite(USONIC1, LOW);
  delayMicroseconds(2);
  digitalWrite(USONIC1, HIGH);
  delayMicroseconds(15);
  digitalWrite(USONIC1, LOW);

  // The same pin is used to read back the returning signal, so must be set back to input
  pinMode(USONIC1, INPUT);
  duration[0] = pulseIn(USONIC1, HIGH);

  // Set the pin to output, bring it low, then high, then low to generate pulse
  pinMode(USONIC2, OUTPUT);
  digitalWrite(USONIC2, LOW);
  delayMicroseconds(2);
  digitalWrite(USONIC2, HIGH);
  delayMicroseconds(15);
  digitalWrite(USONIC2, LOW);

  // The same pin is used to read back the returning signal, so must be set back to input
  pinMode(USONIC2, INPUT);
  duration[1] = pulseIn(USONIC2, HIGH);

  // Convert to cm using helper function
  cm[0] = uSecToCM(duration[0]);
  cm[1] = uSecToCM(duration[1]);

  distances[0] = cm[0];
  distances[1] = cm[1];
}

// Check if two arrays are different
bool arrayCmp(int *a, int *b){
      // test each element to be the same. if not, return false
      for (int n = 0; n < 2; n++) if (a[n]!=b[n]) return false;

      //ok, if we have not returned yet, they are equal :)
      return true;
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
    absoluteStepsA++;
	}
	else if(dirA == CCW){
		stepsA--;
    absoluteStepsA--;
	}
}

//ISR for reading MOTOR_ENCB
void ENCB_ISR(){
	if(dirB == CCW){
		stepsB++;
    absoluteStepsB++;
	}
	else if(dirB == CW){
		stepsB--;
    absoluteStepsB--;
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
