/*
	generalControl.ino - Arduino Robotics Board
  V2 - Modular design
	General interface for the RPRK Platform
  Author: Alejandro Pascual San Roman (bdh532)
	May 2023
*/

#include "Infrared.h"
#include "Joystick.h"
#include "Motors.h"
#include "Ultrasonics.h"

#include <ARB.h>
#include <Wire.h>

Infrared infrared(IR_BUS_NUMBER);
Joystick joystick;
Motors motors;
Ultrasonics ultrasonics;

void setup() {
	ARBSetup(true); // Setup everything required by the board and enable serial comms

  joystick.initialize();
	motors.initialize();
  ultrasonics.initialize();
  infrared.initialize();

	Serial.begin(9600); // Start serial for debugging
}

// This example loop runs each robot component sequencially
void loop() {
  infrared.readIR();
  ultrasonics.readUltrasonics();
  joystick.readInput();
  motors.runMotors();

  // Call the serialUpdate function at least once per loop
  serialUpdate();
  
  delay(0.1);
}
