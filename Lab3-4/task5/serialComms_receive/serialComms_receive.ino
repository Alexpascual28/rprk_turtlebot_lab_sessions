/*
	serialComms.ino - Arduino Robotics Board
	An Arduino library for controling the Arduino Robotics Board platform,
	either standalone or in conjunction with a Raspberry Pi
	June 2022
	
	This example demonstrates the use of the serial communication between the Arduino
	and the Raspberry Pi and is intended to run with the companion Raspberry Pi example.
*/

#include <ARB.h>
  
void setup(){
	ARBSetup(true); // Setup everything required by the board and enable serial comms
  Serial.begin(9600); // Start serial

	// Setup some dummy data in the registers to be read by the Raspberry Pi
  putRegister(0, 10);
  putRegister(1, 80);
}

void loop(){
	// Main user code loop would go here
  int value = getRegister(0);

  Serial.print("Register 0: ");
  Serial.println(value);

  Serial.print("Register 1: ");
  Serial.println(getRegister(1));
  
	// Call the serialUpdate function at least once per loop
	serialUpdate();

  delay(40);
}
