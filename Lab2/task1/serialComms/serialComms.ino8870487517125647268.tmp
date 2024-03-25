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
	putRegister(0, 'h');
	putRegister(1, 'e');
	putRegister(2, 'l');
	putRegister(3, 'l');
	putRegister(4, 'o');
  putRegister(5, 999);
  putRegister(6, 0xFF);
  putRegister(7, 255);
  putRegister(8, 256);
  putRegister(9, 254);
  putRegister(10, 251);
  putRegister(10, 128);
}

void loop(){
	// Main user code loop would go here
  
  Serial.println(getRegister(30));
  
	// Call the serialUpdate function at least once per loop
	serialUpdate();

  delay(50);
}
