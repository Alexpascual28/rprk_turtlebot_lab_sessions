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
  putRegister(0, -5);
  putRegister(1, 127);
  putRegister(2, 128);
  putRegister(3, 255);
  putRegister(4, 256);
}

void loop(){
	// Main user code loop would go here
  Serial.print("Register 5: ");
  Serial.println(getRegister(5));

  Serial.print("Register 6: ");
  Serial.println(getRegister(6));

  Serial.print("Register 7: ");
  Serial.println(getRegister(7));

  Serial.print("Register 8: ");
  Serial.println(getRegister(8));

  Serial.print("Register 9: ");
  Serial.println(getRegister(9));
  
	// Call the serialUpdate function at least once per loop
	serialUpdate();

  delay(40);
}
