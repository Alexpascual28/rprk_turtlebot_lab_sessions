/*
	I2CMux.ino - Arduino Robotics Board
	An Arduino library for controling the Arduino Robotics Board platform,
	either standalone or in conjunction with a Raspberry Pi
	June 2022
	
	This example shows how the I2C multiplexer on the board can be used to
	connect to 4 separate I2C buses, allowing for multiple of the same device
	with the same address to be used.
	
	For this example you will need 1 GP2Y0E02B IR range sensors, connected to the
	first I2C bus. The data is then transmitted to the Raspberry Pi through serial
*/

#include <ARB.h>
#include <Wire.h>

#define IR_SENSOR_ADDRESS      0x80 >> 1  // 0xEE >> 1  // 0x80 >> 1 // Address for IR sensor, shifted to provide 7-bit version
#define IR_SENSOR_DISTANCE_REG 0x5E // Register address to read distance from
#define IR_SENSOR_SHIFT_REG    0x35 // Register address to read shift value from

const int BUS_NUMBER = 0; // I2C bus number to read from. For the IR sensor.
const int IR_SERIAL_REGISTER = 0; // Serial register to send the read data to

int shift; //Stores shift value from sensor

void setup() {
	ARBSetup(true); // Setup everything required by the board and enable serial comms
	Serial.begin(9600); // Start serial

  clearSerialRegisters();
  setupI2C(BUS_NUMBER);
}

void loop() {
	int infrared_distance = readI2CSensor(BUS_NUMBER);

  putRegister(IR_SERIAL_REGISTER, infrared_distance);

  // Call the serialUpdate function at least once per loop
  serialUpdate();

	// Delay before loop starts over
	delay(50);
}

void clearSerialRegisters(){
  for(int i = 0; i < 128; i++){
    putRegister(i, NULL);
  }
}

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
