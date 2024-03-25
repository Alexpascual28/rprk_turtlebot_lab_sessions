#
# task2.py - Arduino Robotics Board
# May 2023
#
# @author: Alejandro Pascual San Roman (bdh532)
# @organistation: Department of Physics, Engineering and Technology (PET). University of York
# @title: (Autumn Term) Laboratory Session 5 Task 2
#
# READ IN THE ENCODER COUNTS
#

from ARBPi import *
import time

REG_SEND_DATA_ENCODER_A_1 = 36 # Encoder A send messages (Rasperry receives)
REG_SEND_DATA_ENCODER_A_2 = 37 # Encoder A send data (Raspberry recieves)

REG_SEND_DATA_ENCODER_B_1 = 38 # Encoder B send messages (Raspberry receives)
REG_SEND_DATA_ENCODER_B_2 = 39 # Encoder B send data (Raspberry receives)

REG_RECEIVE_SPEED_DATA = 40 # Arduino receives speed data (Raspberry sends)
REG_RECEIVE_MSG_DRIVE = 41 #// Arduino receives drive control data (Raspberry sends)
REG_SEND_MSG_DRIVE = 42 # Arduino sends drive control data (current state) (Raspberry receives)

REG_SEND_DISTANCE_A = 43 # Calculated distance in wheel A
REG_SEND_DISTANCE_A_DEC = 44 # Calculated distance in wheel A (decimal part)

REG_SEND_DISTANCE_B = 45 # Calculated distance in wheel B
REG_SEND_DISTANCE_B_DEC = 46 # Calculated distance in wheel B (decimal part)

REG_SEND_SPEED_A = 47 # Calculated speed of wheel A
REG_SEND_SPEED_A_DEC = 48 # Calculated speed of wheel A (decimal part)

REG_SEND_SPEED_B = 49 # Calculated speed of wheel B
REG_SEND_SPEED_B_DEC = 50 # Calculated speed of wheel B (decimal part)

def setup():
    # Setup the ARB functions
    print("Setting up ARB")
    ARBPiSetup(SERIAL)

def main():
    try:
        # Sets initial values for variables
        setup();
        prev_state = 0
        
        prev_stepsA = 0
        prev_stepsB = 0
        prev_distanceA = 0
        prev_distanceB = 0
        prev_speedA = 0
        prev_speedB = 0
        
        while True:
            # Read user input
            userinput = input("Enter robot direction (WASD) or speed (0-9):")

            speedInput = ("0", "1", "2", "3", "4", "5", "6", "7", "8", "9")

            # If the user input is 0-9
            if userinput in speedInput:
                # Send data
                register = REG_RECEIVE_SPEED_DATA
                value = int(userinput)
                putRegister(register, value)

            directionInput = {"W": 1, "A": 3, "S": 2, "D": 4,
                              "w": 1, "a": 3, "s": 2, "d": 4}

            # If the user input is WASD
            if userinput in directionInput:
                # Send data
                register = REG_RECEIVE_MSG_DRIVE
                value = directionInput.get(userinput) # Get associated signal in directionInput
                putRegister(register, value)

            # Read state of the robot
            robot_state = getRegister(REG_SEND_MSG_DRIVE)
            
            # If the state has changed, print current state
            if((robot_state != prev_state) and (robot_state > 0) and (robot_state <= 5)):
                robot_direction = {1:"moving forward", 2:"moving backward", 3:"moving left", 4:"moving right", 5:"stopped"}
                robot_state = robot_direction.get(robot_state)
                prev_state = robot_state
                print("The robot is " + robot_state)
                
            # Read steps
            stepsA = getStepsFromRegisters(REG_SEND_DATA_ENCODER_A_1, REG_SEND_DATA_ENCODER_A_2)
            stepsB = getStepsFromRegisters(REG_SEND_DATA_ENCODER_B_1, REG_SEND_DATA_ENCODER_B_2)
            
            # Read distances
            distanceA = getDecimalFromRegisters(REG_SEND_DISTANCE_A, REG_SEND_DISTANCE_A_DEC)
            distanceB = getDecimalFromRegisters(REG_SEND_DISTANCE_B, REG_SEND_DISTANCE_B_DEC)
            
            # Read speeds
            speedA = getDecimalFromRegisters(REG_SEND_SPEED_A, REG_SEND_SPEED_A_DEC)
            speedB = getDecimalFromRegisters(REG_SEND_SPEED_B, REG_SEND_SPEED_B_DEC)
            
            # If any of these values has changed, print them (Motor A)
            if(stepsA != prev_stepsA):
                print("A steps: " + str(stepsA))
                prev_stepsA = stepsA
            if(distanceA != prev_distanceA):
                print("A distance: " + str(distanceA) + " cm")
                prev_distanceA = distanceA
            if(speedA != prev_speedA):
                print("A speed: " + str(speedA) + " cm/s")
                prev_speedA = speedA
            
            # If any of these values has changed, print them (Motor B)
            if(stepsB != prev_stepsB):
                print("B steps: " + str(stepsB))
                prev_stepsB = stepsB
            if(distanceB != prev_distanceB):
                print("B distance: " + str(distanceB) + " cm")
                prev_distanceB = distanceB
            if(speedB != prev_speedB):
                print("B speed: " + str(speedB) + " cm/s")
                prev_speedB = speedB
                
            time.sleep(0.2)

    except KeyboardInterrupt:
        print('interrupted!')

# Reads two values from separate registers and turns them into a 16bit signed number
def getStepsFromRegisters(register1, register2):
    firstPart = getRegister(register2)
    secondPart = getRegister(register1)
    
    steps = (firstPart << 7) + secondPart
    return steps

# Reads two values from separate registers and turns them into a 2-fractionary decimal number
def getDecimalFromRegisters(register1, register2):
    wholePart = getRegister(register1)
    fractPart = getRegister(register2)
    
    resultFrac = wholePart + fractPart/100
    return resultFrac

# Check if the node is executing in the main path
if __name__ == '__main__':
    main();
    