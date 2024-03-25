#!/usr/bin/python3

"""
Created on 27/04/2023 at 16:19

@filename: task6.py
@compile: gcc -o task6 task6.c -lpigpio -lm
@run: python3 pigpio_example.py  or  ./pigpio_example.py

@title: Lab Session 1 Task 6: BLINK THE LED USING PYTHON AND THE PIGPIOD DAEMON
@author: Alejandro Pascual San Roman (bdh532)
@organisation: School of Physics, Engineering and Technology. University of York
"""

import time
import pigpio # http://abyz.co.uk/rpi/pigpio/python.html

LED = 5

pi = pigpio.pi()

# Set LED pin as output
pi.set_mode(LED, pigpio.OUTPUT)

# Get the Pi to blink the LEDs at a faster or slower rate, and a different number of 
# times, then recompile your code and test it
blink_rate = 0.1
number_of_blinks = 5

# # Main loop
# for i in range(0,number_of_blinks):
	# # Toggle LED on and off
	# pi.write(LED, 1)
	# time.sleep(blink_rate)
	# pi.write(LED, 0)
	# time.sleep(blink_rate)
    
# Adapt your code so that it runs continuously until Ctrl-C is pressed
while True:
    # Toggle LED on and off
	pi.write(LED, 1)
	time.sleep(blink_rate)
	pi.write(LED, 0)
	time.sleep(blink_rate)

pi.stop()

