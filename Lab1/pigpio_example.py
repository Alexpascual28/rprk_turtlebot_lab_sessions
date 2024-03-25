#!/usr/bin/python3

import time
import pigpio # http://abyz.co.uk/rpi/pigpio/python.html

LED = 5

pi = pigpio.pi()

# Set LED pin as output
pi.set_mode(LED, pigpio.OUTPUT)

# Main loop
for i in range(0,10):
	# Toggle LED on and off
	pi.write(LED, 1)
	time.sleep(0.5)
	pi.write(LED, 0)
	time.sleep(0.5)

pi.stop()

