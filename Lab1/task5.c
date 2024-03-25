/*
	Created on 27/04/2023 at 15:25

	@filename: task5.c
	@compile: gcc -o task5 task5.c -lpigpio -lm
	@run: sudo ./task5
	
	@title: Lab Session 1 Task 5: WRITE A C PROGRAM TO BLINK THE LED USING PIGPIO
	@author: Alejandro Pascual San Roman (bdh532)
	@organisation: School of Physics, Engineering and Technology. University of York
*/

#include <stdio.h>
#include <pigpio.h>
#include <math.h>

/* LED pin number */
#define LED 5

int main(int argc, char *argv[])
{
	if (gpioInitialise() < 0)
	{
		printf("pigpio initialisation failed\n");
		return 1;
	}

	/* Set GPIO modes */
	gpioSetMode(LED, PI_OUTPUT);
	
	/* Get the Pi to blink the LEDs at a faster or slower rate, and a different number of 
	times, then recompile your code and test it */
	float blinkRate = 0.1; // decrease this value to make it faster, increment for slower
	float numberOfBlinks = 5; // set number of blinks
	
	/* Blink LED X times */
	int i;
	for(i=0; i<numberOfBlinks; i++)
	{
		gpioWrite(LED, 1);
		time_sleep(blinkRate);
		gpioWrite(LED, 0);
		time_sleep(blinkRate);
	}
	
	/* Adapt your code so that it runs continuously until Ctrl-C is pressed */
	// while(1)
	// {
		// gpioWrite(LED, 1);
		// time_sleep(blinkRate);
		// gpioWrite(LED, 0);
		// time_sleep(blinkRate);
	// }

	/* Stop DMA, release resources */
	gpioTerminate();

	return 0;
}

