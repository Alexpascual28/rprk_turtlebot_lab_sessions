/*
	pigpio_example.c

	Compile with:
	gcc -o pigpio_example pigpio_example.c -lpigpio

	Run with:
	sudo ./pigpio_example
*/

#include <stdio.h>
#include <pigpio.h>

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

	/* Blink LED 10 times */
	int i;
	for(i=0; i<10; i++)
	{
		gpioWrite(LED, 1);
		time_sleep(0.5);
		gpioWrite(LED, 0);
		time_sleep(0.5);
	}

	/* Stop DMA, release resources */
	gpioTerminate();

	return 0;
}

