#include "uart.h"

#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>


#define PIN_MASK 0xF0 // maschera per i 4 pin (da PB4 a PB7....da PIN 10 a PIN 13)

volatile uint8_t int_occurred = 0;
volatile uint16_t int_count = 0;

// interrupt routine for position PCINT0
ISR(PCINT0_vect)
{
	int_occurred = 1;
	int_count++;
}

int main(void)
{

	uart_init();

	DDRB &= ~PIN_MASK; // set PIN_MASK pins as input
	PORTB |= PIN_MASK; // enable pull up resistors

	PCICR |= (1 << PCIE0); // set interrupt on change, looking up PCMSK0
	PCMSK0 |= PIN_MASK;	   // set PCINT0 to trigger an interrupt on state change
	sei();

	while (1)
	{
		while (!int_occurred);
		// we reset the flag;
		int_occurred = 0;
		for (uint8_t x = 4; x < 8; x++)
		{
			if (bit_is_set(PINB, x))
			{

				nota *np = (nota *)malloc(sizeof(nota));
				np->pin = x + 6;
				np->tipo_evento = 0;
				arduino_send_packet(np);

				loop_until_bit_is_clear(PINB, x);

				nota *nr = (nota *)malloc(sizeof(nota));
				nr->pin = x + 6;
				nr->tipo_evento = 1;
				arduino_send_packet(nr);

				free(np);
				free(nr);
			}
		}
	}
}
