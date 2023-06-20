#pragma once

#include "nota.h"

#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/iom2560.h>



#define F_CPU 16000000UL
#define BAUD 57600
#define MYUBRR F_CPU/16/BAUD-1
#include <util/setbaud.h>


void uart_init();

void arduino_send_packet(nota* n);

