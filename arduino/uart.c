#include "uart.h"
#include "nota.h"


uint8_t data_to_send[2];

void uart_init() {
    
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    UCSR0A |= (1<<U2X0);


    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */ 
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);   /* Enable RX and TX */  
    
    sei();

}

ISR(USART0_UDRE_vect){
	for(int i = 0; i<2; i++){
	
		UDR0 = data_to_send[i];
		
	}
	UCSR0B &= ~_BV(UDRIE0);
}

void arduino_send_packet(nota* n){

	data_to_send[0] = n->pin;
	data_to_send[1] = n->tipo_evento;
	
	UCSR0B |=  _BV(UDRIE0); //attivo l'interrupt per inviare
}





