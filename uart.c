/*
 * uart.c
 *
 * Created: 14/10/2018 6:07:11 PM
 *  Author: sshe325
 */ 

#include "uart.h"

void uart_init(uint16_t ubrr){
	UCSRB = (1<<TXEN)| (1<<RXEN); //set transmit and receive bit
	UCSRC &= ~(1<<UMSEL); //Set Asynchronous operation
	UCSRC &= ~(1<<UCSZ2); //set 8 bit character size
	UCSRC |= ((1<<UCSZ1) | (1<<UCSZ0)); //set 8 bit character size
	UBRR = ubrr; //load baud rate register with calculated value
}

//Transmitter
void uart_transmit_string(char* stringtransmit){
	while (*stringtransmit != 0x00){
		usart_transmit(*stringtransmit);
		stringtransmit++;
	}
}

void uart_transmit(uint8_t data){
	while (UDRE == 0){ //wait for buffer to empty
	}
	UDR0 = data; //load data register with data byte
}

//Receiver
void uart_receive_string(char* stringtrreceive){
	while (*stringreceive != 0x00){
		usart_receive(*stringreceive);
		stringreceive++;
	}
}
void uart_receive(uint8_t data){
	while (UDRE == 0){ //wait for buffer to empty
	}
	UDR0 = data; //load data register with data byte
}