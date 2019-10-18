/*
 * uart.c
 *
 * Created: 14/10/2018 6:07:11 PM
 *  Author: sshe325
 */ 

#include "uart.h"

void uart_init(){
	UCSRB |= (1<<TXEN); //set transmit and receive bit
	UCSRB |= (1<<RXCIE);
	UBRRH = (ubrrvalue >> 8); //load baud rate register with calculated value
	UBRRL = ubrrvalue;
	UCSRC |= ((1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0)); //set 8 bit character size
	UCSRC &= ~(1<<UMSEL); //Set Asynchronous operation
	UCSRC &= ~(1<<UCSZ2); //set 8 bit character size
}

//Transmitter
void uart_transmit_string(char* stringtransmit){
	while (*stringtransmit != 0x00){
		uart_transmit(*stringtransmit);
		stringtransmit++;
	}
}

void uart_transmit(uint8_t data){
	while (UDREMPTY == 0){ //wait for buffer to empty
	}
	UDR = data; //load data register with data byte
}

//Receiver
char uart_receive(){
	while (RXCOMP == 0){ //wait for receive to be completed
	}
	return UDR; //load data register with data byte
}