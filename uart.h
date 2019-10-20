/*
 * uart.h
 *
 * Created: 14/10/2018 6:07:25 PM
 *  Author: sshe325
 */ 


#ifndef UART_H_
#define UART_H_

#include <avr/io.h>

#define UDREMPTY (UCSRA & (1<<UDRE)) //check if data register is empty
#define RXCOMP (UCSRA & (1<<RXC)) //check if RXC flag has been set
#define TXCOMP (UCSRA & (1<<TXC)) //check if RXC flag has been set
#define ubrrvalue 103 //F_CPU/(16*BAUDRATE)-1 //Baud rate of 9600

void uart_init();//initialise uart
void uart_transmit(uint8_t data);//transmit character over uart
void uart_transmit_string(char* stringtransmit);//transmit string
char uart_receive(); //receive a character over uart

extern volatile unsigned char received_char;
#endif /* UART_H_ */