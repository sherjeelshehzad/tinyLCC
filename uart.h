/*
 * uart.h
 *
 * Created: 14/10/2018 6:07:25 PM
 *  Author: sshe325
 */ 


#ifndef UART_H_
#define UART_H_

#include <avr/io.h>

#define UDRE (UCSR0A & (1<<UDRE0)) //check if data register is empty

void usart_init(uint16_t ubrr);//initialise uart
void usart_transmit(uint8_t data);//transmit character over uart
void usart_transmit_string(char* stringtransmit);//transmit string

#endif /* UART_H_ */