/*
 * EE311.c
 *
 * Created: 5/09/2019 2:02:59 PM
 * Author : ktan404
 */ 

//switch 1-4 control signals = PB4-PB7
//left PMOS = PB2
//left NMOS = PB1
//right PMOS = PD7
//right NMOS = PB0

#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <avr/iom8.h>
#include <avr/interrupt.h>
#include <util/delay.h> //header for delay function

#include "uart.h"
#include "interrupts.h"

int main(void)
{
	//set PWM ports as output
	DDRB |= ((1<<PB2)|(1<<PB1)|(1<<PB0));
	DDRD |= (1<<PD7);
	
	//set normal timer mode
	TCCR2 &= ~(1<<WGM20);
	TCCR2 &= ~(1<<WGM21);
	
	//disconnect OC2 port
	TCCR2 &= ~((1<<COM21) | (1<<COM20));
	
	//set output compare value for timer 2 between 0-255 (OCR2/255 % duty cycle)
	OCR2 = 255;
	
	//set prescaler of 64 (gives effective PWM frequency of 980Hz)
	//TCCR2 |= (1<<CS22);
	//TCCR2 &= ~((1<<CS21) | (1<<CS20));
	//set prescaler of 256 (gives effective PWM frequency of 245Hz)
	TCCR2 |= ((1<<CS22) | (1<<CS21));
	TCCR2 &= ~((1<<CS20));
	
	//turn off PMOSes, turn on NMOSes so motor brakes?!
	//turn off left PMOS
	PORTB &= ~(1<<PB2);
	//turn on left NMOS
	PORTB &= ~(1<<PB1);
	//turn off right PMOS
	PORTD &= ~(1<<PD7);
	//turn on right NMOS
	PORTB &= ~(1<<PB0);
	
	//set timer 1 to normal mode
	TCCR1B &= ~((1<<WGM13) | (1<<WGM12));
	TCCR1A &= ~((1<<WGM11) | (1<<WGM10));
	
	//set prescaler to /256
	TCCR1B |= (1<<CS12);
	TCCR1B &= ~((1<<CS11) | (1<<CS10));
	
	//set timer 1 overflow A and B compare values
	OCR1A = 1250; //for T/4
	OCR1B = 2500; //for 12.5Hz resonant frequency (for T/2)
	//enable timer 2 compare match interrupt and overflow interrupt
	TIMSK |= ((1<<OCIE2) | (1<<TOIE2));
	//enable timer 1 compare match A and B interrupts
	TIMSK |= ((1<<OCIE1A) | (1<<OCIE1B));
	//mark T PWM pulse to be handled next
	t2 = 0;
	//enable global interrupts
	sei();
    while (1)
    {
		if (data_received == 1){//data has been received, parse json
			const char left_bracket = '{';
			char** splitstrings; //pointer to hold arrays of c-strings
			unsigned int k = 1; //iterator for array of c-strings
			
			splitstrings = (char**)malloc(10*sizeof(char*));
			
			for (int j = 0; j < 10; ++j){
				splitstrings[j] = (char*) malloc(10);
			}
			
			splitstrings[0] = strtok(str_buffer,left_bracket);
			
			while (splitstrings[k-1] != NULL){
				splitstrings[k] = strtok(NULL,left_bracket);
				++k;
			}
			//DONT FORGET TO FREE MEMORY
		}
    }
}

