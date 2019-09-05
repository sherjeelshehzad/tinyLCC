/*
 * EE311.c
 *
 * Created: 5/09/2019 2:02:59 PM
 * Author : ktan404
 */ 
#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> //header for delay function

//interrupt to handle T/4 time crossing
ISR(TIMER1_COMPA_vect){
	//turn off PWM
	TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<CS20));
	//diconnect OC2
	TCCR2 &= ~((1<<COM21) | (1<<COM20));
	//turn output high (inverted logic)
	PORTB |= (1<<PB3);
	//reset PWM timer counter
	TCNT2 = 0;
}

//interrupt to handle T time crossing
ISR(TIMER1_COMPB_vect){
	//turn off timer1
	TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
	//reset timer1 counter
	TCNT1 = 0;
	//turn on timer 1 again (prescaler /64)
	TCCR1B &= ~(1<<CS12);
	TCCR1B |= ((1<<CS11) | (1<<CS10));
	//set inverted PWM mode
	TCCR2 |= ((1<<COM21) | (1<<COM20));
	//turn on PWM timer counter (prescaler /64)
	TCCR2 |= (1<<CS22);
	TCCR2 &= ~((1<<CS21) | (1<CS20));
	
}

int main(void)
{
	unsigned int pin_buffer;
	DDRC = 0xFF;
	//set OC2 pin as output
	DDRB |= ((1<<PB3)|(1<<PB5)|(1<<PB6));
	//set phase correct PWM mode
	TCCR2 |= (1<<WGM20);
	TCCR2 &= ~(1<<WGM21);
	//set inverted PWM mode
	TCCR2 |= ((1<<COM21) | (1<<COM20));
	//set output compare value (OCR2/255 %)
	OCR2 = 128;
	//set prescaler of 64 (gives effective PWM frequency of 240Hz)
	TCCR2 |= (1<<CS22);
	TCCR2 &= ~((1<<CS21) | (1<CS20));
	
	
	//set timer 1 to normal mode
	TCCR1B &= ~((1<<WGM13) | (1<<WGM12));
	TCCR1A &= ~((1<<WGM11) | (1<<WGM10));
	
	//set prescaler to /64
	TCCR1B &= ~(1<<CS12);
	TCCR1B |= ((1<<CS11) | (1<<CS10));
	
	//set timer 1 overflow A and B compare values
	OCR1A = 2500; //for T/4
	OCR1B = 10000; //for 12.5Hz resonant frequency
	
	//enable timer 1 overflow A and B interrupts
	TIMSK |= ((1<<OCIE1A) | (1<<OCIE1B));
	//enable global interrupts
	sei();
	pin_buffer = PINB3;
    while (1)
    {
		if (PINB3){
			PORTB |= (1<<PB6);
			pin_buffer = PINB3;
		}
		else{
			PORTB &= ~(1<<PB6);
			pin_buffer = PINB3;
		}
    }
}

