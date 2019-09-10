/*
 * EE311.c
 *
 * Created: 5/09/2019 2:02:59 PM
 * Author : ktan404
 */ 

//switch 1-4 control signals = PB4-PB7
//left PMOS = PB4
//left NMOS = PB5
//right PMOS = PB6
//right NMOS = PB7

#define F_CPU 8000000UL

#include <stdio.h>
#include <avr/io.h>
#include <avr/iom8.h>
#include <avr/interrupt.h>
#include <util/delay.h> //header for delay function

extern volatile unsigned int t2 = 0; //flag for if T/2 time crossing is next to be handled

//interrupt to handle T/4 or 3T/4 time crossing
ISR(TIMER1_COMPA_vect){
	//turn off PWM
	TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
	//disable PWM
	//open all switches so motor coasts along
	//turn off left PMOS
	//PORTB &= ~(1<<PB4);
	//turn off left NMOS
	//PORTB |= (1<<PB5);
	//turn off right PMOS
	//PORTB &= ~(1<<PB6);
	//turn off right NMOS
	//PORTB |= (1<<PB7);
	
	//turn off PMOSes, turn on NMOSes so motor brakes?!
	//turn off left PMOS
	PORTB &= ~(1<<PB4);
	//turn on left NMOS
	PORTB &= ~(1<<PB5);
	//turn off right PMOS
	PORTB &= ~(1<<PB6);
	//turn on right NMOS
	PORTB &= ~(1<<PB7);
	//reset PWM timer counter
	TCNT2 = 0;
}

//interrupt to handle T or T/2 time crossing
ISR(TIMER1_COMPB_vect){
	if (!(t2)){
		//turn off timer1
		TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
		//turn off PWM
		TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
		//reset timer1 counter
		TCNT1 = 0;
		//reset PWM timer counter
		TCNT2 = 0;
		//turn on timer 1 again (prescaler /64)
		TCCR1B &= ~(1<<CS12);
		TCCR1B |= ((1<<CS11) | (1<<CS10));
		//turn on PWM timer counter (prescaler /64)
		//TCCR2 |= (1<<CS22);
		//TCCR2 &= ~((1<<CS21) | (1<<CS20));
		//turn on PWM timer counter (prescaler /128)
		TCCR2 |= ((1<<CS22) | (1<<CS20));
		TCCR2 &= ~((1<<CS21));
		//T time crossing has been handled, T/2 is next
		t2 = 1;
	}
	else{
		//turn off timer1
		TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
		//turn off PWM
		TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
		//reset timer1 counter
		TCNT1 = 0;
		//reset PWM timer counter
		TCNT2 = 0;
		//turn on timer 1 again (prescaler /64)
		TCCR1B &= ~(1<<CS12);
		TCCR1B |= ((1<<CS11) | (1<<CS10));
		//turn on PWM timer counter (prescaler /64)
		//TCCR2 |= (1<<CS22);
		//TCCR2 &= ~((1<<CS21) | (1<<CS20));
		//turn on PWM timer counter (prescaler /128)
		TCCR2 |= ((1<<CS22) | (1<<CS20));
		TCCR2 &= ~((1<<CS21));
		t2 = 0;
	}
}

//PWM duty cycle expired, shut off signal
ISR(TIMER2_COMP_vect){
	//open all switches so motor coasts along
	//turn off left PMOS
	//PORTB &= ~(1<<PB4);
	//turn off left NMOS
	//PORTB |= (1<<PB5);
	//turn off right PMOS
	//PORTB &= ~(1<<PB6);
	//turn off right NMOS
	//PORTB |= (1<<PB7);
	
	//turn off PMOSes, turn on NMOSes so motor brakes?!
	//turn off left PMOS
	PORTB &= ~(1<<PB4);
	//turn on left NMOS
	PORTB &= ~(1<<PB5);
	//turn off right PMOS
	PORTB &= ~(1<<PB6);
	//turn on right NMOS
	PORTB &= ~(1<<PB7);
}

//PWM restarted, change switches according to current current direction
ISR(TIMER2_OVF_vect){
	if ((t2)){
		//right to left current
		//turn off left PMOS
		PORTB &= ~(1<<PB4);
		//turn on left NMOS
		PORTB &= ~(1<<PB5);
		//turn on right PMOS
		PORTB |= (1<<PB6);
		//turn off right NMOS
		PORTB |= (1<<PB7);
	}
	else{
		//left to right current
		//turn on left PMOS
		PORTB |= (1<<PB4);
		//turn off left NMOS
		PORTB |= (1<<PB5);
		//turn off right PMOS
		PORTB &= ~(1<<PB6);
		//turn on right NMOS
		PORTB &= ~(1<<PB7);
	}
}

int main(void)
{
	//set ports as output
	DDRC = 0xFF;
	DDRB = 0xFF;
	//set normal timer mode
	TCCR2 &= ~(1<<WGM20);
	TCCR2 &= ~(1<<WGM21);
	
	//disconnect OC2 port
	TCCR2 &= ~((1<<COM21) | (1<<COM20));
	
	//set output compare value for timer 2 between 0-255 (OCR2/255 % duty cycle)
	OCR2 = 128;
	
	//set prescaler of 64 (gives effective PWM frequency of 490Hz)
	//TCCR2 |= (1<<CS22);
	//TCCR2 &= ~((1<<CS21) | (1<<CS20));
	//set prescaler of 128 (gives effective PWM frequency of 245Hz)
	TCCR2 |= ((1<<CS22) | (1<<CS20));
	TCCR2 &= ~((1<<CS21));
	
	//turn off PMOSes, turn on NMOSes so motor brakes?!
	//turn off left PMOS
	PORTB &= ~(1<<PB4);
	//turn on left NMOS
	PORTB &= ~(1<<PB5);
	//turn off right PMOS
	PORTB &= ~(1<<PB6);
	//turn on right NMOS
	PORTB &= ~(1<<PB7);
	
	//set timer 1 to normal mode
	TCCR1B &= ~((1<<WGM13) | (1<<WGM12));
	TCCR1A &= ~((1<<WGM11) | (1<<WGM10));
	
	//set prescaler to /64
	TCCR1B &= ~(1<<CS12);
	TCCR1B |= ((1<<CS11) | (1<<CS10));
	
	//set timer 1 overflow A and B compare values
	OCR1A = 2500; //for T/4
	OCR1B = 5000; //for 12.5Hz resonant frequency (for T/2)
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
    }
}

