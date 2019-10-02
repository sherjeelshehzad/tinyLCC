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
#define STOPCYCLE 100

#include <stdio.h>
#include <avr/io.h>
#include <avr/iom8.h>
#include <avr/interrupt.h>
#include <util/delay.h> //header for delay function

volatile unsigned int t2 = 0; //flag for if T/2 time crossing is next to be handled
volatile unsigned int stop_counter = 0; //counter to stop PWM every STOPCYCLE/2 number of cycles
//interrupt to handle T/4 or 3T/4 time crossing
ISR(TIMER1_COMPA_vect){
		//turn off PWM
		TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
		//disable PWM
		//open all switches so motor coasts along
		//turn off left PMOS
		//PORTB &= ~(1<<PB2);
		//turn off left NMOS
		//PORTB |= (1<<PB1);
		//turn off right PMOS
		//PORTD &= ~(1<<PD7);
		//turn off right NMOS
		//PORTB |= (1<<PB0);
			
		//turn off PMOSes, turn on NMOSes so motor brakes?!
		//turn off left PMOS
		PORTB &= ~(1<<PB2);
		//turn on left NMOS
		PORTB &= ~(1<<PB1);
		//turn off right PMOS
		PORTD &= ~(1<<PD7);
		//turn on right NMOS
		PORTB &= ~(1<<PB0);
		//reset PWM timer counter
		TCNT2 = 0;
}

//interrupt to handle T or T/2 time crossing
ISR(TIMER1_COMPB_vect){
	
	/*if (stop_counter == STOPCYCLE){
		//turn off timer1
		TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
		//turn off PWM
		TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
		//reset timer1 counter
		TCNT1 = 0;
		//reset PWM timer counter
		TCNT2 = 0;
		//open all switches so motor coasts along
		//turn off left PMOS
		PORTB &= ~(1<<PB2);
		//turn off left NMOS
		PORTB |= (1<<PB1);
		//turn off right PMOS
		PORTD &= ~(1<<PD7);
		//turn off right NMOS
		PORTB |= (1<<PB0);
		//turn on timer 1 again (prescaler /256)
		TCCR1B |= (1<<CS12);
		TCCR1B &= ~((1<<CS11) | (1<<CS10));
		stop_counter = 0;
	}*/

		if (t2){
			t2 = 0;
			//turn off timer1
			TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
			//turn off PWM
			TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
			//reset timer1 counter
			TCNT1 = 0;
			//reset PWM timer counter
			TCNT2 = 0;
			//turn on timer 1 again (prescaler /256)
			TCCR1B |= (1<<CS12);
			TCCR1B &= ~((1<<CS11) | (1<<CS10));
			//turn on PWM timer counter (prescaler /64)
			//TCCR2 |= (1<<CS22);
			//TCCR2 &= ~((1<<CS21) | (1<<CS20));
			//turn on PWM timer counter (prescaler /256)
			TCCR2 |= ((1<<CS22) | (1<<CS21));
			TCCR2 &= ~((1<<CS20));
			//T time crossing has been handled, T/2 is next
		}
		else{
			t2 = 1;
			//turn off timer1
			TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
			//turn off PWM
			TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
			//reset timer1 counter
			TCNT1 = 0;
			//reset PWM timer counter
			TCNT2 = 0;
			//turn on timer 1 again (prescaler /256)
			TCCR1B |= (1<<CS12);
			TCCR1B &= ~((1<<CS11) | (1<<CS10));
			//turn on PWM timer counter (prescaler /64)
			//TCCR2 |= (1<<CS22);
			//TCCR2 &= ~((1<<CS21) | (1<<CS20));
			//turn on PWM timer counter (prescaler /256)
			TCCR2 |= ((1<<CS22) | (1<<CS21));
			TCCR2 &= ~((1<<CS20));
		}
		//++stop_counter;
		
}

//PWM duty cycle expired, shut off signal
ISR(TIMER2_COMP_vect){
	//open all switches so motor coasts along
	//turn off left PMOS
	//PORTB &= ~(1<<PB2);
	//turn off left NMOS
	//PORTB |= (1<<PB1);
	//turn off right PMOS
	//PORTD &= ~(1<<PD7);
	//turn off right NMOS
	//PORTB |= (1<<PB0);
	
	//turn off PMOSes, turn on NMOSes so motor brakes?!
	//turn off left PMOS
	//PORTB &= ~(1<<PB2);
	//turn on left NMOS
	//PORTB &= ~(1<<PB1);
	//turn off right PMOS
	//PORTD &= ~(1<<PD7);
	//turn on right NMOS
	//PORTB &= ~(1<<PB0);
	/*if (stop_counter >= STOPCYCLE - 1){
		//open all switches so motor coasts along
		//turn off left PMOS
		PORTB &= ~(1<<PB2);
		//turn off left NMOS
		PORTB |= (1<<PB1);
		//turn off right PMOS
		PORTD &= ~(1<<PD7);
		//turn off right NMOS
		PORTB |= (1<<PB0);
	}*/
		if (t2){
			//right to left current
			//turn off PMOSes, turn on right NMOS so motor brakes
			//turn off left PMOS
			PORTB &= ~(1<<PB2);
			//turn off left NMOS
			PORTB |= (1<<PB1);
			//turn off right PMOS
			PORTD &= ~(1<<PD7);
			//turn on right NMOS
			PORTB &= ~(1<<PB0);
		}
		else{
			//left to right current
			//turn off PMOSes, turn on left NMOS so motor brakes?!
			//turn off left PMOS
			PORTB &= ~(1<<PB2);
			//turn on left NMOS
			PORTB &= ~(1<<PB1);
			//turn off right PMOS
			PORTD &= ~(1<<PD7);
			//turn off right NMOS
			PORTB |= (1<<PB0);
		}
}

//PWM restarted, change switches according to current current direction
ISR(TIMER2_OVF_vect){
	//turn off PWM
	TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
	if (t2){
		//right to left current
		//turn off left PMOS
		PORTB &= ~(1<<PB2);
		//turn on left NMOS
		PORTB &= ~(1<<PB1);
		//turn on right PMOS
		PORTD |= (1<<PD7);
		//turn off right NMOS
		PORTB |= (1<<PB0);
	}
	else{
		//left to right current
		//turn on left PMOS
		PORTB |= (1<<PB2);
		//turn off left NMOS
		PORTB |= (1<<PB1);
		//turn off right PMOS
		PORTD &= ~(1<<PD7);
		//turn on right NMOS
		PORTB &= ~(1<<PB0);
	}
	//turn on PWM timer counter (prescaler /256)
	TCCR2 |= ((1<<CS22) | (1<<CS21));
	TCCR2 &= ~((1<<CS20));
}


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
    }
}

