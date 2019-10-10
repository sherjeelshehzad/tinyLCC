/*
 * interrupts.c
 *
 * Created: 10/10/2019 6:33:45 PM
 *  Author: sshe325
 */ 

#include "interrupts.h"
#include "uart.h"

t2 = 0; //flag for if T/2 time crossing is next to be handled
stop_counter = 0; //counter to stop PWM every STOPCYCLE/2 number of cycles
data_received = 0; //set data received flag to 0 initially

ISR(USART_RXC_vect){
	//dynamically allocate and reallocate space for string
	str_buffer = calloc(1,sizeof(char));
	char* buffer_ptr = str_buffer;
	int size = 1;
	*str_buffer = uart_receive();
	//only end receive when we receive a newline
	while (*buffer_ptr != '\n'){
		str_buffer = realloc(str_buffer,(++size)*sizeof(char));
		strcat(str_buffer,uart_receive());
		++buffer_ptr;
	}
	//turn off receiver to prevent any more interrupts before we have parsed the current data
	UCSRB &= ~(1<<RXEN);
	//set flag to indicate received data
	data_received = 1;
}

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