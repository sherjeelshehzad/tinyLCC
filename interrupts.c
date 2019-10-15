/*
 * interrupts.c
 *
 * Created: 10/10/2019 6:33:45 PM
 *  Author: sshe325
 */ 

#include "interrupts.h"

ISR(USART_RXC_vect){
	//dynamically allocate and reallocate space for string
	str_buffer = calloc(1,sizeof(char));
	char* buffer_ptr = str_buffer;
	int size = 1;
	*str_buffer = uart_receive();
	//only end receive when we receive a newline
	while ((*buffer_ptr != '\n') || (*buffer_ptr != '\r')){
		str_buffer = realloc(str_buffer,(++size)*sizeof(char));
		//make a null terminated string for strcat()
		char uart_char[2];
		uart_char[0] = uart_receive();
		uart_char[1] = '\0';
		strcat(str_buffer,uart_char);
		++buffer_ptr;
	}
	//add null terminator to turn it into proper c-string
	str_buffer = realloc(str_buffer,(++size)*sizeof(char));
	*(++buffer_ptr) = '\0';
	//turn off receiver to prevent any more interrupts before we have parsed the current data
	UCSRB &= ~(1<<RXEN);
	//set flag to indicate received data
	data_received = 1;
}

//interrupt to increment overflow counter
ISR(TIMER0_OVF_vect){
	++timer0_ovf_count;
}

//interrupt to handle T/4 or 3T/4 time crossing
ISR(TIMER1_COMPA_vect){
		//turn off PWM timer
		TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
		
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
		
		//indicate that RHS/LHS voltage are both off
		voltage_right_on = 0;
		voltage_left_on = 0;
		
		//since we have stopped pulsing PWM, stop reading current ADC
		readadci = 0;
		//disable PWM signals
		if (!t2){
			//right to left current
			//turn off PMOSes, turn on left NMOS so motor generates back emf
			//turn off left PMOS
			PORTB &= ~(1<<PB2);
			//turn on left NMOS
			PORTB &= ~(1<<PB1);
			//turn off right PMOS
			PORTD &= ~(1<<PD7);
			//turn off right NMOS
			PORTB |= (1<<PB0);
			//indicate that we are ready to start reading motor LHS back emf
			readadc = 1;
			readadcmotorleft = 1;
			backemffound = 0;
			//reset backemf index
			backemf_i = 0;
		}
		else{
			//left to right current
			//turn off PMOSes, turn on right NMOS so motor generates back emf
			//turn off left PMOS
			PORTB &= ~(1<<PB2);
			//turn off left NMOS
			PORTB |= (1<<PB1);
			//turn off right PMOS
			PORTD &= ~(1<<PD7);
			//turn on right NMOS
			PORTB &= ~(1<<PB0);
			//indicate that we are ready to start reading motor RHS back emf
			readadc = 1;
			readadcmotorright = 1;
			backemffound = 0;
			//reset backemf index
			backemf_i = 0;
		}
		//turn on timer 0 with prescaler /256 to measure backemf time
		TCCR0 |= ((1<<CS02));
		TCCR0 &= ~((1<<CS01)|(1<<CS00));
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
		//back emf pulse has expired (we are driving the motor now), stop reading the adc
		readadcmotorleft = 0;
		readadcmotorright = 0;
		if (backemffound == 0){
			backemftime += 10000; //add 10ms to back emf time (since we didn't find it previously, we must have driven the motor too fast)
		}
		if (t2){
			//T time crossing has been handled, T/2 is next
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
			//turn on PWM signals
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
				//indicate that we are ready to read adc from motor RHS to measure voltage and current
				readadc = 1;
				readadcv = 1;
				readadci = 1;
				//indicate that RHS voltage is on
				voltage_right_on = 1;
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
				//indicate that we are ready to read adc from motor LHS to measure voltage and current
				readadc = 1;
				readadcv = 1;
				readadci = 1;
				//indicate that LHS voltage is on
				voltage_left_on = 1;
			}
			//turn on PWM timer counter (prescaler /256)
			TCCR2 |= ((1<<CS22) | (1<<CS21));
			TCCR2 &= ~((1<<CS20));
			
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
			//turn on PWM signals
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
				//indicate that we are ready to read adc from motor RHS to measure voltage and current
				readadc = 1;
				readadcv = 1;
				readadci = 1;
				//indicate that RHS voltage is on
				voltage_right_on = 1;
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
				//indicate that we are ready to read adc from motor LHS to measure voltage and current
				readadc = 1;
				readadcv = 1;
				readadci = 1;
				//indicate that LHS voltage is on
				voltage_left_on = 1;
			}
			//turn on PWM timer counter (prescaler /256)
			TCCR2 |= ((1<<CS22) | (1<<CS21));
			TCCR2 &= ~((1<<CS20));
			
		}
		//++stop_counter;
		
}

//PWM duty cycle expired, shut off PWM signals
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
	//turn off PWM
	TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
	
	if (t2){
		//right to left current
		//turn off PMOSes, turn on left NMOS so motor brakes
		//turn off left PMOS
		PORTB &= ~(1<<PB2);
		//turn on left NMOS
		PORTB &= ~(1<<PB1);
		//turn off right PMOS
		PORTD &= ~(1<<PD7);
		//turn off right NMOS
		PORTB |= (1<<PB0);
		//indicate that RHS voltage is off
		voltage_right_on = 0;
	}
	else{
		//left to right current
		//turn off PMOSes, turn on right NMOS so motor brakes?!
		//turn off left PMOS
		PORTB &= ~(1<<PB2);
		//turn off left NMOS
		PORTB |= (1<<PB1);
		//turn off right PMOS
		PORTD &= ~(1<<PD7);
		//turn on right NMOS
		PORTB &= ~(1<<PB0);
		//indicate that LHS voltage is off
		voltage_left_on = 0;
	}
	//turn on PWM timer counter (prescaler /256)
	TCCR2 |= ((1<<CS22) | (1<<CS21));
	TCCR2 &= ~((1<<CS20));
}

//PWM restarted, change switches according to current current direction
ISR(TIMER2_OVF_vect){
	//turn off PWM
	TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
	TCCR2 = 0;
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
		//indicate that RHS voltage is on
		voltage_right_on = 1;
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
		//indicate that LHS voltage is on
		voltage_left_on = 1;
	}
	//turn on PWM timer counter (prescaler /256)
	TCCR2 |= ((1<<CS22) | (1<<CS21));
	TCCR2 &= ~((1<<CS20));
}