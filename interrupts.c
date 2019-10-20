/*
 * interrupts.c
 *
 * Created: 10/10/2019 6:33:45 PM
 *  Author: sshe325
 */ 

#include "interrupts.h"
#include "uart.h"

//json receiver and parser
ISR(USART_RXC_vect){
	uart_char = UDR;
	switch (state){
		case ID:
			if (uart_char == '3') {state = MFC; mfcstate = M; clrstate = clrC; reqstate = req1;}
			break;
		case MFC:
			switch (mfcstate){
				case M:
					if (uart_char == 'm') {mfcstate = F;}
					break;
				case F:
					if (uart_char == 'f') {mfcstate = C;}
					break;
				case C:
					if (uart_char == 'c') {mfcstate = R;}
					break;
				case R:
					if (uart_char == 'r') {mfcstate = E;}
					break;
				case E:
					if (uart_char == 'e') {mfcstate = Q;}
					break;
				case Q:
					if (uart_char == 'q') {
						state = REQ; 
						mfcstate = M;
						clrstate = clrC;
						reqstate = req1;
						//do reqval
					}
					break;
				default:
					break;
			}
			break;
		case REQ:
			switch(reqstate){
				case req1:
					if ((uart_char >= 48) && (uart_char <= 57)){
						reqval = (uart_char - 48) * 100;
						reqstate = req2;
					}
					break;
				case req2:
					if ((uart_char >= 48) && (uart_char <= 57)){
						reqval = reqval + ((uart_char - 48) * 10);
						reqstate = req3;
					}
					break;
				case req3:
					if ((uart_char >= 48) && (uart_char <= 57)){
						reqval = reqval + (uart_char - 48);
						if ((reqval >= 0) && (reqval <= 255)){
							duty_request = reqval; //set duty cycle request if valid value detected
						}
						state = CLR;
						reqstate = req1;
					}
					break;
				default:
					break;
			}
			break;
		case CLR:
			switch(clrstate){
				case clrC:
					if (uart_char == 'c') {clrstate = L;}
					break;
				case L:
					if (uart_char == 'l') {clrstate = clrR;}
					break;
				case clrR:
					if (uart_char == 'r') {clrstate = clrE;}
					break;
				case clrE:
					if (uart_char == 'e') {clrstate = W;}
					break;
				case W:
					if (uart_char == 'w') {
						clrstate = br1;
					}
					break;
				case br1:
					if (uart_char == '}') {
						clrstate = br1;
					}
				case br2:
					if (uart_char == '}') {
						state = ID;
						mfcstate = M;
						reqstate = req1;
						clrstate = clrC;
						clear_errors = 1;
						transmit_data = 1;
					}
				default:
					break;
			}
			break;
		default:
			break;
	}
	wdr();
}
//interrupt to increment overflow counter
ISR(TIMER0_OVF_vect){
	++timer0_ovf_count;
}

//interrupt to handle T/4 or 3T/4 time crossing
ISR(TIMER1_COMPA_vect){
		//turn off PWM timer
		TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
		//indicate that RHS/LHS voltage are both off
		voltage_right_on = 0;
		voltage_left_on = 0;
		
		//since we have stopped pulsing PWM, stop reading current ADC
		readadci = 0;
		adcdone = 0;
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
			readadcmotorright = 1;
			backemffound = 0;
			//reset backemf index
			backemfreadingindex = 0;
			PORTD &= ~(1<<PD5);
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
			readadcmotorleft = 1;
			backemffound = 0;
			//reset backemf index
			backemfreadingindex = 0;
			PORTD |= (1<<PD5);
		}
		//turn on timer 0 with prescaler /256 to measure backemf time
		TCNT0 = 0;
		TCCR0 |= ((1<<CS02));
		TCCR0 &= ~((1<<CS01)|(1<<CS00));
		//reset PWM timer counter
		TCNT2 = 0;
}

//interrupt to handle T or T/2 time crossing
ISR(TIMER1_COMPB_vect){
		//back emf pulse has expired (we are driving the motor now), stop reading the adc
		readadcmotorleft = 0;
		readadcmotorright = 0;
		if (numcycles >= 50){
			if (backemffound){
				if (validbackemffound == 0){
					backemftime += 1; //add 0.5ms to back emf time (since we didn't find it previously, we must have driven the motor too fast)
					//backemffreq = 1000000000/backemftime;
					backemffound = 1; //indicate that a back emf value has been "found"
				}
			}
			else{
				backemftime += 5; //add 0.5ms to back emf time (since we didn't find it previously, we must have driven the motor too fast)
				//backemffreq = 1000000000/backemftime;
				backemffound = 1; //indicate that a back emf value has been "found"
			}
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
			//turn on PWM timer counter (prescaler /256)
			TCCR2 |= ((1<<CS22) | (1<<CS21));
			TCCR2 &= ~((1<<CS20));
			
		}
		//turn on PWM signals if we are not braking
		if (OCR2 != 0){
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
		}
		
		
		//indicate that we are ready to read adc from motor RHS to measure voltage and current
		readadc = 1;
		readadcv = 1;
		readadci = 1;
		//reset ADC indices
		current_i = 0;
		currentreadingindex = 0;
		//++stop_counter;
		//stop back emf timer since the time to measure it has expired
		TCCR0 &= ~((1<<CS02)|(1<<CS01)|(1<<CS00));
		TCNT0 = 0;
		timer0_ovf_count = 0;
		if (numcycles != 50) ++numcycles;
}

//PWM duty cycle expired, shut off PWM signals
ISR(TIMER2_COMP_vect){
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
	}
	readadc = 1;
	//indicate that voltage is off
	voltage_right_on = 0;
	voltage_left_on = 0;
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
		readadc = 1;
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
		readadc = 1;
		voltage_left_on = 1;
	}
	//turn on PWM timer counter (prescaler /256)
	TCCR2 |= ((1<<CS22) | (1<<CS21));
	TCCR2 &= ~((1<<CS20));
}