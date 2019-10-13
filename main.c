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
#include <stdlib.h>
#include <string.h>

#include "uart.h"
#include "interrupts.h"

//global variable section
//left = left of the decimal point
//right = right of the decimal point
volatile unsigned int duty_request; //the currently requested duty cycle value
volatile unsigned int transmit_data = 0;//indicate to the UART to transmit a json packet
volatile unsigned int powerleft = 1;
volatile unsigned int powerright = 38;
volatile unsigned int freqleft = 12;
volatile unsigned int freqright = 5;
volatile unsigned int current = 1200;
volatile unsigned int voltleft = 12;
volatile unsigned int voltright = 21;

int main(void)
{
	t2 = 0; //flag for if T/2 time crossing is next to be handled
	stop_counter = 0; //counter to stop PWM every STOPCYCLE/2 number of cycles
	data_received = 0; //set data received flag to 0 initially
	//set PWM ports as output
	DDRB |= ((1<<PB2)|(1<<PB1)|(1<<PB0));
	DDRD |= (1<<PD7);
	
	//set normal timer mode
	TCCR2 &= ~(1<<WGM20);
	TCCR2 &= ~(1<<WGM21);
	
	//disconnect OC2 port
	TCCR2 &= ~((1<<COM21) | (1<<COM20));
	
	//set output compare value for timer 2 between 0-255 (OCR2/255 % duty cycle)
	duty_request = 255;
	OCR2 = duty_request;
	
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
	
	//enable and initialise UART
	uart_init();
	//enable global interrupts
	sei();
    while (1)
    {
		//add gradual increase logic to couple with resonant frequency detection module
		if (OCR2 != duty_request)
			OCR2 = duty_request; //if the duty cycle has been changed, set it to the newly received value
		
		//if data has been received, parse json
		if (data_received){
			const char left_bracket[2] = "{";
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
			
			//extract and validate data from JSON
			//check if id == 3
			if (splitstrings[0][1] == '3'){
				//if true, check mfc key
				char check_mfc[4];
				memcpy(check_mfc, &splitstrings[1][1], 3);
				check_mfc[3] = '\0';
				//check for valid MFC key
				if (strcmp(check_mfc,"mfc") == 0){
					//if true, check if req key is present
					char check_req[4];
					memcpy(check_req, &splitstrings[2][1], 3);
					check_req[3] = '\0';
					if (strcmp(check_req,"req") == 0){
						//if true, check mass flow request value
						char check_reqval[8];
						memcpy(check_reqval, &splitstrings[2][8], 7);
						//isolate value from key-value pair using apostrophe as the delimiter
						check_reqval[7] = '\0';
						char check_val[4];
						char* commapos = strchr(check_reqval, '"');
						memcpy(check_val, check_reqval, (commapos - check_reqval));
						check_val[commapos - check_reqval] = '\0';
						if (check_val[0] != '\0'){
							//if value was not empty
							unsigned int notzero = 0;
							//check if value is 0
							//we need to check if we received a 0 since strtol() returns a 0 on invalid inputs as well
							for (int n = 0; n < (commapos - check_reqval);++n){
								if (check_val[n] != '0'){
									notzero = 1;
								}
							}
							if (notzero == 1){
								int duty_received = strtol(check_val,NULL,10); //turn duty cycle into a base 10 value
								if ((duty_received >= 0) && (duty_received <= 255)){
									//if true, duty request value is valid
									duty_request = duty_received;
								}
							}
							else{
								duty_request = 0; //stop the motor
							}
						}
					}
				}
				//we have processed everything
				//free memory, and continue to next iteration
				data_received = 0;
				transmit_data = 1;
				//DONT FORGET TO FREE MEMORY
				free(str_buffer);
				free(splitstrings);
			}
			
			if (transmit_data){
				//disable receiver while transmitting to avoid echo-back
				UCSRB &= ~(1<<RXEN);
				char transmit_buffer[150];
				sprintf(transmit_buffer, "{\n%3s\"3\":\n%3s{\n%7s\"mfc\":\n%7s{\n%11s\"req\": \"%3d\",\n%11s\"cur\": \"%3d\"\n%7s},\n%7s\"ver\": \"1.0.0\",\n%7s\"param\":\n%7s{\n%11s\"pwr\":  \"%d.%dW\",\n%11s\"freq\": \"%d.%dHz\",\n%11s\"curr\": \"%dmA\",\n%11s\"volt\": \"%d.%dV\",\n%7s}\n%3s}\n}", "", "", "", "", "", duty_request, "", OCR2, "", "", "", "", "", powerleft, powerright, "", freqleft, freqright, "", current, "", voltleft, voltright, "", "");
				//TODO: add code to detect and report errors/error messages
				//TODO: also add code to re-transmit errors if detected
				uart_transmit_string(transmit_buffer);
				//wait for last transmission to fully complete
				while (UDREMPTY == 0);
				//reenable receiver after transmission is complete
				UCSRB |= (1<<RXEN);
				transmit_data = 0;
			}
		}
    }
}

