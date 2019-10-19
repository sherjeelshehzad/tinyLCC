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
#define NUMSAMPLESI 50
#define LHSVOLTAGECHANNEL 0
#define RHSVOLTAGECHANNEL 1
#define ISHIFTEDCHANNEL 2
#define HALLSENSOR1CHANNEL 3
#define HALLSENSOR2CHANNEL 4

#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/iom8.h>
#include <avr/interrupt.h>
#include <util/delay.h> //header for delay function
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "uart.h"
#include "interrupts.h"
#include "adc.h"

//global variable section
//left = left of the decimal point
//right = right of the decimal point
unsigned int reading = 0;
unsigned int duty_request; //the currently requested duty cycle value
unsigned int transmit_data = 0; //indicate to the UART to transmit a json packet
unsigned int clear_errors = 0; //indicate to the LCC to clear the errors
unsigned int error = 0; //flag that indicates if we have an error
unsigned int stalled = 0; //flag that indicates if the motor is stalled
unsigned int collision = 0; //flag that indicates if there has been a head collision
volatile unsigned int powerleft = 1;
volatile unsigned int powerright = 38;
unsigned int freqleft = 12;
unsigned int freqright = 5;
unsigned int current = 1200;
unsigned int voltleft = 12;
unsigned int voltright = 21;
unsigned int resonant_done = 1;
unsigned int voltagereading = 0;
unsigned int currentreading = 0;
unsigned int backemfreading[3];
volatile unsigned int shortcircuit = 0;
volatile uint64_t backemftime = 20000; //20ms for T/4 
volatile uint64_t backemffreq = 50000; //50Hz for T/4 (by default)
volatile unsigned int numcycles = 0;
unsigned int backemf_i = 0;
unsigned int backemf_i_mod = 0;
uint32_t currentarray[NUMSAMPLESI];
uint32_t powerarray[NUMSAMPLESI];
uint32_t backemfarray[NUMSAMPLESI];
uint64_t currentvalue = 0;
uint64_t powervalue = 0;

int main(void)
{
	//char transmit_buffer[255];
	//sprintf(transmit_buffer, "{\n\"3\":\n{\n\"mfc\":{\"req\":\"\",\"cur\":\"\"\},\n\"ver\": \"001.002.003\",\n\"param\":{\"pwr\":\".W\",\"freq\":\".Hz\",\"curr\":\"mA\",\"volt\": \".V\"},\n\"clr\":\"ew\",\n\"ew\":[\"cmprStalled\",\"pistonCollision\"]\n}\n}");
	//sprintf(transmit_buffer, "{\n%3s\"3\":\n%3s{\n%7s\"mfc\":\n%7s{\n%11s\"req\": \"%3s\",\n%11s\"cur\": \"%3s\"\n%7s},\n%7s\"ver\": \"1.0.0\",\n%7s\"param\":\n%7s{\n%11s\"pwr\":  \"%s.%sW\",\n%11s\"freq\": \"%s.%sHz\",\n%11s\"curr\": \"%smA\",\n%11s\"volt\": \"%s.%sV\",\n%7s}\n%3s}\n}", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "");
	//eeprom_write_block(transmit_buffer,0,strlen(transmit_buffer));	
	//eeprom_write_byte(0,'0');
	
	t2 = 0; //flag for if T/2 time crossing is next to be handled
	stop_counter = 0; //counter to stop PWM every STOPCYCLE/2 number of cycles
	data_received = 0; //set data received flag to 0 initially
	timer0_ovf_count = 0;//set timer0 overflow count to 0 initially
	current_i = 0; //initialise current reading index
	backemfreadingindex = 0; //initialise backemf reading index
	readadc = 0;
	readadcv = 0;
	readadci = 0;
	numcycles = 0;
	voltagereference = 5000;
	//set PWM ports as output
	DDRB |= ((1<<PB2)|(1<<PB1)|(1<<PB0));
	DDRD |= ((1<<PD7)|(1<<PD5));
	
	//set normal timer mode
	TCCR2 &= ~(1<<WGM20);
	TCCR2 &= ~(1<<WGM21);
	
	//disconnect OC2 port
	TCCR2 &= ~((1<<COM21) | (1<<COM20));
	
	//set output compare value for timer 2 between 0-255 (OCR2/255 % duty cycle)
	duty_request = 128;
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
	PORTB |= (1<<PB1);
	//turn off right PMOS
	PORTD &= ~(1<<PD7);
	//turn on right NMOS
	PORTB |= (1<<PB0);
	
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
	//enable timer 0 overflow interrupt
	TIMSK |= (1<<TOIE0);
	//mark T PWM pulse to be handled next
	t2 = 0;
	
	//enable and initialise UART
	uart_init();
	//enable and initialise ADC
	adc_init();
	//enable global interrupts
	sei();
    while (1)
    {
		//gradual duty cycle increase logic to couple with resonant frequency detection module
		//this is because changing the PWM duty cycle also changes the resonant frequency
		if (OCR2 < duty_request)
			OCR2 = duty_request + 1;
		else if (OCR2 > duty_request){
			OCR2 = duty_request - 1;
		}
		
		//if data has been received, parse json
		/*if (data_received){
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
				transmit_data = 1;
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
								int duty_received;
								duty_received = (((check_val[0] - 48) * 100) + (check_val[1] - 48) * 10) + (check_val[2] - 48)));
								if (check)
								strtol(check_val,NULL,10); //turn duty cycle into a base 10 value
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
					//TODO: add check for "clr" object
				}
			}
			//we have processed everything
			//free memory, and reset flags
			data_received = 0;
			//DONT FORGET TO FREE MEMORY
			free(str_buffer);
			free(splitstrings);
		}*/
		
		if (data_received == 1){//using fixed-size fixed-order JSON
			if (str_buffer[2] == '3') {
				transmit_data = 1; //id detected, send transmission
				if (str_buffer[7] == 'm') {
					if (str_buffer[8] == 'f') {
						if (str_buffer[9] == 'c') { //if mfc object detected, check request value
							if (str_buffer[14] == 'r') {
								if (str_buffer[15] == 'e') {
									if (str_buffer[16] == 'q') { //valid request detected, check request value
										int reqval = (((str_buffer[20] - 48) * 100) + ((str_buffer[21] - 48) * 10) + (str_buffer[22] - 48));
										if ((reqval >= 0) && (reqval <= 255)){
											duty_request = reqval; //set duty cycle request if valid value detected
										}
										if (str_buffer[27] == 'c') {
											if (str_buffer[28] == 'l') {
												if (str_buffer[29] == 'r') {
													if (str_buffer[33] == 'e') {
														if (str_buffer[34] == 'w') {//if clear error detected, set error clear flag
															//printf("clr requested\n");
															clear_errors = 1;
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
		
		if (transmit_data){
			//disable receiver while transmitting to avoid echo-back
			UCSRB &= ~(1<<RXEN);
			//char transmit_buffer[200];
			//sprintf(transmit_buffer, "{\n%3s\"3\":\n%3s{\n%7s\"mfc\":\n%7s{\n%11s\"req\": \"%3d\",\n%11s\"cur\": \"%3d\"\n%7s},\n%7s\"ver\": \"1.0.0\",\n%7s\"param\":\n%7s{\n%11s\"pwr\":  \"%d.%dW\",\n%11s\"freq\": \"%d.%dHz\",\n%11s\"curr\": \"%dmA\",\n%11s\"volt\": \"%d.%dV\",\n%7s}\n%3s}\n}", "", "", "", "", "", duty_request, "", OCR2, "", "", "", "", "", powerleft, powerright, "", freqleft, freqright, "", current, "", voltleft, voltright, "", "");
			for (int i = 0; i < 260; ++i){
				if (i == 23){
					//req flowrate value
					uart_transmit((duty_request/100) + 48);
					uart_transmit(((duty_request/10) % 10) + 48);
					uart_transmit((duty_request % 10) + 48);
				}
				if (i == 32){
					//current flowrate value
					uart_transmit((OCR2/100) + 48);
					uart_transmit(((OCR2/10) % 10) + 48);
					uart_transmit((OCR2 % 10) + 48);
				}
				if (i == 74){
					//pwr left value
					if ((powerleft / 10) == 0){
						uart_transmit(powerleft + 48);//transmit the number right away
					}
					else{//split it into two digits
						uart_transmit((powerleft/10) + 48);
						uart_transmit((powerleft % 10) + 48);
					}
				}
				if (i == 75){
					//pwr right value
					uart_transmit((powerright/100) + 48);//1st DP
					uart_transmit(((powerright/10) % 10) + 48);//2nd DP
				}
				if (i == 86){
					//freq left value
					uart_transmit((freqleft/10) + 48);
					uart_transmit((freqleft%10) + 48);
				}
				if (i == 87){
					//freq right value
					//uart_transmit((freqright/(pow(10,log(freqright)))) + 48);//only transmit 1st DP
				}
				if (i == 99){
					//current value
					uart_transmit((current/1000) + 48);
					uart_transmit(((current/100) % 10) + 48);
					uart_transmit(((current/10) % 10) + 48);
					uart_transmit((current % 10) + 48);
				}
				if (i == 112){
					//voltage left value
					uart_transmit((voltleft/10) + 48);
					uart_transmit((voltleft % 10) + 48);
				}
				if (i == 113){
					//voltage right value
					uart_transmit((voltright/100) + 48);
					uart_transmit(((voltright/10) % 10) + 48);
					uart_transmit((voltright % 10) + 48);
				}
				if ((i >= 118) && (i <= 168)){//error condition check
					if (error){
						if (i <= 136){
							uart_transmit(eeprom_read_byte((uint8_t*)i));//transmit clr object and error object header
						}
						else{//check to see which errors we have and transmit accordingly
							if (i <= 148){
								if (stalled){
									uart_transmit(eeprom_read_byte((uint8_t*)i));//transmit cmprstalled
								}
								else{
									uart_transmit(' '); //transmit whitespace
								}
							}
							else if (i == 149){
								if (stalled && collision){
									uart_transmit(eeprom_read_byte((uint8_t*)i));//transmit comma if both errors present
								}
							}
							else if (i <= 166){
								if (collision){
									uart_transmit(eeprom_read_byte((uint8_t*)i));//transmit pistoncollision
								}
								else{
									uart_transmit(' '); //transmit whitespace
								}
							}
							else{
								uart_transmit(eeprom_read_byte((uint8_t*)i));//transmit the closing brace and newline
							}
						}
					}
					else{
						uart_transmit(' '); //transmit whitespace
					}
				}
				else{//if no error condition, transmit json as usual from eeprom
					uart_transmit(eeprom_read_byte((uint8_t*)i));//we have stored the formatted json string in the eeprom to avoid PROGMEM overfill issues
				}
			}
			//TODO: add code to detect and report errors/error messages
			//TODO: also add code to re-transmit errors if detected
			//uart_transmit_string(transmit_buffer);
			//wait for last transmission to fully complete
			while (UDREMPTY == 0);
			//reenable receiver after transmission is complete
			UCSRB |= (1<<RXEN);
			transmit_data = 0;
		}
		//uart_transmit(48);
		//if adc has to be read
		if (numcycles >= 50){
			if (readadc){
				//TODO: fix the order of these if-conditions to properly read ADC at the correct intervals
				//if (!backemffound){
					if (readadcmotorleft){
						//turn off timer 0
						TCCR0 &= ~((1<<CS02)|(1<<CS01)|(1<<CS00));
						//store time value for current sample
						backemftime = (TCNT0 + (timer0_ovf_count*256)); //store time value in 16 microsecond ticks
						//turn on timer 0 with prescaler /256 to keep measuring time
						TCCR0 |= ((1<<CS02));
						TCCR0 &= ~((1<<CS01)|(1<<CS00));
						
						reading = adc_convert((adc_read(LHSVOLTAGECHANNEL)));
						if (reading < 3000){
							if (backemfreadingindex >= 3){
								backemfreadingindex = 2;
								backemfreading[0] = backemfreading[1];
								backemfreading[1] = backemfreading[2];
								backemfreading[2] = 0;
							}
							if ((backemf_i_mod % 2) == 0){
								if (backemf_i != NUMSAMPLESI){
									backemfarray[backemf_i] = reading;
									backemf_i++;
								}
								if (backemf_i_mod > 10){
									backemfreading[backemfreadingindex] = reading;
									if (backemfreadingindex < 3) ++backemfreadingindex;
								}
							}
							backemf_i_mod++;
						}
						//if we have gathered at least 3 samples so far, start testing for maximum point reachage
						if (backemfreadingindex == 3){
							if (((backemfreading[2] - backemfreading[1]) < 5) && ((backemfreading[1] - backemfreading[0]) < 5)){//hysteresis of 50mV
								//we have found max, reset timer 0 count and overflow counter, and calculate frequency using stored time
								backemffound = 1;
								//backemf_i = 0;
								backemf_i_mod = 0;
								//char buf[10];
								//sprintf(buf,"%d",backemftime);
								//uart_transmit_string(buf);
								TCCR0 &= ~((1<<CS02)|(1<<CS01)|(1<<CS00));
								TCNT0 = 0;
								timer0_ovf_count = 0;
								backemffreq = 1000000000/backemftime; //find backemf frequency in millihertz
								readadcmotorleft = 0;//we have successfully found back-emf, no need to keep reading
							}
						}
						
						//left side motor back emf is ready to be read, start timer and keep reading (and storing current and previous timer value each time) until max value stabilises
						//possibly enable ADC interrupt and write a separate function to do non-blocking ADC converts and timer stores to not block other code
						//once max has stabilised and corresponding time has been obtained, calculate resonant frequency and adjust duty cycle on next while () iteration (if required) and do resonant_done = 1;
					}
					else if (readadcmotorright){
						//turn off timer 0
						TCCR0 &= ~((1<<CS02)|(1<<CS01)|(1<<CS00));
						//store time value for current sample
						backemftime = (TCNT0 + (timer0_ovf_count*256)); //store time value in 16 microseconds
						//turn on timer 0 with prescaler /256 to keep measuring time
						TCCR0 |= ((1<<CS02));
						TCCR0 &= ~((1<<CS01)|(1<<CS00));
						//read back emf voltage
						reading = adc_convert((adc_read(RHSVOLTAGECHANNEL)));
						if (reading < 3000){
							if (backemfreadingindex >= 3){
								backemfreadingindex = 2;
								backemfreading[0] = backemfreading[1];
								backemfreading[1] = backemfreading[2];
								backemfreading[2] = 0;
							}
							if ((backemf_i_mod % 2) == 0){
								if (backemf_i != NUMSAMPLESI){
									backemfarray[backemf_i] = reading;
									backemf_i++;
								}
								if (backemf_i_mod > 10){
									backemfreading[backemfreadingindex] = reading;
									if (backemfreadingindex < 3) ++backemfreadingindex;
								}
							}
							backemf_i_mod++;
						}
						
						//if we have gathered at least 3 samples so far, start testing for maximum point reachage
						if (backemfreadingindex == 3){
							if (((backemfreading[2] - backemfreading[1]) < 5) && ((backemfreading[1] - backemfreading[0]) < 5)){//hysteresis of 50mV
								//we have found max, reset timer 0 count and overflow counter, and calculate frequency using stored time
								backemffound = 1;
								//backemf_i = 0;
								backemf_i_mod = 0;
								//char buf[10];
								//sprintf(buf,"%d",backemftime);
								//uart_transmit_string(buf);
								TCCR0 &= ~((1<<CS02)|(1<<CS01)|(1<<CS00));
								TCNT0 = 0;
								timer0_ovf_count = 0;
								backemffreq = 1000000000/backemftime; //find backemf frequency in millihertz
								readadcmotorright = 0;//we have successfully found back-emf, no need to keep reading
							}
						}
						//left side motor back emf is ready to be read, start timer and keep reading (and storing current and previous timer value each time) until max value stabilises
						//possibly enable ADC interrupt and write a separate function to do non-blocking ADC converts and timer stores to not block other code
						//once max has stabilised and corresponding time has been obtained, calculate resonant frequency and adjust duty cycle on next while () iteration (if required) and do resonant_done = 1;
					}
			//	}
				//else{//resonant frequency calculation has priority, so do everything else in this else block
				if (readadcv){
					/*if (voltage_left_on){
						//read motor_left for VCC
						reading = adc_convert(adc_read(LHSVOLTAGECHANNEL));
						voltagereading = (reading - 220)*2.6997; //store actual voltage value
					}*/
					if (voltage_right_on){
						//read motor_right for VCC
						reading = adc_convert(adc_read(RHSVOLTAGECHANNEL));
						voltagereading = (reading - 220)*2.6997; //store actual voltage value
					}
					//we have read voltage value (only need to do this once because we are assuming Vcc remains constant between PWM pulses), turn off voltage reading flag
					readadcv = 0;
				}
				else if (readadci){
					if (currentreadingindex != NUMSAMPLESI){
						//uart_transmit(voltage_left_on + 48);
						//uart_transmit(voltage_right_on + 48);
						//uart_transmit('\n');
						if ((voltage_left_on) || (voltage_right_on)){
							//store current ADC samples into array, do NOT reset the index between sampling intervals so we can build up a full array
							//also at the same time, multiply adc current value by the voltage value to get the instantaneous power array
							//store only alternate samples (i % 50 == 0) (so we don't get more than ~50 samples for the whole of T/4)
							reading = adc_convert(adc_read(ISHIFTEDCHANNEL));
							if((current_i % 2) == 0){
								currentarray[currentreadingindex] = reading; //store i_sense_shifted voltage
								//powerarray[currentreadingindex] = currentarray[currentreadingindex] * voltagereading; //store power in uW
								++currentreadingindex;
								if (reading > 3000) {//short circuit condition test
									shortcircuit = 1;
								}
							}
						}
						else if ((!voltage_left_on) && (!voltage_right_on)){
							//uart_transmit('A');
							//if PWM is turned off, current is 0
							//therefore store a 0 in the array (again, do not reset the index yet)
							//but make sure to read still ADC to generate the correct number of samples (because we don't have a spare timer)
							//also at the same time, multiply adc current value by the voltage value to get the instantaneous power array
							//store only alternate samples (every 25th sample using (i % 50 == 0)) (so we don't get more than ~50 samples)
							adc_convert(adc_read(ISHIFTEDCHANNEL));
							if ((current_i % 2) == 0){
								currentarray[currentreadingindex] = 0;
								//powerarray[currentreadingindex] = 0;
								++currentreadingindex;
							}
						}
						++current_i;
					}
				}
				//}
			}
		}
		
		//driving frequency adjustment here
		if (backemffound){
			char buf[10];
			if (TCNT1 < OCR1A){
				//if (((backemftime - OCR1A) < 1000) || (OCR1A - (backemftime) < 1000)){
				backemffound = 0; //reset found flag
				backemf_i = 0;
				backemf_i_mod = 0;
				char buf[10];
				//sprintf(buf,"freq: %u\n",backemffreq * 0.25);
				//uart_transmit_string(buf);
				//backemftime = 1250;
				if ((backemftime > 1100) && (backemftime < 1600)){
					OCR1A = backemftime; //t/4 = backemf/16
					OCR1B = backemftime * 2;//t/2 is always 2 * t/4
				}
				//}
			}
			
			
			sprintf(buf,"natural time: %u\n",backemftime);
			uart_transmit_string(buf);
			sprintf(buf,"driving time: %u\n",OCR1A);
			uart_transmit_string(buf);
			//for (int i = 0; i < backemf_i; i++){
				//sprintf(buf,"%u\n",backemfarray[i]);
				//uart_transmit_string(buf);
			//}
			//backemf_i = 0;
			//backemf_i_mod = 0;
			//for (int i = 0; i < NUMSAMPLESI; i++){
			//	backemfarray[i] = 0;
			//	sprintf(buf,"%u\n",backemfarray[i]);
				//uart_transmit_string(buf);
			//}
			//backemffound = 0;
		}
		
		//once we have done adc readings, do rms/power calcs and store the values in their proper places
		if (currentreadingindex == NUMSAMPLESI){
			//calculate RMS current
			//uart_transmit_string("works\n");
			
			for (int i = 0; i < NUMSAMPLESI; ++i){
				if (currentarray[i] != 0){
					currentarray[i] = (((currentarray[i] - 200) * 0.2445)) * 10; //store actual current value (mA)
					//currentarray[i] = (((currentarray[i] - 220) * 0.2074)) * 10; //store actual current value (mA)
				}
				powerarray[i] = currentarray[i] * voltagereading;
				
				currentarray[i] = square(currentarray[i]); //square
				
				powervalue += powerarray[i]; //summation of power 
				currentvalue += currentarray[i];//summation of current^2
			}
			currentvalue /= NUMSAMPLESI; //mean
			currentvalue = sqrt(currentvalue); //root - gives RMS current in (mA)
			currentvalue /= 2; //since we are operating bidirectional current and only reading the "ON" period of the signals for (T/4 + T/4) = T/2, we need to halve our obtained RMS value
			
			powervalue = currentvalue * voltagereading; //power mean (uW)
			powervalue /= 1000; //gives average power in (mW)
			//sprintf(buf,"current = %u\n",powervalue);
			//uart_transmit_string(buf);
			//put all the found circuit parameters into the proper format for transmission)
			voltleft = voltagereading/1000;
			voltright = voltagereading % 1000;
			
			uint32_t freqfull = backemffreq * 0.25; //total frequency = f(T/4) / 4
			freqleft = freqfull/1000;
			freqright = freqfull % 1000;
			
			current = currentvalue; //current is already in mA
			
			powerleft = powervalue / 1000;
			powerright = powervalue % 1000;
			
			//reset ADC indices
			current_i = 0;
			currentreadingindex = 0;
		}
		//TODO: add clear error logic here
		//TODO: add stall detection here
		//TODO: add short circuit detection here
		
		//voltage attenation = motor_left * 0.3704 + 220mV; //or motor_right * 0.3704 + 220mV;
		//therefore motor_left = (lhs_voltage - 220mV) * 2.6997
		//current amplification = (i_sense_v * 4.8214) + 220mV
		//i_sense_v = i*0.1, so i = i_sense_v * 10 (milliAMPS)
    }
}

