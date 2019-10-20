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

# define wdr()  __asm__ __volatile__ ("wdr" ::: "memory")

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
unsigned int error = 0; //flag that indicates if we have an error
unsigned int stalled = 0; //flag that indicates if the motor is stalled
unsigned int collision = 0; //flag that indicates if there has been a head collision

//set initial values for measured parameters
unsigned int powerleft = 1;
unsigned int powerright = 38;
unsigned int freqleft = 12;
unsigned int freqright = 5;
unsigned int current = 9500;
unsigned int voltleft = 12;
unsigned int voltright = 21;
unsigned int resonant_done = 1;
unsigned int voltagereading = 0;
unsigned int currentreading = 0;
unsigned int backemfreading[3];
volatile unsigned int shortcircuit = 0;
volatile uint64_t backemftime = 20000; //20ms for T/4 
volatile uint64_t backemffreq = 50000; //50Hz for T/4 (by default)
//index variables to count various things
volatile unsigned int numcycles = 0;
unsigned int backemf_i = 0;
unsigned int backemf_i_mod = 0;
unsigned int current_rms_index = 0;
//arrays to hold current and power readings
uint32_t currentarray[NUMSAMPLESI];
uint32_t powerarray[NUMSAMPLESI];

//variables used in current and power calculations
uint32_t currentrmsavg = 0;
uint64_t currentvalue = 0;
uint64_t powervalue = 0;

int main(void)
{
	//the following block of code is used to fill the EEPROM with our fixed-size fixed-order JSON string
	//it was only executed once because the EEPROM is not erased on chip-reprogram
	//it has been left here to show 
	//char transmit_buffer[255];
	//sprintf(transmit_buffer, "{\n\"3\":\n{\n\"mfc\":{\"req\":\"\",\"cur\":\"\"\},\n\"ver\": \"001.002.003\",\n\"param\":{\"pwr\":\".W\",\"freq\":\".Hz\",\"curr\":\"mA\",\"volt\": \".V\"},\n\"clr\":\"ew\",\n\"ew\":[\"cmprStalled\",\"pistonCollision\"]\n}\n}");
	//eeprom_write_block(transmit_buffer,0,strlen(transmit_buffer));	
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
	adcdone = 0;
	transmit_data = 0;
	clear_errors = 0;
	voltagereference = 5000;
	buffer_ptr = 0;
	//set PWM ports as output
	DDRB |= ((1<<PB2)|(1<<PB1)|(1<<PB0));
	DDRD |= ((1<<PD7)|(1<<PD5));
	
	WDTCR |= ((1<<WDP2)|(1<<WDP2)|(1<<WDP2));
	//enable watchdog timer
	WDTCR |= (1<<WDE);
	//set normal timer mode
	TCCR2 &= ~(1<<WGM20);
	TCCR2 &= ~(1<<WGM21);
	
	//disconnect OC2 port
	TCCR2 &= ~((1<<COM21) | (1<<COM20));
	
	//set output compare value for timer 2 between 0-255 (OCR2/255 % duty cycle)
	duty_request = 128;
	OCR2 = duty_request;

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
		wdr(); //reset watchdog timer
		//gradual duty cycle increase logic to couple with resonant frequency detection module
		//this is because changing the PWM duty cycle also changes the resonant frequency
		if (duty_request == 0){
			OCR2 = 0;
		}
		else if (duty_request == 255){
			OCR2 = 255;
		}
		else{
			if (OCR2 < duty_request){
				OCR2 = OCR2 + 1;
			}
			else if (OCR2 > duty_request){
				OCR2 = OCR2 - 1;
			}
		}
		

		if (transmit_data){//transmitting fixed-size fixed order JSON
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
					uart_transmit((freqright/1000) + 48);//1st DP
					uart_transmit(((freqright/100) % 10) + 48);//2nd DP
					uart_transmit((freqright % 10) + 48);//3nd DP
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
				wdr();//reset watchdog timer
			}
			uart_transmit('\n');//transmit a newline at the end of the transmission
			//wait for last transmission to fully complete
			while (TXCOMP == 0);
			transmit_data = 0;
			mfcinvalid = 0;
			clrinvalid = 0;
		}
		//uart_transmit(48);
		//if adc has to be read
		if (numcycles >= 50){
			if (readadc){
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
									backemf_i++;
								}
								if (backemf_i_mod > 30){
									backemfreading[backemfreadingindex] = reading;
									if (backemfreadingindex < 3) ++backemfreadingindex;
								}
							}
							backemf_i_mod++;
						}
						//if we have gathered at least 3 samples so far, start testing for maximum point reachage
						if (backemfreadingindex == 3){
							if (((backemfreading[2] - backemfreading[1]) < 10) && ((backemfreading[1] - backemfreading[0]) < 10)){//hysteresis of 50mV
								//we have found max, reset timer 0 count and overflow counter, and calculate frequency using stored time
								backemffound = 1;
								backemf_i_mod = 0;
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
									//backemfarray[backemf_i] = reading;
									backemf_i++;
								}
								if (backemf_i_mod > 30){
									backemfreading[backemfreadingindex] = reading;
									if (backemfreadingindex < 3) ++backemfreadingindex;
								}
							}
							backemf_i_mod++;
						}
						
						//if we have gathered at least 3 samples so far, start testing for maximum point reachage
						if (backemfreadingindex == 3){
							if (((backemfreading[2] - backemfreading[1]) < 10) && ((backemfreading[1] - backemfreading[0]) < 10)){//hysteresis of 50mV
								//we have found max, reset timer 0 count and overflow counter, and calculate frequency using stored time
								backemffound = 1;
								//backemf_i = 0;
								backemf_i_mod = 0;
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
				if (readadcv){
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
			}
		}
		
		//driving frequency adjustment here
		if (backemffound){
			char buf[10];
			if (TCNT1 < OCR1A){
				backemf_i_mod = 0;
				backemf_i = 0;
				if ((backemfreading[2] < 260) && (backemfreading[1] < 260) && (backemfreading[0] < 260)){ //check for either stalling or head collision if our back emf is too low
					error = 1;
				}
				backemfreading[0] = 0;
				backemfreading[1] = 0;
				backemfreading[2] = 0;
			}
			if ((backemftime > 1150) && (backemftime < 1500)){//ignore garbage/incorrect resonant frequency values
				OCR1A = backemftime; //t/4 = backemf/16
				OCR1B = backemftime * 2;//t/2 is always 2 * t/4
				validbackemffound = 1;
			}
		}
		
		//once we have done adc readings, do rms/power calcs and store the values in their proper places
		if (currentreadingindex == NUMSAMPLESI){
			//calculate RMS current
			for (int i = 0; i < NUMSAMPLESI; ++i){
				if (currentarray[i] != 0){
					currentarray[i] = (((currentarray[i] - 200) * 0.2345)) * 10; //store actual current value (mA)
				}
				powerarray[i] = currentarray[i] * voltagereading;
				
				currentarray[i] = square(currentarray[i]); //square
				
				powervalue += powerarray[i]; //summation of power 
				currentvalue += currentarray[i];//summation of current^2
			}
			currentvalue /= NUMSAMPLESI; //mean
			currentvalue = sqrt(currentvalue); //root - gives RMS current in (mA)
			currentvalue /= 2; //since we are operating bidirectional current and only reading the "ON" period of the signals for (T/4 + T/4) = T/2, we need to halve our obtained RMS value
			char buf[10];
			
			powervalue = currentvalue * voltagereading; //power mean (uW)
			powervalue /= 1000; //gives average power in (mW)
			current = currentvalue; //current is already in mA
			//put all the found circuit parameters into the proper format for transmission)
			voltleft = voltagereading/1000;
			voltright = voltagereading % 1000;
			
			uint32_t freqfull = (backemffreq * 16)/1000; //total frequency = f(T/4) / 4
			freqfull = freqfull/4;
			freqleft = freqfull/1000;
			freqright = freqfull % 1000;
			
			powerleft = powervalue / 1000;
			powerright = powervalue % 1000;
			
			//reset ADC indices
			current_i = 0;
			currentreadingindex = 0;
			//we have read the adc
			adcdone = 1;
			currentvalue = 0;
			powervalue = 0;
			//voltagereading = 0;
		}
		
		//clear error logic
		if (clear_errors){
			error = 0;
			stalled = 0;
			collision = 0;
			clear_errors = 0;
		}
		
		//error detection here
		if (current != 9500){//if we have actually read a current value
			if (error){
				if (adcdone){
					if ((((uint32_t)current*1000)/OCR2) > ((uint32_t)((3*OCR2) + 4600))){//check to see if we are within the statistically allowed range of Irms for our duty cycle value
						stalled = 1;
						transmit_data = 1;
					}
					//the following block of code checks for head-collision conditions
					//however, during testing we discovered too many false positives over all duty cycles
					//this is likely due to micro-adjustments made in the resonant frequency adjustment module
					//we have left this code here as a commented block so it may be marked, but it has been disabled for the implementation
					/*else{
						collision = 1;
						transmit_data = 1;
					}*/
				}
			}
		}
		
		if (shortcircuit){
			cli(); //disable global interrupts
			//this will also disable the receiver, which will not work while we have a short-circuit condition to prevent damage to external circuitry
			//turn off PWM timer
			TCCR2 &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
			//turn off timer1
			TCCR1B &= ~((1<<CS12) | (1<<CS11) | (1<<CS10));
			//turn off timer0 (back emf measurement timer)
			TCCR0 &= ~((1<<CS02)|(1<<CS01)|(1<<CS00));
			//reset all timer counts
			TCNT0 = 0;
			TCNT1 = 0;
			TCNT2 = 0;
			//turn off PMOSes, turn on NMOSes so motor is disconnected
			//turn off left PMOS
			PORTB &= ~(1<<PB2);
			//turn on left NMOS
			PORTB |= (1<<PB1);
			//turn off right PMOS
			PORTD &= ~(1<<PD7);
			//turn on right NMOS
			PORTB |= (1<<PB0);
			
			//busy-wait 5 seconds, turn on one MOSFET pair, and check adc
			_delay_ms(5000);
			//left to right current
			//turn on left PMOS
			PORTB |= (1<<PB2);
			//turn off left NMOS
			PORTB |= (1<<PB1);
			//turn off right PMOS
			PORTD &= ~(1<<PD7);
			//turn on right NMOS
			PORTB &= ~(1<<PB0);
			
			reading = adc_convert(adc_read(ISHIFTEDCHANNEL));
			if (reading < 3000){
				//short circuit no longer exists, reset status
				shortcircuit = 0;
				//turn on main timer and set prescaler to /256
				TCCR1B |= (1<<CS12);
				TCCR1B &= ~((1<<CS11) | (1<<CS10));
				sei();//reenable global interrupts so timers and UART can begin working again
			}
			else{
				//short circuit has not been resolved, disconnect the motor again
				//turn off PMOSes, turn on NMOSes
				//turn off left PMOS
				PORTB &= ~(1<<PB2);
				//turn on left NMOS
				PORTB |= (1<<PB1);
				//turn off right PMOS
				PORTD &= ~(1<<PD7);
				//turn on right NMOS
				PORTB |= (1<<PB0);
			}
		}
    }
}

