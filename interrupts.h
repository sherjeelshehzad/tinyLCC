/*
 * interrupts.h
 *
 * Created: 10/10/2019 6:34:31 PM
 *  Author: sshe325
 */ 


#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#include "uart.h"
#include "adc.h"
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/iom8.h>
#include <avr/interrupt.h>

#define STOPCYCLE 100
# define wdr()  __asm__ __volatile__ ("wdr" ::: "memory")

enum RXSTATE {ID,MFC,REQ,CLR};
enum MFCSTATE {M,F,C,R,E,Q};
enum REQSTATE {req1,req2,req3};
enum CLRSTATE {clrC,L,clrR,clrE,W,br1,br2};

ISR(TIMER1_COMPA_vect); //interrupt to handle T/4 or 3T/4 time crossing
ISR(TIMER1_COMPB_vect); //interrupt to handle T or T/2 time crossing
ISR(TIMER2_COMP_vect); //PWM duty cycle expired, shut off signal
ISR(TIMER2_OVF_vect); //PWM restarted, change switches according to current current direction
ISR(USART_RXC_vect); //receiver interrupt

volatile unsigned int t2; //flag to check if T/2 is next time crossing to be handled
volatile unsigned int stop_counter; //flag to count cycles and see if we are going to stop the motor to measure backemf
volatile unsigned int data_received; //flag to indicate to main that data has been recieved

volatile unsigned int voltage_left_on; //flag to indicate if the left side of the motor is currently in the PWM ON state
volatile unsigned int voltage_right_on; //flag to indicate if the right side of the motor is currently in the PWM ON state
volatile uint32_t timer0_ovf_count; //keeps track of how many times timer0 has overflown
volatile unsigned int current_i; //current adc reading index
volatile unsigned int currentreadingindex; //current adc array storage index
volatile unsigned int backemfreadingindex;  //backemf adc reading index
volatile unsigned int backemffound; //flag to indicate if backemf frequency was found last time
volatile unsigned int validbackemffound; //flag to indicate if the backemf frequency found was valid
volatile uint64_t backemffreq; //back emf frequency
volatile uint64_t backemftime; //back emf time 

//adc read flags
volatile unsigned int readadcv;
volatile unsigned int readadci;
volatile unsigned int readadc;
volatile unsigned int readadcmotorleft;
volatile unsigned int readadcmotorright;
volatile unsigned int adcdone;

volatile unsigned int numcycles; //count how many cycles we have had so far

volatile char str_buffer[50]; //string buffer to receive data
volatile int buffer_ptr; //pointer to current position of buffer
char uart_char; //char to store UART data
volatile unsigned int transmit_data; //transmission flag if id detected
volatile unsigned int idvalid; //transmission flag if id detected
volatile unsigned int mfcinvalid;
volatile unsigned int clrinvalid;
volatile unsigned int duty_request; //the currently requested duty cycle value
volatile unsigned int clear_errors; //flag to indicate to the LCC to clear the errors
volatile unsigned int reqval; //new (unconfirmed) value of duty cycle
volatile unsigned char received_char;
volatile enum RXSTATE state;
volatile enum MFCSTATE mfcstate;
volatile enum CLRSTATE clrstate;
volatile enum REQSTATE reqstate;

#endif /* INTERRUPTS_H_ */