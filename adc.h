/*
 * adc.h
 *
 * Created: 14/10/2019 12:29:58 PM
 *  Author: sshe325
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>
#include <avr/iom8.h>
#include <avr/interrupt.h> //library for interrupts

volatile unsigned int voltagereference; //5V reference value in mV

void adc_init();
uint16_t adc_read(uint8_t channel);
uint32_t adc_convert(uint16_t rawvalue);
static void update_vref();//update the 5V voltage reference value

#endif /* ADC_H_ *//