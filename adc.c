/*
 * adc.c
 *
 * Created: 14/10/2019 12:28:56 PM
 *  Author: sshe325
 */ 

#include "adc.h"

#define ADCSTATUS (ADCSRA & (1<<ADIF))

uint16_t adc_read(uint8_t channel){
	ADMUX &= 0xF0; //clear previous setting
	ADMUX |= channel; //add channel setting
	
	ADCSRA |= (1<<ADSC); //start conversion
	
	while (ADCSTATUS == 0){ //block until conversion finishes
	}
	return ADC; //return value in mV  
}

uint32_t adc_convert(uint16_t rawvalue){
	return (uint32_t)((uint32_t)rawvalue*voltagereference/1024);
}

static void update_vref(){
	ADMUX |= 14; //set channel to bandgap reference
	ADCSRA |= (1<<ADSC); //start conversion
	while (ADCSTATUS == 0){ //block until conversion finishes
	}
	uint16_t bgrawvalue = ADC; //store ADC value for bandgap (1.30V)
	voltagereference = (uint32_t)((uint32_t)1300*1024/bgrawvalue); //calculate vref using bandgap
}

void adc_init(){
	ADMUX &= ~(1<<REFS1);
	ADMUX |= (1<<REFS0); //use AVcc reference
	ADCSRA |= (1<<ADEN); //enable ADC
	ADCSRA |= ((1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2)); //use clock divider of /128 for 125khz ADC frequency
	voltagereference = 5000;
	//update_vref();
}