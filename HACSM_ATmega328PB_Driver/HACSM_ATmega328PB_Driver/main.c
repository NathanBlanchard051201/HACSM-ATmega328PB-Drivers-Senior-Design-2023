/*
 * HACSM_ATmega328PB_Driver.c
 *
 * Created: 9/5/2023 2:35:01 PM
 * Authors: Nathan Blanchard, Abdalla Mohamad, Mark J. Lim
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define F_CPU 16000000UL
#define LEDON PORTB |= (1 << 5)								// Turn-On LED shortcut
#define LEDOFF PORTB &= ~(1 << 5)							// Turn-Off LED shortcut

uint8_t ADC_channel_number = 0;
uint16_t ADC_data[3];

// Function to set the ADMUX channel --> May be more efficient with contents dispersed throughout the program instead.
void Set_ADMUX_Channel (uint8_t channelNum){
	ADMUX &= 0xF0;																	// Clear the ADMUX bits
	switch (channelNum){
		case 0: ADMUX |= (1 << MUX0) | (0 << MUX1) | (0 << MUX2) | (0 << MUX3);		// Set to channel 0
		case 1: ADMUX |= (0 << MUX0) | (1 << MUX1) | (0 << MUX2) | (0 << MUX3);		// Set to channel 1
		case 2: ADMUX |= (0 << MUX0) | (0 << MUX1) | (1 << MUX2) | (0 << MUX3);		// Set to channel 2
		case 3: ADMUX |= (0 << MUX0) | (0 << MUX1) | (0 << MUX2) | (1 << MUX3);		// Set to channel 3
		default: ADMUX |= (1 << MUX0) | (0 << MUX1) | (0 << MUX2) | (0 << MUX3);	// Set to channel 0
	}
}

void ADC_init()
{
	DDRC &= ~(1 << 0) | ~(1 << 1) | ~(1 << 2) | ~(1 << 3);	// Set analog input at Port C, Pin PC0, PC1, PC2, and PC3
	ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADATE);		// Enable ADC and Enable ADC Interrupts and Enable AD Auto-Trigger
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// Set Pre-Scaler to 128 ----------------------------> ??? Do we need this?
	ADCSRB |= (1 << ADTS1) | (1 << ADTS0);					// Enable Timer 0 Compare Match A for Auto Trigger --> ??? Do we need this?
	Set_ADMUX_Channel(0);									// Set ADMUX channel to 0
	ADMUX |= (1 << REFS0);									// Set VCC ref and ADC1
	ADCSRA |= (1 << ADSC);									// Start conversion
}

void SPI_init()
{
	// Select output pin for ESP8266
	// Select output pin for Speaker
}

void SYS_init(){
	
}

float Sensor_Percentage(uint16_t sensorVal){
	// We get a 10-bit value from the ADC --> 2^10 = 1024
	// Dividing the data by 1024 gives us the sensor's percentage
	return (float) (sensorVal / 1024) * 100;
}

int main(void)
{
	ADC_init();
	SPI_init();
	SYS_init();
    sei();
    while (1) 
    {
    }
}

// Interrupt for when the ADC finishes converting sensor values
ISR (ADC_vect) {
	
	switch (ADC_channel_number){
		case 0:
			Set_ADMUX_Channel(0);
			ADC_data[0] = ADC;			// Get data from the ADC at channel 0
			ADC_channel_number = 1;		// Update ADC_channel_number to go to channel 1
			break;
			
		case 1:
			Set_ADMUX_Channel(1);
			ADC_data[1] = ADC;			// Get data from the ADC at channel 0
			ADC_channel_number = 2;		// Update ADC_channel_number to go to channel 1
			break;
			
		case 2:
			Set_ADMUX_Channel(2);
			ADC_data[2] = ADC;			// Get data from the ADC at channel 0
			ADC_channel_number = 3;		// Update ADC_channel_number to go to channel 1
			break;
			
		case 3:
			Set_ADMUX_Channel(3);
			ADC_data[3] = ADC;			// Get data from the ADC at channel 0
			ADC_channel_number = 0;		// Update ADC_channel_number to go to channel 1
			break;
			
		default:
			Set_ADMUX_Channel(0);
			ADC_data[0] = ADC;			// Get data from the ADC at channel 0
			ADC_channel_number = 1;		// Update ADC_channel_number to go to channel 1
			break;
	}
}

// Interrupt for SPI -> May need multiple for Rx/Tx
ISR(){
	
}