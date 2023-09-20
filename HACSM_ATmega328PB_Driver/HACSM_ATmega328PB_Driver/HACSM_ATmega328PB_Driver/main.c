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
#define BAUD 9600									// BAUD defined as 9600.
#define BAUDRATE ((F_CPU / (BAUD * 16UL)) - 1)		// BAUDRATE for UBRR -> 103
#define LEDON PORTB |= (1 << 5)						// Turn-On LED shortcut
#define LEDOFF PORTB &= ~(1 << 5)					// Turn-Off LED shortcut

#include <util/delay.h>

uint8_t ADC_channel_number = 0;
uint16_t ADC_data[4];
float sensorPer_Val = 0;
char returnNewLine[3] = "\r\n";

// Function to set the ADMUX channel --> May be more efficient with contents dispersed throughout the program instead.
void Set_ADMUX_Channel (uint8_t channelNum){
	ADMUX &= 0xF0;																	// Clear the ADMUX bits
	switch (channelNum){
		case 0: ADMUX |= (0 << MUX0) | (0 << MUX1) | (0 << MUX2) | (0 << MUX3);		// Set to channel ADC0
		case 1: ADMUX |= (0 << MUX0) | (0 << MUX1) | (0 << MUX2) | (1 << MUX3);		// Set to channel ADC1
		case 2: ADMUX |= (0 << MUX0) | (0 << MUX1) | (1 << MUX2) | (0 << MUX3);		// Set to channel ADC2
		case 3: ADMUX |= (0 << MUX0) | (0 << MUX1) | (1 << MUX2) | (1 << MUX3);		// Set to channel ADC3
		default: ADMUX |= (1 << MUX0) | (0 << MUX1) | (0 << MUX2) | (0 << MUX3);	// Set to channel ADC0 (Default)
	}
}

void ADC_Init()
{
	DDRC &= ~(1 << 0) | ~(1 << 1) | ~(1 << 2) | ~(1 << 3);	// Set analog input at Port C, Pin PC0, PC1, PC2, and PC3
	ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADATE);		// Enable ADC and Enable ADC Interrupts and Enable AD Auto-Trigger
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// Set Pre-Scaler to 128 ----------------------------> ??? Do we need this?
	ADCSRB |= (1 << ADTS1) | (1 << ADTS0);					// Enable Timer 0 Compare Match A for Auto Trigger --> ??? Do we need this?
	Set_ADMUX_Channel(0);									// Set ADMUX channel to 0
	ADMUX |= (1 << REFS0);									// Set VCC ref to 5V
	ADCSRA |= (1 << ADSC);									// Start conversion
}

// TODO:
// - Select output pin for ESP8266
// - Select output pin for Speaker

// Taken Directly from **page 149** of ATmega328P's manual
void USART_Init(unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	}

// Taken Directly from **page 150** of ATmega328P's manual
void USART_Transmit(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while (!(UCSR0A & (1<<UDRE0)))
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

// Taken Directly from **page 152** of ATmega328P's manual
unsigned char USART_Receive(void)
{
	/* Wait for data to be received */
	while (!(UCSR1A & (1<<RXC0)));
	/* Get and return received data from buffer */
	return UDR1;
}

int main(void)
{
	ADC_Init();
	USART_Init(BAUDRATE);
    sei();
    while (1) 
    {
		
    }
}

// Interrupt for when the ADC finishes converting sensor values
ISR (ADC_vect) {
	
	// Obtain data from ADC channel and place it in the corresponding slot in the data array.
	switch (ADC_channel_number){
		case 0:
			ADC_data[0] = ADC;			// Get data from the ADC at channel 0
			break;
			
		case 1:
			ADC_data[1] = ADC;			// Get data from the ADC at channel 0
			break;
			
		case 2:
			ADC_data[2] = ADC;			// Get data from the ADC at channel 0
			break;
			
		case 3:
			ADC_data[3] = ADC;			// Get data from the ADC at channel 0
			break;
			
		default:
			ADC_data[0] = ADC;			// Get data from the ADC at channel 0
			break;
	}
	
	// Get Sensor Percentage Value
	// We get a 10-bit value from the ADC --> 2^10 = 1024
	// Dividing the data by 1024 gives us the sensor's percentage
	sensorPer_Val = ((ADC_data[ADC_channel_number] / 1024.0) * 100.0);
	
	// Check channel number and either increment +1 or reset to 0.
	if (ADC_channel_number >= 3){
		ADC_channel_number = 0;
	} else {
		ADC_channel_number++;
	}
	
	Set_ADMUX_Channel(ADC_channel_number);		// Set ADC channel to new ADC channel number
	ADCSRA |= (1 << ADSC);						// Start conversion
	
}

// Interrupt for SPI -> May need multiple for Rx/Tx
//ISR(void){
	//
//}