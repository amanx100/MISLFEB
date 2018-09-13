/*
 * miSL_FEB.c
 *
 * Created: 6/22/2018 8:33:32 PM
 * Author : amanu
 */ 

#define printerPrintDisabled (PINB & (1<<PB0))

#define printCommandOutput_Status (PIND & (1<<PD4))
#define printCommandOutput_ON (PORTD |= (1<<PD4))
#define printCommandOutput_OFF (PORTD &= ~(1<<PD4))

#define printSuceessOutput_Status (PIND & (1<<PD5))
#define printSuccessOutput_ON (PORTD |= (1<<PD5))
#define printSuccessOutput_OFF (PORTD &= ~(1<<PD5))

#define visualPulseOutput_TOGGLE (PORTD ^= (1<<PD6))

#define printCommandPulseTimeConst 10000
#define printSuccessOutputPulseTimeConst 11000
#define printSuccessOutputDelayTimeConst 2600

#define F_CPU 12000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

// Global variables
uint8_t iMarkSensor = 0x00;
uint8_t cutterSensor = 0x00;

uint8_t printCommand = 0x00;


// External Interrupt 0 service routine
// To sense the iMark sensor
ISR(INT0_vect)
{
	cutterSensor = 0x00;
	iMarkSensor = 0xFF;
}

// External Interrupt 1 service routine
// To sense cutter sensor
ISR(INT1_vect)
{
	 cutterSensor = 0xFF;
}

// External Interrupt 2 service routine
// To sense print command
ISR(INT2_vect)
{
	 printCommand = 0xFF;
}


int main(void)
{
    // Input/Output Ports initialization
    // Port A initialization
    DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (0<<DDA2) | (0<<DDA1) | (0<<DDA0);
    PORTA=(0<<PA7) | (0<<PA6) | (0<<PA5) | (0<<PA4) | (0<<PA3) | (0<<PA2) | (0<<PA1) | (0<<PA0);

    // Port B initialization
    DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
    PORTB=(0<<PB7) | (0<<PB6) | (0<<PB5) | (0<<PB4) | (0<<PB3) | (0<<PB2) | (0<<PB1) | (0<<PB0);

    // Port C initialization
    DDRC=(0<<DDC7) | (0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
    PORTC=(0<<PC7) | (0<<PC6) | (0<<PC5) | (0<<PC4) | (0<<PC3) | (0<<PC2) | (0<<PC1) | (0<<PC0);

    // Port D initialization
    DDRD=(0<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
    PORTD=(0<<PD7) | (0<<PD6) | (0<<PD5) | (0<<PD4) | (0<<PD3) | (0<<PD2) | (0<<PD1) | (0<<PD0);

	
	// External Interrupts initialization
	// INT0: On
	// INT0 Mode: Rising Edge
	// INT1: On
	// INT1 Mode: Rising Edge
	// INT2: On
	// INT2 Mode: Rising Edge
	GICR|=(1<<INT1) | (1<<INT0) | (1<<INT2);
	MCUCR=(1<<ISC11) | (1<<ISC10) | (1<<ISC01) | (1<<ISC00);
	MCUCSR=(1<<ISC2);
	GIFR=(1<<INTF1) | (1<<INTF0) | (1<<INTF2);
	
	// Global Interrupt Enable
	sei();
	
	// Enabling Watchdog Timer
	wdt_enable(WDTO_250MS);
	
	// Process Execution Variables
	uint16_t printSuccessOutputDelayTime = 0;
	uint16_t visualPulseTime = 0;
	uint16_t printCommandOutputTime = 0;
	
	
    while (1) 
    {
		// To execute visual pulse output
		if (visualPulseTime++ == 0) {
			visualPulseOutput_TOGGLE;
			// Resetting the watchdog timer
			wdt_reset();
		}
		
		// To execute the print signal
		if (iMarkSensor && cutterSensor)
		{
			if (printCommandOutput_Status)
			{
				if (printCommandOutputTime-- == 0)
				{
					printCommandOutput_OFF;
					iMarkSensor = 0x00;
					cutterSensor = 0x00;
				}
			}
			else
			{
				printCommandOutput_ON;
				printCommandOutputTime = printCommandPulseTimeConst;
			}
		}
		
		// to execute the successful print signal
		if (printCommand)
		{
			if (++printSuccessOutputDelayTime > printSuccessOutputDelayTimeConst)
			{
				if (printSuceessOutput_Status)
				{
					if (printSuccessOutputDelayTime > printSuccessOutputPulseTimeConst)
					{
						printSuccessOutput_OFF;
						printSuccessOutputDelayTime = 0;
						printCommand = 0x00;
					}
				}
				else if (printerPrintDisabled)
				{
					printSuccessOutputDelayTime = 0;
					printCommand = 0x00;
				}
				else
				{
					printSuccessOutput_ON;
				}
			}
		}
		_delay_us(1);
    }
}

