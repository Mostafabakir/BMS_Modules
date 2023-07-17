// EVMS3 CAN Current Sensor
// Open Source version, released under MIT License (see Readme file)
// Last modified by Ian Hooper (ZEVA), August 2021

// Code for ATmega16M1 (also suitable for ATmega32M1, ATmega64m1)
// Fuses: 8Mhz+ external crystal, CKDIV8 off, brownout 4.2V


#define SHUNT_VERSION	1 // 1 or 0, same code works for hall sensor or shunt interface types

#define HALL_MULTIPLIER	1L	// Compared with 300A sensor, so 600A is 2, 1200A is 4
#define DEADBAND        80L // Tolerance for noise/offset around zero, 80L equates to about +/-0.2A with 300a sensor

#define CAN_BAUD_RATE		250 // Code knows how to do 125, 250, 500, 1000kbps
#define USE_29BIT_IDS		1

#define CAN_ID		40

#include <inttypes.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/wdt.h>

#define EEPROM_OFFSET	4
enum { EEPROM_BLANK, EEPROM_CORRUPT, EEPROM_CORRECT };

#define SELECTOR1	(!(PIND & (1<<PD3)))
#define SELECTOR2	(!(PIND & (1<<PD2)))
#define SELECTOR3	(!(PIND & (1<<PD4)))

// Function declarations
long ScaleForShunt(long raw);
void CanTX(long packetID, unsigned char* data, unsigned char length);
void SetupPorts();
char LoadSettingsFromEEPROM();
void SaveSettingsToEEPROM();

uint8_t data[8];

int ticks = 0;

long currentBuffer = 0;
int offset, amplifiedOffset;
char useAmplifiedCurrent = false;

char calibrateCurrentRequested = false;

int buttonDownTimer;

SIGNAL(TIMER0_OVF_vect) // Called at 31372Hz, ticks used for main loop timer
{
	ticks++;
}

SIGNAL(CAN_INT_vect) // Called when a new CAN message is received
{
	int8_t length, savedCANPage;
	uint8_t data[8];
	
	savedCANPage = CANPAGE; // Saves current MOB
	CANPAGE = CANHPMOB & 0xF0; // Selects MOB with highest priority interrupt
	if (CANSTMOB & (1<<RXOK))
	{
		length = CANCDMOB & 0x0F; // Number of bytes to receive is bottom four bits of this reg
		for (int8_t i=0; i<length; i++) data[i] = CANMSG; // This autoincrements when read

		// ID has top 8 bits in IDT1 and bottom 3 bits at TOP of IDT2
		long packetID;
		if (USE_29BIT_IDS)
			packetID = (((long)CANIDT1)<<21) + (((long)CANIDT2)<<13) + (CANIDT3<<5) + (CANIDT4>>3);
		else
			packetID = (CANIDT1<<3) + (CANIDT2>>5);

		// Check packets in here

		if (packetID == 41) calibrateCurrentRequested = true;
		
		CANCDMOB = (1<<CONMOB1) | (8<<DLC0) | ((1<<IDE) * USE_29BIT_IDS); // Enable reception, data length 8
		// Note: The DLC field of CANCDMOB register is updated by the received MOB, and if it differs from above, an error is set
	}
	CANSTMOB = 0x00; // Reset interrupt reason on selected channel
	CANPAGE = savedCANPage;
}

#define ADC_VREF_TYPE (1<<REFS0) // AVCC used as ADC reference voltage, with capacitor on AREF pin
int ReadADC(unsigned char channel)
{
	ADMUX = channel | ADC_VREF_TYPE;
	_delay_us(10);				// Delay needed for the stabilization of the ADC input voltage
	ADCSRA |= (1<<ADSC);				// Start the AD conversion
	while ((ADCSRA & 0x10) == 0);	// Wait for the AD conversion to complete
	ADCSRA |= 0x10;
	return ADCW;
}

int GetAmplifiedCurrent() // ADC 15 is the differential input, has built-in differential amplifier
{
	int sample = ReadADC(15);
	if (sample > 511) sample -= 1024;
	return sample;
}

int GetCurrent() // Used when the value is too big for differential channel (would have saturated)
{
	return (ReadADC(9) - ReadADC(8))*10; // multiplier to get it into same scaling as amplified input
}

void CalibrateZeroPoint()
{
	_delay_ms(200); // Let current settle

	amplifiedOffset = 0;
	offset = 0;
	for (int n=0; n<32; n++)
	{
		_delay_ms(10);
		offset += GetCurrent();
	}
	for (int n=0; n<32; n++)
	{
		_delay_ms(10);
		amplifiedOffset += GetAmplifiedCurrent();
	}

	SaveSettingsToEEPROM(); // This will revert to the defaults
}

int main()
{
	SetupPorts();

	char result = LoadSettingsFromEEPROM();
	if (result == EEPROM_BLANK || result == EEPROM_CORRUPT)
		CalibrateZeroPoint();

	for (int n=0; n<8; n++) data[n] = 0; // Initialise TX buffer to zero

	sei();
	char slowerTicks = 0;
	unsigned int verySlowTicks = 0;
    
	while (1)
	{
		if (ticks > 245) // Ticks happen at 31372Hz so 245 is every 1/128 of a second
		{
			ticks -= 245;

			int current = 0;
			if (useAmplifiedCurrent)
				current = GetAmplifiedCurrent();
			else
				current = GetCurrent();
		
			currentBuffer += current;

			if (current > -400 && current < 400) // Amplified signal safely within ADC limits
				useAmplifiedCurrent = true;
			else
				useAmplifiedCurrent = false;

			slowerTicks++;
		}

		if (slowerTicks >= 32) // 4Hz loop
		{
			slowerTicks -= 32;
			verySlowTicks++;

			if (useAmplifiedCurrent)
				currentBuffer -= amplifiedOffset; // zero offset
			else
				currentBuffer -= offset;

			if (currentBuffer < DEADBAND && currentBuffer > -DEADBAND) // about 0.2A deadband for 300A sensor, scales with size
			{
				PORTB |= (1<<PB1);
				currentBuffer = 0;
			}
			else if (verySlowTicks&1)
				PORTB |= (1<<PB1);
			else
				PORTB &= ~(1<<PB1);

			long multiplier = 1L;
			if (SHUNT_VERSION)
				currentBuffer = ScaleForShunt(currentBuffer);
			else
				multiplier = HALL_MULTIPLIER;
			
			long scaledValue = 8388608L + 10L*currentBuffer*multiplier/4L;

			currentBuffer = 0;

            data[0] = scaledValue>>16;
			data[1] = scaledValue>>8;
			data[2] = scaledValue;
			CanTX(CAN_ID, data, 8);

			if (!(PIND&(1<<PD5))) // Calibration button pressed
				buttonDownTimer++;
			else
				buttonDownTimer = 0;

			if (buttonDownTimer == 4 || calibrateCurrentRequested)
			{
				CalibrateZeroPoint();
				calibrateCurrentRequested = false;
			}
		} // End of 4Hz loop
	}

	return 0; // Never gets here but compiler wants to see it
}

long ScaleForShunt(long raw)
{
	long result = raw;

	// Selector 2 and 1 used for current size.. 00 means 100A, 01 means 200A, 10 means 500A ? 11 means 50A?

	if (!SELECTOR1 && !SELECTOR2) // 100A shunt
		result *= 2L;
	else if (SELECTOR1 && !SELECTOR2) // 200A shunt
		result *= 4L;
	else if (!SELECTOR1 && SELECTOR2) // 500A shunt
		result *= 10L;
	
    if (SELECTOR3) // 50mV shunt instead of 75mV, current will be 1.5x higher
		result = result*3L/2L;

	return result*102L/190L; // was result*10L/19L but needed to be 2% higher
}

void CanTX(long packetID, unsigned char* data, unsigned char length)
{
	CANPAGE = 0x00; // Select MOB0 for transmission
	while (CANEN2 & (1<<ENMOB0)); // Wait for MOB0 to be free
	CANSTMOB = 0x00;

	if (USE_29BIT_IDS) // CAN 2.0b is 29-bit IDs, CANIDT4 has bits 0-4 in top 5 bits, CANID3 has 5-12
	{
		CANIDT1 = packetID>>21;
		CANIDT2 = packetID>>13;
		CANIDT3 = packetID>>5;
		CANIDT4 = (packetID & 0b00011111)<<3;
	}
	else // CAN 2.0a is 11-bit IDs, IDT1 has top 8 bits, IDT2 has bottom three bits BUT at top of byte!
	{
		CANIDT1 = (packetID>>3); // Packet ID
		CANIDT2 = (packetID & 0x07)<<5;
		CANIDT3 = 0x00;
		CANIDT4 = 0x00;
	}

	for (int8_t i=0; i<length; i++) CANMSG = data[i];

	CANCDMOB = (1<<CONMOB0) | (length<<DLC0) | ((1<<IDE)*USE_29BIT_IDS); // Enable transmission, 8-bit data

	while (!(CANSTMOB & (1<<TXOK)) && !(CANSTMOB & (1<<AERR))); // Wait for transmission to finish (via setting of TXOK flag)

	CANCDMOB = 0x00; // Disable transmission
	CANSTMOB = 0x00; // Clear TXOK flag
}

void SetupPorts()
{
	DDRB = (1<<PB1) + (1<<PB0); // PB1 is LED output, PB0 is STB pin on CAN transceiver, enabled by pulling low
	PORTD |= 0b00111100; // Enable pull up on button and three prog port pins (used for shunt size selection)
    
    // Timer0 is used to increment ticks variable, for main loop timing
	TCCR0B = (1<<CS00); // Clock on, 1:1 prescaler
	TIMSK0 = (1<<TOIE0); // Interrupt on overflow (increments ticks)

	// ADC settings
	ADMUX |= ADC_VREF_TYPE;
	ADCSRA |= 0b10000110; // ADEN plus 110 prescaler = /64 for 125kHz ADC clock
	ADCSRB |= (1<<AREFEN);
	AMP1CSR |= (1<<AMP1EN) + (1<<AMP1G1); // 20x gain on differential channel
	
	// CAN init stuff
	CANGCON = (1<<SWRES); // Software reset
	CANTCON = 0; // CAN timing prescaler set to 0
	if (CAN_BAUD_RATE == 1000)
	{
		CANBT1 = 0x00;
		CANBT2 = 0x04;
		CANBT3 = 0x12;
	}
	else
	{
		if (CAN_BAUD_RATE == 500)
			CANBT1 = 0x02;
		else if (CAN_BAUD_RATE == 250)
			CANBT1 = 0x06;
		else
			CANBT1 = 0x0E;
		CANBT2 = 0x04;
		CANBT3 = 0x13; // These set baud rate to 125kb/s (see ATmega16M1 manual, page 203)
	}

	for (int8_t mob=0; mob<6; mob++)
	{
		CANPAGE = (mob<<4); // Select MOB 0-5
		CANCDMOB = 0x00; // Disable MOB
		CANSTMOB = 0x00; // Clear MOB status register
	}

	CANPAGE = (1<<MOBNB0); // Select MOB1
	CANIE2 = (1<<IEMOB1); // Enable interrupts on MOB1 for reception and transmission
	CANGIE = (1<<ENIT) | (1<<ENRX); // Enable interrupts on receive and transmit
	CANIDM1 = 0x00;
	CANIDM2 = 0x00;
	CANIDM3 = 0x00;
	CANIDM4 = 0x00; // CAN ID mask, zero will let all IDs pass
	CANCDMOB = (1<<CONMOB1) | (8<<DLC0) + ((1<<IDE)*USE_29BIT_IDS); // Enable reception, 11-bit IDE, 1-bit data length
	CANGCON |= (1<<ENASTB); // Enable mode. CAN channel enters enable mode after 11 recessive bits have been read
}

char LoadSettingsFromEEPROM()
{
	if (eeprom_read_dword((uint32_t*)(EEPROM_OFFSET + 6)) != 0xC0FFEE) return EEPROM_BLANK;
	
	offset = eeprom_read_word((uint16_t*)(EEPROM_OFFSET));
	amplifiedOffset = eeprom_read_word((uint16_t*)(EEPROM_OFFSET + 2));

	if (offset+amplifiedOffset != eeprom_read_word((uint16_t*)(EEPROM_OFFSET + 4))) return EEPROM_CORRUPT;

	eeprom_write_byte(0, 0); // Park EEPROM pointer

	return EEPROM_CORRECT;
}

void SaveSettingsToEEPROM()
{
	eeprom_write_word((uint16_t*)(EEPROM_OFFSET), offset);
	eeprom_write_word((uint16_t*)(EEPROM_OFFSET+2), amplifiedOffset);
	eeprom_write_word((uint16_t*)(EEPROM_OFFSET + 4), offset+amplifiedOffset); // Add checksum
	eeprom_write_dword((uint32_t*)(EEPROM_OFFSET + 6), 0xC0FFEE); // Add watermark

	eeprom_write_byte(0, 0); // Park EEPROM pointer
}

