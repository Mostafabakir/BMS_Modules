// EVMS3: Electric Vehicle Management System V3
// Open Source version, released under MIT License (see Readme file)
// Last modified by Ian Hooper (ZEVA), August 2021

// Code for ATmega16M1 (also suitable for ATmega32M1, ATmega64m1)
// Fuses: 8Mhz+ external crystal with 0ms startup time, CKDIV8 off, brownout 4.2V

// Note: Various #defined build options found in Common.h file

#include <inttypes.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdbool.h>
#include <avr/wdt.h>

#include "Common.h"

#define TACHO_200A_OFFSET 0 // Offsets zero point so you can see up to 200A of regen current below

// ADCs
#define HV_POS	3
#define HV_NEG	8
#define TEMP_SENSE	9
#define AUX_VOLTAGE 7

// Inputs
#define KEY_ON				(PINC & (1<<PC0))
#define CHARGE_SENSE		!(PINB & (1<<PB5))
#define PRECHARGE_DONE		(PIND & (1<<PD5))	// This goes HIGH when precharge has completed (or hasn't started)
#define MPI					(PINB & (1<<PB2))

// Outputs
#define	MPO1				(1<<PD4)
#define MPO2				(1<<PD1)
#define MPO_PORT			PORTD
#define CAN_BUS_ENABLE		(1<<PC7)
#define CAN_CHIP_DISABLE	(1<<PB0)
#define OPAMP_5V			(1<<PB1)
#define MAIN_CTR			(1<<PC6)
#define AUX_CTR				(1<<PB3)
#define CHARGE_ENABLE		(1<<PB4)
#define PRECHARGE_ENABLE	(1<<PC1)

// Function declarations
void DoMPO(char which);
void SetState(char newState);
void SetError(char err);
void GoToSleep();
void CalculateNumCells();
void CanTX(long packetID, unsigned char* data, unsigned char length);
void SetupPorts();
char LoadSettingsFromEEPROM();
void SaveSettingsToEEPROM();

// Global variables
short cellVoltages[16][12]; // Buffer for holding last cell voltages
unsigned char bmsTemps[16][2];
volatile unsigned char bmsRequestCounter[16];
short numCells = 0;

uint8_t txData[8]; // Transmission buffer for CAN messages

volatile short ticks = 0;

unsigned short voltage;
long current = 0; // in milliamps
short currentSensorTimeout = 0;

short chargeEndTimer = 0;

unsigned short overcurrentTimer = 0;
unsigned short tachPeriod;

unsigned short soc = 0;

unsigned short voltageBuffer;
unsigned short measuredVoltage;
long sumOfCellVoltages;
short averageCellVoltage;
short midpointCellVoltage;

short isolationBuffer;
short isolation;

unsigned char auxVoltage;

unsigned short evsePowerLimit = 0;

// ADC vals for temps: -40, -20    0   20   40   60   80 100 120 140 160 dec C
short tempData[11] = { 990, 950, 783, 568, 356, 204, 114, 65, 38, 23, 15 };
short tempBuffer;
short temperature;

long charge; // In hundredths of amp seconds
long maxCharge; // Also hundredths of amp seconds

volatile char error = NO_ERROR;

// BMS variables for EV version
char  bmsHighErrorFlag = false;
char  bmsLowErrorFlag = false;
char  bmsUndertempFlag = false;
char  bmsOvertempFlag = false;
short bmsHighErrorTimer = 0;
short bmsLowErrorTimer = 0;
short bmsUndertempTimer = 0;
short bmsOvertempTimer = 0;

// BMS variables for stationary version
char bmsHighEnabled;
char bmsLowEnabled;
enum { UNDER_VOLT, OK_VOLT, OVER_VOLT };
char cellStatus[16][12];

short mainCtrSwTimer = 0;

char prechargeDidStart = 0;

unsigned long errorAckFlags = 0;

char saveToEepromFlag = false;
char setStateRequest = -1;
short gaugeValueRequest;

char configRequested = false;

short canPowerDownTimer = 0;
short lowAuxVoltageTimer = 0;

char slowerTicks = 0; // Used for polling loops of different frequency
char verySlowTicks = 0;

volatile char countingUp;
SIGNAL(TIMER0_OVF_vect) // Called at 15686Hz, used for timing of the main loop
{
	ticks++;
	countingUp = true; // Flag used for phase correct PWM below
}

SIGNAL(TIMER0_COMPB_vect) // Timer 0 used for generating PWM on MPO outputs. Looks a bit weird because we're doing phase correct PWM
{
	if (countingUp)
	{
		if (settings[MPO1_FUNCTION] == MPO_TEMP_GAUGE) MPO_PORT &= ~MPO1;
		if (settings[MPO2_FUNCTION] == MPO_TEMP_GAUGE) MPO_PORT &= ~MPO2;
		countingUp = false;
	}
	else
	{
		if (settings[MPO1_FUNCTION] == MPO_TEMP_GAUGE) MPO_PORT |= MPO1;
		if (settings[MPO2_FUNCTION] == MPO_TEMP_GAUGE) MPO_PORT |= MPO2;
	}
}

SIGNAL(TIMER1_OVF_vect) // Timer 1 used for generating tacho PWM output signal, 50% duty at varying frequency
{
	ICR1 = tachPeriod;
	OCR1A = tachPeriod>>1;
}

SIGNAL(CAN_INT_vect) // Interrupt function for when we've received a CAN message
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

		if (packetID >= BMS_BASE_ID && packetID < BMS_BASE_ID+170) // In range of BMS modules
		{
			int moduleID = (packetID - BMS_BASE_ID)/10;
			int packetType = packetID - BMS_BASE_ID - moduleID*10;
		
			switch (packetType)
			{
				case BMS_REPLY1:
					for (int n=0; n<4; n++)
						cellVoltages[moduleID][n] = (data[n*2]<<8) + data[n*2+1];
					break;

				case BMS_REPLY2:
					for (int n=0; n<4; n++)
						cellVoltages[moduleID][n+4] = (data[n*2]<<8) + data[n*2+1];
					break;

				case BMS_REPLY3:
					for (int n=0; n<4; n++)
						cellVoltages[moduleID][n+8] = (data[n*2]<<8) + data[n*2+1];
					break;

				case BMS_REPLY4:
					bmsTemps[moduleID][0] = data[0];
					bmsTemps[moduleID][1] = data[1];
					bmsRequestCounter[moduleID] = 0;
					break;
			}
		}
		else if (packetID == CORE_ACKNOWLEDGE_ERROR)
		{
			if (data[0] < NUM_ERRORS) errorAckFlags |= (1L<<data[0]);
			if (error == data[0]) error = NO_ERROR;
		}
		else if (packetID == CORE_SET_STATE)
		{
			setStateRequest = data[0];
			gaugeValueRequest = data[1];
		}
		else if (packetID == CORE_RECEIVE_CONFIG1)
		{
			for (int n=0; n<8; n++) settings[n] = Cap(data[n], minimums[n], maximums[n]);
		}
		else if (packetID == CORE_RECEIVE_CONFIG2)
		{
			for (int n=0; n<8; n++) settings[n+8] = Cap(data[n], minimums[n+8], maximums[n+8]);
		}
		else if (packetID == CORE_RECEIVE_CONFIG3)
		{
			for (int n=0; n<8; n++) settings[n+16] = Cap(data[n], minimums[n+16], maximums[n+16]);
		}
		else if (packetID == CORE_RECEIVE_CONFIG4)
		{
			for (int n=0; n<8; n++) settings[n+24] = Cap(data[n], minimums[n+24], maximums[n+24]);
			saveToEepromFlag = true; // Only do this after 4th packet is received
		}
		else if (packetID == CORE_RECEIVE_CELL_NUMS)
		{
			for (int n=0; n<8; n++)
			{
				bmsCellCounts[n*2] = data[n] & 0x0F; // Bottom four bits only
				bmsCellCounts[n*2+1] = data[n]>>4; // Top four bits moved into bottom four bits
			}
		}
		else if (packetID == CORE_RESET_SOC)
		{
			if (length > 0 && data[0] <= 100)
				charge = maxCharge/100L*(long)data[0];
			else
				charge = maxCharge;
		}
		else if (packetID == CAN_CURRENT_SENSOR_ID)
		{
			current = ((long)data[2]) + ((long)data[1]<<8) + ((long)data[0]<<16) - 8388608L;
			//if (current == 0) current = 200; // For Roborigger, wanted 200mA minimum to account for quiescent current
			currentSensorTimeout = 4; // 1 second timeout in 4hz loop
		}
		else if (packetID == CORE_REQUEST_CONFIG)
			configRequested = true;	
		else if (packetID == CAN_EVSE_INTERFACE)
		{
			evsePowerLimit = ((unsigned short)data[0]<<8) + data[1];
		}

		CANCDMOB = (1<<CONMOB1) | (8<<DLC0) | ((1<<IDE)*USE_29BIT_IDS); // Enable reception, data length 8
		// Note: The DLC field of CANCDMOB register is updated by the received MOB, and if it differs from above, an error is set
	}
	CANSTMOB = 0x00; // Reset interrupt reason on selected channel
	CANPAGE = savedCANPage;
}

SIGNAL(INT1_vect)
{
	// Associated with MPI, does nothing but needs to be here, pin change interrupt is used to wake EVMS when sleeping
}

SIGNAL(INT2_vect)
{
	// Associated with charge sense, does nothing but needs to be here, pin change interrupt is used to wake EVMS when sleeping
}

SIGNAL(INT3_vect)
{
	// Associated with key in, does nothing but needs to be here, pin change interrupt is used to wake EVMS when sleeping
}

#define ADC_VREF_TYPE (1<<REFS0) // AVCC ref with cap on AREF pin
int ReadADC(unsigned char channel)
{
	ADMUX = channel | ADC_VREF_TYPE;
	_delay_us(10);                  // Delay needed for the stabilization of the ADC input voltage
	ADCSRA |= (1<<ADSC);			// Start the AD conversion
	while ((ADCSRA & 0x10) == 0);	// Wait for the AD conversion to complete
	ADCSRA |= 0x10;
	return ADCW;
}

int main()
{
	SetupPorts();

	_delay_ms(100); // wait for things to stabilise

	PORTC |= CAN_BUS_ENABLE; // CAN intialization 
	PORTB &= ~CAN_CHIP_DISABLE; // Initially start with CAN bus and chip enabled
	PORTB |= OPAMP_5V;  //

	char result = LoadSettingsFromEEPROM(); //data memory organisation
	if (result == EEPROM_BLANK || result == EEPROM_CORRUPT)
	{
		SetError(CORRUPT_EEPROM_ERROR);
		SaveSettingsToEEPROM(); // This will revert to the defaults
	}

	maxCharge = (long)settings[PACK_CAPACITY]*PACK_CAPACITY_MULTIPLIER*360000L; //calculate max pack charge
	charge = maxCharge / 100L * (long)Cap(eeprom_read_byte((uint8_t*)123), 0, 100);

	sei(); // correction of the charge capacity 
	wdt_enable(WDTO_250MS);
	state = IDLE;
	
	setStateRequest = 0;

	uint8_t bmsRequestNum = 0;
	
    // Init cell status array - applies to stationary version only
    for (int id=0; id<16; id++)
		for (int c=0; c<12; c++)
			cellStatus[id][c] = OK_VOLT;
	
	while (1) // Main loop
	{
		if (saveToEepromFlag) 
		{
			saveToEepromFlag = false;
			SaveSettingsToEEPROM();
		}

		if (configRequested)
		{
			configRequested = false;
			for (int n=0; n<8; n++) txData[n] = settings[n];
			CanTX(CORE_SEND_CONFIG1, txData, 8);
			for (int n=0; n<8; n++) txData[n] = settings[n+8];
			CanTX(CORE_SEND_CONFIG2, txData, 8);
			for (int n=0; n<8; n++) txData[n] = settings[n+16];
			CanTX(CORE_SEND_CONFIG3, txData, 8);
			for (int n=0; n<8; n++) txData[n] = settings[n+24];
			CanTX(CORE_SEND_CONFIG4, txData, 8);
			for (int n=0; n<8; n++) txData[n] = bmsCellCounts[n*2] + bmsCellCounts[n*2+1]*16;
			CanTX(CORE_SEND_CELL_NUMS, txData, 8);
		}

		if (ticks > 490) // Every 1/32 of a second, or roughly 31ms. Do fast/frequent things in here.
		{
			ticks -= 490;

			// Request next BMS cell data, increment through required packets one 32hz loop at a time
			if (numCells > 0)
			{
				uint8_t lastBmsRequestNum = bmsRequestNum;
				do
				{
					bmsRequestNum++;
					if (bmsRequestNum > 15) bmsRequestNum = 0;
					if (bmsRequestNum == lastBmsRequestNum) break; // Full loop without finding more modules
				} while (bmsCellCounts[bmsRequestNum] == 0);

				uint16_t shuntVoltage = 2000+settings[BALANCE_VOLTAGE]*10; // Shunt at set voltage
				if (settings[BALANCE_VOLTAGE] == 251) // Dynamic shunt voltage
					// Oct 2020: Changed from averageCellVoltage to midpointCellVoltage, works better with single low cells
					shuntVoltage = midpointCellVoltage + BALANCE_TOLERANCE; // small margin to avoid shunt oscillation when nearly balanced
				else if (settings[BALANCE_VOLTAGE] == 252) // Balancing off
					shuntVoltage = 0;

				txData[0] = txData[1] = 0;
				if (state == CHARGING || (state == RUNNING && 
					(settings[BALANCE_VOLTAGE] < 251 || settings[STATIONARY_VERSION] == true)))
				{
					txData[0] = shuntVoltage>>8; // Send balance voltage to BMS modules
					txData[1] = shuntVoltage&0xFF;
				}
				
				CanTX(BMS_BASE_ID + bmsRequestNum*10 + BMS_REQUEST_DATA, txData, 2);
				if (bmsRequestCounter[bmsRequestNum] < 100) bmsRequestCounter[bmsRequestNum]++;
			}

            // Use the HV+ / HV- terminals to measure voltage and isolation
            // Voltage will actually come from sum of cell voltages from BMS modules if possible since it's more accurate
			int voltageA = 1023 - ReadADC(HV_POS);
			int voltageB = 1023 - ReadADC(HV_NEG);
			voltageBuffer += (voltageA + voltageB);
			isolationBuffer += (voltageA - voltageB);

			// 8Mhz clock / 256 prescaler = 31250 timer counting rate
			// 1A = 10rpm, 10rpm = 0.1666 pulses per second (for 1ppr)
			// For/per 1A, timer counts should be 31250/0.1666 = 187500
			// BUT, we're in phase correct mode, so it's counting UP and DOWN each period, so need 187500/2 = 93750
			if (TACHO_200A_OFFSET)
				tachPeriod = 93750L / (long)Cap(current/1000L+200L, 5, 1200) / settings[TACHO_PPR];
			else
				tachPeriod = 93750L / (long)Cap(Abs(current/1000L), 5, 1200) / settings[TACHO_PPR];
			// June 2016: Changed minimum from 2 to 5 so it's not too slow, was causing noticeable response delay sometimes

			tempBuffer += ReadADC(TEMP_SENSE);

			slowerTicks++;
			if (ticksInCurrentState < 30000) ticksInCurrentState++;
		}

		if (slowerTicks >= 8) // 4Hz loop
		{
			slowerTicks -= 8;
			verySlowTicks++;

			if ((state == IDLE || state == STOPPED) && canPowerDownTimer < 30000 && settings[CAN_POWER_DOWN_DELAY] != 6)
				canPowerDownTimer++; // (setting of "6" means OFF, i.e never power down)

			if (canPowerDownTimer == settings[CAN_POWER_DOWN_DELAY]*240) // CAN bus stays on for a while after EVMS goes IDLE
				GoToSleep();

			if (currentSensorTimeout > 0)
				currentSensorTimeout--;
			else
				current = 0; // Haven't received current data for over 1 sec - reset current so we don't accumulate error
	
			charge = CapLong(charge - current/40L, 0, maxCharge); // /40L converts milliamps into hundredths of an amp hour per 1/4 second

			isolation = 100-Cap(Abs(isolationBuffer*10)/(voltageBuffer/10), 0, 100); // Scales leakage to 0-100% (100% being dead short to one rail)
			isolationBuffer = 0;

			// Pack voltage measured based on 200K on 2K divider: 5/1023 * 202/2 = 0.4936 ADCs per volt
			// But buffer is 8x oversampled so we also divide by 8 to get volts, but x10 to get tenths of a volt
			// So it's 0.4936 /8 *10 = 0.617, which is very close to v*2/3 - v/20
			measuredVoltage = (voltageBuffer*2/3) - (voltageBuffer/20);
			if (measuredVoltage > 24)
				measuredVoltage += 18; // +1.8V to compensate for unexpected offset		
			else
            {
				measuredVoltage = 0; // Small deadband for under 5V
				isolation = 101; // Above 100 signals to monitor that no isolation measurement is available
			}
			voltageBuffer = 0;

            // Do some calcs with cell voltages from BMS module(s)
			sumOfCellVoltages = 0;
			int minCellVoltage = 5000;
			int maxCellVoltage = 0;
			for (int n=0; n<16; n++)
				for (int c=0; c<bmsCellCounts[n]; c++)
				{
					sumOfCellVoltages += cellVoltages[n][c];
					if (cellVoltages[n][c] < minCellVoltage) minCellVoltage = cellVoltages[n][c]; // Added Oct 2020, for new balance threshold scheme
					if (cellVoltages[n][c] > maxCellVoltage) maxCellVoltage = cellVoltages[n][c];
				}

			averageCellVoltage = 0;
			if (numCells > 0) averageCellVoltage = sumOfCellVoltages/(long)numCells;
			midpointCellVoltage = (minCellVoltage + maxCellVoltage)/2; // Added Oct 2020, for new balance threshold scheme

			sumOfCellVoltages /= 100L*(long)settings[NUM_PARALLEL_STRINGS]; // To tenths of a volt & consider parallel strings

			if (sumOfCellVoltages > 0)
				voltage = sumOfCellVoltages;
			else
				voltage = measuredVoltage;

			if (state != IDLE && state != SETUP) // i.e if precharging, running or charging
			{
				if (Abs(current/1000L) > settings[CURRENT_TRIP]*10 && settings[CURRENT_TRIP] < 121)
					overcurrentTimer++;
				else
					overcurrentTimer = 0;

				if (settings[FULL_VOLTAGE] > 0)
				{
					if (voltage >= settings[FULL_VOLTAGE]*20*PACK_VOLTAGE_MULTIPLIER && chargeEndTimer == 0) chargeEndTimer = 1;

					// Added this noise filter - needs four consecutive samples (1 second) over full voltage or timer will be reset
	                // Stationary version should always reset whenever voltage drops below full voltage - i.e auto resets
					if (voltage < settings[FULL_VOLTAGE]*20*PACK_VOLTAGE_MULTIPLIER
						&& (chargeEndTimer <= 4 || settings[STATIONARY_VERSION])) chargeEndTimer = 0;
				}

				#define CHARGE_END_TIMEOUT	4*3600	// 1 hour
				if (chargeEndTimer > 0 && chargeEndTimer < CHARGE_END_TIMEOUT) chargeEndTimer++;

				if (chargeEndTimer == CHARGE_END_TIMEOUT && (state == CHARGING || settings[STATIONARY_VERSION]))
				{
					charge = maxCharge;
 					if (!settings[STATIONARY_VERSION]) SetState(STOPPED); // Kills charger and aux contactor, though CAN bus stays up for 1 minute
				}

				// Check cell voltages
				int minBms = 1500+settings[BMS_MIN_VOLTAGE]*10;
				int maxBms = 2000+settings[BMS_MAX_VOLTAGE]*10;

				bmsLowErrorFlag = false;
				bmsHighErrorFlag = false;
				bmsUndertempFlag = false;
				bmsOvertempFlag = false;
				for (int id=0; id<16; id++)
                {
					if (bmsCellCounts[id] > 0)
					{
						for (int n=0; n<bmsCellCounts[id]; n++)
						{
							if (settings[STATIONARY_VERSION])
                            {
								// [April 2019] Changed hysteresis to only affect reset threshold, not trip (safer this way)
                                if (cellVoltages[id][n] > maxBms)
                                    cellStatus[id][n] = OVER_VOLT;
                                if (cellVoltages[id][n] < minBms)
                                    cellStatus[id][n] = UNDER_VOLT;
                                if (cellVoltages[id][n] > minBms+settings[BMS_HYSTERESIS]*10 && cellStatus[id][n] == UNDER_VOLT)
                                    cellStatus[id][n] = OK_VOLT;
                                if (cellVoltages[id][n] < maxBms-settings[BMS_HYSTERESIS]*10 && cellStatus[id][n] == OVER_VOLT)
                                    cellStatus[id][n] = OK_VOLT;
                            }
                            else
                            {
                                if (cellVoltages[id][n] > maxBms) bmsHighErrorFlag = true;
                                if (cellVoltages[id][n] < minBms) bmsLowErrorFlag = true;
                            }
						}
						
						for (int n=0; n<2; n++)
						{
							if (bmsTemps[id][n] != 0)
							{
								if (bmsTemps[id][n] < settings[BMS_MIN_TEMP] && settings[BMS_MIN_TEMP] > 0)
									bmsUndertempFlag = true;

								if (bmsTemps[id][n] > settings[BMS_MAX_TEMP] && settings[BMS_MAX_TEMP] < 141)
									bmsOvertempFlag = true;
							}
						}							
					}
				}
                    
                if (settings[STATIONARY_VERSION])
                {
                    bmsHighEnabled = true;
                    bmsLowEnabled = true;
                    for (int id=0; id<16; id++)
                    {
                        if (bmsCellCounts[id] > 0)
                        {
                            for (int n=0; n<bmsCellCounts[id]; n++)
                            {
                                if (cellStatus[id][n] == OVER_VOLT) bmsHighEnabled = false;
                                if (cellStatus[id][n] == UNDER_VOLT) bmsLowEnabled = false;
                            }
                        }
                    }
                }
			}

			short tempADC = tempBuffer>>3;
			tempBuffer = 0;
			temperature = -40; // i.e "not available" if the algorithm below doesn't find it (-20 to 160degC)
			for (int n=0; n<10; n++)
			{
				if (tempADC <= tempData[n] && tempADC > tempData[n+1]) // we're between samples  
				{
					temperature = -40 + n*20 + 20*(tempADC - tempData[n])/(tempData[n+1] - tempData[n]);
					break;
				}
			}

			// Aux voltage, voltage divider: 20K low, 100K high. ADCs/V = 1023 / 5 / 6 = 34.1
			auxVoltage = ReadADC(AUX_VOLTAGE)*10/34; // Tenths of a volt, 3 is for rounding and offset

			if (auxVoltage < (short)settings[MIN_AUX_VOLTAGE]*10)
			{
				if (lowAuxVoltageTimer < 1000) lowAuxVoltageTimer++;
			}
			else
				lowAuxVoltageTimer = 0;

			// Calculate SoC as a percentage
			// ATmega16M1 seems to have problems with dividing longs? If we >>16 then convert to short, 100Ah = 550
			// To avoid overflow, we make maxCharge 4x smaller, and multiply by 25 instead of 100 at the start
			// We also add charge>>24 to add about 0.4% to make it like rounding instead of truncating
			soc = (201L*(charge>>8)/(maxCharge>>8))>>1; // Effectively *100.5 so we get rounding instead of truncating

            // Override gauge outputs if we're editing associated settings, so can see setting on gauge
            if (setStateRequest == CORE_SETUP_EDIT_FUEL_GAUGE)
				OCR0A = gaugeValueRequest*51/20;
			else
				OCR0A = ((short)settings[FUEL_GAUGE_EMPTY] + (short)(settings[FUEL_GAUGE_FULL]-settings[FUEL_GAUGE_EMPTY])*soc/100)*51/20;
			
			if (setStateRequest == CORE_SETUP_EDIT_TEMP_GAUGE)
				OCR0B = gaugeValueRequest*51/20;
			else
				OCR0B = ((short)settings[TEMP_GAUGE_COLD] + (short)(settings[TEMP_GAUGE_HOT]-settings[TEMP_GAUGE_COLD])*temperature/settings[OVER_TEMP])*51/20;

			// Some error detection
			if (settings[STATIONARY_VERSION])
            {
                if (!bmsHighEnabled && state == RUNNING)
                {
                    SetError(BMS_HIGH_WARNING);
                    PORTB &= ~CHARGE_ENABLE;
                }
                else if (bmsHighEnabled && state == RUNNING)
                {
                    if (error == BMS_HIGH_WARNING) error = 0; // This one auto-resets
                    if (state != STOPPED) PORTB |= CHARGE_ENABLE;
                }
                
                if (!bmsLowEnabled && state == RUNNING)
                {
                    SetError(BMS_LOW_WARNING); // Need a second deadband to allow BMS modules to come up
                    PORTC &= ~MAIN_CTR;
                }
                else if (bmsLowEnabled && state == RUNNING)
                {
                    if (error == BMS_LOW_WARNING) error = 0;
                    if (state != STOPPED) PORTC |= MAIN_CTR;
                }
            }
            else
            {
                if (bmsHighErrorFlag && (state == RUNNING || state == CHARGING))
                {
                    if (bmsHighErrorTimer < 4 || state == RUNNING) // 1 sec timeout if charging, always just warning if running
                    {
                        SetError(BMS_HIGH_WARNING);
                        bmsHighErrorTimer++;
                    }
                    else
                    {
                        SetState(STOPPED); // Kill charger (and open aux contactor(s))..
                        SetError(BMS_ENDED_CHARGE_ERROR);
                    }
                }
                else
                {
                    bmsHighErrorTimer = 0;
                    if (error == BMS_HIGH_WARNING) error = 0; // This one auto-resets
                }

                if (bmsLowErrorFlag && (state == RUNNING || state == CHARGING))
                {
                    if (bmsLowErrorTimer < 40) // 10 sec
                    {
                        if (bmsLowErrorTimer > 10) SetError(BMS_LOW_WARNING); // Need a second deadband to allow BMS modules to come up
                        bmsLowErrorTimer++;
                    }
                    else
                    {
                        SetState(STOPPED); // Kill drive
                        SetError(SHUTDOWN_BY_BMS_ERROR);
                    }
                }
                else
                {
                    bmsLowErrorTimer = 0;
                    if (error == BMS_LOW_WARNING) error = 0;
                }

                if (settings[MPI_FUNCTION] == MPI_CTR_AUX_SWITCH && (((PINC & MAIN_CTR) && MPI) || (!(PINC & MAIN_CTR) && !MPI)))
					// Note inverted MPI logic, MPI will be pulled down by aux switch
                {
                    mainCtrSwTimer++;
                    if (mainCtrSwTimer == 4)
                        SetError(CONTACTOR_SW_FAULT);
                }
                else
                    mainCtrSwTimer = 0;
            }

			if (bmsUndertempFlag)
			{
				if (bmsUndertempTimer < 1000) bmsUndertempTimer++;
			}
			else
				bmsUndertempTimer = 0;

			if (bmsOvertempFlag)
			{
				if (bmsOvertempTimer < 1000) bmsOvertempTimer++;
			}
			else
				bmsOvertempTimer = 0;

			if (bmsOvertempTimer > 4 && state != IDLE && state != SETUP && !(errorAckFlags&(1L<<BMS_OVERTEMP)))
			{
				if (settings[MPO1_FUNCTION] != MPO_OVERTEMP_SIGNAL && settings[MPO2_FUNCTION] != MPO_OVERTEMP_SIGNAL
						&& (state == CHARGING || (state == RUNNING && bmsOvertempTimer > 40)))
					SetState(STOPPED); // Only stop if MPO isn't being used for overtemps
				SetError(BMS_OVERTEMP);
			}

			if (bmsUndertempTimer > 4 && state != IDLE && state != SETUP && !(errorAckFlags&(1L<<BMS_UNDERTEMP)))
			{
				if (settings[MPO1_FUNCTION] != MPO_UNDERTEMP_SIGNAL && settings[MPO2_FUNCTION] != MPO_UNDERTEMP_SIGNAL
						&& (state == CHARGING || (state == RUNNING && bmsUndertempTimer > 40)))
					SetState(STOPPED); // Only stop if MPO isn't being used for undertemps
				SetError(BMS_UNDERTEMP);
			}

			if (Abs(current/1000L) > settings[CURRENT_WARNING]*10 && settings[CURRENT_WARNING] < 121
					&& !(errorAckFlags&(1L<<OVERCURRENT_WARNING)))
				SetError(OVERCURRENT_WARNING);
			else if (overcurrentTimer > 4) // kind of 1 second digital fuse
			{
				SetError(OVERCURRENT_SHUTDOWN);
				SetState(STOPPED);
			}
			else if (state == RUNNING && soc <= settings[SOC_WARNING] && !(errorAckFlags&(1L<<LOW_SOC_ERROR)))
				SetError(LOW_SOC_ERROR);
			else if (state != IDLE && state != SETUP && temperature > settings[OVER_TEMP]
				&& settings[OVER_TEMP] != 151 && ticksInCurrentState > 32 && !(errorAckFlags&(1L<<OVERTEMP_ERROR)))
				SetError(OVERTEMP_ERROR);
			else if (isolation < (short)settings[MIN_ISOLATION] && !(errorAckFlags&(1L<<ISOLATION_ERROR))
					&& state != IDLE && state != SETUP && ticksInCurrentState > 32)
				SetError(ISOLATION_ERROR);
			else if (lowAuxVoltageTimer == 20) // after 5 seconds of low aux voltage
				SetError(LOW_12V_ERROR);
			
			if (state != IDLE && state != SETUP) // BMS errors shouldn't be raised in IDLE or SETUP
			{
				for (int n=0; n<16; n++)
				{
					if (bmsCellCounts[n] > 0 && bmsRequestCounter[n] > 4) // Then probably lost comms with a BMS module
					{		
						if (!(errorAckFlags&(1L<<BMS_COMMS_ERROR))) SetError(BMS_COMMS_ERROR);
						SetState(STOPPED);
					}
				}
			}

			if (settings[MPI_FUNCTION] != MPI_HEADLIGHT_SENSE)
				PORTB |= (1<<PB2); // Enable MPI pin pull-up for everything other than headlight sense
			else
				PORTB &= ~(1<<PB2); // Pull up off for headlight sense - uses 200Kohm pull-down on PCB instead

			DoMPO(MPO1_FUNCTION);
			DoMPO(MPO2_FUNCTION);

			// Broadcast status packet (whenever not in deep sleep)

			// Calculate amp-hours left in battery. "charge" is hundredths of amp seconds
			// We want to send tenths of amp hours, so divide by 36000
			unsigned short ampHours = charge / 36000L;

			char headlightsOn = false;
			if (settings[MPI_FUNCTION] == MPI_HEADLIGHT_SENSE && MPI) headlightsOn = true;

			uint8_t data[8];
			data[0] = state + (error<<3);	// Status in bottom 3 bits, error top 5
			data[1] = ampHours>>8;
			data[2] = ampHours&0xFF;
			data[3] = voltage>>8;
			data[4] = voltage&0xFF;
			data[5] = auxVoltage;
			data[6] = isolation + (headlightsOn<<7);
			data[7] = temperature+40;
			CanTX(CORE_BROADCAST_STATUS, data, 8);

            // Prepare and send message(s) to TC Charger(s)
			if (state == CHARGING || (state == RUNNING && settings[STATIONARY_VERSION]))
			{
				// Voltage stored in first byte, plus a 9th bit in top bit of second byte, and sent to charger 0.1V resolution
				int targetVoltage = (short)settings[CHARGER_VOLTAGE]*10 + (short)(settings[CHARGER_CURRENT]&0b10000000)*20;
				// Current stored in bottom 7 bits of second byte
				int currentLimit = (short)(settings[CHARGER_CURRENT]&0b01111111)*10;

				if (settings[MPI_FUNCTION] == MPI_DUAL_CHARGE_RATE && !MPI) // Pull down to activate alt charge settings
				{
					targetVoltage = (short)settings[CHARGER_VOLTAGE2]*10 + (short)(settings[CHARGER_CURRENT2]&0b10000000)*20;
					currentLimit = (short)(settings[CHARGER_CURRENT2]&0b01111111)*10;
				}

				if (evsePowerLimit > 0) // Then we've received info about how much EVSE can supply, make sure charger doesn't exceed
				{
					unsigned short power = evsePowerLimit; // Make copy so we don't modify original
					if (power > 13333) power = 13333; // 13.3kW max (per phase), plenty for J1772 AFAIK
					power -= (power/10); // Take 10% off to allow for charger inefficiency (worst case scenario, chargers usually 95% efficient)
					unsigned short evseCurrentLimit = 0;
					if (voltage > 0) // protect against divide by zero
						evseCurrentLimit = (long)power * 100L / (long)voltage; // Should yield tenths of amps
					if (evseCurrentLimit < currentLimit) currentLimit = evseCurrentLimit;
				}
				
				targetVoltage *= PACK_VOLTAGE_MULTIPLIER;
				
				data[0] = targetVoltage>>8;
				data[1] = targetVoltage%256;

				data[2] = currentLimit>>8;
				data[3] = currentLimit%256;
			
				if (settings[STATIONARY_VERSION])
                	data[4] = !bmsHighEnabled; // Invert because 0 means OK to charge
				else if (bmsHighErrorFlag && state == RUNNING)
					data[4] = 1; // Stop if BMS over voltage error
				else
					data[4] = (state == STOPPED); // 0 for OK to charge, 1 for protection / stop.
			
				data[5] = data[6] = data[7] = 0; // Reserved, zero
	
				CanTX(TC_CHARGER1_RX_ID, data, 8); // Send control packet to TC Charger
				CanTX(TC_CHARGER2_RX_ID, data, 8); // Any harm always sending all three charger control packets?
				CanTX(TC_CHARGER3_RX_ID, data, 8);
			}
		} // End of 4Hz loop

		// State management, i.e switching between Idle, Running, Charging, Setup, etc
		if (settings[STATIONARY_VERSION]) // Only running mode is available, no charging or precharging modes
        {
			if (state == IDLE && KEY_ON && !error) SetState(RUNNING);
            if (state == RUNNING && !KEY_ON) SetState(IDLE);
		}
		else // Normal EV version
		{
            if (state == IDLE && KEY_ON && !error)
            {
                if (settings[ENABLE_PRECHARGE])
                    SetState(PRECHARGING);
                else
                    SetState(RUNNING);
            }

            if (state == PRECHARGING && !KEY_ON) SetState(IDLE);
            
            if (state == PRECHARGING && !prechargeDidStart && !PRECHARGE_DONE) prechargeDidStart = true;

            if (state == PRECHARGING && ((!prechargeDidStart && ticksInCurrentState > 16) // Precharge doesn't start within 0.5s
                || (state == PRECHARGING && ticksInCurrentState > 160))) // Or, 5s maximum precharge time
            {
                SetError(PRECHARGE_FAILED_ERROR);
                SetState(STOPPED);
            }
            
            if (state == PRECHARGING && prechargeDidStart && PRECHARGE_DONE && ticksInCurrentState > 2)
                SetState(RUNNING); // Successful precharge -> switch to Running mode
			
			if (state == RUNNING && !KEY_ON) SetState(IDLE);
			if (state == IDLE && CHARGE_SENSE) SetState(CHARGING);
			if (state == CHARGING && !CHARGE_SENSE) SetState(IDLE);
		}

		if (state == STOPPED && !KEY_ON && !CHARGE_SENSE) SetState(IDLE);

		if (state == IDLE && setStateRequest > 0) SetState(SETUP);
		if (state == SETUP && setStateRequest == 0) SetState(IDLE);

		wdt_reset();
	}

	return 0; // Never gets here but compiler wants to see it
}

void DoMPO(char mpo_num) // Implement behaviour for Multi Purpose Output terminal(s)
{
	unsigned char pin = MPO1;
	if (mpo_num == MPO2_FUNCTION) pin = MPO2;

	switch (settings[mpo_num])
	{
		case MPO_GROUND:
			MPO_PORT &= ~pin;
			break;
		
		// Note: Temp gauge case is handled in interrupt

		case MPO_LOW_SOC_SIGNAL:
			if (state != IDLE && state != SETUP && soc <= settings[SOC_WARNING])
				MPO_PORT |= pin;
			else
				MPO_PORT &= ~pin;
			break;

		case MPO_OVERTEMP_SIGNAL:
			if (state != IDLE && state != SETUP && ticksInCurrentState > 32 &&
					(temperature >= settings[OVER_TEMP] || bmsOvertempFlag))
				MPO_PORT |= pin;
			else
				MPO_PORT &= ~pin;
			break;
		
		case MPO_UNDERTEMP_SIGNAL:
			if (state != IDLE && state != SETUP && bmsUndertempFlag && ticksInCurrentState > 32)
				MPO_PORT |= pin;
			else
				MPO_PORT &= ~pin;
			break;
		
		case MPO_ERROR_BUZZER:
			if (error != 0)
				MPO_PORT |= pin;
			else
				MPO_PORT &= ~pin;
			break;
		
		case MPO_STATUS_LIGHT:
			if (state == RUNNING || (state == CHARGING && (soc == 100 || verySlowTicks&0x2)))
				MPO_PORT |= pin; // Output ON when running or fully charged, flash while charging
			else
				MPO_PORT &= ~pin; // Off otherwise
		break;

		// No default case because don't want to override temp gauge
	}
}

void SetState(char newState)
{
	for (int n=0; n<16; n++) bmsRequestCounter[n] = 0; // Reset all request counters to start with

	if ((state == RUNNING || state == CHARGING) && (newState == IDLE || newState == STOPPED))
		eeprom_write_byte((uint8_t*)123, soc); // Save SoC to EEPROM

	// Enable CAN bus whenever we do a state transition
	PORTB &= ~CAN_CHIP_DISABLE;
	PORTC |= CAN_BUS_ENABLE;
	PORTB |= OPAMP_5V;
	canPowerDownTimer = 0;

	switch (newState)
	{
		default:
		case IDLE:
			PORTC &= ~(MAIN_CTR + PRECHARGE_ENABLE);
			PORTB &= ~(CHARGE_ENABLE + AUX_CTR);
			SetError(NO_ERROR); // Reset any pending errors automatically when changing to IDLE
			break;
		
		case PRECHARGING:
			PORTB |= AUX_CTR;
			PORTC |= PRECHARGE_ENABLE;
			prechargeDidStart = false;
			errorAckFlags = 0;
			break;

		case RUNNING:
			PORTC |= MAIN_CTR;
			PORTB |= AUX_CTR;
			PORTC &= ~PRECHARGE_ENABLE;
			errorAckFlags = 0; // Re-enable all warnings when entering run mode
			break;

		case CHARGING:
			PORTB |= CHARGE_ENABLE + AUX_CTR;
			errorAckFlags = 0; // Re-enable all warnings when entering charge mode
			break;

		case STOPPED:
			PORTC &= ~(MAIN_CTR + PRECHARGE_ENABLE);
			PORTB &= ~(AUX_CTR + CHARGE_ENABLE); // Kill everything - except CAN bus
			break;
	
		case SETUP:
			// Nothing changes, same outputs as Idle mode
			break;
	}

	state = newState;
	ticksInCurrentState = 0;
	chargeEndTimer = 0;
}

void SetError(char err)
{
	error = err;
}

void GoToSleep()
{
    // Before going to sleep, we switch of as many peripheral circuits as we can
    
	PORTC &= ~CAN_BUS_ENABLE;
	PORTB |= CAN_CHIP_DISABLE; // Turn off CAN bus and transceiver
	PORTB &= ~OPAMP_5V; // Disable supply to op amp (used for isolation monitoring circuit)

	TCCR1A = 0;
	TCCR1B = 0;
	PORTD &= ~(1<<PD2); // Turn tach gauge off

	MPO_PORT &= ~(MPO1 + MPO2); // Turn off MPOs

	wdt_disable();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Power down leaves only external INT0-3 and watchdog active
	sleep_enable();
	//sleep_bod_disable();
	sleep_cpu(); // Code execution stops here until external interrupt (level change to key, charge sense or MPI terminals)

	// Wake up from sleep here
	sleep_disable();
	wdt_enable(WDTO_250MS);
    
    // Ree-enable peripheral circuits after waking up
	PORTC |= CAN_BUS_ENABLE;
	PORTB &= ~CAN_CHIP_DISABLE;
	PORTB |= OPAMP_5V;

	// Tach gauge back on
	TCCR1A = (1<<COM1A1) + (1<<WGM11); // Non-inverting
	TCCR1B = (1<<WGM13) + (1<<CS12); // with line above = phase correct, top at ICR1, /256 prescaler

	canPowerDownTimer = 0; // Reset power down timer
}

void CalculateNumCells()
{
	numCells = 0;
	for (int n=0; n<16; n++) numCells += bmsCellCounts[n];
}

// Function to transmit CAN bus message
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

	char timeout = 0;
	while (!(CANSTMOB & (1<<TXOK)) && !(CANSTMOB & (1<<AERR)))
	{	// Wait for transmission to finish (via setting of TXOK flag)
		_delay_us(100);
		if (timeout++ > 100) break;	
	}
	CANCDMOB = 0x00; // Disable transmission
	CANSTMOB = 0x00; // Clear TXOK flag
}


void SetupPorts()
{
    // Direction registers (i.e setting which pins are outputs)
	DDRB = CHARGE_ENABLE + AUX_CTR + CAN_CHIP_DISABLE + OPAMP_5V;
	DDRC = CAN_BUS_ENABLE + MAIN_CTR + PRECHARGE_ENABLE;
	DDRD = MPO1 + MPO2 + (1<<PD2) + (1<<PD3); // Last two are fuel gauge and tach gauge outputs
	
	PORTB = (1<<PB5); // Charge sense pull up
	PORTD = (1<<PD5); // Precharge detect pull up

    // Timer 0 is used for main loop polling timer, and generating PWM for temp gauge
	TCCR0A = (1<<COM0A1) + (1<<WGM00); // Clear OC0A on compare match, phase correct PWM
	TCCR0B = (1<<CS00); // Clock on, 1:1 prescaler
	TIMSK0 = (1<<TOIE0) + (1<<OCIE0B); // Interrupt on overflow and compare match B

	// Timer 1 is used for tacho output
    // Non-inverted, phase correct, top at ICR1, /256 prescaler @ 8Mhz = 31250Hz counting
	TCCR1A = (1<<COM1A1) + (1<<WGM11); // Non-inverting
	TCCR1B = (1<<WGM13) + (1<<CS12); // with line above = phase correct, top at ICR1, /256 prescaler
	TIMSK1 = (1<<TOIE1); // Interrupt on overflow
	ICR1 = 1000;
	OCR1A = 500;

    // Enabling external interrupts
	EICRA = 0b01010100; // 01 for INT1, INT2, INT3 means interrupt on any logical change
	EIMSK = 0b00001110; // Enables INT1,2,3

    // Setting up Analog to Digital Conversion
	ADMUX |= ADC_VREF_TYPE;
	ADCSRA |= 0b10000110; // ADEN plus 110 prescaler = /64 for 125kHz ADC clock
	ADCSRB |= (1<<AREFEN);
		
	// CAN init stuff
	CANGCON = (1<<SWRES); // Software reset
	CANTCON = 0; // CAN timing prescaler set to 0
	
	if (CAN_BAUD == 1000)
		CANBT1 = 0x00;
	else if (CAN_BAUD == 500)
		CANBT1 = 0x02;
	else if (CAN_BAUD == 250)
		CANBT1 = 0x06;
	else
		CANBT1 = 0x0E;
	CANBT2 = 0x04;
	CANBT3 = 0x13;
	if (CAN_BAUD == 1000) CANBT3 = 0x12;

	for (int8_t mob=0; mob<6; mob++)
	{
		CANPAGE = (mob<<4); // Select MOB 0-5
		CANCDMOB = 0x00; // Disable MOB
		CANSTMOB = 0x00; // Clear MOB status register
	}
	
	CANPAGE = (1<<MOBNB0); // Select MOB1
	CANIE2 = (1<<IEMOB1); // Enable interrupts on MOB1 for reception and transmission
	CANGIE = (1<<ENIT) | (1<<ENRX); // Enable interrupts on receive and transmit
	CANIDM1 = CANIDM2 = CANIDM3 = CANIDM4 = 0x00; // CAN ID mask, zero will let all IDs pass
	CANCDMOB = (1<<CONMOB1) + (8<<DLC0) + ((1<<IDE)*USE_29BIT_IDS);
	CANGCON |= (1<<ENASTB); // Enable mode. CAN channel enters enable mode after 11 recessive bits have been read
}

char LoadSettingsFromEEPROM()
{
	unsigned char tempSettings[NUM_SETTINGS];
	unsigned char tempCellCount[16];
	
	if (eeprom_read_dword((uint32_t*)(EEPROM_OFFSET + NUM_SETTINGS + 8 + 2)) != 0xC0FFEE) // Arbitrary watermark
		return EEPROM_BLANK;
	
	unsigned short checksum = 0;
	for (int i=0; i<NUM_SETTINGS; i++)
	{
		tempSettings[i] = eeprom_read_byte((uint8_t*)(EEPROM_OFFSET + i));
		checksum += tempSettings[i];
	}

	for (int i=0; i<8; i++)
	{
		unsigned char byte = eeprom_read_byte((uint8_t*)(EEPROM_OFFSET + NUM_SETTINGS + i));
		tempCellCount[i*2] = byte & 0x0F;
		tempCellCount[i*2+1] = byte>>4;
		checksum += byte;
	}

	if (checksum != eeprom_read_word((uint16_t*)(EEPROM_OFFSET + NUM_SETTINGS + 8))) return EEPROM_CORRUPT;

	// Otherwise, settings are correct and we can copy to the real thing
	for (int i=0; i<NUM_SETTINGS; i++) settings[i] = tempSettings[i];
	for (int i=0; i<16; i++) bmsCellCounts[i] = tempCellCount[i];
	CalculateNumCells();

	EEAR = 0; // Park EEPROM pointer at sacrificial location 0

	return EEPROM_CORRECT;
}

void SaveSettingsToEEPROM()
{
    // Disable watchdog timer and interrupts because writing to EEPROM is so slow
    wdt_disable();
	cli();

	unsigned short checksum = 0;
	for (int i=0; i<NUM_SETTINGS; i++)
	{
		eeprom_write_byte((uint8_t*)(EEPROM_OFFSET + i), settings[i]);
		checksum += settings[i];
	}

	for (int i=0; i<8; i++)
	{
		unsigned char byte = bmsCellCounts[i*2] + bmsCellCounts[i*2+1]*16;
		eeprom_write_byte((uint8_t*)(EEPROM_OFFSET + NUM_SETTINGS + i), byte);
		checksum += byte;
	}

	eeprom_write_word((uint16_t*)(EEPROM_OFFSET + NUM_SETTINGS + 8), checksum); // Add checksum
	eeprom_write_dword((uint32_t*)(EEPROM_OFFSET + NUM_SETTINGS + 8 + 2), 0xC0FFEE); // Add watermark

	sei();
	wdt_enable(WDTO_250MS);

	EEAR = 0; // Park EEPROM pointer at sacrificial location 0

	maxCharge = (long)settings[PACK_CAPACITY]*PACK_CAPACITY_MULTIPLIER*360000L;
	charge = CapLong(charge, 0, maxCharge); // Make sure charge isn't bigger than (possibly new) max charge

	CalculateNumCells();
}
