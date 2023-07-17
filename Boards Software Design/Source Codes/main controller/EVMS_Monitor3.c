// EVMS Monitor V3
// Open Source version, released under MIT License (see Readme file)
// Last modified by Ian Hooper (ZEVA), August 2021

// Code for AT90CAN64 (also suitable for AT90CAN128)
// Fuses: 16Mhz external crystal, CKDIV8 off, JTAGEN off

// Note: Various #defined build options found in Common.h file


#define DISPLAY_TYPE	ILI9341		// SSD1289 or ILI9325 or ILI9341
#define SHOW_TOUCH_LOCATION	0 // Used for debugging touchscreen - writes touched coords in top left
#define FAKE_EVMS	0 // Use to test things if no EVMS is present
#define MONITOR // Modifies some stuff in the Common.h header

#define MAX_BMS_MODULES	16

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdbool.h>

#include "Common.h"
#include "Touchscreen.h"
#include "config.h"
#include "can_lib.h"

// Colour theme - only partially implemented
#define BGND_COLOUR		BLACK
#define LABEL_COLOUR	BLUE // RED
#define TEXT_COLOUR		WHITE // RED
#define CHARGING_COLOUR	BLUE // RED
#define RUNNING_COLOUR	GREEN // RED
#define SOC_COLOUR		0		// or 0 to use green-to-red dynamic colouring

#define BUZZER	(1<<PB7)
#define BUZZER2	(1<<PB6)

#define CONFIG_LOCK	(PIND & (1<<PD0)) // Config lock normally pulled down (off) by jumper, i.e remove jumper to lock
#define CONFIG_LOCK2 (PINE & (1<<PE5)) // PCB version 3.1 reassigned this - but code should support both

#ifdef NEW_LCD
#define BACKLIGHT		(1<<PF7)
#define BACKLIGHT_PORT	PORTF
#define BACKLIGHT_DDR	DDRF
#else
#define BACKLIGHT		(1<<PD7)
#define BACKLIGHT_PORT	PORTD
#define BACKLIGHT_DDR	DDRD
#endif

// Initialising all the buttons
typedef struct
{
	int x, y, width;
	U16 colour;
	char* text;
	bool isTouched;
} Button;

Button enterSetupButton = { 160, 30, 220, BLUE, "Enter Setup", false };
Button resetSocButton = { 160, 70, 220, GREEN, "Reset SoC", false };
Button zeroCurrentButton = { 160, 110, 220, GREEN, "Zero Current", false };
Button displayOffButton = { 160, 150, 220, GREEN, "Display Off", false };
Button exitOptionsButton = { 160, 190, 220, ORANGE, "Exit Options", false };

Button nextBmsModuleButton = { 260, 200, 100, GREEN, "Next", false };
Button prevBmsModuleButton = { 60, 200, 100, GREEN, "Prev", false };

Button changeSetupPageButtonLeft = { 40, 25, 80, BLUE, "<", false };
Button changeSetupPageButtonRight = { 280, 25, 80, BLUE, ">", false };
Button changeParameterButtonLeft = { 40, 90, 80, BLUE, "<", false };
Button changeParameterButtonRight = { 280, 90, 80, BLUE, ">", false };
Button changeValueButtonLeft = { 40, 155, 80, BLUE, "<", false };
Button changeValueButtonRight = { 280, 155, 80, BLUE, ">", false };
Button exitSetupButton = { 160, 207, 160, ORANGE, "Exit Setup", false };

Button* touchedButton = 0;

short cellVoltages[MAX_BMS_MODULES][12]; // Buffer for holding last cell voltages
U8 bmsTemps[MAX_BMS_MODULES][2];

long current = 0;
int currentSensorTimeout = 0;
char haveReceivedCurrentData = false;

unsigned char mcStatusBytes[8];
short mcCanTimeout = 0;

char haveReceivedEVMSData = 0;
char haveReceivedMCData = 0;

short chargerCommsTimeout[3] = { 0, 0, 0 };

short ticksSincePowerOn = 0;

// Display pages
enum { EVMS_CORE, MOTOR_CONTROLLER, TC_CHARGER, BMS_SUMMARY, BMS12_DETAILS, NUM_KNOWN_DEVICES }; 

// Function declarations
void PrepareCanRX(unsigned char mob);
void ProcessCanRX(unsigned char mob);
void HandleTouchDown();
void HandleTouchUp();
void DoSetupButtons(char isKeyRepeat);
void TransmitSettings();
void TransmitGaugeState();
inline void Beep(short ticks);
inline void UpdateBuzzer();
void AddDecimalPoint(char* buffer);
void AddDecimalPoint2(char* buffer);
void SetupPorts();
char LoadSettingsFromEEPROM();
void SaveSettingsToEEPROM();
void CanTX(long packetID, unsigned char* data, unsigned char length, unsigned char delayAfterSending);
void RenderStartupScreen();
void RenderMainView();
void RenderMainViewNoCurrentSensor();
void RenderLiteIdleScreen();
void RenderMCStatus();
void RenderChargerStatus();
void RenderThreeChargerStatus();
void RenderBMSSummary();
void RenderBMSDetails();
void RenderWarningOverlay();
void RenderOptionsButtons();
inline void RenderBorderBox(int lx, int ly, int rx, int ry, U16 Fcolor, U16 Bcolor);
void RenderButton(Button* button, bool needsRedraw);
void RenderSettings();

// Global variables

#define NUM_RX_MOBS	4 // Four RX MOBs for CAN messages
st_cmd_t rxMsg[NUM_RX_MOBS];
U8 rxData[NUM_RX_MOBS][8];

U8 txData[8]; // CAN transmit buffer

char buffer[30]; // Used for sprintf functions

short ticks = 0; // For main loop timing

volatile uint8_t evmsStatusBytes[8];

signed char displayedPage = EVMS_CORE;

volatile bool displayNeedsFullRedraw = true;
bool showOptionsButtons;
bool showStartupScreen = true;

short coreStatus;
char isBMS16 = false; // Monitor can tell the difference based on CAN data, changes displays etc a little
char isActuallyBMS12i = false; // has a bit of a dirty hack to differentiate BMS16 and BMS12i

unsigned char error = NO_ERROR;
char evmsCommsTimer = 0;
char resetSoCsent = false;

short beepTimer = 0;
short errorBeeperTimeout = 0;
short touchTimer;
int touchX, touchY;
int touchBufferX[10], touchBufferY[10];

enum { NOTHING_TO_SEND, SEND_RESET_SOC, SEND_ZERO_CURRENT, SEND_ENTER_SETUP, SEND_GAUGE_STATE, SEND_SETTINGS, SEND_ACK_ERROR };
short canToGo;

short displayBrightness = 255;
short targetDisplayBrightness; // (Used for smoothly fading brightness)
bool  displayDimmed;
bool  headlightsOn = 0;

char canTXing = false;

bool setupMode = false;

int numCells = 0;

enum { GENERAL_SETTINGS, MC_SETTINGS, PACK_SETUP };
char settingsPage = GENERAL_SETTINGS;
short currentParameter = 0;
short currentBmsModule = 0;

short mcCurrentParameter = 0;

typedef struct {
	// These come from the charger
	short instVoltage;
	short instCurrent;
	unsigned char statusBits;
	unsigned char temp;
	// These go to the charger
	short targetVoltage;
	short targetCurrent;
	char controlBit;
} ChargerData;
ChargerData charger[3] = {{ 0, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0, 0 }};
char haveReceivedChargerData = false;
char numChargers = 1; // Gets set to 3 if we receive data from third charger

inline bool ButtonTouched(Button* button)
{
	return (touchX >= button->x-button->width/2 && touchX <= button->x+button->width/2
		&& touchY >= button->y && touchY <= button->y + 32);
}

inline void DisplayOn(bool on, bool shouldSaveToEEPROM)
{
	if (on)
	{
		if (displayDimmed || headlightsOn)
			targetDisplayBrightness = 254 - settings[NIGHT_BRIGHTNESS]*settings[NIGHT_BRIGHTNESS] * 5/2;
			// May 2016: Testing quadratic display brightness
			// April 2019: Using 254 so there's a difference between 0% night brightness and actually off (255), though both are black
		else
			targetDisplayBrightness = 0; // Full bright
	}
	else
		targetDisplayBrightness = 255;

	if (shouldSaveToEEPROM)
	{	
		eeprom_write_byte((U8*)EEPROM_DISPLAY_BRIGHTNESS, targetDisplayBrightness);
		eeprom_write_byte(0, 0); // Park EEPROM pointer to prevent corruption
	}
}

char toggley;
SIGNAL(TIMER0_OVF_vect) // Called at 7812Hz, i.e every 2048 cycles of 16Mhz clock
{
	ticks++; // Used for main loop timing

#ifdef NEW_LCD
	BACKLIGHT_PORT &= ~BACKLIGHT;

	if (beepTimer > 0)
	{
		if (toggley)
		{
			PORTB |= BUZZER;
			PORTB &= ~BUZZER2;
			toggley = 0;
		}
		else
		{
			PORTB &= ~BUZZER;
			PORTB |= BUZZER2;
			toggley = 1;
		}
	}
	else
		PORTB &= ~(BUZZER + BUZZER2); // Turn both off if not beeping
#else
	BACKLIGHT_PORT |= BACKLIGHT;
#endif

	if (!canTXing && (ticks & 0x07) == 0) // every 8th interrupt, or around 1000hz
	{
		for (int mob=0; mob<NUM_RX_MOBS; mob++)
		{
			U8 temp = can_get_status(&rxMsg[mob]);

			if (temp == CAN_STATUS_COMPLETED) // Then we have received a frame
				ProcessCanRX(mob);

			if (temp == CAN_STATUS_ERROR)
				PrepareCanRX(mob); // And flad a comms error?
		}
	}
}

SIGNAL(TIMER0_COMP_vect)
{
	if (displayBrightness < 254) // 254 is for 0% night brightness, and 255 is for actually off, but both should have no backlight
#ifdef NEW_LCD
		BACKLIGHT_PORT |= BACKLIGHT;	
#else
		BACKLIGHT_PORT &= ~BACKLIGHT;
#endif
}

SIGNAL(TIMER1_OVF_vect) // Interrupts at about 30Hz
{
	// Update display brightness
	if (targetDisplayBrightness > displayBrightness)
		displayBrightness += Cap(targetDisplayBrightness-displayBrightness, 0, 15); // Change by 15 maximum
	if (targetDisplayBrightness < displayBrightness)
		displayBrightness -= Cap (displayBrightness-targetDisplayBrightness, 0, 15);

	// Stay dark for half second to receive data from available devices - suppress initial "Waiting for data" page
	if (ticksSincePowerOn < 1) displayBrightness = 255;

	OCR0A = displayBrightness; // Updates backlight PWM, inverted due to PNP transistor
	
	// Poll touchscreen
	if (Touch_DataAvailable())
	{
		Touch_Read();

		touchTimer++;
		//touchX = Touch_GetX(); // these now in function below - delete here
		//touchY = Touch_GetY();
		HandleTouchDown();
	}
	else
	{
		if (touchTimer > 0) HandleTouchUp();
		
		touchTimer = 0;
		touchX = -1;
		touchY = -1;
	}

	UpdateBuzzer();
}

void PrepareCanRX(unsigned char mob)
{
	rxMsg[mob].pt_data = rxData[mob];
	for (int n=0; n<8; n++) rxData[mob][n] = 0;

	if (USE_29BIT_IDS) rxMsg[mob].ctrl.ide = true;

	rxMsg[mob].cmd = CMD_RX_DATA;

	while (can_cmd(&rxMsg[mob]) != CAN_CMD_ACCEPTED) { } // Wait for RX command to be completed
}

// This function gets called when a new CAN message is received
void ProcessCanRX(unsigned char mob)
{
	long packetID = rxMsg[mob].id.std;
	if (USE_29BIT_IDS) packetID = rxMsg[mob].id.ext;

	if (packetID >= BMS_BASE_ID && packetID < BMS_BASE_ID+MAX_BMS_MODULES*10+10) // Packet ID within BMS module range
	{
		int moduleID = (packetID - BMS_BASE_ID)/10;
		int packetType = packetID - BMS_BASE_ID - moduleID*10;

		if (moduleID < MAX_BMS_MODULES)
		{
			switch (packetType)
			{
				case BMS_REPLY1:
					for (int n=0; n<4; n++)
						cellVoltages[moduleID][n] = (rxData[mob][n*2]<<8) + rxData[mob][n*2+1];
					break;

				case BMS_REPLY2:
					for (int n=0; n<4; n++)
						cellVoltages[moduleID][n+4] = (rxData[mob][n*2]<<8) + rxData[mob][n*2+1];
					break;

				case BMS_REPLY3:
					for (int n=0; n<4; n++)
						cellVoltages[moduleID][n+8] = (rxData[mob][n*2]<<8) + rxData[mob][n*2+1];
					if (moduleID == 0 && isBMS16) isActuallyBMS12i = true; // If received this third voltages set, sender must be BMS12i
					break;

				case BMS_REPLY4:
					bmsTemps[moduleID][0] = rxData[mob][0];
					bmsTemps[moduleID][1] = rxData[mob][1];
					break;
			}
		}
	}
	else switch (packetID)
	{
		case CORE_BROADCAST_STATUS:
			for (int n=0; n<8; n++) evmsStatusBytes[n] = rxData[mob][n];
			evmsCommsTimer = 0;
			haveReceivedEVMSData = true;
			if (evmsStatusBytes[5] == 255)
			{
				isBMS16 = true;
				displayOffButton.text = "Power off";
				//displayOffButton.colour = BLUE;
			}
			headlightsOn = evmsStatusBytes[6]>>7;
			DisplayOn(targetDisplayBrightness != 255, false);
			break;

		case CAN_CURRENT_SENSOR_ID:
			current = ((long)rxData[mob][0]<<16) + ((long)rxData[mob][1]<<8) + (long)rxData[mob][2] - 8388608L;
			currentSensorTimeout = 4; // 1 second timeout
			if (!haveReceivedCurrentData)
			{
				haveReceivedCurrentData = true;
				displayNeedsFullRedraw = true;
			}
			break;

		case MC_STATUS_PACKET_ID:
			for (int n=0; n<8; n++) mcStatusBytes[n] = rxData[mob][n];
			mcCanTimeout = 4;
			haveReceivedMCData = true;
			/*
			if (showStartupScreen && ticksSincePowerOn >= 2) // MC has lower priority than EVMS, let EVMS go first
			{
				showStartupScreen = false;
				displayNeedsFullRedraw = true;
				displayedPage = MOTOR_CONTROLLER;
			}
			*/
			break;

		case MC_SEND_SETTINGS_ID:
			for (int n=0; n<4; n++) mcSettings[n] = rxData[mob][n]; // First four match 1:1
			mcSettings[MC_RAMP_RATE] = rxData[mob][4]&0b00001111; // Bottom 4 bits hold ramp rate
			mcSettings[MC_SPEED_CONTROL_TYPE] = (rxData[mob][4]&0b00110000)>>4;
			mcSettings[MC_TORQUE_CONTROL_TYPE] = (rxData[mob][4]&0b11000000)>>6;
			for (int n=5; n<8; n++) mcSettings[n+2] = rxData[mob][n];
			break;

		case TC_CHARGER1_RX_ID:
			charger[0].targetVoltage = rxData[mob][0]*256+rxData[mob][1];
			charger[0].targetCurrent = rxData[mob][2]*256+rxData[mob][3];
			charger[0].controlBit = rxData[mob][4];
			break;

		case TC_CHARGER1_TX_ID:
			charger[0].instVoltage = rxData[mob][0]*256+rxData[mob][1];
			charger[0].instCurrent = rxData[mob][2]*256+rxData[mob][3];
			charger[0].statusBits = rxData[mob][4];
			charger[0].temp = rxData[mob][5];
			chargerCommsTimeout[0] = 12; // Three seconds with 4hz loop
			haveReceivedChargerData = true;
			break;

		case TC_CHARGER2_RX_ID:
			charger[1].targetVoltage = rxData[mob][0]*256+rxData[mob][1];
			charger[1].targetCurrent = rxData[mob][2]*256+rxData[mob][3];
			charger[1].controlBit = rxData[mob][4];
			break;

		case TC_CHARGER2_TX_ID:
			charger[1].instVoltage = rxData[mob][0]*256+rxData[mob][1];
			charger[1].instCurrent = rxData[mob][2]*256+rxData[mob][3];
			charger[1].statusBits = rxData[mob][4];
			charger[1].temp = rxData[mob][5];
			chargerCommsTimeout[1] = 12; // Three seconds with 4hz loop
			if (numChargers<2) numChargers = 2;
			break;

		case TC_CHARGER3_RX_ID:
			charger[2].targetVoltage = rxData[mob][0]*256+rxData[mob][1];
			charger[2].targetCurrent = rxData[mob][2]*256+rxData[mob][3];
			charger[2].controlBit = rxData[mob][4];
			break;

		case TC_CHARGER3_TX_ID:
			charger[2].instVoltage = rxData[mob][0]*256+rxData[mob][1];
			charger[2].instCurrent = rxData[mob][2]*256+rxData[mob][3];
			charger[2].statusBits = rxData[mob][4];
			charger[2].temp = rxData[mob][5];
			chargerCommsTimeout[2] = 12; // Three seconds with 4hz loop
			numChargers = 3;
			break;
	}

	PrepareCanRX(mob);
}

void SetError(U8 newError)
{
	if (error != newError)
	{
		displayNeedsFullRedraw = true;
		showOptionsButtons = false;
		DisplayOn(true, false);
		errorBeeperTimeout = 120; // Beep 120 times = 60 seconds max with new error
	}
	error = newError;
}

void CalculateNumCells()
{
	numCells = 0;
	for (int id=0; id<MAX_BMS_MODULES; id++) numCells += bmsCellCounts[id];
}

void TestBeep(int delay, int osc)
{
	for (int n=0; n<osc; n++)
	{
		PORTB |= BUZZER;
		PORTB &= ~BUZZER2;
		_delay_us(delay);
		PORTB &= ~BUZZER;
		PORTB |= BUZZER2;
		_delay_us(delay);
	}
}

int main()
{
	SetupPorts();

	OCR0A = 255;

	_delay_ms(100); // Wait for LCD to power up
	TFT_Init(DISPLAY_TYPE, DISPLAY_TYPE == ILI9325);

	Touch_Init();

	can_init(0);

	char result = LoadSettingsFromEEPROM();
	if (result == EEPROM_BLANK || result == EEPROM_CORRUPT)
	{	
		SetError(CORRUPT_EEPROM_ERROR);
		SaveSettingsToEEPROM();
	}
	
	mcStatusBytes[0] = 0;
	CalculateNumCells();

	int lastDisplayBrightness = eeprom_read_byte((U8*)EEPROM_DISPLAY_BRIGHTNESS);
	eeprom_write_byte(0, 0); // Park EEPROM pointer to prevent corruption
	if (lastDisplayBrightness == 0) displayDimmed = false; else displayDimmed = true;
	DisplayOn(lastDisplayBrightness != 255, false); // Sends true unless old display brightness was off

	for (int mob=0; mob<NUM_RX_MOBS; mob++) PrepareCanRX(mob);

#ifdef NEW_LCD
	if (settings[BUZZER_ON]) // Do a bit of a startup chirp
	{
		TestBeep(160, 50);
		TestBeep(40, 200);
	}
#endif

	sei(); // Enable interrupts
	
	while (1)
	{
		// Timed polling for things like comms timeouts
		while (ticks > 1952) // 4Hz
		{
			ticks -= 1952;

			if (ticksSincePowerOn < 100) ticksSincePowerOn++;

			// Check for comms timeouts
			if (evmsCommsTimer < 100) evmsCommsTimer++;
			if (evmsCommsTimer == 4)
			{
				if (mcStatusBytes[0] == 0)
					SetError(CORE_COMMS_ERROR); // No core OR motor controller detected - set error
				else
					displayNeedsFullRedraw = true; // We detected a motor controller, so redraw to view that
			}

			if (error == CORE_COMMS_ERROR && evmsCommsTimer < 4) SetError(NO_ERROR); // Self-reset if received data

			if (currentSensorTimeout > 0)
				currentSensorTimeout--;
			else
				current = 0;
			
			if (mcCanTimeout > 0) mcCanTimeout--; // Only used in the MC status page

			for (int n=0; n<3; n++)
			{
				if (chargerCommsTimeout[n] > 0)
					chargerCommsTimeout[n]--;
				else
				{	// No data for a while - set values to zero
					charger[n].instVoltage = 0;
					charger[n].instCurrent = 0;
					charger[n].statusBits = 0;
				}
			}

			if (FAKE_EVMS) // Then pretend we have received EVMS status message, and transmit BMS ID 0 request
			{
				int ampHours = 10000;
				int voltage = 1234;
				evmsStatusBytes[0] = 0b00000000;	// Status in bottom 3 bits, error top 5
				evmsStatusBytes[1] = ampHours>>8;
				evmsStatusBytes[2] = ampHours&0xFF;
				evmsStatusBytes[3] = voltage>>8;
				evmsStatusBytes[4] = voltage&0xFF;
				evmsStatusBytes[5] = 123; // Aux voltage
				evmsStatusBytes[6] = 100;// Isolation
				evmsStatusBytes[7] = 23+40; // Temperature
				evmsCommsTimer = 0;
				haveReceivedEVMSData = true;

				txData[0] = txData[1] = 0; // Zero shunt voltage (i.e shunts off)
				CanTX(BMS_BASE_ID + BMS_REQUEST_DATA, txData, 2, 0);
			}
		}

		if (canToGo)
		{
			switch (canToGo)
			{
				case SEND_RESET_SOC:	CanTX(CORE_RESET_SOC, txData, 0, 5); break;
				case SEND_ZERO_CURRENT: CanTX(CAN_ZERO_CURRENT, txData, 0, 5); break;
				case SEND_ENTER_SETUP:
					txData[0] = CORE_SETUP_STATE;
					CanTX(CORE_SET_STATE, txData, 1, 5);
					txData[0] = 0;
					CanTX(MC_RECEIVE_SETTINGS_ID, txData, 1, 0);
					break;
				case SEND_SETTINGS:		TransmitSettings(); break;
				case SEND_GAUGE_STATE:	TransmitGaugeState(); break;
				case SEND_ACK_ERROR: CanTX(CORE_ACKNOWLEDGE_ERROR, &error, 1, 5); break;
				case POWER_OFF: CanTX(POWER_OFF, txData, 0, 5); break;
			}
			canToGo = NOTHING_TO_SEND;
		}

		// LCD update stuff - happens whenever there's free time
		char oldCoreStatus = coreStatus;
		coreStatus = evmsStatusBytes[0]&0x07; // Bottom 3 bits are status
		if (oldCoreStatus != coreStatus) displayNeedsFullRedraw = true;
		
		char newError = evmsStatusBytes[0]>>3; // Top 5 bytes hold error codes
		if (error < CORE_COMMS_ERROR) SetError(newError); // Only update error with Core error status if a Monitor error isn't pending

		if (showStartupScreen)
		{
			if (haveReceivedEVMSData)
			{
				displayNeedsFullRedraw = true;
				showStartupScreen = false;
			}
			else if (haveReceivedMCData && ticksSincePowerOn > 2)
			{
				displayNeedsFullRedraw = true;
				displayedPage = MOTOR_CONTROLLER;
				showStartupScreen = false;
			}				
		}

		if (isBMS16)
		{
			if (displayedPage == EVMS_CORE && settings[SHUNT_SIZE] == 0 /* no shunt */ && !haveReceivedCurrentData)
			{
				displayedPage = BMS_SUMMARY;
				if (!setupMode) displayNeedsFullRedraw = true;
			}
		}

		// Write to display
		if (setupMode)
			RenderSettings();
		else if (error != NO_ERROR && (!settings[STATIONARY_VERSION] || (error != BMS_LOW_WARNING && error != BMS_HIGH_WARNING)))
			RenderWarningOverlay();
		else if (showStartupScreen)
			RenderStartupScreen();
		else if (showOptionsButtons)
			RenderOptionsButtons();
		else if (displayedPage == MOTOR_CONTROLLER)
			RenderMCStatus();
		else if (displayedPage == TC_CHARGER)
			RenderChargerStatus();
		else if (displayedPage == EVMS_CORE)
		{
			if (haveReceivedCurrentData || isBMS16)
				RenderMainView();
			else
				RenderMainViewNoCurrentSensor();
		}
		else if (displayedPage == BMS12_DETAILS)
			RenderBMSDetails();
		else if (displayedPage == BMS_SUMMARY)
			RenderBMSSummary();

		if (SHOW_TOUCH_LOCATION)
		{
			char temp[5];
			itoa(touchX, buffer, 10);
			strcat(buffer, ",");
			itoa(touchY, temp, 10);
			strcat(buffer, temp);
			strcat(buffer, " ");
			
			TFT_Text(buffer, 0, 0, 1, GREEN, BLACK);
		}
	}
	return 0; // Compiler wants to see it
}

inline void CheckTouchedButton(Button* button)
{
	if (ButtonTouched(button)) touchedButton = button;
}

void HandleTouchDown()
{
	touchX = Touch_GetX();
	touchY = Touch_GetY();

	if (touchTimer <= 10)
	{
		touchBufferX[touchTimer-1] = touchX;
		touchBufferY[touchTimer-1] = touchY;
	}

	if (targetDisplayBrightness == 255)
	{
		if (touchTimer == 30)
		{
			DisplayOn(true, true);
			Beep(2);
			showOptionsButtons = false;
			displayNeedsFullRedraw = true;
		}
		return;
	}

	if (!setupMode && touchTimer == 30 && !showOptionsButtons) // Held down for 1 second
	{
		showOptionsButtons = true;	
		displayNeedsFullRedraw = true;
	}
	else if (touchTimer == 3)
	{
		// Ignore touches which changed location rapidly, might be erroneous data
		if (Abs(touchBufferX[0]-touchBufferX[1]) > 10 || Abs(touchBufferY[0]-touchBufferY[1]) > 10
			|| Abs(touchBufferX[1]-touchBufferX[2]) > 10 || Abs(touchBufferY[1]-touchBufferY[2]) > 10)
			return;
		
		if (showOptionsButtons)
		{
			CheckTouchedButton(&resetSocButton);
			if ((coreStatus == IDLE || isBMS16) && (!CONFIG_LOCK || !CONFIG_LOCK2)) CheckTouchedButton(&enterSetupButton); // This button only responds in IDLE mode
			CheckTouchedButton(&zeroCurrentButton);
			CheckTouchedButton(&displayOffButton);
			CheckTouchedButton(&exitOptionsButton);
		}
		else if (setupMode) // Then we're in setup mode
		{
			CheckTouchedButton(&changeSetupPageButtonLeft);
			CheckTouchedButton(&changeSetupPageButtonRight);
			
			// The same parameter and value button are also used for BMS configuration (module ID and cell count respectively)
			CheckTouchedButton(&changeParameterButtonLeft);
			CheckTouchedButton(&changeParameterButtonRight);
			CheckTouchedButton(&changeValueButtonLeft);
			CheckTouchedButton(&changeValueButtonRight);

			CheckTouchedButton(&exitSetupButton);

		}
		else if (error != NO_ERROR) // Acknowledging error
		{
			Beep(2);
		}
		else // One of the regular three screens - always beep because if not clicked in a button, we go to next screen
		{
			if (displayedPage == BMS12_DETAILS && !isBMS16)
			{
				CheckTouchedButton(&nextBmsModuleButton);
				CheckTouchedButton(&prevBmsModuleButton);
			}

			Beep(2);		
		}
		
		if (touchedButton) Beep(2); // Only beep if a button was touched
	}
	else if (touchTimer == 6) // then check for swipes
	{
		char swipingDown = true, swipingUp = true;
		for (int n=2; n<6; n++)
		{
			if (touchBufferY[n] - touchBufferY[n-1] < 5) swipingDown = false;
			if (touchBufferY[n] - touchBufferY[n-1] > -5) swipingUp = false;			
		}
		if (swipingDown)
		{
			displayDimmed = true;
			DisplayOn(true, true);
		}
		if (swipingUp)
		{
			displayDimmed = false;
			DisplayOn(true, true);
		}
	}
	else if (touchTimer >= 30 && setupMode) // Do key repeats
	{
		if (touchTimer%6 == 0) DoSetupButtons(true); // Passing "true" for isKeyRepeat
	}
}

void HandleTouchUp()
{
	if (targetDisplayBrightness == 255) return; // Do nothing if display is off

	// Ignore too-fast touches and swipes
	if (touchTimer < 3 || Abs(touchBufferX[0]-touchBufferX[1]) > 10 || Abs(touchBufferY[0]-touchBufferY[1]) > 10
		|| Abs(touchBufferX[1]-touchBufferX[2]) > 10 || Abs(touchBufferY[1]-touchBufferY[2]) > 10)
			return;
	
	if (showOptionsButtons)
	{
		if (ButtonTouched(&resetSocButton) && touchedButton == &resetSocButton)
		{
			canToGo = SEND_RESET_SOC;
			showOptionsButtons = false;
			displayNeedsFullRedraw = true;
		}

		if (ButtonTouched(&zeroCurrentButton) && touchedButton == &zeroCurrentButton)
		{
			canToGo = SEND_ZERO_CURRENT;
			showOptionsButtons = false;
			displayNeedsFullRedraw = true;
		}

		if (ButtonTouched(&enterSetupButton) && touchedButton == &enterSetupButton
			&& (coreStatus == IDLE || isBMS16) && (!CONFIG_LOCK || !CONFIG_LOCK2))
		{
			// Todo: set a flag for the main thread to do a CAN TX instead
			canToGo = SEND_ENTER_SETUP;
			setupMode = true;
			if (isBMS16)
				for (int n=0; n<NUM_SETTINGS; n++)
					if (settings[n] > bms16maximums[n]) settings[n] = bms16maximums[n]; // Cap to BMS16 maximums
			showOptionsButtons = false;
			displayNeedsFullRedraw = true;
		}

		if (ButtonTouched(&displayOffButton) && touchedButton == &displayOffButton)
		{
			if (isBMS16)
				canToGo = POWER_OFF;
			else
				DisplayOn(false, true);
		}
		if (ButtonTouched(&exitOptionsButton) && touchedButton == &exitOptionsButton)
		{
			showOptionsButtons = false;
			displayNeedsFullRedraw = true;
		}
	}
	else if (setupMode) // Then we're in setup mode
	{
		DoSetupButtons(false); // False = isKeyRepeat; some actions don't want key repeat
	}
	else if (error != NO_ERROR && (!settings[STATIONARY_VERSION] || (error != BMS_LOW_WARNING && error != BMS_HIGH_WARNING)))
	{
		if (error >= CORE_COMMS_ERROR)
			SetError(NO_ERROR);
		else
			canToGo = SEND_ACK_ERROR; // Send acknowledge to Core for its error reported

		displayNeedsFullRedraw = true;
	}
	else // One of the regular three screens
	{
		if (displayedPage == BMS12_DETAILS && ButtonTouched(&nextBmsModuleButton) && !isBMS16)
		{
			char startModule = currentBmsModule;
			do {
				currentBmsModule++;
				if (currentBmsModule >= MAX_BMS_MODULES) currentBmsModule = 0;
				if (currentBmsModule == startModule) break; // No modules to change to, avoids infinite loop
			} while (bmsCellCounts[currentBmsModule] == 0);
		}
		else if (displayedPage == BMS12_DETAILS && ButtonTouched(&prevBmsModuleButton) && !isBMS16)
		{
			char startModule = currentBmsModule;
			do {
				currentBmsModule--;
				if (currentBmsModule < 0) currentBmsModule = 15;
				if (currentBmsModule == startModule) break; // No modules found, avoids infinite loop
			} while (bmsCellCounts[currentBmsModule] == 0);
		}
		else if (touchedButton == 0 && touchTimer < 30) // Wasn't a touch down in a button, and we're running/charging
		{
			char oldPage = displayedPage;

			if (touchX > 160) // right side touched
			{
				displayedPage++;
				if (displayedPage == MOTOR_CONTROLLER && mcStatusBytes[0] == 0) displayedPage++; // Skip past MC page if none detected
				if (displayedPage == TC_CHARGER && !haveReceivedChargerData) displayedPage++;
				if (displayedPage == BMS_SUMMARY && numCells == 0) displayedPage++;
				if (displayedPage == BMS12_DETAILS && numCells == 0) displayedPage++;
				if (displayedPage == NUM_KNOWN_DEVICES) displayedPage = 0;
				if (displayedPage == EVMS_CORE && !haveReceivedEVMSData) displayedPage++;
			}
			else
			{
				displayedPage--;
				if (displayedPage == EVMS_CORE && !haveReceivedEVMSData) displayedPage--;
				if (displayedPage < EVMS_CORE) displayedPage = BMS12_DETAILS; // Wrap around
				if (displayedPage == BMS12_DETAILS && numCells == 0) displayedPage--;
				if (displayedPage == BMS_SUMMARY && numCells == 0) displayedPage--; // Skip past BMS pages if no cells being monitored
				if (displayedPage == TC_CHARGER && !haveReceivedChargerData) displayedPage--; // Skip if no charger
				if (displayedPage == MOTOR_CONTROLLER && mcStatusBytes[0] == 0) displayedPage--; // Skip past MC page if none detected
				if (displayedPage == EVMS_CORE && isBMS16 && settings[SHUNT_SIZE] == 0 && !haveReceivedCurrentData) displayedPage = BMS12_DETAILS;
			}

			if (displayedPage != oldPage) displayNeedsFullRedraw = true;
		}
		
	}

	touchedButton = 0;
}

void DoSetupButtons(char isKeyRepeat)
{
	// Always visible on Setup page are buttons to change (toggle) the settings page
	if (ButtonTouched(&changeSetupPageButtonLeft))
	{
		settingsPage--;
		if (settingsPage == PACK_SETUP && isBMS16) settingsPage--; // Skip BMS setup if using BMS16 (single Num Cells in general settings instead)
		if (settingsPage == MC_SETTINGS && mcStatusBytes[0] == 0) settingsPage--; // Skip past motor controller if none connected
		if (settingsPage < 0) settingsPage = PACK_SETUP;
	}
	else if (ButtonTouched(&changeSetupPageButtonRight)) // Only 2 pages so either button = swap pages
	{
		settingsPage++;
		if (settingsPage == MC_SETTINGS && mcStatusBytes[0] == 0) settingsPage++; // Skip past motor controller if none connected
		if (settingsPage == PACK_SETUP && isBMS16) settingsPage++; // Skip past BMS setup if using BMS16
		if (settingsPage > PACK_SETUP)
			settingsPage = GENERAL_SETTINGS;
	}
	else if (settingsPage == GENERAL_SETTINGS) // General settings has four buttons: Left and right to change parameter, and left and right for value
	{
		if (ButtonTouched(&changeParameterButtonLeft))
		{
			currentParameter--;
			if (isBMS16) // If we're connected to a BMS16, skip past nonapplicable parameters
				while (bms16maximums[currentParameter] == 0) currentParameter--;

			if (currentParameter < 0) currentParameter = NUM_SETTINGS-1;
		}

		if (ButtonTouched(&changeParameterButtonRight))
		{
			currentParameter++;
			if (isBMS16)
				while (bms16maximums[currentParameter] == 0) currentParameter++;

			if (currentParameter == NUM_SETTINGS) currentParameter = 0;	
		}

		if (ButtonTouched(&changeValueButtonLeft) || ButtonTouched(&changeValueButtonRight))
		{
			int delta = 1;
			if (ButtonTouched(&changeValueButtonLeft)) delta = -1;					

			// Prevent charger current from using 8th bit, since charger voltage needs it
			if (currentParameter == CHARGER_CURRENT && delta == 1 && settings[CHARGER_CURRENT] == 127) delta = 0;

			// If charger voltage is about to overflow, set 9th bit in charger current byte and zero voltage byte
			if (currentParameter == CHARGER_VOLTAGE && delta == 1 && settings[CHARGER_VOLTAGE] == 255)
			{
				settings[CHARGER_CURRENT] |= 0b10000000; // Set high bit of current byte
				settings[CHARGER_VOLTAGE] = 0;
				delta = 0;
			}
			else if (currentParameter == CHARGER_VOLTAGE && delta == -1 && settings[CHARGER_VOLTAGE] == 0
				&& settings[CHARGER_CURRENT]&0b10000000) // Our 9th bit is set
			{
				settings[CHARGER_CURRENT] &= 0b01111111; // Clear high bit of current byte
				settings[CHARGER_VOLTAGE] = 255;
				delta = 0;
			}

			if (currentParameter == CHARGER_VOLTAGE2 && delta == 1 && settings[CHARGER_VOLTAGE2] == 255)
			{
				settings[CHARGER_CURRENT2] |= 0b10000000; // Set high bit of current byte
				settings[CHARGER_VOLTAGE2] = 0;
				delta = 0;
			}
			else if (currentParameter == CHARGER_VOLTAGE2 && delta == -1 && settings[CHARGER_VOLTAGE2] == 0
				&& settings[CHARGER_CURRENT2]&0b10000000) // Our 9th bit is set
			{
				settings[CHARGER_CURRENT2] &= 0b01111111; // Clear high bit of current byte
				settings[CHARGER_VOLTAGE2] = 255;
				delta = 0;
			}

			int maximum = maximums[currentParameter];
			if (isBMS16) maximum = bms16maximums[currentParameter];
			if (isBMS16 && isActuallyBMS12i && currentParameter == NUM_CELLS)
			{
				minimums[NUM_CELLS] = 4; maximum = 12;
			}
			settings[currentParameter] = Cap(settings[currentParameter]+delta,
				minimums[currentParameter], maximum);

			if (isBMS16 && currentParameter == SHUNT_SIZE && settings[currentParameter] > 3)
				settings[currentParameter] = 3;

			if (currentParameter >= FUEL_GAUGE_FULL && currentParameter <= TEMP_GAUGE_COLD) TransmitGaugeState();
			if (currentParameter == NIGHT_BRIGHTNESS) DisplayOn(true, false); // Updates target brightness
		}
	}
	else if (settingsPage == MC_SETTINGS) // Four buttons: Left and right to change parameter, and left and right for value
	{
		if (ButtonTouched(&changeParameterButtonLeft))
		{
			mcCurrentParameter--;
			if (mcCurrentParameter < 0) mcCurrentParameter = MC_NUM_SETTINGS-1;
		}

		if (ButtonTouched(&changeParameterButtonRight))
		{
			mcCurrentParameter++;
			if (mcCurrentParameter == MC_NUM_SETTINGS) mcCurrentParameter = 0;
		}

		if (ButtonTouched(&changeValueButtonLeft) || ButtonTouched(&changeValueButtonRight))
		{
			int delta = 1;
			if (ButtonTouched(&changeValueButtonLeft)) delta = -1;

			if ((mcStatusBytes[0]&0x0F) == MC600C) // Need to change maximums for smaller controller
			{
				mcMaximums[MC_MAX_MOTOR_CURRENT] = 60;
				mcMaximums[MC_MAX_BATT_CURRENT] = 60;
			}

			mcSettings[mcCurrentParameter] = Cap(mcSettings[mcCurrentParameter]+delta,
				mcMinimums[mcCurrentParameter], mcMaximums[mcCurrentParameter]);
		}
	}
	else if (settingsPage == PACK_SETUP) // Pack setup.. 16 buttons for selecting cell, 13 buttons for selecting number of cells
	{
		if (ButtonTouched(&changeParameterButtonRight))
			currentBmsModule = Cap(currentBmsModule+1, 0, 15);

		if (ButtonTouched(&changeParameterButtonLeft))
			currentBmsModule = Cap(currentBmsModule-1, 0, 15);
		
		if (ButtonTouched(&changeValueButtonLeft)) // Reduce by 1
		{
			bmsCellCounts[currentBmsModule] = Cap(bmsCellCounts[currentBmsModule]-1, 0, 12);
			if (bmsCellCounts[currentBmsModule] < 4) bmsCellCounts[currentBmsModule] = 0; // Can't be 1-3
		}
		if (ButtonTouched(&changeValueButtonRight)) // Increase by 1
			bmsCellCounts[currentBmsModule] = Cap(bmsCellCounts[currentBmsModule]+1, 4, 12); // Always >0, can't be 1-3
	}

	if (ButtonTouched(&exitSetupButton) && !isKeyRepeat)
	{
		canToGo = SEND_SETTINGS;

		setupMode = false;
		currentBmsModule = 0;
		//if (isBMS16) haveReceivedCurrentData = false;
		displayNeedsFullRedraw = true;
	}
}

void TransmitSettings()
{
	// Pack and send expected BMS cell counts		
	for (int n=0; n<8; n++)
	{
		txData[n] = bmsCellCounts[n*2];
		txData[n] += bmsCellCounts[n*2+1]<<4;
	}
	
	CanTX(CORE_RECEIVE_CELL_NUMS, txData, 8, 20);
	// Pack and send updated core settings
	for (int n=0; n<8; n++) txData[n] = settings[n];
	CanTX(CORE_RECEIVE_CONFIG1, txData, 8, 20);
	for (int n=0; n<8; n++) txData[n] = settings[n+8];
	CanTX(CORE_RECEIVE_CONFIG2, txData, 8, 20);
	for (int n=0; n<8; n++) txData[n] = settings[n+16];
	CanTX(CORE_RECEIVE_CONFIG3, txData, 8, 20);
	for (int n=0; n<8; n++) txData[n] = settings[n+24];
	CanTX(CORE_RECEIVE_CONFIG4, txData, 8, 20); // -> Remember, Core saves to EEPROM only after receiving this packet
	
	// Send motor controller settings
	for (int n=0; n<4; n++) txData[n] = mcSettings[n];
	txData[4] = mcSettings[MC_RAMP_RATE] + (mcSettings[MC_SPEED_CONTROL_TYPE]<<4) + (mcSettings[MC_TORQUE_CONTROL_TYPE]<<6);
	for (int n=5; n<8; n++) txData[n] = mcSettings[n+2];
	CanTX(MC_RECEIVE_SETTINGS_ID, txData, 8, 20);

	if (isBMS16)
	{
		for (int n=0; n<MAX_BMS_MODULES; n++) bmsCellCounts[n] = 0; // Make sure the rest are zero
		if (isActuallyBMS12i)
			bmsCellCounts[0] = settings[NUM_CELLS];
		else
		{
			bmsCellCounts[0] = 8;
			bmsCellCounts[1] = settings[NUM_CELLS]-8;
		}
	}
	
	SaveSettingsToEEPROM();

	// And now that updating is all done, tell Core to go back to Idle
	txData[0] = CORE_IDLE_STATE;
	CanTX(CORE_SET_STATE, txData, 1, 5);

	CalculateNumCells(); // In case it has changed

	//wdt_enable(WDTO_500MS);
}

void TransmitGaugeState()
{
	if (settingsPage == GENERAL_SETTINGS)
	{
		txData[0] = CORE_SETUP_STATE;
		txData[1] = settings[currentParameter];
		switch (currentParameter)
		{
			case FUEL_GAUGE_FULL:
				txData[0] = CORE_SETUP_EDIT_FUEL_GAUGE;
				break;	

			case FUEL_GAUGE_EMPTY:
				txData[0] = CORE_SETUP_EDIT_FUEL_GAUGE;
				break;	

			case TEMP_GAUGE_HOT:
				txData[0] = CORE_SETUP_EDIT_TEMP_GAUGE;
				break;	

			case TEMP_GAUGE_COLD:
				txData[0] = CORE_SETUP_EDIT_TEMP_GAUGE;
				break;	
		}
		CanTX(CORE_SET_STATE, txData, 2, 0);
	}
}

void Beep(short ticks)
{
	if (settings[BUZZER_ON])
	{
		PORTB |= BUZZER;
		beepTimer = ticks;
	}
}

void UpdateBuzzer()
{
	if (beepTimer > -100) beepTimer--;
		
	if (beepTimer == 0) PORTB &= ~BUZZER;

	if (beepTimer < -8 && error != NO_ERROR
		&& (!settings[STATIONARY_VERSION] || (error != BMS_LOW_WARNING && error != BMS_HIGH_WARNING))
		&& errorBeeperTimeout>0)
	{
		Beep(8);

		errorBeeperTimeout--;
	}
}

void AddDecimalPoint(char* buffer) // (to a number stored as a string)
{
	short i = strlen(buffer);
	if (i == 1) // i.e single character, we want to add a leading zero
	{	
		buffer[2] = 0;
		buffer[1] = buffer[0];
		buffer[0] = '0';
		i++;
	}

	buffer[i+1] = buffer[i]; // move final zero
	buffer[i] = buffer[i-1]; // move last character
	buffer[i-1] = '.'; // insert decimal
}

void AddDecimalPoint2(char* buffer) // (to a number stored in a string, to 2 decimal places)
{
	char temp[10];
	while (strlen(buffer) < 3) // Add leading zeroes until we have at least 3 digits
	{
		strcpy(temp, "0");
		strcat(temp, buffer);
		strcpy(buffer, temp);
	}

	short i = strlen(buffer);
	buffer[i+1] = buffer[i]; // move final zero
	buffer[i] = buffer[i-1]; // move last character
	buffer[i-1] = buffer[i-2]; // move 2nd last character
	buffer[i-2] = '.'; // insert decimal
}

void AddDecimalPoint3(char* buffer) // (to a number stored in a string, to 3 decimal places)
{
	char temp[10];
	while (strlen(buffer) < 4) // Add leading zeroes until we have at least 4 digits
	{
		strcpy(temp, "0");
		strcat(temp, buffer);
		strcpy(buffer, temp);
	}

	short i = strlen(buffer);
	buffer[i+1] = buffer[i]; // move final zero
	buffer[i] = buffer[i-1]; // move last character
	buffer[i-1] = buffer[i-2]; // move 2nd last character
	buffer[i-2] = buffer[i-3]; // move 3rd last character
	buffer[i-3] = '.'; // insert decimal
}

void WriteTemp(char* buffer, short celcius)
{
	if (settings[USE_FAHRENHEIT])
	{
		itoa(celcius*9/5+32, buffer, 10);
		strcat(buffer, "~F "); // ~ has been modified to display the degree sign
	}
	else
	{
		itoa(celcius, buffer, 10);
		strcat(buffer, "~C ");
	}
}

void CanTX(long packetID, unsigned char* data, unsigned char length, unsigned char delayAfterSending)
{
	canTXing = true; // Semaphor so it doesn't RX while TXing
	
	st_cmd_t canFrame;
	canFrame.pt_data = data;
	if (USE_29BIT_IDS)
	{
		canFrame.ctrl.ide = 1; // CAN 2.0B
		canFrame.id.ext = packetID;
	}
	else
	{
		canFrame.ctrl.ide = 0; // CAN 2.0A
		canFrame.id.std = packetID;
	}
	canFrame.dlc = length;
	canFrame.cmd = CMD_TX_DATA;
	while (can_cmd(&canFrame) != CAN_CMD_ACCEPTED) {} // Wait for MOB to accept frame
	while (can_get_status(&canFrame) == CAN_STATUS_NOT_COMPLETED) {} // Wait for TX completion

	if (delayAfterSending > 0) _delay_ms(delayAfterSending);

	canTXing = false;
}



void SetupPorts()
{
	// TODO: Move this into Touchscreen.c?
	DP_Lo_DDR = 0b11111111; // DP_Lo
	DDRB = BUZZER + BUZZER2;
	DP_Hi_DDR = 0b11111111; // DP_Hi
	BACKLIGHT_DDR |= BACKLIGHT;
	
#ifdef NEW_LCD
	DDRG |= RST + WR;
	DDRD |= CS + RS + RD;
	PORTD |= (1<<PD0); // Pull up on config lock input
	PORTE |= (1<<PE5); // Pull up on new config lock input
#else
	DDRF |= RST + CS + RD + WR + RS; // LCD control pins
	DDRD |= (1<<PD0); // PD0 is config lock on the new LCD, but on the old one we have to just pull it low internally
	DDRE |= (1<<PE5); // As above, but for new config lock pin
#endif


	// Timer0 used for ticks and display backlight PWM
	TCCR0A = (1<<CS01) /* + (1<<WGM01) + (1<<WGM00) + (1<<COM0A1) */ ; // clk/8 counting rate = 1Mhz, overflows at 7812Hz, PWM OFF - was fast PWM, non inverting
	TIMSK0 = (1<<TOIE0) + (1<<OCIE0A); // Interrupt on overflow and output compare

	// Timer 1 used for touchscreen polling interrupt
	TCCR1B = (1<<CS11); // Timer running with clk/8 prescaler -> about 30hz overflows at 16Mhz clock
	TIMSK1 = (1<<TOIE1); // Enable interrupt on overflow
}

char LoadSettingsFromEEPROM()
{
	unsigned char tempSettings[NUM_SETTINGS];
	unsigned char tempCellCount[MAX_BMS_MODULES];
	
	if (eeprom_read_dword((uint32_t*)(EEPROM_OFFSET + NUM_SETTINGS + MAX_BMS_MODULES/2 + 2)) != 0xC0FFEE)
		return EEPROM_BLANK;
	
	unsigned short checksum = 0;
	for (int i=0; i<NUM_SETTINGS; i++)
	{
		tempSettings[i] = eeprom_read_byte((uint8_t*)(EEPROM_OFFSET + i));
		checksum += tempSettings[i];
	}

	for (int i=0; i<MAX_BMS_MODULES/2; i++)
	{
		unsigned char byte = eeprom_read_byte((uint8_t*)(EEPROM_OFFSET + NUM_SETTINGS + i));
		tempCellCount[i*2] = byte & 0x0F;
		tempCellCount[i*2+1] = byte>>4;
		checksum += byte;
	}

	if (checksum != eeprom_read_word((uint16_t*)(EEPROM_OFFSET + NUM_SETTINGS + MAX_BMS_MODULES/2))) return EEPROM_CORRUPT;

	// Otherwise, settings are correct and we can copy to the real thing
	for (int i=0; i<NUM_SETTINGS; i++) settings[i] = tempSettings[i];
	for (int i=0; i<MAX_BMS_MODULES; i++) bmsCellCounts[i] = tempCellCount[i];

	eeprom_write_byte(0, 0); // Park EEPROM pointer at sacrificial location 0

	return EEPROM_CORRECT;
}

void SaveSettingsToEEPROM()
{
	cli(); // Disable interrupts

	unsigned short checksum = 0;
	for (int i=0; i<NUM_SETTINGS; i++)
	{
		eeprom_write_byte((uint8_t*)(EEPROM_OFFSET + i), settings[i]);
		checksum += settings[i];
	}

	for (int i=0; i<MAX_BMS_MODULES/2; i++)
	{
		unsigned char byte = bmsCellCounts[i*2];
		byte += bmsCellCounts[i*2+1]*16;
		eeprom_write_byte((uint8_t*)(EEPROM_OFFSET + NUM_SETTINGS + i), byte);
		checksum += byte;
	}

	eeprom_write_word((uint16_t*)(EEPROM_OFFSET + NUM_SETTINGS + MAX_BMS_MODULES/2), checksum); // Add checksum
	eeprom_write_dword((uint32_t*)(EEPROM_OFFSET + NUM_SETTINGS + MAX_BMS_MODULES/2 + 2), 0xC0FFEE); // Add watermark

	eeprom_write_byte(0, 0);

	sei();
}

// Functions for writing to display
void DrawTitlebar(char* text)
{
	U16 col = L_GRAY;
	switch (coreStatus)
	{
		case PRECHARGING:	col = ORANGE; break;
		case RUNNING:		col = RUNNING_COLOUR; break;
		case CHARGING:		col = CHARGING_COLOUR; break;
		case STOPPED:		col = RED; break;
		case SETUP:			col = ORANGE; break;
	}
	if (settings[STATIONARY_VERSION] && (error == BMS_HIGH_WARNING || error == BMS_LOW_WARNING))
	{
		col = RED;
		if (displayedPage != BMS12_DETAILS && error == BMS_HIGH_WARNING)
			text = "EVMS: Charge Disabled";
		else if (displayedPage != BMS12_DETAILS && error == BMS_LOW_WARNING)
			text = "EVMS: Discharge disabled";
	}

	TFT_Fill(BGND_COLOUR);
	TFT_Box(0, 0, 319, 19, col);
	TFT_CentredText(text, 160, 2, 1, BLACK, col);
}

void RenderStartupScreen()
{
	if (displayNeedsFullRedraw) TFT_Fill(BGND_COLOUR);
	displayNeedsFullRedraw = false;

	TFT_CentredText("ZEVA EVMS Monitor", 160, 90, 1, TEXT_COLOUR, BGND_COLOUR);
	TFT_CentredText("Waiting for data..", 160, 130, 1, BLUE, BGND_COLOUR);
	
	// Draw crosshairs at touch location
	if (touchX > 20 && touchY > 20 && touchX < 300 && touchY < 300)
	{
		TFT_Box(touchX-20, touchY, touchX+20, touchY, D_GRAY);
		TFT_Box(touchX, touchY-20, touchX, touchY+20, D_GRAY);
	}
}

void RenderMainView()
{
	int temperature = evmsStatusBytes[7];
	int voltage = (evmsStatusBytes[3]<<8) + evmsStatusBytes[4];
	int isolation = evmsStatusBytes[6] & 0b01111111; // Bottom 7 bits only

	if (displayNeedsFullRedraw) // Render static parts
	{
		displayNeedsFullRedraw = false;

		char statusText[20];
		strcpy_P(statusText, (char*)pgm_read_word(&(coreStatuses[coreStatus])));
		if (isBMS16)
			strcpy(buffer, "BMS Status: ");
		else
			strcpy(buffer, "EVMS: ");
		strcat(buffer, statusText);
		
		DrawTitlebar(buffer);
		
		TFT_Text("Voltage", 16, 30, 1, LABEL_COLOUR, BGND_COLOUR);

		TFT_Text("Current", 16, 88, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Power", 16, 146, 1, LABEL_COLOUR, BGND_COLOUR);

		if (isBMS16)
		{	// Used to only show temp if a sensor was plugged in, but I think it looks better to show title always and '-' value
			/*if (evmsStatusBytes[7] > 0)*/ TFT_Text("Temp", 16, 202, 1, LABEL_COLOUR, BGND_COLOUR);
		}
		else
			TFT_Text("Aux V", 16, 202, 1, LABEL_COLOUR, BGND_COLOUR);
		if (temperature > 0 && !isBMS16) TFT_Text("Temp", 100, 202, 1, LABEL_COLOUR, BGND_COLOUR);
		if (isolation <= 100 && !isBMS16) TFT_Text("Isol", 172, 202, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("SoC", 244, 202, 1, LABEL_COLOUR, BGND_COLOUR);		

		TFT_Box(243, 36, 279, 48, L_GRAY);
		TFT_Box(245, 38, 277, 46, D_GRAY);
		TFT_Box(222, 46, 300, 192, L_GRAY);
	}

	// Dynamic parts
	long power = (long)voltage*(long)(current/100L); // Current was in milliamps, don't need such accuracy
	if (power < 0) power = -power;
	power = power/10000L; // Gets it into tenths of a kilowatt

	if (voltage == 0)
		strcpy(buffer, " -    ");
	else
	{
		if (voltage < 1000 && numCells > 0)
		{
			itoa(voltage, buffer, 10);
			AddDecimalPoint(buffer);
		}
		else
			itoa(voltage/10, buffer, 10);
	
		strcat(buffer, "V  ");
	}
	TFT_Text(buffer, 16, 48, 2, TEXT_COLOUR, BGND_COLOUR);

	int currenty = (current+50L)/100L; // round to 0.1A resolution 16 bit

	if (settings[REVERSE_CURRENT_DISPLAY]) currenty = -currenty;

	if (currentSensorTimeout == 0 && !isBMS16)
		strcpy(buffer, " -    ");
	else
	{
		if (Abs(currenty) < 1000)
		{
			itoa(currenty, buffer, 10);
			AddDecimalPoint(buffer);
		}
		else
			itoa(currenty/10, buffer, 10);

		strcat(buffer, "A  ");
	}
	TFT_Text(buffer, 16, 106, 2, TEXT_COLOUR, BGND_COLOUR);

	if (currentSensorTimeout == 0 && !isBMS16)
		strcpy(buffer, " -    ");
	else
	{
		if (power < 1000) // Under 100kW
		{
			itoa(Abs(power), buffer, 10);
			AddDecimalPoint(buffer); // Display in tenths of a kilowatt
		}
		else
			itoa(Abs(power/10), buffer, 10); // Display whole kilowatts only
		strcat(buffer, "kW ");
	}
	TFT_Text(buffer, 16, 164, 2, TEXT_COLOUR, BGND_COLOUR);

	if (!isBMS16)
	{
		int auxV = evmsStatusBytes[5];
		itoa(auxV, buffer, 10); // Aux voltage
		AddDecimalPoint(buffer);
		strcat(buffer, "V  ");
		TFT_Text(buffer, 16, 220, 1, TEXT_COLOUR, BGND_COLOUR);
	}

	if (temperature > 0)
	{
		WriteTemp(buffer, temperature-40);
		TFT_Text(buffer, 100-84*isBMS16, 220, 1, TEXT_COLOUR, BGND_COLOUR);
	}
	else if (isBMS16) // Always showing Temp label for BMS16, but '-' if no temp available (evens up GUI appearance)
		TFT_Text(" -  ", 16, 220, 1, TEXT_COLOUR, BGND_COLOUR);
	
	if (isolation <= 100 && !isBMS16)
	{
		int isol = ((isolation+5)/10)*10; // Do some rounding to nearest 10% so it doesn't jiggle too much
		
		itoa(isol, buffer, 10); // Leakage
		strcat(buffer, "%  ");
		TFT_Text(buffer, 172, 220, 1, TEXT_COLOUR, BGND_COLOUR);
	}
	
	int ampHours = (evmsStatusBytes[1]<<8) + evmsStatusBytes[2];

	int soc = Cap(201L*(long)ampHours/2L/(long)(settings[PACK_CAPACITY]*PACK_CAPACITY_MULTIPLIER*10), 0, 100); // 201L/2L is for rounding instead of truncating

	if (settings[SOC_DISPLAY] == SOC_AMPHOURS)
	{
		if (ampHours < 100)
		{
			itoa(ampHours, buffer, 10);
			AddDecimalPoint(buffer);
			strcat(buffer, "Ah ");
		}
		else
		{
			itoa((ampHours+5)/10, buffer, 10);
			strcat(buffer, "Ah ");
		}
	}
	else
	{	
		itoa(soc, buffer, 10);
		strcat(buffer, "%  ");
	}
	TFT_Text(buffer, 244, 220, 1, TEXT_COLOUR, BGND_COLOUR);

	int height = 142 * soc / 100;
	unsigned int redAmount = Cap(0b11111 /* 5 bits */ * (100-soc) / 100 *3/2, 0, 0b11111);
	unsigned int greenAmount = Cap(0b111111 /* 6 bits */ * 2* soc / 100, 0, 0b111111);
	unsigned int colour = (redAmount<<11) + (greenAmount<<5);

	if (SOC_COLOUR > 0) colour = SOC_COLOUR;
	TFT_Box(224, 48, 298, 190-height, D_GRAY);	// Background part
	TFT_Box(224, 190-height, 298, 190, colour);	// SoC part
}

void DrawCellsBarGraph()
{
	int balanceVoltage = 5000;
	if (settings[BALANCE_VOLTAGE] < 251 && (coreStatus == CHARGING || isBMS16))
		balanceVoltage = 2000+settings[BALANCE_VOLTAGE]*10;
	else if (settings[BALANCE_VOLTAGE] == 251 && numCells > 0 && (coreStatus == CHARGING || isBMS16
		|| (coreStatus == RUNNING && settings[STATIONARY_VERSION] == true)))
	{
		long packVoltage = 0;
		int minCellVoltage = 5000;
		int maxCellVoltage = 0;
		for (int id=0; id<MAX_BMS_MODULES; id++)
			for (int n=0; n<bmsCellCounts[id]; n++)
			{
				packVoltage += cellVoltages[id][n];
				if (cellVoltages[id][n] < minCellVoltage) minCellVoltage = cellVoltages[id][n];
				if (cellVoltages[id][n] > maxCellVoltage) maxCellVoltage = cellVoltages[id][n];
			}

		//balanceVoltage = (long)packVoltage/(long)numCells + BALANCE_TOLERANCE;
		balanceVoltage = (minCellVoltage + maxCellVoltage) / 2 + BALANCE_TOLERANCE; // Oct 2020: New balance scheme, works better for single low cells
	}
	
	int width = 0;
	if (numCells > 0) width = 320 / numCells;
	int margin = (320 - numCells*width)/2;
	int gap = 1;
	if (numCells > 160) gap = 0; // Have to do single pixel bars with no gap

	int max = 200+settings[BMS_MAX_VOLTAGE]; // In hundredths of a volt
	int min = 150+settings[BMS_MIN_VOLTAGE];

	if (settings[STATIONARY_VERSION]) max += settings[BMS_HYSTERESIS]; // allow for hysteresis
	if (settings[STATIONARY_VERSION]) min -= settings[BMS_HYSTERESIS];

	int range = max - min; // Typical range something like 130 ?

	int n=0;
	for (uint8_t m=0; m<MAX_BMS_MODULES; m++)
	{
		char cells = bmsCellCounts[m];
		for (uint8_t c=0; c<cells; c++)
		{
			int v = cellVoltages[m][c]/10;

			U16 col = GREEN;
			if (v < min)
			{	col = BLUE;
				v = min;
			}
			else if (v > max)
			{
				col = RED;
				v = max;
			}
			else if (cellVoltages[m][c] > balanceVoltage)
				col = ORANGE;

			unsigned int height = 5 + (v - min)*40/range;
			
			TFT_Box(margin+n*width+1, 185, margin+(n+1)*width-gap, 238-height, BGND_COLOUR); // Blank out anything above bars
			TFT_Box(margin+n*width+1, 239-height, margin+(n+1)*width-gap, 239, col);
			
			// Add dotted line, one bar at a time to minimise flashing
			int start = margin+n*width;
			int end = margin+(n+1)*width;
			for (int a=start/3; a<=end/3; a++)
			{
				TFT_H_Line(a*3+1, a*3+1, 194, WHITE);
				TFT_H_Line(a*3+1, a*3+1, 234, WHITE);

				if (settings[STATIONARY_VERSION])
				{
					// how many pixels is 0.4V?

					int offset = settings[BMS_HYSTERESIS] * 80 / range;

					TFT_H_Line(a*3+1, a*3+1, 194+offset, WHITE);
					TFT_H_Line(a*3+1, a*3+1, 234-offset, WHITE);
				}
			}
			n++;
		}
	}
}

void RenderMainViewNoCurrentSensor()
{
	if (displayNeedsFullRedraw)
	{
		displayNeedsFullRedraw = false;

		char statusText[20];
		strcpy_P(statusText, (char*)pgm_read_word(&(coreStatuses[coreStatus])));
		if (isBMS16)
			strcpy(buffer, "BMS: ");
		else
			strcpy(buffer, "EVMS: ");
		strcat(buffer, statusText);
		
		DrawTitlebar(buffer);
		
		TFT_Text("Pack voltage", 16, 40, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Temperature", 170, 40, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Isolation", 16, 110, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Aux voltage", 170, 110, 1, LABEL_COLOUR, BGND_COLOUR);			
	}

	int voltage = (evmsStatusBytes[3]<<8) + evmsStatusBytes[4];
	if (voltage == 0)
		strcpy(buffer, " -    ");
	else
	{
		if (voltage < 1000 && numCells > 0)
		{
			itoa(voltage, buffer, 10);
			AddDecimalPoint(buffer);
		}
		else
			itoa(voltage/10, buffer, 10);
	
		strcat(buffer, "V ");
	}
	TFT_Text(buffer, 16, 60, 2, TEXT_COLOUR, BGND_COLOUR);
	
	int temperature = evmsStatusBytes[7];
	if (temperature == 0)
		strcpy(buffer, " -    ");
	else
		WriteTemp(buffer, temperature-40);
	
	TFT_Text(buffer, 170, 60, 2, TEXT_COLOUR, BGND_COLOUR);
	
	if (voltage == 0)
		strcpy(buffer, " -    ");
	else
	{
		int isolation = evmsStatusBytes[6] & 0b01111111; // Bottom 7 bits only
		int isol = ((isolation+5)/10)*10; // Do some rounding to nearest 10% so it doesn't jiggle too much
		itoa(isol, buffer, 10); // Leakage
		strcat(buffer, "%  ");
	}
	TFT_Text(buffer, 16, 130, 2, TEXT_COLOUR, BGND_COLOUR);

	int auxV = evmsStatusBytes[5];
	itoa(auxV, buffer, 10); // Aux voltage
	AddDecimalPoint(buffer);
	strcat(buffer, "V ");
	TFT_Text(buffer, 170, 130, 2, TEXT_COLOUR, BGND_COLOUR);

	if (numCells > 0) DrawCellsBarGraph();
}

void RenderMCStatus()
{
	if (displayNeedsFullRedraw) // Render static parts
	{
		displayNeedsFullRedraw = false;

		//char statusText[20];
		switch (mcStatusBytes[0] & 0x0F)
		{
			case MC600C: DrawTitlebar("MC600C Status"); break;
			case MC1000C: DrawTitlebar("MC1000C Status"); break;
			default: DrawTitlebar("(Unknown Controller)"); break;
		}		

		TFT_Text("Batt Volts", 16, 30, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Batt Amps", 170, 30, 1, LABEL_COLOUR, BGND_COLOUR);

		TFT_Text("Motor Volts", 16, 88, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Motor Amps", 170, 88, 1, LABEL_COLOUR, BGND_COLOUR);

		TFT_Text("Temp", 16, 146, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Throttle", 170, 146, 1, LABEL_COLOUR, BGND_COLOUR);
	}

	// Dynamic parts

	if (mcCanTimeout > 0) // 1 second
	{
		int battVolts = mcStatusBytes[1] + (mcStatusBytes[6]&0b10000000)*2;
		int pwm = mcStatusBytes[7];
		int motorVolts = (long)battVolts * (long)pwm / 255L;
		
		itoa(battVolts, buffer, 10); // Batt volts
		strcat(buffer, "V  ");
		TFT_Text(buffer, 16, 48, 2, TEXT_COLOUR, BGND_COLOUR);
	
		itoa(mcStatusBytes[2]*5, buffer, 10); // Batt amps
		strcat(buffer, "A  ");
		TFT_Text(buffer, 170, 48, 2, TEXT_COLOUR, BGND_COLOUR);
	
		itoa(motorVolts, buffer, 10); // Motor volts
		strcat(buffer, "V  ");
		TFT_Text(buffer, 16, 106, 2, TEXT_COLOUR, BGND_COLOUR);

		itoa(mcStatusBytes[4]*5, buffer, 10); // Motor amps
		strcat(buffer, "A  ");
		TFT_Text(buffer, 170, 106, 2, TEXT_COLOUR, BGND_COLOUR);

		WriteTemp(buffer, mcStatusBytes[5]); // Temp
		TFT_Text(buffer, 16, 164, 2, TEXT_COLOUR, BGND_COLOUR);

		itoa(mcStatusBytes[6]&0b01111111, buffer, 10); // Throttle
		strcat(buffer, "%   ");
		TFT_Text(buffer, 170, 164, 2, TEXT_COLOUR, BGND_COLOUR);

		int mcError = mcStatusBytes[0]>>4;
		unsigned short col = RED;
		if (mcError == MC_THERMAL_CUTBACK_ERROR) col = ORANGE;
		if (mcError == MC_SLEEPING) col = L_GRAY;
		if (mcError == MC_NO_ERROR) col = GREEN;

		TFT_CentredText(mcErrors[mcError], 160, 210, 1, col, BGND_COLOUR);
	}
	else // Comms error
	{
		strcpy(buffer, " -   ");
		TFT_Text(buffer, 16, 48, 2, TEXT_COLOUR, BGND_COLOUR);
		TFT_Text(buffer, 170, 48, 2, TEXT_COLOUR, BGND_COLOUR);
		TFT_Text(buffer, 16, 106, 2, TEXT_COLOUR, BGND_COLOUR);
		TFT_Text(buffer, 170, 106, 2, TEXT_COLOUR, BGND_COLOUR);
		TFT_Text(buffer, 16, 164, 2, TEXT_COLOUR, BGND_COLOUR);
		TFT_Text(buffer, 170, 164, 2, TEXT_COLOUR, BGND_COLOUR);

		TFT_CentredText("   COMMS ERROR!   ", 160, 210, 1, RED, BGND_COLOUR);
	}
}

void RenderChargerStatus()
{
	if (numChargers == 3)
	{
		RenderThreeChargerStatus();
		return;
	}
	
	if (displayNeedsFullRedraw) // Render static parts
	{
		displayNeedsFullRedraw = false;

		DrawTitlebar("TC Charger Status");		

		TFT_Text("Output Volt", 16, 40, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Output Amps", 170, 40, 1, LABEL_COLOUR, BGND_COLOUR);

		TFT_Text("Target Volt", 16, 110, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Target Amps", 170, 110, 1, LABEL_COLOUR, BGND_COLOUR);
	}

	// Dynamic parts
	if (charger[0].instVoltage > 0)
	{
		itoa(charger[0].instVoltage/10, buffer, 10); // Output volts
		strcat(buffer, "V  ");
	}
	else
		strcpy(buffer, " -   ");
	TFT_Text(buffer, 16, 60, 2, TEXT_COLOUR, BGND_COLOUR);
	
	if (charger[0].instCurrent > 0)
	{
		itoa(charger[0].instCurrent, buffer, 10); // Output amps
		AddDecimalPoint(buffer);
		strcat(buffer, "A  ");
	}
	else
		strcpy(buffer, " -    ");
	TFT_Text(buffer, 170, 60, 2, TEXT_COLOUR, BGND_COLOUR);

	itoa(charger[0].targetVoltage/10, buffer, 10); // Target volts
	strcat(buffer, "V  ");
	TFT_Text(buffer, 16, 130, 2, TEXT_COLOUR, BGND_COLOUR);

	itoa(charger[0].targetCurrent, buffer, 10); // Target amps
	AddDecimalPoint(buffer);
	strcat(buffer, "A  ");
	TFT_Text(buffer, 170, 130, 2, TEXT_COLOUR, BGND_COLOUR);

	if (chargerCommsTimeout[0] == 0)
		TFT_CentredText("No comms to charger", 160, 200, 1, RED, BGND_COLOUR);
	else if (charger[0].controlBit)
		TFT_CentredText("  Shutdown by BMS  ", 160, 200, 1, RED, BGND_COLOUR);
	else if (charger[0].statusBits & 0b00000001)
		TFT_CentredText(" Hardware failure! ", 160, 200, 1, RED, BGND_COLOUR);
	else if (charger[0].statusBits & 0b00000010)
		TFT_CentredText(" Overtemp shutdown ", 160, 200, 1, RED, BGND_COLOUR);
	else if (charger[0].statusBits & 0b00000100)
		TFT_CentredText("Input voltage error", 160, 200, 1, RED, BGND_COLOUR);
	else if (charger[0].statusBits & 0b00001000)
		TFT_CentredText("   Battery Fault   ", 160, 200, 1, RED, BGND_COLOUR);
	else if (charger[0].statusBits & 0b00010000)
		TFT_CentredText("   Comms timeout   ", 160, 200, 1, RED, BGND_COLOUR);
	else
		TFT_CentredText(" Charger status OK ", 160, 200, 1, GREEN, BGND_COLOUR);
}

void RenderThreeChargerStatus()
{
	int divisor = 1;
	if (charger[0].targetCurrent*numChargers > 1000) divisor = 10; // If dealing with 100.0A or more, drop decimal point

	if (displayNeedsFullRedraw) // Render static parts
	{
		displayNeedsFullRedraw = false;

		DrawTitlebar("Charger Status");		

		TFT_Text("Output Volts", 16, 30, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Total Amps", 170, 30, 1, LABEL_COLOUR, BGND_COLOUR);
		
		TFT_Text("Target Volts", 16, 90, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Target Amps", 170, 90, 1, LABEL_COLOUR, BGND_COLOUR);

		TFT_Text("#", 16, 150, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Volts", 60, 150, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Amps", 132, 150, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Status", 204, 150, 1, LABEL_COLOUR, BGND_COLOUR);
	}

	// Dynamic parts
	int voltage = 0;
	for (int n=0; n<numChargers; n++) if (charger[n].instVoltage > voltage) voltage = charger[n].instVoltage;
	itoa(voltage/10, buffer, 10); // Output volts
	strcat(buffer, "V  ");
	TFT_Text(buffer, 16, 50, 2, TEXT_COLOUR, BGND_COLOUR);

	int current = 0;
	for (int n=0; n<3; n++) current += charger[n].instCurrent;
	itoa(current/divisor, buffer, 10); // Output amps
	if (divisor == 1) AddDecimalPoint(buffer);
	strcat(buffer, "A  ");
	TFT_Text(buffer, 170, 50, 2, TEXT_COLOUR, BGND_COLOUR);
	
	voltage = settings[CHARGER_VOLTAGE];
	if (settings[CHARGER_CURRENT] & 0b10000000) voltage += 256;
	current = (settings[CHARGER_CURRENT]&0b01111111)*numChargers;
	
	itoa(voltage, buffer, 10); // Target volts - same for all chargers
	strcat(buffer, "V  ");
	TFT_Text(buffer, 16, 110, 2, TEXT_COLOUR, BGND_COLOUR);

	itoa(current*10/divisor, buffer, 10); // Target amps
	if (divisor == 1) AddDecimalPoint(buffer);
	strcat(buffer, "A  ");
	TFT_Text(buffer, 170, 110, 2, TEXT_COLOUR, BGND_COLOUR);

	for (int n=0; n<3; n++)
	{
		itoa(n+1, buffer, 10);
		TFT_Text(buffer, 16, 170+n*20, 1, TEXT_COLOUR, BGND_COLOUR);

		itoa(charger[n].instVoltage/10, buffer, 10); // Output volts
		strcat(buffer, "V ");
		TFT_Text(buffer, 60, 170+n*20, 1, TEXT_COLOUR, BGND_COLOUR);

		itoa(charger[n].instCurrent/divisor, buffer, 10); // Output amps
		if (divisor == 1) AddDecimalPoint(buffer);
		strcat(buffer, "A ");
		TFT_Text(buffer, 132, 170+n*20, 1, TEXT_COLOUR, BGND_COLOUR);

		if (chargerCommsTimeout[n] == 0)
			TFT_Text("No comms", 204, 170+n*20, 1, RED, BGND_COLOUR);
		else if (charger[n].controlBit)
			TFT_Text("BMS Stop", 204, 170+n*20, 1, RED, BGND_COLOUR);
		else if (charger[n].statusBits & 0b00000001)
			TFT_Text("HW Fault", 204, 170+n*20, 1, RED, BGND_COLOUR);
		else if (charger[n].statusBits & 0b00000010)
			TFT_Text("Overtemp", 204, 170+n*20, 1, RED, BGND_COLOUR);
		else if (charger[n].statusBits & 0b00000100)
			TFT_Text("AC fault", 204, 170+n*20, 1, RED, BGND_COLOUR);
		else if (charger[n].statusBits & 0b00001000)
			TFT_Text("BatError", 204, 170+n*20, 1, RED, BGND_COLOUR);
		else if (charger[n].statusBits & 0b00010000)
			TFT_Text("No comms", 204, 170+n*20, 1, RED, BGND_COLOUR);
		else
			TFT_Text("OK      ", 204, 170+n*20, 1, GREEN, BGND_COLOUR);
	}
}

void RenderBMSSummary()
{
	// Do the calculations
	unsigned short minVoltage = 5000, maxVoltage = 0;
	unsigned char minModule = 0, minCell = 0, maxModule = 0, maxCell = 0;

	long packVoltage = 0;
	for (int id=0; id<MAX_BMS_MODULES; id++)
	{
		if (bmsCellCounts[id] > 0)
		{
			for (int n=0; n<bmsCellCounts[id]; n++)
			{
				if (cellVoltages[id][n] < minVoltage)
				{
					minVoltage = cellVoltages[id][n];
					minModule = id;
					minCell = n+1;
				}
				if (cellVoltages[id][n] > maxVoltage)
				{
					maxVoltage = cellVoltages[id][n];
					maxModule = id;
					maxCell = n+1;
				}
				packVoltage += cellVoltages[id][n];
			}
		}
	}

	long avgVoltage = 0;
	if (numCells > 0) avgVoltage = (long)packVoltage/(long)numCells;

	int avgTemp = 0, numTempSensors = 0;
	for (int n=0; n<MAX_BMS_MODULES; n++)
	{
		if (bmsCellCounts[n] > 0)
		{
			for (int i=0; i<2; i++)
			{
				if (bmsTemps[n][i] > 0)
				{
					avgTemp += bmsTemps[n][i];
					numTempSensors++;
				}
			}
		}
	}
	if (numTempSensors > 1) avgTemp /= numTempSensors;

	if (displayNeedsFullRedraw)
	{
		displayNeedsFullRedraw = false;

		char stringy[4];
		itoa(numCells, stringy, 10);
		strcpy(buffer, "BMS Summary: ");
		strcat(buffer, stringy);
		strcat(buffer, " cells");

		DrawTitlebar(buffer);
		
		if (isBMS16 && settings[SHUNT_SIZE] == 0 && !haveReceivedCurrentData) 
			TFT_Text("Pack voltage", 16, 40, 1, LABEL_COLOUR, BGND_COLOUR);
		else
			TFT_Text("Avg voltage", 16, 40, 1, LABEL_COLOUR, BGND_COLOUR);
		
		
		
		if (isBMS16)
			TFT_Text("Temperature", 170, 40, 1, LABEL_COLOUR, BGND_COLOUR);
		else
			TFT_Text("Avg temp", 170, 40, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Min voltage", 16, 110, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_Text("Max voltage", 170, 110, 1, LABEL_COLOUR, BGND_COLOUR);			
	}
	
	if (isBMS16 && settings[SHUNT_SIZE] == 0 && !haveReceivedCurrentData)
	{
		itoa(packVoltage/100, buffer, 10);
		AddDecimalPoint(buffer);
	}
	else
	{
		itoa((avgVoltage+5)/10, buffer, 10); // The +5 is so it rounds not truncates
		AddDecimalPoint2(buffer);
	}
	strcat(buffer, "V ");
	TFT_Text(buffer, 16, 60, 2, TEXT_COLOUR, BGND_COLOUR);
	
	if (isBMS16 && evmsStatusBytes[7] > 0)
	{
		WriteTemp(buffer, evmsStatusBytes[7]-40);
		TFT_Text(buffer, 170, 60, 2, TEXT_COLOUR, BGND_COLOUR);
	}
	else if (numTempSensors > 0)
	{
		WriteTemp(buffer, avgTemp-40);
		TFT_Text(buffer, 170, 60, 2, TEXT_COLOUR, BGND_COLOUR);
	}
	else
		TFT_Text(" -   ", 170, 60, 2, TEXT_COLOUR, BGND_COLOUR);

	itoa((minVoltage+5)/10, buffer, 10);
	AddDecimalPoint2(buffer);
	strcat(buffer, "V");
	TFT_Text(buffer, 16, 130, 2, TEXT_COLOUR, BGND_COLOUR);

	char texty[12];
	strcpy(texty, "M");
	itoa(minModule, buffer, 10);
	strcat(texty, buffer);
	strcat(texty, " C");
	itoa(minCell, buffer, 10);
	strcat(texty, buffer);
	strcat(texty, " ");
	TFT_Text(texty, 16, 165, 1, L_GRAY, BGND_COLOUR);

	itoa((maxVoltage+5)/10, buffer, 10);
	AddDecimalPoint2(buffer);
	strcat(buffer, "V");
	TFT_Text(buffer, 170, 130, 2, TEXT_COLOUR, BGND_COLOUR);

	strcpy(texty, "M");
	itoa(maxModule, buffer, 10);
	strcat(texty, buffer);
	strcat(texty, " C");
	itoa(maxCell, buffer, 10);
	strcat(texty, buffer);
	strcat(texty, " ");
	TFT_Text(texty, 170, 165, 1, L_GRAY, BGND_COLOUR);

	DrawCellsBarGraph();
}

void RenderBMSDetails()
{
	// Calculate average cell voltage, for showing shunts
	int balanceVoltage = 5000;
	if (settings[BALANCE_VOLTAGE] < 251 && (coreStatus == CHARGING || isBMS16))
		balanceVoltage = 2000+settings[BALANCE_VOLTAGE]*10;
	else if (settings[BALANCE_VOLTAGE] == 251 && numCells > 0 && (coreStatus == CHARGING || isBMS16))
	{
		long packVoltage = 0;
		int minCellVoltage = 5000;
		int maxCellVoltage = 0;
		for (int id=0; id<MAX_BMS_MODULES; id++)
			for (int n=0; n<bmsCellCounts[id]; n++)
			{
				packVoltage += cellVoltages[id][n];
				if (cellVoltages[id][n] < minCellVoltage) minCellVoltage = cellVoltages[id][n];
				if (cellVoltages[id][n] > maxCellVoltage) maxCellVoltage = cellVoltages[id][n];
			}

		//balanceVoltage = (long)packVoltage/(long)numCells + BALANCE_TOLERANCE;
		balanceVoltage = (minCellVoltage + maxCellVoltage) / 2 + BALANCE_TOLERANCE; // Oct 2020: New balance scheme, works better for single low cells
	}

	bool fullRedraw = false;
	if (displayNeedsFullRedraw)
	{	
		displayNeedsFullRedraw = false; // Have to sort of double buffer this in case of touch interrupts while drawing
		fullRedraw = true;

		if (isBMS16)
		{
			char stringy[4];
			itoa(numCells, stringy, 10);
			strcpy(buffer, "BMS Details: ");
			strcat(buffer, stringy);
			strcat(buffer, " cells");
			DrawTitlebar(buffer);
		}
		else
		{
			DrawTitlebar("BMS Details: Module  ");
			TFT_Text("Temp1:", 12, 165, 1, LABEL_COLOUR, BGND_COLOUR);
			TFT_Text("Temp2:", 162, 165, 1, LABEL_COLOUR, BGND_COLOUR);
		}
		TFT_Text("Cell Voltages", 12, 40, 1, LABEL_COLOUR, BGND_COLOUR);
	}

	U16 col = RUNNING_COLOUR;
	if (isBMS16)
		currentBmsModule = 0;
	else
	{
		itoa(currentBmsModule, buffer, 10);
		strcat(buffer, " ");
		if (coreStatus == CHARGING) col = CHARGING_COLOUR;
		if (coreStatus == IDLE) col = L_GRAY;
		if (coreStatus == STOPPED) col = RED;
		if (settings[STATIONARY_VERSION] && (error == BMS_HIGH_WARNING || error == BMS_LOW_WARNING)) col = RED;
		TFT_Text(buffer, 274, 2, 1, BLACK, col);
	}

	// Matrix of voltages
	int max = 12;
	if (isBMS16 && !isActuallyBMS12i) max = 8;
	for (int n=0; n<max; n++)
	{
		if (n < bmsCellCounts[currentBmsModule])
		{
			itoa(cellVoltages[currentBmsModule][n], buffer, 10);
			AddDecimalPoint3(buffer);
		}
		else
			strcpy(buffer, "     "); // blanking spaces to clear any old entries
		TFT_Text(buffer, 12+75*(n&0x03), 70+30*(n/4), 1, TEXT_COLOUR, BGND_COLOUR);
	}

	if (isBMS16) // Write next 8 cells
	{
		if (!isActuallyBMS12i) // Second set of voltages only if is really BMS16
		{
			for (int n=0; n<8; n++)
			{
				if (n < bmsCellCounts[1])
				{
					itoa(cellVoltages[1][n], buffer, 10);
					AddDecimalPoint3(buffer);
				}
				else
					strcpy(buffer, "     "); // blanking spaces to clear any old entries
				TFT_Text(buffer, 12+75*(n&0x03), 130+30*(n/4), 1, TEXT_COLOUR, BGND_COLOUR);
			}
		}
		DrawCellsBarGraph();
	}
	else
	{
		if (bmsTemps[currentBmsModule][1] == 0)
			strcpy(buffer, " -   ");
		else
			WriteTemp(buffer, bmsTemps[currentBmsModule][1]-40);
		TFT_Text(buffer, 96, 165, 1, TEXT_COLOUR, BGND_COLOUR);

		if (bmsTemps[currentBmsModule][0] == 0)
			strcpy(buffer, " -   ");
		else
			WriteTemp(buffer, bmsTemps[currentBmsModule][0]-40);
		TFT_Text(buffer, 246, 165, 1, TEXT_COLOUR, BGND_COLOUR);

		for (int n=0; n<12; n++)
		{
			col = BGND_COLOUR;
			if (cellVoltages[currentBmsModule][n] > balanceVoltage) col = ORANGE; // +5 mV tolerance for balancing
			TFT_Box(12+75*(n&0x03), 88+30*(n/4), 72+75*(n&0x03), 89+30*(n/4), col);
		}
	
		RenderButton(&nextBmsModuleButton, fullRedraw);
		RenderButton(&prevBmsModuleButton, fullRedraw);
	}
}

void RenderWarningOverlay()
{
	if (displayNeedsFullRedraw)
	{
		displayNeedsFullRedraw = false;

		for (int x=0; x<320; x+=2) TFT_Box(x, 0, x, 239, D_GRAY); // Sort of grays out the background
	
		TFT_Box(20, 70, 299, 169, RED);
		TFT_Box(24, 74, 295, 165, BLACK);

		strcpy_P(buffer, (char*)pgm_read_word(&(errorStrings[error])));
	
		TFT_CentredText("Warning:", 160, 90, 1, RED, BLACK);
		TFT_CentredText(buffer, 160, 130, 1, WHITE, BLACK);
	}
}

void RenderOptionsButtons()
{
	// Overlay with button options: Reset SOC, Enter Setup, Display Off, Exit Options
	bool needsRedraw = false;
	if (displayNeedsFullRedraw)
	{
		displayNeedsFullRedraw = false; // This has to happen immediately after the "if" function in case of touch interrupts while drawing
		for (int x=0; x<320; x+=2) TFT_Box(x, 0, x, 239, D_GRAY); // Sort of grays out the background
		RenderBorderBox(40, 20, 280, 232, D_GRAY, BLACK); // Size = 240 x 160
		needsRedraw = true;
	}

	if ((coreStatus == IDLE || isBMS16) && (!CONFIG_LOCK || !CONFIG_LOCK2))
		enterSetupButton.colour = BLUE;
	else
		enterSetupButton.colour = D_GRAY;

	RenderButton(&enterSetupButton, needsRedraw);
	RenderButton(&resetSocButton, needsRedraw);
	RenderButton(&zeroCurrentButton, needsRedraw);
	RenderButton(&displayOffButton, needsRedraw);
	RenderButton(&exitOptionsButton, needsRedraw);
}

inline void RenderBorderBox(int lx, int ly, int rx, int ry, U16 Fcolor, U16 Bcolor)
{
	TFT_Box(lx, ly, rx, ry, Fcolor);
	TFT_Box(lx+2, ly+2, rx-2, ry-2, Bcolor);
}

void RenderButton(Button* button, bool needsRedraw)
{
	bool touched = ButtonTouched(button) && touchedButton == button;
	
	if ((button->isTouched && !touched) || (!(button->isTouched) && touched) || needsRedraw)
	{
		// Needs redraw if state has changed		
		U16 Bcolor = BLACK;
		if (touchX > 0 && touchY > 0 && touched) Bcolor = button->colour;
		RenderBorderBox(button->x-button->width/2, button->y, button->x+button->width/2, button->y+32, button->colour, Bcolor);
		TFT_CentredText(button->text, button->x, button->y+8, 1, WHITE, Bcolor);
		button->isTouched = touched;
	}
}

void RenderSettings()
{
	bool fullRedraw = false;
	if (displayNeedsFullRedraw)
	{
		displayNeedsFullRedraw = false;
		fullRedraw = true;
		
		if (isBMS16)
			DrawTitlebar("BMS Setup");
		else
			DrawTitlebar("EVMS: Setup");
		
		if (!isBMS16 || haveReceivedMCData) TFT_Text("<", 8, 30, 2, TEXT_COLOUR, BGND_COLOUR);
		TFT_Text("<", 8, 90, 2, TEXT_COLOUR, BGND_COLOUR);
		TFT_Text("<", 8, 150, 2, TEXT_COLOUR, BGND_COLOUR);
		if (!isBMS16 || haveReceivedMCData) TFT_Text(">", 288, 30, 2, TEXT_COLOUR, BGND_COLOUR);
		TFT_Text(">", 288, 90, 2, TEXT_COLOUR, BGND_COLOUR);
		TFT_Text(">", 288, 150, 2, TEXT_COLOUR, BGND_COLOUR);	
	}

	RenderButton(&exitSetupButton, fullRedraw);

	if (settingsPage == PACK_SETUP)
	{
		TFT_CentredText("BMS Configuration", 160, 40, 1, GREEN, BGND_COLOUR);
		TFT_CentredText("Module ID:", 160, 90, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_CentredText("Cell count:", 160, 150, 1, LABEL_COLOUR, BGND_COLOUR);

		char temp[4];
		itoa(currentBmsModule, temp, 10);
		strcpy(buffer, "          "); // Blanking spaces either side in case change of length
		strcat(buffer, temp);
		strcat(buffer, "          ");
		TFT_CentredText(buffer, 160, 110, 1, TEXT_COLOUR, BGND_COLOUR);

		itoa(bmsCellCounts[currentBmsModule], temp, 10);
		strcpy(buffer, "          ");
		strcat(buffer, temp);
		strcat(buffer, "          ");
		TFT_CentredText(buffer, 160, 170, 1, TEXT_COLOUR, BGND_COLOUR);
	}
	else if (settingsPage == MC_SETTINGS)
	{
		TFT_CentredText(" Motor Controller ", 160, 40, 1, GREEN, BGND_COLOUR);
		TFT_CentredText("Parameter:", 160, 90, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_CentredText("   Value:   ", 160, 150, 1, LABEL_COLOUR, BGND_COLOUR);

		TFT_CentredText(mcNames[mcCurrentParameter], 160, 110, 1, TEXT_COLOUR, BGND_COLOUR);

		int value = mcSettings[mcCurrentParameter];
		if (mcCurrentParameter == MC_MAX_MOTOR_CURRENT || mcCurrentParameter == MC_MAX_BATT_CURRENT || mcCurrentParameter == MC_IDLE_CURRENT)
			value *= 10;

		if (mcCurrentParameter == MC_THROTTLE_TYPE)
			strcpy(buffer, mcThrottleTypes[value]);
		else if (mcCurrentParameter == MC_SPEED_CONTROL_TYPE || mcCurrentParameter == MC_TORQUE_CONTROL_TYPE)
			strcpy(buffer, mcControlTypes[value]);
		else
		{
			char temp[20];
			itoa(value, temp, 10);
		
			strcpy(buffer, "      ");
			strcat(buffer, temp);
			strcat(buffer, mcUnits[mcCurrentParameter]);
			strcat(buffer, "      ");
		}
		TFT_CentredText(buffer, 160, 170, 1, TEXT_COLOUR, BGND_COLOUR);
	}
	else
	{
		TFT_CentredText(" General Settings ", 160, 40, 1, GREEN, BGND_COLOUR);
		TFT_CentredText("Parameter:", 160, 90, 1, LABEL_COLOUR, BGND_COLOUR);
		TFT_CentredText("   Value:   ", 160, 150, 1, LABEL_COLOUR, BGND_COLOUR);

		char temp[20];
		strcpy_P(temp, (char*)pgm_read_word(&(generalSettingsLabels[currentParameter])));
		
		// Couple of reassigned settings for BMS16
		if (isBMS16 && currentParameter == NUM_CELLS) strcpy(temp, "  Num Cells  ");
		if (isBMS16 && currentParameter == SHUNT_SIZE) strcpy(temp, "  Shunt Size  ");
		
		strcpy(buffer, " ");
		strcat(buffer, temp);
		strcat(buffer, " "); // White space to clean up previous label if longer
		TFT_CentredText(buffer, 160, 110, 1, TEXT_COLOUR, BGND_COLOUR);

		int value = 0;
		char* units = allSettingsUnits[currentParameter];
		if (maximums[currentParameter] == 1) // It's a Yes/No one
		{
			if (settings[currentParameter] == 1)
				strcpy(temp, "YES");
			else
				strcpy(temp, "NO");
		}
		else
		{
			value = settings[currentParameter];
			if (currentParameter == PACK_CAPACITY) value *= PACK_CAPACITY_MULTIPLIER;
			if (currentParameter == CURRENT_WARNING || currentParameter == CURRENT_TRIP) value *= 10;
			if (currentParameter == BMS_MIN_VOLTAGE) value += 150;
			if (currentParameter == BMS_MAX_VOLTAGE) value += 200;
			if (currentParameter == BALANCE_VOLTAGE) value += 200;
			if (currentParameter == BMS_MIN_TEMP || currentParameter == BMS_MAX_TEMP) value -= 40;
			if (currentParameter == FULL_VOLTAGE && !isBMS16) value *= 2;
			if (currentParameter == NIGHT_BRIGHTNESS) value *= 10; // converts 0-10 into 0-100%

			// Adds ninth bit to voltage display, from spare bit in current byte
			if (currentParameter == CHARGER_VOLTAGE) value += (settings[CHARGER_CURRENT]&0b10000000)*2;
			if (currentParameter == CHARGER_CURRENT) value = (value&0b01111111);
				// Only bottom 7 bits, and multiply by number of chargers (steps in 1A per charger)
			if (currentParameter == CHARGER_VOLTAGE2) value += (settings[CHARGER_CURRENT2]&0b10000000)*2;
			if (currentParameter == CHARGER_CURRENT2) value = (value&0b01111111);

			if (currentParameter == CHARGER_VOLTAGE || currentParameter == CHARGER_VOLTAGE2
				|| currentParameter == FULL_VOLTAGE) value *= PACK_VOLTAGE_MULTIPLIER;
			
			if (*units == 'C' && settings[USE_FAHRENHEIT])
			{
				value = value*9/5+32;
				units = "F";
			}

			itoa(value, temp, 10);
			if (currentParameter == BMS_MIN_VOLTAGE || currentParameter == BMS_MAX_VOLTAGE
				|| currentParameter == BMS_HYSTERESIS || currentParameter == BALANCE_VOLTAGE)
				AddDecimalPoint2(temp);
		}

		if (((currentParameter == CURRENT_WARNING || currentParameter == CURRENT_TRIP) && value > 1200)
			|| (currentParameter == FULL_VOLTAGE && value == 0) || (currentParameter == MIN_AUX_VOLTAGE && value == 0)
			|| (currentParameter == OVER_TEMP && value == 151) || (currentParameter == BMS_MIN_TEMP && value == -40)
			|| (currentParameter == BMS_MAX_TEMP && value == 101) || (currentParameter == CAN_POWER_DOWN_DELAY && value == 6)
			|| (currentParameter == BALANCE_VOLTAGE && value == 452))
		{
			strcpy(temp, "OFF");
			units = "";
		}

		if (currentParameter == BALANCE_VOLTAGE && value == 451)
		{
			strcpy(temp, "Dynamic");
			units = "";
		}

		if (currentParameter == MPI_FUNCTION) strcpy_P(temp, (char*)pgm_read_word(&(mpiStrings[value])));
		if (currentParameter == MPO1_FUNCTION || currentParameter == MPO2_FUNCTION)
			strcpy_P(temp, (char*)pgm_read_word(&(mpoStrings[value])));

		if (currentParameter == SOC_DISPLAY)
		{
			if (settings[currentParameter] == SOC_PERCENT) strcpy(temp, "Percent");
			else strcpy(temp, "Amp-hours");
		}

		if (isBMS16 && currentParameter == NUM_CELLS) units = "";
		if (isBMS16 && currentParameter == SHUNT_SIZE)
		{
			units = "";
			char* shuntStrings[4] = { "None", "100A", "200A", "500A" };
			if (value > 3) value = 3;
			strcpy(temp, shuntStrings[value]);
		}

		if (currentParameter == NUM_PARALLEL_STRINGS)
			strcpy(buffer, "     "); // Need extra space because it's so short compared with previous label
		else
			strcpy(buffer, "    ");
		strcat(buffer, temp);
		strcat(buffer, units);
		if (currentParameter == NUM_PARALLEL_STRINGS)
			strcat(buffer, "     ");
		else
			strcat(buffer, "    ");

		TFT_CentredText(buffer, 160, 170, 1, TEXT_COLOUR, BGND_COLOUR);
	}	
}
