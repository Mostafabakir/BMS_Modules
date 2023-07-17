// Touchscreen.h
// Functions for controlling 320x240 pixel TFT touchscreens (SSD1289/ILI9325/ILI9341)
// For AT90CAN64/128 microcontrollers
// By Ian Hooper (ZEVA), released under open source MIT License.

#include <avr/io.h>

#define NEW_LCD // Enable this for the new LCD with Monitor V3 PCB
#define ROTATE180 // Rotate 180 degrees (some panels have better contrast from above or below)

enum { SSD1289, ILI9325, ILI9341 };

#ifdef NEW_LCD // Newer LCD with 34-pin interface
	// TFT pins
	#define	RST			(1<<PG1)
	#define RST_PORT	PORTG
	#define CS			(1<<PD7)
	#define CS_PORT		PORTD
	#define RD			(1<<PD3)
	#define RD_PORT		PORTD
	#define WR			(1<<PG0)
	#define WR_PORT		PORTG
	#define RS			(1<<PD4)
	#define RS_PORT		PORTD

	#define DP_Lo		PORTC
	#define DP_Lo_DDR	DDRC
	#define	DP_Hi		PORTA
	#define DP_Hi_DDR	DDRA

	// Touch Screen stuff, I/O pins we need - all on PORT E
	#define T_DDR		DDRF // applied to T_CS, T_CLK, T_DIN
	#define	T_CLK		(1<<PF1)
	#define T_CLK_PORT	PORTF
	#define T_CS		(1<<PF0)
	#define T_CS_PORT	PORTF
	#define T_DIN		(1<<PF4)
	#define T_DIN_PORT	PORTF
	#define T_DOUT		(1<<PF3)
	#define T_DOUT_PIN	PINF
	#define T_IRQ		(1<<PF2)
	#define T_IRQ_PIN	PINF

#else // Older LCD with 40-pin interface
	// TFT pins
	#define	RST			(1<<PF0)
	#define RST_PORT	PORTF
	#define CS			(1<<PF1)
	#define CS_PORT		PORTF
	#define RD			(1<<PF2)
	#define RD_PORT		PORTF
	#define WR			(1<<PF3)
	#define WR_PORT		PORTF
	#define RS			(1<<PF4)
	#define RS_PORT		PORTF

	#define DP_Lo		PORTA
	#define DP_Lo_DDR	DDRA
	#define	DP_Hi		PORTC
	#define DP_Hi_DDR	DDRC

	// Touch Screen stuff, I/O pins we need - all on PORT E
	#define T_DDR		DDRE // applied to T_CS, T_CLK, T_DIN
	#define	T_CLK		(1<<PE6)
	#define T_CLK_PORT	PORTE
	#define T_CS		(1<<PE5)
	#define T_CS_PORT	PORTE
	#define T_DIN		(1<<PE4)
	#define T_DIN_PORT	PORTE
	#define T_DOUT		(1<<PE3)
	#define T_DOUT_PIN	PINE
	#define T_IRQ		(1<<PE2)
	#define T_IRQ_PIN	PINE
#endif

// Colour format is RRRRR GGGGGG BBBBB
#define BLACK 0
#define RED 63488
#define GREEN 2016
#define BLUE 31
#define WHITE 65535
#define PURPLE 61727
#define YELLOW 65504
#define ORANGE	0b1111110000000000 // R31 G16 B0
#define CYAN 2047
#define D_GRAY 21130
#define L_GRAY 31727


unsigned short TP_X, TP_Y; // Variables holding raw touch data


// TFT functions
void TFT_Init(int displayType, char swapXtouch);
void TFT_WriteCommand(unsigned int command);
void TFT_WriteData(unsigned int data);
void TFT_WriteCommandData(unsigned int command,unsigned int data);
void TFT_SetBounds(unsigned int PX1,unsigned int PY1,unsigned int PX2,unsigned int PY2);
void TFT_Fill(unsigned int color);
void TFT_Box(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2,unsigned int color);
void TFT_Dot(unsigned int x,unsigned int y,unsigned int color);
void TFT_H_Line(unsigned int x1, unsigned int x2,unsigned int y_pos,unsigned int color);
void TFT_Char(char C,unsigned int x,unsigned int y,char DimFont,unsigned int Fcolor,unsigned int Bcolor);
void TFT_Text(char* S, unsigned int x, unsigned int y, char scale, unsigned int Fcolor, unsigned int Bcolor);
void TFT_CentredText(char* S, unsigned int x, unsigned int y, char scale, unsigned int Fcolor, unsigned int Bcolor);

// Touch functions
void Touch_Init();
void Touch_Read();
char Touch_DataAvailable();
unsigned short Touch_GetX();
unsigned short Touch_GetY();
void Touch_SetPrecision(char precision);
void Touch_CalibrateRead();
void Touch_WriteData(unsigned char data);
unsigned short Touch_ReadData();


