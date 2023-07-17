// Touchscreen.c
// Functions for controlling 320x240 pixel TFT touchscreens (SSD1289/ILI9325/ILI9341)
// For AT90CAN64/128 microcontrollers
// By Ian Hooper (ZEVA), released under open source MIT License.

#include "Touchscreen.h"

#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#include "Fonts.h"

// Private utility function
inline unsigned char ReverseByte(unsigned char x);

static int type;
static char swapX;

#define TOUCH_SCALING	1 // 7/6 was used on one of the screens to correct scaling

void TFT_Init(int displayType, char swapXtouch)
{
	type = displayType;
	swapX = swapXtouch;

#ifdef NEW_LCD
	swapX = 1;
#endif

    RD_PORT |= RD;	// TFT_RD = 1;
    RST_PORT |= RST;	// TFT_RST=1;
    _delay_ms(5);
    RST_PORT &= ~RST;	// TFT_RST=0;
    _delay_ms(15);
    RST_PORT |= RST;	// TFT_RST=1;
    _delay_ms(15);
    CS_PORT |= CS;	// TFT_CS =0;
	WR_PORT |= WR;
	_delay_ms(20);

	if (type == ILI9341)
	{

#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0D
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR    0x30
#define ILI9341_MADCTL   0x36
#define ILI9341_VSCRSADD 0x37
#define ILI9341_PIXFMT   0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1


		static const uint8_t init_commands[] = {
			4, 0xEF, 0x03, 0x80, 0x02,
			4, 0xCF, 0x00, 0XC1, 0X30,
			5, 0xED, 0x64, 0x03, 0X12, 0X81,
			4, 0xE8, 0x85, 0x00, 0x78,
			6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
			2, 0xF7, 0x20,
			3, 0xEA, 0x00, 0x00,
			2, ILI9341_PWCTR1, 0x23, // Power control
			2, ILI9341_PWCTR2, 0x10, // Power control
			3, ILI9341_VMCTR1, 0x3e, 0x28, // VCM control
			2, ILI9341_VMCTR2, 0x86, // VCM control2
			2, ILI9341_MADCTL, 0x48, // Memory Access Control
			2, ILI9341_PIXFMT, 0x55,
			3, ILI9341_FRMCTR1, 0x00, 0x18,
			4, ILI9341_DFUNCTR, 0x08, 0x82, 0x27, // Display Function Control
			2, 0xF2, 0x00, // Gamma Function Disable
			2, ILI9341_GAMMASET, 0x01, // Gamma curve selected
			16, ILI9341_GMCTRP1, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
				0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
			16, ILI9341_GMCTRN1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
				0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
			0
		};

		const uint8_t *addr = init_commands;
		while (1)
		{
			uint8_t count = *addr++;
			if (count-- == 0) break;
			TFT_WriteCommand(*addr++);
			while (count-- > 0) {
				TFT_WriteData(*addr++);
			}
		}
		
		TFT_WriteCommand(ILI9341_SLPOUT);
		_delay_ms(120);
		TFT_WriteCommand(ILI9341_DISPON);
	}
	else if (type == ILI9325)
	{
		TFT_WriteCommandData(0xE5, 0x78F0); // set SRAM internal timing
		TFT_WriteCommandData(0x01, 0x0100); // set Driver Output Control  
		TFT_WriteCommandData(0x02, 0x0200); // set 1 line inversion  
		TFT_WriteCommandData(0x03, 0x1030); // set GRAM write direction and BGR=1.  
		TFT_WriteCommandData(0x04, 0x0000); // Resize register  
		TFT_WriteCommandData(0x08, 0x0207); // set the back porch and front porch  
		TFT_WriteCommandData(0x09, 0x0000); // set non-display area refresh cycle ISC[3:0]  
		TFT_WriteCommandData(0x0A, 0x0000); // FMARK function  
		TFT_WriteCommandData(0x0C, 0x0000); // RGB interface setting  
		TFT_WriteCommandData(0x0D, 0x0000); // Frame marker Position  
		TFT_WriteCommandData(0x0F, 0x0000); // RGB interface polarity  
		//*************Power On sequence ****************  
		TFT_WriteCommandData(0x10, 0x0000); // SAP, BT[3:0], AP, DSTB, SLP, STB  
		TFT_WriteCommandData(0x11, 0x0007); // DC1[2:0], DC0[2:0], VC[2:0]  
		TFT_WriteCommandData(0x12, 0x0000); // VREG1OUT voltage  
		TFT_WriteCommandData(0x13, 0x0000); // VDV[4:0] for VCOM amplitude  
		TFT_WriteCommandData(0x07, 0x0001);  
		_delay_ms(200); // Dis-charge capacitor power voltage  
		TFT_WriteCommandData(0x10, 0x1690); // SAP, BT[3:0], AP, DSTB, SLP, STB  
		TFT_WriteCommandData(0x11, 0x0227); // Set DC1[2:0], DC0[2:0], VC[2:0]  
		_delay_ms(50); // Delay 50ms  
		TFT_WriteCommandData(0x12, 0x000D); // 0012  
		_delay_ms(50); // Delay 50ms  
		TFT_WriteCommandData(0x13, 0x1200); // VDV[4:0] for VCOM amplitude  
		TFT_WriteCommandData(0x29, 0x000A); // 04  VCM[5:0] for VCOMH  
		TFT_WriteCommandData(0x2B, 0x000D); // Set Frame Rate  
		_delay_ms(50); // Delay 50ms  
		TFT_WriteCommandData(0x20, 0x0000); // GRAM horizontal Address  
		TFT_WriteCommandData(0x21, 0x0000); // GRAM Vertical Address  
		// ----------- Adjust the Gamma Curve ----------//  
		TFT_WriteCommandData(0x30, 0x0000);  
		TFT_WriteCommandData(0x31, 0x0404);  
		TFT_WriteCommandData(0x32, 0x0003);  
		TFT_WriteCommandData(0x35, 0x0405);  
		TFT_WriteCommandData(0x36, 0x0808);  
		TFT_WriteCommandData(0x37, 0x0407);  
		TFT_WriteCommandData(0x38, 0x0303);  
		TFT_WriteCommandData(0x39, 0x0707);  
		TFT_WriteCommandData(0x3C, 0x0504);  
		TFT_WriteCommandData(0x3D, 0x0808);  
		//------------------ Set GRAM area ---------------//  
		TFT_WriteCommandData(0x50, 0x0000); // Horizontal GRAM Start Address  
		TFT_WriteCommandData(0x51, 0x00EF); // Horizontal GRAM End Address  
		TFT_WriteCommandData(0x52, 0x0000); // Vertical GRAM Start Address  
		TFT_WriteCommandData(0x53, 0x013F); // Vertical GRAM Start Address  
		TFT_WriteCommandData(0x60, 0xA700); // Gate Scan Line  
		TFT_WriteCommandData(0x61, 0x0001); // NDL,VLE, REV   
		TFT_WriteCommandData(0x6A, 0x0000); // set scrolling line  
		//-------------- Partial Display Control ---------//  
		TFT_WriteCommandData(0x80, 0x0000);  
		TFT_WriteCommandData(0x81, 0x0000);  
		TFT_WriteCommandData(0x82, 0x0000);  
		TFT_WriteCommandData(0x83, 0x0000);  
		TFT_WriteCommandData(0x84, 0x0000);  
		TFT_WriteCommandData(0x85, 0x0000);  
		//-------------- Panel Control -------------------//  
		TFT_WriteCommandData(0x90, 0x0010);  
		TFT_WriteCommandData(0x92, 0x0000);  
		TFT_WriteCommandData(0x07, 0x0133); // 262K color and display ON        
	}
	else // SSD 1289
	{
		TFT_WriteCommandData(0x00,0x0001);
		TFT_WriteCommandData(0x03,0xA8A4);
		TFT_WriteCommandData(0x0C,0x0000);
		TFT_WriteCommandData(0x0D,0x080C);
		TFT_WriteCommandData(0x0E,0x2B00);
		TFT_WriteCommandData(0x1E,0x00B7);
		TFT_WriteCommandData(0x01,0x2B3F);
		TFT_WriteCommandData(0x02,0x0600);
		TFT_WriteCommandData(0x10,0x0000);
		TFT_WriteCommandData(0x11,0x6070);
		TFT_WriteCommandData(0x05,0x0000);
		TFT_WriteCommandData(0x06,0x0000);
		TFT_WriteCommandData(0x16,0xEF1C);
		TFT_WriteCommandData(0x17,0x0003);
		TFT_WriteCommandData(0x07,0x0233);
		TFT_WriteCommandData(0x0B,0x0000);
		TFT_WriteCommandData(0x0F,0x0000);
		TFT_WriteCommandData(0x41,0x0000);
		TFT_WriteCommandData(0x42,0x0000);
		TFT_WriteCommandData(0x48,0x0000);
		TFT_WriteCommandData(0x49,0x013F);
		TFT_WriteCommandData(0x4A,0x0000);
		TFT_WriteCommandData(0x4B,0x0000);
		TFT_WriteCommandData(0x44,0xEF00);
		TFT_WriteCommandData(0x45,0x0000);
		TFT_WriteCommandData(0x46,0x013F);
		TFT_WriteCommandData(0x30,0x0707);
		TFT_WriteCommandData(0x31,0x0204);
		TFT_WriteCommandData(0x32,0x0204);
		TFT_WriteCommandData(0x33,0x0502);
		TFT_WriteCommandData(0x34,0x0507);
		TFT_WriteCommandData(0x35,0x0204);
		TFT_WriteCommandData(0x36,0x0204);
		TFT_WriteCommandData(0x37,0x0502);
		TFT_WriteCommandData(0x3A,0x0302);
		TFT_WriteCommandData(0x3B,0x0302);
		TFT_WriteCommandData(0x23,0x0000);
		TFT_WriteCommandData(0x24,0x0000);
		TFT_WriteCommandData(0x25,0x8000);
		TFT_WriteCommandData(0x4f,0x0000);
		TFT_WriteCommandData(0x4e,0x0000);
		TFT_WriteCommand(0x22);   
	}

    CS_PORT |= CS;	// TFT_CS =1;
}

void TFT_WriteCommand(unsigned int command)
{
    RS_PORT &= ~RS;		// TFT_RS = 0;
	CS_PORT &= ~CS;
#ifdef NEW_LCD
	DP_Hi = ReverseByte(command>>8);// TFT_DP_Hi = wcommand >> 8;
#else
	DP_Hi = (command>>8);// TFT_DP_Hi = wcommand >> 8;
#endif
    DP_Lo = command;	// TFT_DP_Lo = wcommand ;
    WR_PORT &= ~WR;		// TFT_WR = 0;
    WR_PORT |= WR;		// TFT_WR = 1;
	CS_PORT |= CS;
}

void TFT_WriteData(unsigned int data)
{
    RS_PORT |= RS;		// TFT_RS = 1 ;
	CS_PORT &= ~CS;
#ifdef NEW_LCD
	DP_Hi = ReverseByte(data>>8);	// TFT_DP_Hi = Wdata >>8 ;
#else
	DP_Hi = (data>>8);	// TFT_DP_Hi = Wdata >>8 ;
#endif
    DP_Lo = data;	// TFT_DP_Lo = wdata;
	WR_PORT &= ~WR;		// TFT_WR = 0;
    WR_PORT |= WR;		// TFT_WR = 1 ;
	CS_PORT |= CS;
}

void TFT_WriteCommandData(unsigned int command,unsigned int data)
{
    TFT_WriteCommand(command);
    TFT_WriteData(data);
}

void TFT_SetBounds(unsigned int PX1,unsigned int PY1,unsigned int PX2,unsigned int PY2)
{
	// We're using landscape so have to swap some things around
	unsigned int temp;
	temp = PX1; PX1 = PY1; PY1 = temp;
	temp = PX2; PX2 = PY2; PY2 = temp;	

#ifdef ROTATE180 // Then it's like the old panel.. swap Y (which is actually X because we're landscape)
	PY1 = 320-PY1;
	PY2 = 320-PY2;
	temp = PY1; PY1 = PY2; PY2 = temp;
#elif defined NEW_LCD
	PX1 = 239-PX1;
	PX2 = 239-PX2;
	temp = PX1; PX1 = PX2; PX2 = temp;
#else
	PY1 = 320-PY1;
	PY2 = 320-PY2;
	temp = PY1; PY1 = PY2; PY2 = temp;
#endif

	if (type == ILI9341)
	{
		TFT_WriteCommand(ILI9341_CASET);
		TFT_WriteData(PX1 >> 8);
		TFT_WriteData(PX1 & 0xFF);
		TFT_WriteData(PX2 >> 8);
		TFT_WriteData(PX2 & 0xFF);

		TFT_WriteCommand(ILI9341_PASET);
		TFT_WriteData(PY1 >> 8);
		TFT_WriteData(PY1 & 0xFF);
		TFT_WriteData(PY2 >> 8);
		TFT_WriteData(PY2 & 0xFF);

		TFT_WriteCommand(ILI9341_RAMWR);

	}
	else if (type == ILI9325)
	{
		TFT_WriteCommandData(0x20,PX1);
		TFT_WriteCommandData(0x21,PY1);
		TFT_WriteCommandData(0x50,PX1);
		TFT_WriteCommandData(0x52,PY1);
		TFT_WriteCommandData(0x51,PX2);
		TFT_WriteCommandData(0x53,PY2);
		TFT_WriteCommand(0x22); 
	}
	else // SSD1289
	{
		TFT_WriteCommandData(0x44,(PX2<<8)+PX1);
		TFT_WriteCommandData(0x45,PY1);
		TFT_WriteCommandData(0x46,PY2);
		TFT_WriteCommandData(0x4e,PX1);
		TFT_WriteCommandData(0x4f,PY1);
		TFT_WriteCommand(0x22); 
	}
}

void TFT_Fill(unsigned int color)
{
    TFT_Box(0, 0, 320, 239, color);
}

void TFT_Box(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2,unsigned int color)
{
    unsigned int  i,j;
    TFT_SetBounds(x1,y1,x2,y2);
    TFT_WriteData(color);
    CS_PORT &= ~CS;	// TFT_CS  = 0;
    for(i = y1; i <= y2; i++)
    {
        for(j = x1; j <= x2; j++)
        {
            WR_PORT &= ~WR;	// TFT_WR = 0;
            WR_PORT |= WR;	// TFT_WR = 1;
        }
    }
    CS_PORT |= CS;	// TFT_CS  = 1;
}

void TFT_H_Line(unsigned int x1, unsigned int x2, unsigned int y_pos,unsigned int color)
{
    TFT_Box(x1,y_pos,x2,y_pos,color);
}

void TFT_V_Line(unsigned int y1,unsigned int y2,char x_pos,unsigned int color)
{
    TFT_Box(x_pos,y1,x_pos+1,y2,color);
}

void TFT_Rectangle(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2,unsigned int color)
{
    TFT_H_Line(x1,x2,y1,color);
    TFT_H_Line(x1,x2,y2,color);
    TFT_V_Line(y1,y2,x1,color);
    TFT_V_Line(y1,y2,x2,color);
}

void TFT_Char(char c,unsigned int x,unsigned int y, char scale,unsigned int Fcolor,unsigned int Bcolor)
{
	if (x < 0 || y < 0 || x > 320 - 12*scale || y > 240 - 16*scale) return; // Ignore if the character is off screen

   	unsigned char i,ch;
	unsigned short j;
	unsigned short temp; 

	CS_PORT &= ~CS;

	temp = (c-32)*32; // 32 bytes per char

	unsigned short col;
	
	for(j=0; j<32; j+=2) // 32 bytes in char, row stride size of 2 bytes
	{
		for (int b=0; b<scale; b++)
		{
			TFT_SetBounds(x, y+j*scale/2+b, x+28*scale/2-1, y+j*scale/2+b); // This makes a long thin box which we write the row into

#ifdef ROTATE180
			for (int zz=1; zz >= 0; zz--)
#elif defined NEW_LCD
			for (int zz=0; zz <= 1; zz++)
#else
			for (int zz=1; zz >= 0; zz--)
#endif
			{
				ch=pgm_read_byte(&FONT_16x16[temp+zz]);

#ifdef ROTATE180
				// Don't reverse byte
#elif defined NEW_LCD
				ch = ReverseByte(ch);
#endif	
				for(i=0; i<6; i++)
				{   
					char n = i;
#ifdef ROTATE180
					if (zz != 0) n += 2; // Original version
#elif defined NEW_LCD
					if (zz == 0) n += 2;
#else
					if (zz != 0) n += 2;
#endif		
					if((ch&(1<<n)) != 0)
						col = Fcolor;
					else
						col = Bcolor;

					for (int k=0; k<scale; k++) TFT_WriteData(col);
				}
			}
		}
		temp += 2;
	}

	CS_PORT |= CS;
}


void TFT_Text(char* string, unsigned int x, unsigned int y, char scale, unsigned int Fcolor, unsigned int Bcolor)
{
    int length = strlen(string);
	for(int c=0; c<length; c++)
    {
        TFT_Char(string[c], x, y, scale, Fcolor, Bcolor);
        x = x + 12*scale;
    }
}

void TFT_CentredText(char* S, unsigned int x, unsigned int y, char scale, unsigned int Fcolor, unsigned int Bcolor)
{
	int pixelsWide = strlen(S) * 12 * scale;
	TFT_Text(S, x - pixelsWide/2, y, scale, Fcolor, Bcolor);
}


// Touch screen stuff
void Touch_Init()
{
	T_DDR  |= T_CS + T_CLK + T_DIN;
		

	T_CS_PORT |= T_CS;
	T_CLK_PORT |= T_CLK;
	T_DIN_PORT |= T_DIN;
}

void Touch_Read()
{
	T_CS_PORT &= ~T_CS; //cbi(P_CS, B_CS);                    

	int tempx=0, tempy=0, x=0, y=0, samples=0;
	for (int i=0; i<5; i++)
	{

		if (!(T_IRQ_PIN & T_IRQ)) // T_IRQ_PIN was PINE
		{
			Touch_WriteData(0x90);
			T_CLK_PORT |= T_CLK;
			T_CLK_PORT &= ~T_CLK;
			tempx = Touch_ReadData();
		}

		if (!(T_IRQ_PIN & T_IRQ))
		{
			Touch_WriteData(0xD0);
			T_CLK_PORT |= T_CLK;
			T_CLK_PORT &= ~T_CLK;
			tempy = Touch_ReadData();
		}

		if (tempx > 0 && tempx < 4096 && tempy > 0 && tempy < 4096) // Valid sample
		{
			x += tempx;
			y += tempy;
			samples++;
		}
	}

	if (samples > 0)
	{
		TP_X = x/samples;
		TP_Y = y/samples;
	}
	else
	{
		TP_X = -1;
		TP_Y = -1;
	}

	T_CS_PORT |= T_CS; //sbi(P_CS, B_CS);
}

char Touch_DataAvailable()
{
	return !(T_IRQ_PIN & T_IRQ);//avail;
}

unsigned short Touch_GetX()
{
	// Raw X scale goes approx 3950 to 150, i.e range 3800
	if (TP_X < 150) TP_X = 150;		// Cap values
	if (TP_X > 3950) TP_X = 3950;
	int x;
	if (swapX)
		x = (3950-TP_X) * 8 / 95; // Scales 0-3800 down to 0-320
	else
		x = (TP_X-150) * 8 / 95;

	x = 160 + (x-160)*TOUCH_SCALING;

#ifdef ROTATE180
	return 319-x;
#else
	return x;
#endif
}

unsigned short Touch_GetY()
{
	// Raw scale for Y goes ~200 (top) to ~3900 (bottom), i.e range 3700
	if (TP_Y < 200) TP_Y = 200;
	if (TP_Y > 3900) TP_Y = 3900;
	int y = (TP_Y-200) * 6 / 92; // close to * 320 / 3700

	y = 120 + (y-120)*TOUCH_SCALING;

#ifdef ROTATE180
	return 239-y;
#else
	return y;
#endif
}

void Touch_WriteData(unsigned char data)
{
	T_CLK_PORT &= ~T_CLK; //cbi(P_CLK, B_CLK);

	for (char count=0; count<8; count++)
	{
		if(data & 0x80)
			T_DIN_PORT |= T_DIN; //sbi(P_DIN, B_DIN);
		else
			T_DIN_PORT &= ~T_DIN; //cbi(P_DIN, B_DIN);
		data = data<<1; 
		T_CLK_PORT |= T_CLK; //cbi(P_CLK, B_CLK);                
		T_CLK_PORT &= ~T_CLK; //sbi(P_CLK, B_CLK);
	}
}

unsigned short Touch_ReadData()
{
	unsigned short data = 0;

	for (char bit=0; bit<12; bit++)
	{
		data = data<<1;
		T_CLK_PORT |= T_CLK; //sbi(P_CLK, B_CLK);
		T_CLK_PORT &= ~T_CLK; //cbi(P_CLK, B_CLK);                
		if (T_DOUT_PIN & T_DOUT)//rbi(P_DOUT, B_DOUT)) // T_DOUT_PORT was PINE
			data++;
	}

	return data;
}

inline unsigned char ReverseByte(unsigned char x)
{
    static const unsigned char reverso[] = {
        0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
        0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
        0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
        0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
        0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
        0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
        0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
        0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
        0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
        0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
        0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
        0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
        0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
        0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
        0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
        0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
        0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
        0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
        0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
        0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
        0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
        0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
        0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
        0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
        0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
        0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
        0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
        0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
        0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
        0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
        0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
        0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
    };
    return reverso[x];
}
