/*
 * ILI9341.h
 *
 *  Created on: Nov 11, 2023
 *      Author: fahmad
 */

#include "ILI9341.h"
#include "LCD_Pins.h"

void static setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void static pushColor(uint16_t color);
static void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
static void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);

static uint8_t rotationNum = 1;
static uint8_t ColStart, RowStart;

/**
 * initialize the TFT
 */
void ILI9341_Init()
{
	/* display off */
	LCD_Write_Cmd(ILI9341_DISPLAY_OFF);

	/* power control */
	LCD_Write_Cmd(ILI9341_POWER1); // power control
	LCD_Write_Data(0x26);		   // GVDD = 4.75v
	LCD_Write_Cmd(ILI9341_POWER2); // power control
	LCD_Write_Data(0x11);		   // AVDD=VCIx2, VGH=VCIx7, VGL=-VCIx3

	/* VCOM */
	LCD_Write_Cmd(ILI9341_VCOM1); // vcom control
	LCD_Write_Data(0x35);		  // Set the VCOMH voltage (0x35 = 4.025v)
	LCD_Write_Data(0x3e);		  // Set the VCOML voltage (0x3E = -0.950v)
	LCD_Write_Cmd(ILI9341_VCOM2); // vcom control
	LCD_Write_Data(0xbe);

	/* memory access control */
	LCD_Write_Cmd(ILI9341_MAC); // memory access control
	LCD_Write_Data(0x48);

	LCD_Write_Cmd(ILI9341_PIXEL_FORMAT); // pixel format set
	LCD_Write_Data(0x55);				 // 16bit /pixel

	LCD_Write_Cmd(ILI9341_FRC);
	LCD_Write_Data(0);
	LCD_Write_Data(0x1F);

	/* ddram */
	LCD_Write_Cmd(ILI9341_COLUMN_ADDR); // column set
	LCD_Write_Data(0x00);				// x0_HIGH---0
	LCD_Write_Data(0x00);				// x0_LOW----0
	LCD_Write_Data(0x00);				// x1_HIGH---240
	LCD_Write_Data(0x1D);				// x1_LOW----240
	LCD_Write_Cmd(ILI9341_PAGE_ADDR);	// page address set
	LCD_Write_Data(0x00);				// y0_HIGH---0
	LCD_Write_Data(0x00);				// y0_LOW----0
	LCD_Write_Data(0x00);				// y1_HIGH---320
	LCD_Write_Data(0x27);				// y1_LOW----320

	LCD_Write_Cmd(ILI9341_TEARING_OFF); // tearing effect off
	// LCD_write_cmd(ILI9341_TEARING_ON); // tearing effect on
	// LCD_write_cmd(ILI9341_DISPLAY_INVERSION); // display inversion
	LCD_Write_Cmd(ILI9341_Entry_Mode_Set); // entry mode set
	// Deep Standby Mode: OFF
	// Set the output level of gate driver G1-G320: Normal display
	// Low voltage detection: Disable
	LCD_Write_Data(0x07);

	/* display */
	LCD_Write_Cmd(ILI9341_DFC); // display function control
	// Set the scan mode in non-display area
	// Determine source/VCOM output in a non-display area in the partial display mode
	LCD_Write_Data(0x0a);
	// Select whether the liquid crystal type is normally white type or normally black type
	// Sets the direction of scan by the gate driver in the range determined by SCN and NL
	// Select the shift direction of outputs from the source driver
	// Sets the gate driver pin arrangement in combination with the GS bit to select the optimal scan mode for the module
	// Specify the scan cycle interval of gate driver in non-display area when PTG to select interval scan
	LCD_Write_Data(0x82);
	// Sets the number of lines to drive the LCD at an interval of 8 lines
	LCD_Write_Data(0x27);
	LCD_Write_Data(0x00); // clock divisor

	LCD_Write_Cmd(ILI9341_SLEEP_OUT); // sleep out
	delay(100);
	LCD_Write_Cmd(ILI9341_DISPLAY_ON); // display on
	delay(100);
	LCD_Write_Cmd(ILI9341_GRAM); // memory write
	delay(5);
}

// set the address of the pixel in the memory
void static setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	LCD_Write_Cmd(0x2A); // Column addr set
	LCD_Write_Data(0x00);
	LCD_Write_Data(x0 + ColStart); // XSTART
	LCD_Write_Data(0x00);
	LCD_Write_Data(x1 + ColStart); // XEND

	LCD_Write_Cmd(0x2B); // Row addr set
	LCD_Write_Data(0x00);
	LCD_Write_Data(y0 + RowStart); // YSTART
	LCD_Write_Data(0x00);
	LCD_Write_Data(y1 + RowStart); // YEND

	LCD_Write_Cmd(0x2C); // write to RAM
}

// internally needed function to push 16-bit color as 2 8-bit data
void static pushColor(uint16_t color)
{
	LCD_Write_Data((uint8_t)(color >> 8));
	LCD_Write_Data((uint8_t)color);
}

// set cursor function
void ILI9341_SetCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	LCD_Write_Cmd(ILI9341_COLUMN_ADDR);
	LCD_Write_Data(x1 >> 8);
	LCD_Write_Data(x1 & 0xFF);
	LCD_Write_Data(x2 >> 8);
	LCD_Write_Data(x2 & 0xFF);
	LCD_Write_Cmd(ILI9341_PAGE_ADDR);
	LCD_Write_Data(y1 >> 8);
	LCD_Write_Data(y1 & 0xFF);
	LCD_Write_Data(y2 >> 8);
	LCD_Write_Data(y2 & 0xFF);
	LCD_Write_Cmd(ILI9341_GRAM);
}

void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	ILI9341_SetCursorPosition(x, y, x, y);
	LCD_Write_Data(color >> 8);
	LCD_Write_Data(color & 0xFF);
}

void ILI9341_Fill(uint16_t color)
{
	uint32_t n = ILI9341_PIXEL_COUNT;
	//uint16_t myColor = 0xFF;

	if (rotationNum == 1 || rotationNum == 3)
	{
		ILI9341_SetCursorPosition(0, 0, ILI9341_WIDTH - 1, ILI9341_HEIGHT - 1);
	}
	else if (rotationNum == 2 || rotationNum == 4)
	{
		ILI9341_SetCursorPosition(0, 0, ILI9341_HEIGHT - 1, ILI9341_WIDTH - 1);
	}

	while (n)
	{
		n--;
		LCD_Write_Data(color >> 8);
		LCD_Write_Data(color & 0xff);
	}
}

void ILI9341_Fill_Rect(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, uint16_t color)
{
	uint32_t n = ((x1 + 1) - x0) * ((y1 + 1) - y0);
	if (n > ILI9341_PIXEL_COUNT)
		n = ILI9341_PIXEL_COUNT;
	ILI9341_SetCursorPosition(x0, y0, x1, y1);
	while (n)
	{
		n--;
		LCD_Write_Data(color >> 8);
		LCD_Write_Data(color & 0xff);
	}
}

void ILI9341_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	ILI9341_DrawPixel(x0, y0 + r, color);
	ILI9341_DrawPixel(x0, y0 - r, color);
	ILI9341_DrawPixel(x0 + r, y0, color);
	ILI9341_DrawPixel(x0 - r, y0, color);

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ILI9341_DrawPixel(x0 + x, y0 + y, color);
		ILI9341_DrawPixel(x0 - x, y0 + y, color);
		ILI9341_DrawPixel(x0 + x, y0 - y, color);
		ILI9341_DrawPixel(x0 - x, y0 - y, color);
		ILI9341_DrawPixel(x0 + y, y0 + x, color);
		ILI9341_DrawPixel(x0 - y, y0 + x, color);
		ILI9341_DrawPixel(x0 + y, y0 - x, color);
		ILI9341_DrawPixel(x0 - y, y0 - x, color);
	}
}
static void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		if (cornername & 0x4)
		{
			ILI9341_DrawPixel(x0 + x, y0 + y, color);
			ILI9341_DrawPixel(x0 + y, y0 + x, color);
		}
		if (cornername & 0x2)
		{
			ILI9341_DrawPixel(x0 + x, y0 - y, color);
			ILI9341_DrawPixel(x0 + y, y0 - x, color);
		}
		if (cornername & 0x8)
		{
			ILI9341_DrawPixel(x0 - y, y0 + x, color);
			ILI9341_DrawPixel(x0 - x, y0 + y, color);
		}
		if (cornername & 0x1)
		{
			ILI9341_DrawPixel(x0 - y, y0 - x, color);
			ILI9341_DrawPixel(x0 - x, y0 - y, color);
		}
	}
}
static void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		if (cornername & 0x1)
		{
			ILI9341_drawFastVLine(x0 + x, y0 - y, 2 * y + 1 + delta, color);
			ILI9341_drawFastVLine(x0 + y, y0 - x, 2 * x + 1 + delta, color);
		}
		if (cornername & 0x2)
		{
			ILI9341_drawFastVLine(x0 - x, y0 - y, 2 * y + 1 + delta, color);
			ILI9341_drawFastVLine(x0 - y, y0 - x, 2 * x + 1 + delta, color);
		}
	}
}
void ILI9341_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	ILI9341_drawFastVLine(x0, y0 - r, 2 * r + 1, color);
	fillCircleHelper(x0, y0, r, 3, 0, color);
}
void ILI9341_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep)
	{
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1)
	{
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1)
	{
		ystep = 1;
	}
	else
	{
		ystep = -1;
	}

	for (; x0 <= x1; x0++)
	{
		if (steep)
		{
			ILI9341_DrawPixel(y0, x0, color);
		}
		else
		{
			ILI9341_DrawPixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0)
		{
			y0 += ystep;
			err += dx;
		}
	}
}

void ILI9341_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	ILI9341_drawLine(x, y, x + w - 1, y, color);
}

void ILI9341_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	ILI9341_drawLine(x, y, x, y + h - 1, color);
}
// 10. Triangle drawing
void ILI9341_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
	ILI9341_drawLine(x0, y0, x1, y1, color);
	ILI9341_drawLine(x1, y1, x2, y2, color);
	ILI9341_drawLine(x2, y2, x0, y0, color);
}

void ILI9341_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
	int16_t a, b, y, last;

	// Sort coordinates by Y order (y2 >= y1 >= y0)
	if (y0 > y1)
	{
		swap(y0, y1);
		swap(x0, x1);
	}
	if (y1 > y2)
	{
		swap(y2, y1);
		swap(x2, x1);
	}
	if (y0 > y1)
	{
		swap(y0, y1);
		swap(x0, x1);
	}

	if (y0 == y2)
	{ // Handle awkward all-on-same-line case as its own thing
		a = b = x0;
		if (x1 < a)
			a = x1;
		else if (x1 > b)
			b = x1;
		if (x2 < a)
			a = x2;
		else if (x2 > b)
			b = x2;
		ILI9341_drawFastHLine(a, y0, b - a + 1, color);
		return;
	}

	int16_t
		dx01 = x1 - x0,
		dy01 = y1 - y0,
		dx02 = x2 - x0,
		dy02 = y2 - y0,
		dx12 = x2 - x1,
		dy12 = y2 - y1,
		sa = 0,
		sb = 0;

	// For upper part of triangle, find scanline crossings for segments
	// 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
	// is included here (and second loop will be skipped, avoiding a /0
	// error there), otherwise scanline y1 is skipped here and handled
	// in the second loop...which also avoids a /0 error here if y0=y1
	// (flat-topped triangle).
	if (y1 == y2)
		last = y1; // Include y1 scanline
	else
		last = y1 - 1; // Skip it

	for (y = y0; y <= last; y++)
	{
		a = x0 + sa / dy01;
		b = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;

		if (a > b)
			swap(a, b);
		ILI9341_drawFastHLine(a, y, b - a + 1, color);
	}

	// For lower part of triangle, find scanline crossings for segments
	// 0-2 and 1-2.  This loop is skipped if y1=y2.
	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);
	for (; y <= y2; y++)
	{
		a = x1 + sa / dy12;
		b = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;

		if (a > b)
			swap(a, b);
		ILI9341_drawFastHLine(a, y, b - a + 1, color);
	}
}

void ILI9341_setRotation(uint8_t rotate)
{
	switch (rotate)
	{
	case 1:
		rotationNum = 1;
		LCD_Write_Cmd(ILI9341_MEMCONTROL);
		LCD_Write_Data(ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
		break;
	case 2:
		rotationNum = 2;
		LCD_Write_Cmd(ILI9341_MEMCONTROL);
		LCD_Write_Data(ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
		break;
	case 3:
		rotationNum = 3;
		LCD_Write_Cmd(ILI9341_MEMCONTROL);
		LCD_Write_Data(ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR);
		break;
	case 4:
		rotationNum = 4;
		LCD_Write_Cmd(ILI9341_MEMCONTROL);
		LCD_Write_Data(ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
		break;
	default:
		rotationNum = 1;
		LCD_Write_Cmd(ILI9341_MEMCONTROL);
		LCD_Write_Data(ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
		break;
	}
}

void ILI9341_DrawChar(int16_t x, int16_t y, char c, int16_t textColor, int16_t bgColor, uint8_t size)
{
	uint8_t line;								  // horizontal row of pixels of character
	int32_t col, row, i, j;						  // loop indices
	if (((x + 5 * size - 1) >= ILI9341_WIDTH) ||  // Clip right
		((y + 8 * size - 1) >= ILI9341_HEIGHT) || // Clip bottom
		((x + 5 * size - 1) < 0) ||				  // Clip left
		((y + 8 * size - 1) < 0))
	{ // Clip top
		return;
	}

	setAddrWindow(x, y, x + 6 * size - 1, y + 8 * size - 1);

	line = 0x01; // print the top row first
	// print the rows, starting at the top
	for (row = 0; row < 8; row = row + 1)
	{
		for (i = 0; i < size; i = i + 1)
		{
			// print the columns, starting on the left
			for (col = 0; col < 5; col = col + 1)
			{
				if (Font[(c * 5) + col] & line)
				{
					// bit is set in Font, print pixel(s) in text color
					for (j = 0; j < size; j = j + 1)
					{
						pushColor(textColor);
					}
				}
				else
				{
					// bit is cleared in Font, print pixel(s) in background color
					for (j = 0; j < size; j = j + 1)
					{
						pushColor(bgColor);
					}
				}
			}
			// print blank column(s) to the right of character
			for (j = 0; j < size; j = j + 1)
			{
				pushColor(bgColor);
			}
		}
		line = line << 1; // move up to the next row
	}
}

uint16_t ILI9341_Color565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((b & 0xF8) << 8) | ((g & 0xFC) << 3) | (r >> 3);
}

uint32_t ILI9341_DrawString(uint16_t x, uint16_t y, char *pt, int16_t textColor, int16_t BG, uint16_t size)
{
	uint32_t count = 0;

	while (*pt != '\0')
	{

		ILI9341_DrawChar(x * 6 * size, y * 10 * size, *pt, textColor, BG, size);
		pt++;
		x = x + 1;
		count++;
	}
	return count; // number of characters printed
}
