#include "font16.h"
#include "lcd_lib.h"
#include "string.h"
#include "math.h"

#include "config.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

extern uint16_t (*ImgPtr)[CAM_IMG_WIDTH];

void LCD_drawPixel(uint16_t x, uint16_t y, uint8_t layer_id)
{
	if(x < LCD_WIDTH && y < LCD_HEIGHT)
	{
		ImgPtr[y][x] = 0xffff;
	}
}

void LCD_drawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t layer_id)
{
	LCD_drawLine(x0, y0, x1, y0, layer_id);
	LCD_drawLine(x0, y1, x1, y1, layer_id);
	LCD_drawLine(x0, y0, x0, y1, layer_id);
	LCD_drawLine(x1, y0, x1, y1, layer_id);
}

void LCD_drawFilledRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t layer_id)
{
	uint8_t i = 0;
	uint8_t j = 0;

	for( i = x0; i < x1; ++i)
	{
		for( j = y0; j < y1; ++j)
		{
			LCD_drawPixel(i,j, layer_id);
		}
	}
}

void LCD_drawFilledSquare(uint16_t x_center, uint16_t y_center, uint16_t size, uint8_t layer_id)
{
	uint8_t size_2 = size/2;
	LCD_drawFilledRectangle( x_center - size_2, y_center - size_2, x_center + size_2, y_center + size_2, layer_id );
}

void LCD_drawLine_alpha(uint16_t x_0, uint16_t y_0, uint16_t length,  int16_t alpha, uint8_t layer_id)
{
	uint8_t x_1 = 0;
	uint8_t y_1 = 0;

	double cos_val = cos((double)(alpha*M_PI/180));
	double sin_val = sin((double)(alpha*M_PI/180));

	x_1 = x_0 + length * cos_val;
	y_1 = y_0 + length * sin_val;

	LCD_drawLine(x_0, y_0, x_1, y_1, layer_id );
}

void LCD_drawLine_alpha_center(uint16_t x_0, uint16_t y_0, uint16_t length,  int16_t alpha, uint8_t layer_id)
{
	LCD_drawLine_alpha(x_0, y_0, length, alpha, layer_id );
	LCD_drawLine_alpha(x_0, y_0, length, alpha-180, layer_id );
}


void LCD_drawLine(uint16_t x_0, uint16_t y_0, uint16_t x_1, uint16_t y_1, uint8_t layer_id)
{

	double a  = 0;
	double b  = 0;
	double c  = 0;
	double d  = 0;
	double y  = 0;

	c = x_1 - x_0;
	d = y_1 - y_0;


	if( 0 == c ) /* Vertical line */
	{
		for(int16_t y = y_0; y < y_1; ++y)
		{
			LCD_drawPixel(x_0, y, layer_id);
		}
	}
	else if( 0 == d ) /* Horizontal line */
	{
		for(int16_t x = x_0; x < x_1; ++x)
		{
			LCD_drawPixel(x, y_0, layer_id);
		}
	}
	else
	{
		a = d/c;
		b = y_1-a*x_1;

		for(int16_t x = MIN(x_0,x_1); x <=  MAX(x_0,x_1); ++x)
		{
			y = a*x + b;
			LCD_drawPixel(x, y, layer_id);
		}
	}

}

void LCD_putString(uint16_t x, uint16_t y, uint8_t *ptr, int8_t layer_id)
{

	uint16_t col = x;

	  while ((*ptr != 0))
	  {
	    /* Display one character on LCD */
		  LCD_putChar(col, y, *ptr, layer_id);
	    /* Decrement the column position by 16 */
		  col += 16;
	    /* Point on the next character */
	    ++ptr;
	  }

}

void LCD_putChar(uint16_t x, uint16_t y, uint8_t ASCII, int8_t layer_id)
{
	uint16_t x_pos = 0;
	uint16_t y_pos = 0;

	uint16_t *tmp_16 = (uint16_t*)(&Font16_Table[32*(ASCII-32)]);

	for(uint8_t i = 0; i<16; ++i)
	{
		x_pos = 0;
		/* Index starts from 11 because
		 * in Font16 chars are 11 bits wide. */
		for(int8_t j = 11; j>=0; --j)
		{
			if(tmp_16[i] & 1<<j)
			{
				LCD_drawPixel(x+x_pos, y+y_pos, layer_id);
			}
			++x_pos;
		}

		++y_pos;
	}
}

void LCD_clearLayer( uint8_t layer_id )
{
//	memset(frameBuffer[layer_id], 0, LCD_WIDTH*LCD_HEIGHT);
}

void LCD_setActiveLayer(uint8_t layer_id)
{
//	if(0 == layer_id)
//	{
//		LTDC_Layer1->CACR = 255;
//		LTDC_Layer2->CACR = 0;
//	}
//	else
//	{
//		LTDC_Layer1->CACR = 0;
//		LTDC_Layer2->CACR = 255;
//	}
//
//	LTDC->SRCR=2;
}

