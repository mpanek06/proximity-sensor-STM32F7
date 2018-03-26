/*
 * lcd_lib.h
 *
 *  Created on: Mar 26, 2018
 *      Author: Marcin Panek
 */

#ifndef LCD_LIB_H_
#define LCD_LIB_H_

#include <stdint.h>

void LCD_drawPixel(uint16_t x, uint16_t y, uint8_t layer_id);
void LCD_drawRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t layer_id);
void LCD_drawFilledRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t layer_id);
void LCD_drawFilledSquare(uint16_t x_center, uint16_t y_center, uint16_t size, uint8_t layer_id);
void LCD_drawLine(uint16_t x_0, uint16_t y_0, uint16_t x_1, uint16_t y_1, uint8_t layer_id);
void LCD_drawLine_alpha(uint16_t x_0, uint16_t y_0, uint16_t length,  int16_t alpha, uint8_t layer_id);
void LCD_drawLine_alpha_center(uint16_t x_0, uint16_t y_0, uint16_t length,  int16_t alpha, uint8_t layer_id);
void LCD_putString(uint16_t x, uint16_t y, uint8_t *ptr, int8_t layer_id);
void LCD_putChar(uint16_t x, uint16_t y, uint8_t ASCII, int8_t layer_id);
void LCD_clearLayer(uint8_t layer_id);
void LCD_setActiveLayer(uint8_t layer_id);


#endif /* LCD_LIB_H_ */
