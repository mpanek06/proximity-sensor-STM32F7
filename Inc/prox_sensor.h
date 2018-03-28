/*
 * prox_sensor.c
 *
 *  Created on: Feb 15, 2018
 *      Author: Marcin Panek
 */

#ifndef PROX_SENSOR_C_
#define PROX_SENSOR_C_

#include "stdint.h"
#include "config.h"

#define RGB565_R_MSK 0b1111100000000000
#define RGB565_R_POS 11

#define RGB565_G_MSK 0b0000011111100000
#define RGB565_G_POS 5

#define RGB565_B_MSK 0b0000000000011111
#define RGB565_B_POS 0

#define RGB565_GET_R(PIXEL) (((PIXEL & RGB565_R_MSK) >> RGB565_R_POS) * 8 )
#define RGB565_GET_G(PIXEL) (((PIXEL & RGB565_G_MSK) >> RGB565_G_POS) * 4 )
#define RGB565_GET_B(PIXEL) (((PIXEL & RGB565_B_MSK) >> RGB565_B_POS) * 8 )

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xffff
#define COLOR_RED   0xf800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE  0x001F

typedef enum
{
	/* Only red objects are taken into account */
	ProxSensor_Color_R = 0,
	/* Only green objects are taken into account */
	ProxSensor_Color_G,
	/* Only blue objects are taken into account */
	ProxSensor_Color_B,
	/* Objects of all: red, green and blue colors are taken into account */
	ProxSensor_Color_RGB
}ProxSensor_Color_T;

typedef struct
{
	/* Color that is currently being detected */
	ProxSensor_Color_T detectedColor;

	/* Flag to turn on and off whole processing. */
	uint8_t algoActive;

	/* Flag to turn on and off labeling feature. */
	uint8_t labelingActive;

	/* Flag to turn on and off removing small objects from BW image. */
	uint8_t removingSmallObjectsActive;

	/* Flag to turn on and off using float numbers
	 * to calculate greyscale image. */
	uint8_t floatOn;

	/* Flag to turn on and off image output on USB. */
	uint8_t usbOutOn;

	/* Flag to turn on and off half screen mode -
	 * algorithm loop works only on half of screen width. */
	uint8_t halfScreenMode;

	/* Number of continuously occurring pixels
	 * to be detected as object for R, G and B color. */
	uint16_t minNumberOfPixels_R;
	uint16_t minNumberOfPixels_G;
	uint16_t minNumberOfPixels_B;

	/* Threshold for image binarization. */
	uint8_t BwTh_R;
	uint8_t BwTh_G;
	uint8_t BwTh_B;

	/* Threshold for image binarization. */
	float Grayscale_coeff_R;
	float Grayscale_coeff_G;
	float Grayscale_coeff_B;
}ProxSensor_Config_T;

typedef struct
{
	/* Number of pixels that share this label */
	uint16_t numberOfPixels;
	/* Min index on x axis - for bounding box */
	uint16_t x_min;
	/* Max index on x axis - for bounding box */
	uint16_t x_max;
	/* Min index on y axis - for bounding box */
	uint16_t y_min;
	/* Max index on y axis - for bounding box */
	uint16_t y_max;
}ProxSensor_LabelInfo_T;

typedef struct
{


}ProxSensor_CurrentState_T;

void    ProxSensor_Init(uint32_t frameBufferAddr);
uint8_t ProxSensor_Perform(uint32_t frameBufferAddr);

#endif /* PROX_SENSOR_C_ */
