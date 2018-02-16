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

typedef enum
{
	ProxSensor_Color_R = 0,
	ProxSensor_Color_G,
	ProxSensor_Color_B,
	ProxSensor_Color_RGB
}ProxSensor_Color_T;

typedef struct
{
	/* Color that is currently being detected */
	ProxSensor_Color_T detectedColor;

	/* Number of continuously occurring pixels
	 * to be detected as object for R, G and B color */
	uint16_t numberOfPixels_R;
	uint16_t numberOfPixels_G;
	uint16_t numberOfPixels_B;

	/* Threshold for image binarization */
	uint8_t BwTh_R;
	uint8_t BwTh_G;
	uint8_t BwTh_B;
}ProxSensor_Config_T;

ProxSensor_Config_T ProxSensor_Config;

void ProxSensor_Init(uint32_t frameBufferAddr);
uint8_t ProxSensor_Perform();

#endif /* PROX_SENSOR_C_ */
