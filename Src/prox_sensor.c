#include "prox_sensor.h"

volatile ProxSensor_Config_T ProxSensor_Config;

uint16_t (*ImgPtr)[CAM_IMG_WIDTH];

static uint16_t RGB565_To_GreyScale(uint16_t *pixelColor);

void ProxSensor_Init(uint32_t frameBufferAddr)
{
	ImgPtr = frameBufferAddr;

	ProxSensor_Config.Grayscale_coeff_R = 0.3;
	ProxSensor_Config.Grayscale_coeff_G = 0.6;
	ProxSensor_Config.Grayscale_coeff_B = 0.1;
}

uint8_t ProxSensor_Perform()
{
	for(uint16_t y = 0; y < CAM_IMG_HEIGHT/2; ++y)
	{
		for(uint16_t x = 0; x < CAM_IMG_WIDTH/2; ++x)
	  	{
			if ( (RGB565_GET_R(ImgPtr[y][x]) - RGB565_To_GreyScale(&(ImgPtr[y][x]))) > ProxSensor_Config.BwTh_R )
			{
				ImgPtr[y][x] = COLOR_WHITE;
			}
			else
			{
				ImgPtr[y][x] = COLOR_BLACK;
			}
	  	}
	}

	return 0;
}

uint16_t inline RGB565_To_GreyScale(uint16_t *pixelColor)
{
	return  RGB565_GET_R(*pixelColor) * 0.3
	      + RGB565_GET_G(*pixelColor) * 0.6
		  + RGB565_GET_B(*pixelColor) * 0.1;
}
