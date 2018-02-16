#include "prox_sensor.h"

uint16_t (*ImgPtr)[CAM_IMG_WIDTH];

static double RGB565_To_GreyScale(uint16_t pixelColor);

void ProxSensor_Init(uint32_t frameBufferAddr)
{
	ImgPtr = frameBufferAddr;

	ProxSensor_Config.Grayscale_coeff_R = 0.45;
	ProxSensor_Config.Grayscale_coeff_G = 0.3;
	ProxSensor_Config.Grayscale_coeff_B = 0.25;

}

uint8_t ProxSensor_Perform()
{
//	ProxSensor_Console_Perform();
	uint16_t tmp = 0;
	for(uint16_t y = 0; y < CAM_IMG_HEIGHT; ++y)
	{
		for(uint16_t x = 0; x < CAM_IMG_WIDTH; ++x)
	  	{
//			if (RGB565_GET_R(ImgPtr[y][x]) - RGB565_To_GreyScale(ImgPtr[y][x]) > 15)
//			{
//				ImgPtr[y][x] = COLOR_WHITE;
//			}
//			else
//			{
//				ImgPtr[y][x] = COLOR_BLACK;
//			}

			tmp = RGB565_To_GreyScale(ImgPtr[y][x]);
			if ((RGB565_GET_R(ImgPtr[y][x]) - tmp ) > 15)
	  		{
				ImgPtr[y][x] = 0xffff;
	  		}
	  	}
	}

	return 0;
}

//TODO: pointer to pixelColor
double result;
double RGB565_To_GreyScale(uint16_t pixelColor)
{
	result =
			(double)RGB565_GET_R(pixelColor) / 2
	      + (double)RGB565_GET_G(pixelColor) / 3
		  + (double)RGB565_GET_B(pixelColor) / 4;

	return result;
}
