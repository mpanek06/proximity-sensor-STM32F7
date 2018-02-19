#include "prox_sensor.h"

ProxSensor_Config_T       ProxSensor_Config;
ProxSensor_CurrentState_T ProxSensor_CurrentState;

uint16_t (*ImgPtr)[CAM_IMG_WIDTH];

static uint16_t RGB565_To_GreyScale(uint16_t *pixelColor);
static void     ProxSensor_PerformOperationsOnFrame(uint32_t frameBufferAddr);

void ProxSensor_Init(uint32_t frameBufferAddr)
{
	ProxSensor_Config.algoActive = 1;
	ProxSensor_Config.halfScreenMode = 0;

	ProxSensor_Config.Grayscale_coeff_R = 3;
	ProxSensor_Config.Grayscale_coeff_G = 2;
	ProxSensor_Config.Grayscale_coeff_B = 10;
}

uint8_t ProxSensor_Perform(uint32_t frameBufferAddr)
{

	if(ProxSensor_Config.algoActive)
	{
		ProxSensor_PerformOperationsOnFrame(frameBufferAddr);
	}

	return 0;
}

void ProxSensor_PerformOperationsOnFrame(uint32_t frameBufferAddr)
{
	static uint16_t processingWidth = CAM_IMG_WIDTH;

	ImgPtr = frameBufferAddr;

	ProxSensor_CurrentState.numberOfDetectedPixels_R = 0;
	ProxSensor_CurrentState.numberOfDetectedPixels_G = 0;
	ProxSensor_CurrentState.numberOfDetectedPixels_B = 0;

	if(ProxSensor_Config.halfScreenMode)
	{
		processingWidth = CAM_IMG_WIDTH/2;
	}
	else
	{
		processingWidth = CAM_IMG_WIDTH;
	}

	for(uint16_t y = 0; y < CAM_IMG_HEIGHT; ++y)
	{
		for(uint16_t x = 0; x < processingWidth; ++x)
	  	{
			if ( (RGB565_GET_R(ImgPtr[y][x]) - RGB565_To_GreyScale(&(ImgPtr[y][x]))) > ProxSensor_Config.BwTh_R )
			{
				ImgPtr[y][x] = COLOR_WHITE;
				ProxSensor_CurrentState.numberOfDetectedPixels_R += 1;
			}
			else
			{
				ImgPtr[y][x] = COLOR_BLACK;
			}
	  	}
	}
}

uint16_t inline RGB565_To_GreyScale(uint16_t *pixelColor)
{
	return  RGB565_GET_R(*pixelColor) / ProxSensor_Config.Grayscale_coeff_R
	      + RGB565_GET_G(*pixelColor) / ProxSensor_Config.Grayscale_coeff_G
		  + RGB565_GET_B(*pixelColor) / ProxSensor_Config.Grayscale_coeff_B;
}
