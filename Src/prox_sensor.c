#include "prox_sensor.h"
#include "main.h"

#include "usbd_cdc_if.h"

ProxSensor_Config_T       ProxSensor_Config;
ProxSensor_CurrentState_T ProxSensor_CurrentState;

uint16_t (*ImgPtr)[CAM_IMG_WIDTH];

static float RGB565_To_GreyScale(uint16_t *pixelColor);
static void  ProxSensor_PerformOperationsOnFrame(uint32_t frameBufferAddr);

void ProxSensor_Init(uint32_t frameBufferAddr)
{
	ProxSensor_Config.algoActive = 1;
	ProxSensor_Config.floatOn = 1;
	ProxSensor_Config.halfScreenMode = 0;

	ProxSensor_Config.Grayscale_coeff_R = 0.3f;
	ProxSensor_Config.Grayscale_coeff_G = 0.6f;
	ProxSensor_Config.Grayscale_coeff_B = 0.1f;


	ProxSensor_Config.BwTh_R = 38;
	ProxSensor_Config.BwTh_G = 10;
	ProxSensor_Config.BwTh_B = 38;
}

uint8_t ProxSensor_Perform(uint32_t frameBufferAddr)
{

	if(ProxSensor_Config.usbOutOn)
	{
		CDC_Transmit_FS((uint8_t*)(frameBufferAddr) , CAM_IMG_WIDTH*40);
	}

	if(ProxSensor_Config.algoActive)
	{
		ProxSensor_PerformOperationsOnFrame(frameBufferAddr);
	}
	else
	{
		for(uint32_t y = 0; y < CAM_IMG_HEIGHT*CAM_IMG_WIDTH; ++y)
			asm("nop");
	}

	return 0;
}

void ProxSensor_PerformOperationsOnFrame(uint32_t frameBufferAddr)
{
	static uint16_t processingWidth = CAM_IMG_WIDTH;
	uint8_t pixelInGrey = 0U;

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
			pixelInGrey = RGB565_To_GreyScale(&(ImgPtr[y][x]));
			if ( (RGB565_GET_R(ImgPtr[y][x]) - pixelInGrey) > ProxSensor_Config.BwTh_R )
			{
				ImgPtr[y][x] = COLOR_RED;
				ProxSensor_CurrentState.numberOfDetectedPixels_R += 1;
			}
			else if ( (RGB565_GET_G(ImgPtr[y][x]) - pixelInGrey) > ProxSensor_Config.BwTh_G )
			{
				ImgPtr[y][x] = COLOR_GREEN;
				ProxSensor_CurrentState.numberOfDetectedPixels_G += 1;
			}
			else if ( (RGB565_GET_B(ImgPtr[y][x]) - pixelInGrey) > ProxSensor_Config.BwTh_B )
			{
				ImgPtr[y][x] = COLOR_BLUE;
				ProxSensor_CurrentState.numberOfDetectedPixels_B += 1;
			}
			else
			{
				ImgPtr[y][x] = COLOR_BLACK;
			}
	  	}
	}
}

float RGB565_To_GreyScale(uint16_t *pixelColor)
{

	HAL_GPIO_TogglePin(GPIOG, ARDUINO_D4_Pin);
	if(ProxSensor_Config.floatOn != 1)
	{
		return  RGB565_GET_R(*pixelColor) / ProxSensor_Config.Grayscale_coeff_R
		      + RGB565_GET_G(*pixelColor) / ProxSensor_Config.Grayscale_coeff_G
		      + RGB565_GET_B(*pixelColor) / ProxSensor_Config.Grayscale_coeff_B;
	}
	else
	{
		float val_r = (float) RGB565_GET_R(*pixelColor);
		float val_g = (float) RGB565_GET_G(*pixelColor);
		float val_b = (float) RGB565_GET_B(*pixelColor);

		float ret_val = val_r * (float)ProxSensor_Config.Grayscale_coeff_R
                      + val_g * (float)ProxSensor_Config.Grayscale_coeff_G
                      + val_b * (float)ProxSensor_Config.Grayscale_coeff_B;

		return ret_val;
	}
	HAL_GPIO_TogglePin(GPIOC, ARDUINO_D4_Pin);

}
