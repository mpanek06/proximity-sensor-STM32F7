#include "prox_sensor.h"

uint16_t (*ImgPtr)[CAM_IMG_WIDTH];

void ProxSensor_Init(uint32_t frameBufferAddr)
{
	ImgPtr = frameBufferAddr;

}

uint8_t ProxSensor_Perform()
{
	ProxSensor_Console_Perform();

	for(uint16_t y = 0; y < CAM_IMG_HEIGHT; ++y)
	{
		for(uint16_t x = 0; x < CAM_IMG_WIDTH; ++x)
	  	{
			if (((ImgPtr[y][x] & RGB565_R_MSK) >> RGB565_R_POS) > 20)
	  		{
				ImgPtr[y][x] = 0xffff;
	  		}
	  	}
	}

	return 0;
}
