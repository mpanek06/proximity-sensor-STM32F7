#include "prox_sensor.h"
#include "prox_sensor_console.h"
#include "main.h"

#include "usbd_cdc_if.h"

#define  NO_LABEL          0
#define  MAX_NUM_OF_LABELS 2500

ProxSensor_Config_T       ProxSensor_Config;
ProxSensor_CurrentState_T ProxSensor_CurrentState;

uint16_t (*ImgPtr)[CAM_IMG_WIDTH];
uint8_t  workingFrameArray[CAM_IMG_HEIGHT][CAM_IMG_WIDTH] = {0};
uint8_t  numberOfPixelsWithGivenLabel[MAX_NUM_OF_LABELS] = {0};
static uint16_t processingWidth = CAM_IMG_WIDTH;

static float    RGB565_To_GreyScale(uint16_t *pixelColor);
static void     performOperationsOnFrame(uint32_t frameBufferAddr);
static void     performBinarization(void);
static void     performLabeling(void);
static void     removeSmallObjects(void);
static uint16_t getMinNeighbourLabel(uint16_t x, uint16_t y);

void ProxSensor_Init(uint32_t frameBufferAddr)
{
	ProxSensor_Config.algoActive     = 1;
	ProxSensor_Config.labelingActive = 1;
	ProxSensor_Config.floatOn        = 1;
	ProxSensor_Config.halfScreenMode = 0;
	ProxSensor_Config.detectedColor  = ProxSensor_Color_R;

	ProxSensor_Config.Grayscale_coeff_R = 0.3f;
	ProxSensor_Config.Grayscale_coeff_G = 0.6f;
	ProxSensor_Config.Grayscale_coeff_B = 0.1f;

	ProxSensor_Config.numberOfPixels_R = 25;
	ProxSensor_Config.numberOfPixels_G = 25;
	ProxSensor_Config.numberOfPixels_B = 25;

	ProxSensor_Config.BwTh_R = 80;
	ProxSensor_Config.BwTh_G = 19;
	ProxSensor_Config.BwTh_B = 38;

	processingWidth = CAM_IMG_WIDTH;
}

uint8_t ProxSensor_Perform(uint32_t frameBufferAddr)
{
	ImgPtr = frameBufferAddr;

	if(ProxSensor_Config.usbOutOn)
	{
		/* If usb output is on send current frame to PC */
		ProxSensor_Console_SendImgUSB(NULL);
	}

	if(ProxSensor_Config.halfScreenMode)
	{
		/* If half screen mode is on, perform processing
		 * only on the half of the width of the image. */
		processingWidth = CAM_IMG_WIDTH/2;
	}

	if(ProxSensor_Config.algoActive)
	{
//		GPIOG->ODR |= ARDUINO_D4_Pin;
		performOperationsOnFrame(frameBufferAddr);
//		GPIOG->ODR &= ~(ARDUINO_D4_Pin);
	}
	else
	{
		/* TODO: get rid of this workaround once DMA is set up correctly */
		for(uint32_t y = 0; y < CAM_IMG_HEIGHT*CAM_IMG_WIDTH; ++y)
			asm("nop");
	}

	return 0;
}

static void performBinarization()
{
	uint8_t pixelInGrey = 0U;

	for(uint16_t y = 0; y < CAM_IMG_HEIGHT; ++y)
	{
		for(uint16_t x = 0; x < processingWidth; ++x)
	  	{
			/* Convert current pixel into greyscale */
			pixelInGrey = RGB565_To_GreyScale(&(ImgPtr[y][x]));

			if ( ( ProxSensor_Config.detectedColor == ProxSensor_Color_R || ProxSensor_Config.detectedColor == ProxSensor_Color_RGB )
					&& ( ( RGB565_GET_R(ImgPtr[y][x] ) - pixelInGrey ) > ProxSensor_Config.BwTh_R ) )
			{
				ImgPtr[y][x] = COLOR_RED;
				ProxSensor_CurrentState.numberOfDetectedPixels_R += 1;
			}
			else if ( ( ProxSensor_Config.detectedColor == ProxSensor_Color_G || ProxSensor_Config.detectedColor == ProxSensor_Color_RGB )
					&& ( ( RGB565_GET_G(ImgPtr[y][x]) - pixelInGrey ) > ProxSensor_Config.BwTh_G ) )
			{
				ImgPtr[y][x] = COLOR_GREEN;
				ProxSensor_CurrentState.numberOfDetectedPixels_G += 1;
			}
			else if ( ( ProxSensor_Config.detectedColor == ProxSensor_Color_B || ProxSensor_Config.detectedColor == ProxSensor_Color_RGB )
					&& ( RGB565_GET_B(ImgPtr[y][x]) - pixelInGrey ) > ProxSensor_Config.BwTh_B )
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

uint16_t getMinNeighbourLabel(uint16_t x, uint16_t y)
{
	uint8_t  neighbourLabels[8];
	uint16_t minLabel = 0xffff;

	neighbourLabels[0] = workingFrameArray[y-1][x-1];
	neighbourLabels[1] = workingFrameArray[y-1][x];
	neighbourLabels[2] = workingFrameArray[y-1][x+1];
	neighbourLabels[3] = workingFrameArray[y][x-1];
	neighbourLabels[4] = workingFrameArray[y][x+1];
	neighbourLabels[5] = workingFrameArray[y+1][x-1];
	neighbourLabels[6] = workingFrameArray[y+1][x];
	neighbourLabels[7] = workingFrameArray[y+1][x+1];

	for(uint8_t i = 0; i<8; ++i)
	{
		if( NO_LABEL != neighbourLabels[i] && neighbourLabels[i] < minLabel )
		{
			minLabel = neighbourLabels[i];
		}
	}

	if( 0xffff == minLabel )
	{
		/* If minLabel is still equal to 0xfff
		 * return NO_LABEL = 0 to indicate that
		 * this pixel has currently no any
		 * labeled neighbor */
		minLabel = NO_LABEL;
	}

	return minLabel;
}

void performLabeling()
{
	uint16_t currentHighestLabel = 0U;
	uint16_t minNeighborLabel    = 0U;

	memset(workingFrameArray, 0, CAM_IMG_SIZE);

	for(uint16_t y = 1; y < CAM_IMG_HEIGHT - 1; ++y)
	{
		for(uint16_t x = 1; x < processingWidth - 1; ++x)
	  	{
			/* Skip pixel if it belongs to background.
			 * In binary image background is black. */
			if( COLOR_BLACK == ImgPtr[y][x] )
			{
				continue;
			}

			/* Find out what is the lowest label assigned among
			 * neighbors. If this turns out to be value of
			 * NO_LABEL we need to assign new one. */
			minNeighborLabel = getMinNeighbourLabel(x, y);

			if( NO_LABEL == minNeighborLabel )
			{
				/* Create new label */
				workingFrameArray[y][x] = ++currentHighestLabel;
				/* Increase number of pixels with this label */
				numberOfPixelsWithGivenLabel[currentHighestLabel] += 1;
			}
			else
			{
				/* Assign label from one of neighbors  */
				workingFrameArray[y][x] = minNeighborLabel;

				/* Increase number of pixels with this label */
				numberOfPixelsWithGivenLabel[minNeighborLabel] += 1;
			}

	  	}
	}
	asm("nop");
}

void removeSmallObjects()
{
	uint16_t label = 0;
	for(uint16_t y = 1; y < CAM_IMG_HEIGHT - 1; ++y)
	{
		for(uint16_t x = 1; x < processingWidth - 1; ++x)
	  	{
			label = workingFrameArray[y][x];

			/* Skip pixel if it belongs to background */
			if( NO_LABEL == label )
			{
				continue;
			}

			/* If number of pixels with given label is smaller
			 * than threshold value, this pixel has to be set to
			 * black color. */
			if( numberOfPixelsWithGivenLabel[label] < ProxSensor_Config.numberOfPixels_R )
			{
				ImgPtr[y][x] = COLOR_BLACK;
			}
			else
			{
				ImgPtr[y][x] = COLOR_RED;
			}

	  	}
	}
}

void performOperationsOnFrame(uint32_t frameBufferAddr)
{
	ProxSensor_CurrentState.numberOfDetectedPixels_R = 0;
	ProxSensor_CurrentState.numberOfDetectedPixels_G = 0;
	ProxSensor_CurrentState.numberOfDetectedPixels_B = 0;

	performBinarization();

	if(ProxSensor_Config.labelingActive)
	{
		performLabeling();
		removeSmallObjects();
	}
}

float RGB565_To_GreyScale(uint16_t *pixelColor)
{

//	HAL_GPIO_TogglePin(GPIOG, ARDUINO_D4_Pin);
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
//	HAL_GPIO_TogglePin(GPIOC, ARDUINO_D4_Pin);

}
