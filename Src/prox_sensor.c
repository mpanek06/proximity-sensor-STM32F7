#include "prox_sensor.h"
#include "prox_sensor_console.h"
#include "main.h"

#include "usbd_cdc_if.h"

#define  NO_LABEL          0
#define  MAX_NUM_OF_LABELS 2500

ProxSensor_Config_T       ProxSensor_Config;
ProxSensor_CurrentState_T ProxSensor_CurrentState;

uint16_t (*ImgPtr)[CAM_IMG_WIDTH];
uint8_t  labelsArray[CAM_IMG_HEIGHT][CAM_IMG_WIDTH] = {0};
uint8_t  numberOfPixelsWithGivenLabel[MAX_NUM_OF_LABELS] = {0};
static uint16_t processingWidth = CAM_IMG_WIDTH;

static void     performOperationsOnFrame(uint32_t frameBufferAddr);

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
	else
	{
		processingWidth = CAM_IMG_WIDTH;
	}

	if(ProxSensor_Config.algoActive)
	{
		performOperationsOnFrame(frameBufferAddr);
	}
	else
	{
		/* TODO: get rid of this workaround once DMA is set up correctly */
		for(uint32_t y = 0; y < CAM_IMG_HEIGHT*CAM_IMG_WIDTH; ++y)
			asm("nop");
	}

	return 0;
}

void performOperationsOnFrame(uint32_t frameBufferAddr)
{
	SET_DEBUG_PIN2;
	ProxSensor_CurrentState.numberOfDetectedPixels_R = 0;
	ProxSensor_CurrentState.numberOfDetectedPixels_G = 0;
	ProxSensor_CurrentState.numberOfDetectedPixels_B = 0;

	uint16_t currentHighestLabel = 0U;
	uint16_t label               = 0U;
	uint8_t  pixelInGrey         = 0U;
	uint32_t i                   = 0U;
	uint16_t x                   = 0U;
	uint16_t y                   = 0U;
	uint8_t  neighbourLabels[4]  = {NO_LABEL};

	uint16_t *ptr                = (uint16_t*) frameBufferAddr;
	float val_r;
	float val_g;
	float val_b;
	uint16_t pixel = 0;

	memset(labelsArray, 0, CAM_IMG_SIZE);
	memset(numberOfPixelsWithGivenLabel, 0, MAX_NUM_OF_LABELS);

	/* This loop is responsible for transforming image to graysacle,
	 * calculating im diff and performing binarization */
	for( i=0; i < CAM_IMG_SIZE; ++i, ++ptr)
	{
		pixel = *ptr;

		/* Convert current pixel into greyscale */

		val_r = (float) RGB565_GET_R(pixel);
		val_g = (float) RGB565_GET_G(pixel);
		val_b = (float) RGB565_GET_B(pixel);

		pixelInGrey = val_r * (float)ProxSensor_Config.Grayscale_coeff_R
				    + val_g * (float)ProxSensor_Config.Grayscale_coeff_G
					+ val_b * (float)ProxSensor_Config.Grayscale_coeff_B;

		if ( ( ProxSensor_Config.detectedColor == ProxSensor_Color_R || ProxSensor_Config.detectedColor == ProxSensor_Color_RGB )
				&& ( ( RGB565_GET_R(pixel) - pixelInGrey ) > ProxSensor_Config.BwTh_R ) )
		{
			*ptr = COLOR_RED;
			ProxSensor_CurrentState.numberOfDetectedPixels_R += 1;
		}
		else if ( ( ProxSensor_Config.detectedColor == ProxSensor_Color_G || ProxSensor_Config.detectedColor == ProxSensor_Color_RGB )
				&& ( ( RGB565_GET_G(pixel) - pixelInGrey ) > ProxSensor_Config.BwTh_G ) )
		{
			*ptr = COLOR_GREEN;
			ProxSensor_CurrentState.numberOfDetectedPixels_G += 1;
		}
		else if ( ( ProxSensor_Config.detectedColor == ProxSensor_Color_B || ProxSensor_Config.detectedColor == ProxSensor_Color_RGB )
				&& ( RGB565_GET_B(pixel) - pixelInGrey ) > ProxSensor_Config.BwTh_B )
		{
			*ptr = COLOR_BLUE;
			ProxSensor_CurrentState.numberOfDetectedPixels_B += 1;
		}
		else
		{
			*ptr = COLOR_BLACK;
		}


		if( ProxSensor_Config.labelingActive && *ptr != COLOR_BLACK )
		{
			y = i / CAM_IMG_WIDTH;
			x = i - y * CAM_IMG_WIDTH;

			if( 0 != x ) neighbourLabels[1] = labelsArray[y][x-1];

			if (y > 0)
			{
//				if( 0 != x )               neighbourLabels[3] = labelsArray[y-1][x-1];
//				if( CAM_IMG_WIDTH-1 != x ) neighbourLabels[1] = labelsArray[y-1][x+1];
				neighbourLabels[2] = labelsArray[y-1][x];
			}

			for(uint8_t j = 0; j < 4; j++)
			{
				if(neighbourLabels[j] != NO_LABEL)
				{
					labelsArray[y][x] = neighbourLabels[j];
					numberOfPixelsWithGivenLabel[labelsArray[y][x]] += 1;
					break;
				}
			}

			/* If curent pixel still doesnt have label this means it has to have
			 * new label assigned */
			if( NO_LABEL == labelsArray[y][x] )
			{
				labelsArray[y][x] = ++currentHighestLabel;
				/* Increase number of pixels with this label */
				numberOfPixelsWithGivenLabel[currentHighestLabel] += 1;
			}
		}
		/* END OF LABELING */
	}

	if(ProxSensor_Config.labelingActive)
	{
		/* REMOVE SMALL OBJECTS */
		for(uint16_t y = 1; y < CAM_IMG_HEIGHT - 1; ++y)
		{
			for(uint16_t x = 1; x < processingWidth - 1; ++x)
		  	{
				label = labelsArray[y][x];

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

		  	}
		}
		/* END OF REMOVE SMALL OBJECTS */
	}
	RESET_DEBUG_PIN2;
}
