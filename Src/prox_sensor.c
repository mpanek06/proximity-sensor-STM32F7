#include "prox_sensor.h"
#include "prox_sensor_console.h"
#include "main.h"

#include "lcd_lib.h"

#include "usbd_cdc_if.h"

#define  NO_LABEL          0
#define  MAX_NUM_OF_LABELS 400

ProxSensor_Config_T       ProxSensor_Config;
ProxSensor_CurrentState_T ProxSensor_CurrentState;

uint16_t                (*ImgPtr)[CAM_IMG_WIDTH];
uint16_t                labelsArray[CAM_IMG_HEIGHT][CAM_IMG_WIDTH] = {0};
ProxSensor_LabelInfo_T  labelsInfoArray[MAX_NUM_OF_LABELS] = {0};
uint16_t                processingWidth = CAM_IMG_WIDTH;

static void     performOperationsOnFrame(uint32_t frameBufferAddr);

void ProxSensor_Init(uint32_t frameBufferAddr)
{
	ProxSensor_Config.algoActive     = 1;
	ProxSensor_Config.labelingActive = 1;
	ProxSensor_Config.halfScreenMode = 0;
	ProxSensor_Config.detectedColor  = ProxSensor_Color_R;

	ProxSensor_Config.Grayscale_coeff_R = 0.3f;
	ProxSensor_Config.Grayscale_coeff_G = 0.6f;
	ProxSensor_Config.Grayscale_coeff_B = 0.1f;

	ProxSensor_Config.minNumberOfPixels_R = 25;
	ProxSensor_Config.minNumberOfPixels_G = 25;
	ProxSensor_Config.minNumberOfPixels_B = 25;

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

	uint16_t currentHighestLabel = 0U;
	uint16_t label               = 0U;
	uint8_t  pixelInGrey         = 0U;
	uint32_t i                   = 0U;
	uint16_t x                   = 0U;
	uint16_t y                   = 0U;
	uint16_t neighbourLabels[4]  = {NO_LABEL};
	uint16_t minLabel            = MAX_U16_VAL;

	uint16_t *ptr                = (uint16_t*) frameBufferAddr;
	uint16_t pixel               = 0U;
	float    val_r               = 0;
	float    val_g               = 0;
	float    val_b               = 0;

	memset(labelsArray, 0, sizeof(labelsArray));
	memset(labelsInfoArray, 0, sizeof(labelsInfoArray));

	/* This loop is responsible for transforming image to graysacle,
	 * calculating image difference (between color layer and grayscale layer
	 * and performing binarization */
	for( i=0; i < CAM_IMG_SIZE; ++i, ++ptr)
	{
		/* Store value of current pixel so there
		 * is no need to get it from SDRAM
		 * every time it is needed */
		pixel = *ptr;

		/* Get values of RGB colors for current pixel */
		val_r = (float) RGB565_GET_R(pixel);
		val_g = (float) RGB565_GET_G(pixel);
		val_b = (float) RGB565_GET_B(pixel);
		/* Convert current pixel into greyscale according to  */
		pixelInGrey = val_r * (float)ProxSensor_Config.Grayscale_coeff_R
				    + val_g * (float)ProxSensor_Config.Grayscale_coeff_G
					+ val_b * (float)ProxSensor_Config.Grayscale_coeff_B;

		/* If given color is turned on in config (or RGB mode is chosen)
		 * calculate image difference and perform thresholding*/
		if ( ( ProxSensor_Config.detectedColor == ProxSensor_Color_R || ProxSensor_Config.detectedColor == ProxSensor_Color_RGB )
				&& ( ( RGB565_GET_R(pixel) - pixelInGrey ) > ProxSensor_Config.BwTh_R ) )
		{
			*ptr = COLOR_RED;
		}
		else if ( ( ProxSensor_Config.detectedColor == ProxSensor_Color_G || ProxSensor_Config.detectedColor == ProxSensor_Color_RGB )
				&& ( ( RGB565_GET_G(pixel) - pixelInGrey ) > ProxSensor_Config.BwTh_G ) )
		{
			*ptr = COLOR_GREEN;
		}
		else if ( ( ProxSensor_Config.detectedColor == ProxSensor_Color_B || ProxSensor_Config.detectedColor == ProxSensor_Color_RGB )
				&& ( RGB565_GET_B(pixel) - pixelInGrey ) > ProxSensor_Config.BwTh_B )
		{
			*ptr = COLOR_BLUE;
		}
		else
		{
			*ptr = COLOR_BLACK;
		}

		/* Perform labeling if it is turned on in config*/
		if( ProxSensor_Config.labelingActive && *ptr != COLOR_BLACK )
		{
			/* Since we move around the frame by incrementing the value of ptr
			 * we need to calculate value of x, y coordinates in order to be able to
			 * check value of neighbors values */
			y = i / CAM_IMG_WIDTH;
			x = i - ( y * CAM_IMG_WIDTH );

			memset(neighbourLabels, NO_LABEL, sizeof(neighbourLabels));
			/* Getting values of neighbor pixels (N, W, NW, NE) */
			if( 0 != x )
				neighbourLabels[0] = labelsArray[y][x-1];
//			if( 0 != y && x != 0 )
//				neighbourLabels[1] = labelsArray[y-1][x-1];
			if( 0 != y )
				neighbourLabels[2] = labelsArray[y-1][x];
//			if( 0 != y && CAM_IMG_WIDTH-1 != x )
//				neighbourLabels[3] = labelsArray[y-1][x+1];

			/* Picking minimal value of label  */
			for(uint8_t j = 0; j < 4; ++j)
			{
				if(neighbourLabels[j] != NO_LABEL && neighbourLabels[j] < minLabel)
				{
					minLabel = neighbourLabels[j];
				}
			}

			/* If non of neighbor pixel has assigned label
			 * the current one needs to have a new one. */
			if( MAX_U16_VAL == minLabel )
			{
				/* Create new label by incrementing current value of counter. */
				++currentHighestLabel;
				/* Assign new label to current pixel. */
				labelsArray[y][x] = currentHighestLabel;

				/* Set number of pixels with new label to 1 since it is new */
				labelsInfoArray[currentHighestLabel].numberOfPixels = 1;

				/* Initalize x/y min/max values for new pixel in order to be able
				 * to determine those values for bounding box.*/
				labelsInfoArray[currentHighestLabel].x_min = x;
				labelsInfoArray[currentHighestLabel].x_max = x;
				labelsInfoArray[currentHighestLabel].y_min = y;
				labelsInfoArray[currentHighestLabel].y_max = y;
			}
			else
			{
				/* Assign label inherited from one of the neighbors. */
				labelsArray[y][x] = minLabel;
				/* Increase number of pixels with this label. */
				labelsInfoArray[currentHighestLabel].numberOfPixels += 1;

				/* Check if any of x/y min/max values needs to be updated. */
				if( x > labelsInfoArray[currentHighestLabel].x_max )
				{
					labelsInfoArray[currentHighestLabel].x_max = x;
				}
				else if( x < labelsInfoArray[currentHighestLabel].x_min )
				{
					labelsInfoArray[currentHighestLabel].x_min = x;
				}

				if( y > labelsInfoArray[currentHighestLabel].y_max )
				{
					labelsInfoArray[currentHighestLabel].y_max = y;
				}
				else if( y < labelsInfoArray[currentHighestLabel].y_min )
				{
					labelsInfoArray[currentHighestLabel].y_min = y;
				}
			}
		} /* END OF LABELING */
	}

	if(ProxSensor_Config.labelingActive)
	{
		/* REMOVE SMALL OBJECTS */
		for(uint16_t y = 0; y < CAM_IMG_HEIGHT; ++y)
		{
			for(uint16_t x = 0; x < processingWidth; ++x)
		  	{
				label = labelsArray[y][x];
				/* Skip this pixel if it has no label -
				 * it already belongs to background. */
				if( NO_LABEL == label )
					continue;

				/* If number of pixels with given label is smaller
				 * than threshold value, this pixel has to be set to
				 * black color. */
				if( labelsInfoArray[label].numberOfPixels < ProxSensor_Config.minNumberOfPixels_R )
				{
					ImgPtr[y][x] = COLOR_BLACK;
				}

		  	}
		}
	} /* END OF REMOVE SMALL OBJECTS */

	/* Draw a bounding box around each of detected objects. */
	for(uint8_t i = 1; i <= currentHighestLabel; ++i )
	{
		LCD_drawRectangle(labelsInfoArray[i].x_min,
						  labelsInfoArray[i].y_min,
						  labelsInfoArray[i].x_max,
						  labelsInfoArray[i].y_max,
						  0
						  );
	}

	RESET_DEBUG_PIN2;
}
