#include "prox_sensor.h"
#include "prox_sensor_console.h"
#include "main.h"

#include "lcd_lib.h"

#include "usbd_cdc_if.h"

ProxSensor_Config_T       ProxSensor_Config;
ProxSensor_CurrentState_T ProxSensor_CurrentState;

uint16_t                (*ImgPtr)[CAM_IMG_WIDTH];
uint16_t                labelsArray[CAM_IMG_HEIGHT][CAM_IMG_WIDTH] = {0};
ProxSensor_LabelInfo_T  labelsInfoArray[MAX_NUM_OF_LABELS] = {0};
uint16_t                processingWidth = CAM_IMG_WIDTH;

static void             performOperationsOnFrame(uint32_t frameBufferAddr);
static inline uint8_t   isLabelValid(uint16_t labelNumber);

void ProxSensor_Init(uint32_t frameBufferAddr)
{
	ProxSensor_Config.algoActive     = 1;
	ProxSensor_Config.labelingActive = 1;
	ProxSensor_Config.removingSmallObjectsActive = 1;
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

			/* If pixel to the North and to the West are labeled but with different labels
			 * they need to be merged as they belong to the same object, */
			if( labelsArray[y-1][x] != NO_LABEL && labelsArray[y][x-1] != NO_LABEL
					&& labelsArray[y-1][x] != labelsArray[y][x-1] )
			{
				/* Get min label */
				if( labelsArray[y][x-1] < labelsArray[y-1][x])
				{
					/* Pixel to the West has the smallest label number so
					 * we will use this label for current and North pixel as well. */
					label = labelsArray[y][x-1];
					/* Since we remove pixel y-1 from its current label group
					 * we need to decrease number of pixels in this "old" group.*/
					labelsInfoArray[labelsArray[y-1][x]].numberOfPixels -= 1;
					labelsArray[y-1][x] = label;
				}
				else
				{
					/* Pixel to the North has the smallest label number so
					 * we will use this label for current and West pixel as well. */
					label = labelsArray[y-1][x];
					/* Since we remove pixel x-1 from its current label group
					 * we need to decrease number of pixels in this "old" group.*/
					labelsInfoArray[labelsArray[y][x-1]].numberOfPixels -= 1;
					labelsArray[y][x-1] = label;
				}

				labelsArray[y][x] = label;
				/* Label whose value is stored in label variable has been assigned to
				 * two new pixels so we add number two to the number of pixels for this label. */
				labelsInfoArray[label].numberOfPixels += 2;
			}
			else if( labelsArray[y][x-1] != NO_LABEL )
			{
				label = labelsArray[y][x-1];
				labelsArray[y][x] = label;
				labelsInfoArray[label].numberOfPixels += 1;
			}
			else if( labelsArray[y-1][x] != NO_LABEL )
			{
				label = labelsArray[y-1][x];
				labelsArray[y][x] = label;
				labelsInfoArray[label].numberOfPixels += 1;
			}
			else
			{
				/* None of the neighbors has label assigned
				 * so we need to create new one. */
				label = ++currentHighestLabel;
				labelsArray[y][x] = label;

				/* Set number of pixels with new label to 1
				 * since it is assigned to only one pixel so far. */
				labelsInfoArray[label].numberOfPixels = 1;

				/* Initialize x/y min/max values for new pixel in order to be able
				 * to determine those values later for bounding box.*/
				labelsInfoArray[label].x_min = x;
				labelsInfoArray[label].x_max = x;
				labelsInfoArray[label].y_min = y;
				labelsInfoArray[label].y_max = y;
			}

			/* Check if any of x/y min/max values needs to be updated. */
			if( x > labelsInfoArray[label].x_max )
			{
				labelsInfoArray[label].x_max = x;
			}
			else if( x < labelsInfoArray[label].x_min )
			{
				labelsInfoArray[label].x_min = x;
			}

			if( y > labelsInfoArray[label].y_max )
			{
				labelsInfoArray[label].y_max = y;
			}
			else if( y < labelsInfoArray[label].y_min )
			{
				labelsInfoArray[label].y_min = y;
			}

		} /* END OF LABELING */
	}

	if(ProxSensor_Config.labelingActive && ProxSensor_Config.removingSmallObjectsActive)
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
				if( !isLabelValid(label) )
				{
					ImgPtr[y][x] = COLOR_BLACK;
				}

		  	}
		}
	} /* END OF REMOVE SMALL OBJECTS */

	/* Draw a bounding box around each of detected objects. */
	for(uint16_t i = 1; i <= currentHighestLabel; ++i )
	{
		if(isLabelValid(i))
		{
			LCD_drawRectangle(labelsInfoArray[i].x_min,
							  labelsInfoArray[i].y_min,
							  labelsInfoArray[i].x_max,
							  labelsInfoArray[i].y_max,
							  0
							  );
			}
	}
}


static inline uint8_t isLabelValid(uint16_t labelNumber)
{
	return labelsInfoArray[labelNumber].numberOfPixels >= ProxSensor_Config.minNumberOfPixels_R;
}
