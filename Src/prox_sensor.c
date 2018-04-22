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

static void             performOperationsOnFrame_HSV(uint32_t frameBufferAddr);

static inline uint8_t   isLabelValid(uint16_t labelNumber);
static inline uint8_t   isPixelInRange( ProxSensor_HSV_Color_T *hsvColor );
static inline uint32_t  calculateDistance( uint32_t *area );
static inline void      convertRGB2HSV(ProxSensor_RGB_Color_T *rgbColor, ProxSensor_HSV_Color_T *hsvColor);

void ProxSensor_Init(uint32_t frameBufferAddr)
{
	ProxSensor_Config.algoActive     = 1;
	ProxSensor_Config.labelingActive = 0;
	ProxSensor_Config.removingSmallObjectsActive = 1;
	ProxSensor_Config.halfScreenMode = 0;
	ProxSensor_Config.detectedColor  = ProxSensor_Color_R;

	ProxSensor_Config.minNumberOfPixels = 25;

	ProxSensor_Config.BwTh_low_HSV_H = 121;
	ProxSensor_Config.BwTh_up_HSV_H  = 179;

	ProxSensor_Config.BwTh_low_HSV_S = 158;
	ProxSensor_Config.BwTh_up_HSV_S  = 255;

	ProxSensor_Config.BwTh_low_HSV_V =  41;
	ProxSensor_Config.BwTh_up_HSV_V  = 255;

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
		performOperationsOnFrame_HSV(frameBufferAddr);
	}
	else
	{
		/* TODO: get rid of this workaround once DMA is set up correctly */
		for(uint32_t y = 0; y < CAM_IMG_HEIGHT*CAM_IMG_WIDTH; ++y)
			asm("nop");
	}

	return 0;
}

void performOperationsOnFrame_HSV(uint32_t frameBufferAddr)
{
	ProxSensor_HSV_Color_T   hsvColor;
	ProxSensor_RGB_Color_T   rgbColor;
	ProxSensor_BoundingBox_T bBox    = {0,0,0,0};
	ProxSensor_BoundingBox_T bBoxMax = {0,0,0,0};

	uint16_t currentHighestLabel = 0;
	uint16_t label               = 0;
	uint32_t i                   = 0;
	uint16_t x                   = 0;
	uint16_t y                   = 0;

	uint16_t *ptr                = (uint16_t*) frameBufferAddr;
	uint16_t pixel               = 0;

	uint32_t area                = 0;
	uint32_t maxArea             = 0;
	uint32_t maxAreaLabel        = 0;

	uint32_t distanceToObj       = 0;
	static uint32_t prevDistanceToObj   = 0;

	char     osdStr[20]          = {0};

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
		rgbColor.r = RGB565_GET_R(pixel);
		rgbColor.g = RGB565_GET_G(pixel);
		rgbColor.b = RGB565_GET_B(pixel);

		convertRGB2HSV(&rgbColor, &hsvColor);

		/* Check if pixel color is within desired range */
		if (isPixelInRange(&hsvColor))
		{
			*ptr = COLOR_WHITE;
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

	/* Find biggest object detected on the image. */
	for(uint16_t i = 1; i <= currentHighestLabel; ++i )
	{
		if(isLabelValid(i))
		{
			bBox.x_min = labelsInfoArray[i].x_min;
			bBox.x_max = labelsInfoArray[i].x_max;
			bBox.y_min = labelsInfoArray[i].y_min;
			bBox.y_max = labelsInfoArray[i].y_max;

			area = (bBox.x_max - bBox.x_min) * (bBox.y_max - bBox.y_min);

			if( area > maxArea )
			{
				bBoxMax = bBox;
				maxArea = area;
				maxAreaLabel = i;
			}
		}
	}

	/* If maxArea is not equal to zero, we have detected some object
	 * and we should put OSD information on screen. */
	if( 0 != maxArea )
	{
		bBoxMax.x_min = labelsInfoArray[maxAreaLabel].x_min;
		bBoxMax.x_max = labelsInfoArray[maxAreaLabel].x_max;
		bBoxMax.y_min = labelsInfoArray[maxAreaLabel].y_min;
		bBoxMax.y_max = labelsInfoArray[maxAreaLabel].y_max;

		/* Store previous distance and calculate current one.*/
		prevDistanceToObj = distanceToObj;
		distanceToObj = calculateDistance(&maxArea);

		/* Draw a bounding box around each of detected objects. */
		LCD_drawRectangle(bBoxMax.x_min, bBoxMax.y_min, bBoxMax.x_max, bBoxMax.y_max, 0);

		/* Put text info in bounding box. */
		memset(osdStr, 0, strlen(osdStr));
		sprintf(osdStr, "Area: %ld", maxArea);
		LCD_putString( bBoxMax.x_min, bBoxMax.y_min + 10, (uint8_t *) osdStr, 0 );

		/* Put text info in bounding box. */
		memset(osdStr, 0, strlen(osdStr));
		sprintf(osdStr, "Dist: %ldmm ", distanceToObj);

		int32_t distanceDiff = distanceToObj - prevDistanceToObj;

		if( distanceDiff > PROX_PIXEL_TH*10 )
		{
			if(distanceDiff > 0)
			{
				strcat(osdStr, GETTING_CLOSER_MARK);
			}
			else
			{
				strcat(osdStr, GETTING_FURTHER_MARK);
			}
		}

		LCD_putString( bBoxMax.x_min + 10, bBoxMax.y_max + 10, (uint8_t *) osdStr, 0 );
	}
}

static inline uint32_t calculateDistance( uint32_t *area )
{
	return 10 * (OBJ_SIZE_AT_0_DIST - *area ) / 10*PIXEL_CHANGE_PER_PIXEL;
}

static inline uint8_t isPixelInRange( ProxSensor_HSV_Color_T *hsvColor )
{
	 return hsvColor->h >= ProxSensor_Config.BwTh_low_HSV_H
		 && hsvColor->s >= ProxSensor_Config.BwTh_low_HSV_S
		 && hsvColor->v >= ProxSensor_Config.BwTh_low_HSV_V

		 && hsvColor->h <= ProxSensor_Config.BwTh_up_HSV_H
		 && hsvColor->s <= ProxSensor_Config.BwTh_up_HSV_S
		 && hsvColor->v <= ProxSensor_Config.BwTh_up_HSV_V;
}

static inline uint8_t isLabelValid(uint16_t labelNumber)
{
	return labelsInfoArray[labelNumber].numberOfPixels >= ProxSensor_Config.minNumberOfPixels;
}

static inline void convertRGB2HSV(ProxSensor_RGB_Color_T *rgbColor, ProxSensor_HSV_Color_T *hsvColor)
{
	uint8_t  hsv_cmin  = 0;
	uint8_t  hsv_cmax  = 0;
	uint8_t  hsv_delta = 0;

	/* Convert current pixel into HSV */

	hsv_cmax  = MAX(rgbColor->r, MAX(rgbColor->g, rgbColor->b));
	hsv_cmin  = MIN(rgbColor->r, MIN(rgbColor->g, rgbColor->b));
	hsv_delta = hsv_cmax - hsv_cmin;

	/* Calculate V value of HSV */
	hsvColor->v = hsv_cmax;

	if( 0 == hsv_cmax )
	{
		hsvColor->h = 0;
		hsvColor->s = 0;
		return;
	}

	/* Calculate S value of HSV */
	hsvColor->s = 255 * hsv_delta / hsv_cmax;

	if( 0 == hsvColor->s )
	{
		hsvColor->h = 0;
		return;
	}

	/* Calculate H value of HSV */
	if( 0 == hsv_delta )
	{
		hsvColor->h = 0;
	}
	else if( hsv_cmax == rgbColor->r )
	{
		hsvColor->h =   0 + 43 * (rgbColor->g - rgbColor->b) / hsv_delta;
	}
	else if( hsv_cmax == rgbColor->g )
	{
		hsvColor->h =  85 + 43 * (rgbColor->b - rgbColor->r) / hsv_delta;
	}
	else if( hsv_cmax == rgbColor->b )
	{
		hsvColor->h = 171 + 43 * (rgbColor->r - rgbColor->g) / hsv_delta;
	}
	else
	{
		hsvColor->h = 0;
	}
}
