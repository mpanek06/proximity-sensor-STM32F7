/*
 * camera.c
 *
 *  Created on: Apr 4, 2018
 *      Author: Marcin Panek
 */

#include "camera.h"
#include "ov9655.h"

CAMERA_DrvTypeDef *camera_driv;
Camera_Config_T    Camera_Config;
/* Camera module I2C HW address */
static uint32_t CameraHwAddress;

extern DCMI_HandleTypeDef hdcmi;
extern CAMERA_DrvTypeDef  ov9655_drv;

uint8_t CAMERA_Init(uint32_t Resolution) /*Camera initialization*/
{
	uint8_t status = CAMERA_ERROR;
	/* Read ID of Camera module via I2C */
	if(ov9655_ReadID(CAMERA_I2C_ADDRESS) == OV9655_ID)
	{
		camera_driv = &ov9655_drv;/* Initialize the camera driver structure */
		CameraHwAddress = CAMERA_I2C_ADDRESS;

		Camera_Config.brightnessLevel = CAMERA_BRIGHTNESS_LEVEL4;
		Camera_Config.contrastLevel   = CAMERA_CONTRAST_LEVEL0;
		Camera_Config.resolution      = Resolution;

		if (Resolution == CAMERA_R160x120)
		{
			camera_driv->Init(CameraHwAddress, Resolution);
			HAL_DCMI_DisableCROP(&hdcmi);
		}
		else if (Resolution == CAMERA_R320x240)
		{
			camera_driv->Init(CameraHwAddress, Resolution);
			HAL_DCMI_DisableCROP(&hdcmi);
		}
		else if (Resolution == CAMERA_R640x480)
		{
			camera_driv->Init(CameraHwAddress, Resolution);
			HAL_DCMI_ConfigCROP(&hdcmi, 0, 0, 480, 272);
		}
		status = CAMERA_OK; /* Return CAMERA_OK status */

		camera_driv->Config(CameraHwAddress, Camera_Config.brightnessLevel, Camera_Config.contrastLevel, CAMERA_BRIGHTNESS_LEVEL4);

	}
	else
	{
		status = CAMERA_NOT_SUPPORTED; /* Return CAMERA_NOT_SUPPORTED status */
	}

	return status;
}

void CAMERA_SetBrightnessLevel(uint8_t val)
{
	if( CAMERA_BRIGHTNESS_LEVEL0 <= val && CAMERA_BRIGHTNESS_LEVEL4 >= val )
	{
		Camera_Config.brightnessLevel = val;
		camera_driv->Config(CameraHwAddress,
                            CAMERA_CONTRAST_BRIGHTNESS,
                            Camera_Config.contrastLevel,
                            Camera_Config.brightnessLevel);
	}
}

void CAMERA_SetContrastLevel(uint8_t val)
{
	if( CAMERA_CONTRAST_LEVEL0 <= val && CAMERA_CONTRAST_LEVEL4 >= val )
	{
			Camera_Config.contrastLevel = val;
			camera_driv->Config(CameraHwAddress,
	                            CAMERA_CONTRAST_BRIGHTNESS,
	                            Camera_Config.contrastLevel,
	                            Camera_Config.brightnessLevel);
	}
}
