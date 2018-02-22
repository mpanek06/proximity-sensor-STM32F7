/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "dcmi.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* USER CODE BEGIN Includes */
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "config.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_sdram.h"
#include "ov9655.h"
#include "rk043fn48h.h"
#include "fonts.h"

#include "prox_sensor.h"
#include "prox_sensor_console.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

typedef enum
{
	CAMERA_OK = 0x00,
	CAMERA_ERROR = 0x01,
	CAMERA_TIMEOUT = 0x02,
	CAMERA_NOT_DETECTED = 0x03,
	CAMERA_NOT_SUPPORTED = 0x04
} Camera_StatusTypeDef;

typedef struct
{
	uint32_t TextColor;
	uint32_t BackColor;
	sFONT
	*pFont;
}LCD_DrawPropTypeDef;

typedef struct
{
	int16_t X;
	int16_t Y;
}Point, *pPoint;

static LCD_DrawPropTypeDef DrawProp[2];

LTDC_HandleTypeDef hltdc;
LTDC_LayerCfgTypeDef layer_cfg;
static RCC_PeriphCLKInitTypeDef periph_clk_init_struct;
CAMERA_DrvTypeDef *camera_driv;
/* Camera module I2C HW address */
static uint32_t CameraHwAddress;

/* Image size */
uint32_t Im_size = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

uint8_t CAMERA_Init(uint32_t );
static void LTDC_Init(uint32_t , uint16_t , uint16_t , uint16_t, uint16_t);
void LCD_GPIO_Init(LTDC_HandleTypeDef *, void *);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_FMC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  ProxSensor_Init(FRAME_BUFFER);
  ProxSensor_Console_Init();

  LTDC_Init(FRAME_BUFFER, 80, 0, CAM_IMG_WIDTH, CAM_IMG_HEIGHT);
  BSP_SDRAM_Init();
  CAMERA_Init(CAMERA_R320x240);
  //Delay for the camera to output correct data
  HAL_Delay(1000);
  Im_size = CAM_IMG_WIDTH * CAM_IMG_HEIGHT * 2 / 4;
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)FRAME_BUFFER, Im_size);
  uint8_t layer = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ProxSensor_Console_Perform();

	  if(0 == layer)
	  {
		  LTDC_Layer1->CACR = 0;
		  LTDC_Layer2->CACR = 255;
		  LTDC->SRCR=2;

		  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)FRAME_BUFFER, Im_size);
		  DCMI->CR |= DCMI_CR_CAPTURE;
		  HAL_DCMI_Stop(&hdcmi);

		  ProxSensor_Perform(FRAME_BUFFER);
		  layer = 1;
	  }
	  else
	  {
		  LTDC_Layer1->CACR = 255;
		  LTDC_Layer2->CACR = 0;
		  LTDC->SRCR=2;

		  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)FRAME_BUFFER_2, Im_size);
		  DCMI->CR |= DCMI_CR_CAPTURE;
		  HAL_DCMI_Stop(&hdcmi);

		  ProxSensor_Perform(FRAME_BUFFER_2);
		  layer = 0;
	  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void LCD_GPIO_Init(LTDC_HandleTypeDef *hltdc, void *Params)
{
	GPIO_InitTypeDef gpio_init_structure;
	/* Enable the LTDC and DMA2D clocks */
	__HAL_RCC_LTDC_CLK_ENABLE();
	__HAL_RCC_DMA2D_CLK_ENABLE();
	/* Enable GPIOs clock */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOJ_CLK_ENABLE();
	__HAL_RCC_GPIOK_CLK_ENABLE();
	/*** LTDC Pins configuration ***/
	/* GPIOE configuration */
	gpio_init_structure.Pin = GPIO_PIN_4;
	gpio_init_structure.Mode = GPIO_MODE_AF_PP;
	gpio_init_structure.Pull = GPIO_NOPULL;
	gpio_init_structure.Speed = GPIO_SPEED_FAST;
	gpio_init_structure.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOE, &gpio_init_structure);
	/* GPIOG configuration */
	gpio_init_structure.Pin = GPIO_PIN_12;
	gpio_init_structure.Mode = GPIO_MODE_AF_PP;
	gpio_init_structure.Alternate = GPIO_AF9_LTDC;
	HAL_GPIO_Init(GPIOG, &gpio_init_structure);
	/* GPIOI LTDC alternate configuration */
	gpio_init_structure.Pin	= GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_13 |
						      GPIO_PIN_14 | GPIO_PIN_15;
	gpio_init_structure.Mode = GPIO_MODE_AF_PP;
	gpio_init_structure.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOI, &gpio_init_structure);
	/* GPIOJ configuration */
	gpio_init_structure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
	GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_5
	| GPIO_PIN_6 | GPIO_PIN_7 |GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
	GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	gpio_init_structure.Mode = GPIO_MODE_AF_PP;
	gpio_init_structure.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOJ, &gpio_init_structure);
	/* GPIOK configuration */
	gpio_init_structure.Pin 	= GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
	GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	gpio_init_structure.Mode = GPIO_MODE_AF_PP;
	gpio_init_structure.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOK, &gpio_init_structure);
	/* LCD_DISP GPIO configuration */
	gpio_init_structure.Pin = GPIO_PIN_12;
	/* LCD_DISP pin has to be manually controlled */
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOI, &gpio_init_structure);
	/* LCD_BL_CTRL GPIO configuration */
	gpio_init_structure.Pin 	= GPIO_PIN_3;
	/* LCD_BL_CTRL pin has to be manually controlled */
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOK, &gpio_init_structure);
}

static void LTDC_Init(uint32_t FB_Address, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
	/* Timing Configuration */
	hltdc.Init.HorizontalSync = (RK043FN48H_HSYNC - 1);
	hltdc.Init.VerticalSync = (RK043FN48H_VSYNC - 1);
	hltdc.Init.AccumulatedHBP = (RK043FN48H_HSYNC + RK043FN48H_HBP - 1);
	hltdc.Init.AccumulatedVBP = (RK043FN48H_VSYNC + RK043FN48H_VBP - 1);
	hltdc.Init.AccumulatedActiveH = (RK043FN48H_HEIGHT + RK043FN48H_VSYNC +	RK043FN48H_VBP - 1);
	hltdc.Init.AccumulatedActiveW = (RK043FN48H_WIDTH + RK043FN48H_HSYNC + 	RK043FN48H_HBP - 1);
	hltdc.Init.TotalHeigh = (RK043FN48H_HEIGHT + RK043FN48H_VSYNC +	RK043FN48H_VBP + RK043FN48H_VFP - 1);
	hltdc.Init.TotalWidth = (RK043FN48H_WIDTH + RK043FN48H_HSYNC +	RK043FN48H_HBP + RK043FN48H_HFP - 1);
	/* LCD clock configuration */
	periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	periph_clk_init_struct.PLLSAI.PLLSAIN = 192;
	periph_clk_init_struct.PLLSAI.PLLSAIR = RK043FN48H_FREQUENCY_DIVIDER;
	periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
	HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
	/* Initialize the LCD pixel width and pixel height */
	hltdc.LayerCfg->ImageWidth	= RK043FN48H_WIDTH;
	hltdc.LayerCfg->ImageHeight = RK043FN48H_HEIGHT;
	hltdc.Init.Backcolor.Blue = 0;/* Background value */
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Red = 0;
	/* Polarity */
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc.Instance = LTDC;
	if(HAL_LTDC_GetState(&hltdc) == HAL_LTDC_STATE_RESET)
	{
		LCD_GPIO_Init(&hltdc, NULL);
	}
	HAL_LTDC_Init(&hltdc);
	/* Assert display enable LCD_DISP pin */
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);
	/* Assert backlight LCD_BL_CTRL pin */
	HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);
	DrawProp[0].pFont = &Font24;

	/* Layer Init */
	layer_cfg.WindowX0 = Xpos;
	layer_cfg.WindowX1 = Width + Xpos;
	layer_cfg.WindowY0 = Ypos;
	layer_cfg.WindowY1 = Height + Ypos;
	layer_cfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
	layer_cfg.FBStartAdress = FB_Address;
	layer_cfg.Alpha = 255;
	layer_cfg.Alpha0 = 0;
	layer_cfg.Backcolor.Blue = 0;
	layer_cfg.Backcolor.Green = 0;
	layer_cfg.Backcolor.Red = 0;
	layer_cfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
	layer_cfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
	layer_cfg.ImageWidth = Width;
	layer_cfg.ImageHeight = Height;
	HAL_LTDC_ConfigLayer(&hltdc, &layer_cfg, 0);

	/* Layer 2 Init */
	layer_cfg.FBStartAdress = FRAME_BUFFER_2;
	HAL_LTDC_ConfigLayer(&hltdc, &layer_cfg, 1);

	DrawProp[1].BackColor = ((uint32_t)0xFFFFFFFF);
	DrawProp[1].pFont = &Font24;
	DrawProp[1].TextColor = ((uint32_t)0xFF000000);
}

uint8_t CAMERA_Init(uint32_t Resolution) /*Camera initialization*/
{
	uint8_t status = CAMERA_ERROR;
	/* Read ID of Camera module via I2C */
	if(ov9655_ReadID(CAMERA_I2C_ADDRESS) == OV9655_ID)
	{
		camera_driv = &ov9655_drv;/* Initialize the camera driver structure */
		CameraHwAddress = CAMERA_I2C_ADDRESS;

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
	}
	else
	{
		status = CAMERA_NOT_SUPPORTED; /* Return CAMERA_NOT_SUPPORTED status */
	}

	return status;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
