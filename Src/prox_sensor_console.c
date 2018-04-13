
/*
 * prox_sensor_console.c
 *
 *  Created on: Feb 16, 2018
 *      Author: Marcin Panek
 */
#include <ctype.h>

#include "prox_sensor_console.h"
#include "prox_sensor.h"
#include "camera.h"

#include "usbd_cdc_if.h"
#include "stm32f7xx_hal.h"

#define PROX_SENSOR_MENU_ITEM_DESC_LENGTH     40
#define PROX_SENSOR_NO_OF_OPTIONS             30
#define PROX_SENSOR_MENU_BUF_SIZE             500
#define MAX_COMMAND_SIZE                      10
#define RESPONSE_BUFF_SIZE                    500
#define LIVE_MODE_BUFF_SIZE					  500

#define ASCII_BACKSPACE                       8

typedef struct {
	const char index;
	const char desc[ PROX_SENSOR_MENU_ITEM_DESC_LENGTH ];
	void (*optionCallback)( char* );
}ProxSensor_CommandEntry_T;

struct {
	/* Boolean to turn on or off live mode - sending live data on terminal */
	uint8_t liveOutputEnabled;
} ProxSensor_ConsoleConfig;

void ProxSensor_Console_SetBwTh_HSV_H( char* arg );
void ProxSensor_Console_SetBwTh_HSV_S( char* arg );
void ProxSensor_Console_SetBwTh_HSV_V( char* arg );

void ProxSensor_Console_SetBwTh_R( char* arg );
void ProxSensor_Console_SetBwTh_G( char* arg );
void ProxSensor_Console_SetBwTh_B( char* arg );

void ProxSensor_Console_SetGrCoeff_R( char* arg );
void ProxSensor_Console_SetGrCoeff_G( char* arg );
void ProxSensor_Console_SetGrCoeff_B( char* arg );

void ProxSensor_Console_SetNoOfPixels_R( char* arg );
void ProxSensor_Console_SetNoOfPixels_G( char* arg );
void ProxSensor_Console_SetNoOfPixels_B( char* arg );

void ProxSensor_Console_SetDetectedColor( char* arg );

void ProxSensor_Console_SetBrightness( char* arg );
void ProxSensor_Console_SetContrast( char* arg );

void ProxSensor_Console_ToggleAlgo( char* arg );
void ProxSensor_Console_ToggleHalfScreen( char* arg );
void ProxSensor_Console_ToggleLabeling( char* arg );
void ProxSensor_Console_ToggleRemovingSmallObjects( char* arg );
void ProxSensor_Console_CurrParams( char* arg );
void ProxSensor_Console_PrintDebugDataOnUSB( char* arg );
void ProxSensor_Console_ShowHelp( char* arg );
void ProxSensor_Console_ToggleLiveMode( char* arg );
void ProxSensor_Console_RestartuC( char* arg );
void ProxSensor_Console_EnableOutputUSB( char* arg );

//extern uint16_t FRAME_BUFFER[CAM_IMG_WIDTH][CAM_IMG_HEIGHT];
extern uint16_t (*ImgPtr)[CAM_IMG_WIDTH];

ProxSensor_CommandEntry_T ProxSensor_consoleOptions[ PROX_SENSOR_NO_OF_OPTIONS ] =
{
		{ '1', "BwTh_R",     	                          ProxSensor_Console_SetBwTh_R },
//		{ '2', "BwTh_G",     	                          ProxSensor_Console_SetBwTh_G },
//		{ '3', "BwTh_B",     	                          ProxSensor_Console_SetBwTh_B },
//		{ '4', "Grayscale_coeff_R",        	           ProxSensor_Console_SetGrCoeff_R },
//		{ '5', "Grayscale_coeff_G",        	           ProxSensor_Console_SetGrCoeff_G },
//		{ '6', "Grayscale_coeff_B",        	           ProxSensor_Console_SetGrCoeff_B },
		{ '7', "NoOfPixels_R",     	                ProxSensor_Console_SetNoOfPixels_R },
		{ '8', "NoOfPixels_G",     	                ProxSensor_Console_SetNoOfPixels_G },
		{ '9', "NoOfPixels_B",     	                ProxSensor_Console_SetNoOfPixels_B },
		{ 'a', "Toggle algorithm",                       ProxSensor_Console_ToggleAlgo },
		{ 'b', "Toggle half screen mode",          ProxSensor_Console_ToggleHalfScreen },
		{ 'c', "Set detected color",               ProxSensor_Console_SetDetectedColor },
		{ 'd', "Display current parameters",             ProxSensor_Console_CurrParams },
		{ 'e', "Print labels array",            ProxSensor_Console_PrintDebugDataOnUSB },
		{ 'g', "Toggle rem small objs",  ProxSensor_Console_ToggleRemovingSmallObjects },
		{ 'h', "Display this menu",                        ProxSensor_Console_ShowHelp },
		{ 'i', "Set brightness",                      ProxSensor_Console_SetBrightness },
		{ 'j', "Set contrast",                          ProxSensor_Console_SetContrast },
		{ 'l', "Toggle live mode",                   ProxSensor_Console_ToggleLiveMode },
		{ 'r', "Restart STM32 uC",                        ProxSensor_Console_RestartuC },
		{ 's', "Toggle labeling",                    ProxSensor_Console_ToggleLabeling },
		{ 'u', "Send one image frame via USB",           ProxSensor_Console_SendImgUSB },
		{ 'o', "Enable output on USB",              ProxSensor_Console_EnableOutputUSB },
		{ 'x', "BwTh_HSV_H",     	                  ProxSensor_Console_SetBwTh_HSV_H },
		{ 'y', "BwTh_HSV_S",     	                  ProxSensor_Console_SetBwTh_HSV_S },
		{ 'z', "BwTh_HSV_V",     	                  ProxSensor_Console_SetBwTh_HSV_V },
};

static void sendStringToDiagTerminal( char* buffer, size_t size );
static void sendStringOnUSBPort( const char* buffer, size_t size );
static void invokeCallbackForCommand( char* command );
static void getArgFromCommandString( char* command, char* arg );

/* Line separator used in data sent to terminal */
static const char lineSeparator[] = "\r\n";
/* Start sequence send before image frame data */
static const char imgFrameStartSeq[] = "##";
/* Stopt sequence send after image frame data */
static const char imgFrameStopSeq[] = "##";
/* Flag set to true when data has been received on diag port (USB) */
uint8_t USB_RxClbkFlag = 0;
/* Flag set to true when data has been received on diag port (UART) */
uint8_t UART_RxClbkFlag = 0;
/* Buffer for received command */
static char command[ MAX_COMMAND_SIZE ];
/* Buffer for data typed in terminal*/
static char RxBuff[ MAX_COMMAND_SIZE ];
/* Buffer for response sent to terminal by command handler */
char commandResponseBuff[ RESPONSE_BUFF_SIZE ];
/* Buffer for live mode */
char liveModeBuff[ LIVE_MODE_BUFF_SIZE ];
/* Startup string */
const char startString[] = "Proximity Sensor by Marcin Panek. 2018\n\r\n\r";
/* String to name color bing detected. */
const char detectedColorString[][6] = {"RGB", "Red", "Green", "Blue"};

extern uint16_t labelsArray[CAM_IMG_HEIGHT][CAM_IMG_WIDTH];

extern ProxSensor_Config_T ProxSensor_Config;
extern Camera_Config_T     Camera_Config;
extern ProxSensor_CurrentState_T ProxSensor_CurrentState;
extern UART_HandleTypeDef huart1;

void ProxSensor_Console_Init()
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)startString, sizeof(startString));
	HAL_UART_Receive_IT(&huart1, (uint8_t*) RxBuff, 1);
}

void ProxSensor_Console_Perform()
{
	if (UART_RxClbkFlag)
	{
		/* Put received chunk at the end of command buffer */
		sprintf(command, "%s%s", command, RxBuff);

		/* Echo received data */
		HAL_UART_Transmit(&huart1, (uint8_t*) RxBuff, sizeof(RxBuff), 100);
		/* If enter pressed try to invoke command */
		if(RxBuff[strlen(RxBuff)-1] == '\r')
		{
			/* Echo received data */
			HAL_UART_Transmit(&huart1, (uint8_t*) lineSeparator, sizeof(lineSeparator), 100);

			command[strlen(command)-1]  = 0;
			invokeCallbackForCommand(command);
			/* Clear command buffer */
			memset(command, 0, sizeof(command));
		}
		else if( RxBuff[strlen(RxBuff)-1] == ASCII_BACKSPACE )
		{
			/* Remove backspace char from buffer. */
			command[strlen(command)-1] = 0;
			/* Remove char that was the last char in buffer
			 * before backspace was received. */
			command[strlen(command)-1] = 0;
		}

		/* Clear Rx buffer */
		memset(RxBuff, 0, sizeof(RxBuff));
		/* Clear callback flag */
		UART_RxClbkFlag = 0;
	}
	else if(ProxSensor_ConsoleConfig.liveOutputEnabled)
	{
		memset(liveModeBuff, 0, LIVE_MODE_BUFF_SIZE);
		sprintf(liveModeBuff, "Size of object: %ld\n\r", 0L);
		sendStringToDiagTerminal(liveModeBuff, strlen(liveModeBuff));
	}
	/* Start listening for next data */
	HAL_UART_Receive_IT(&huart1, (uint8_t*) RxBuff, 1);
}

void ProxSensor_Console_SetBwTh_HSV_H( char* arg )
{
	ProxSensor_Config.BwTh_HSV_H = atoi(arg);
}

void ProxSensor_Console_SetBwTh_HSV_S( char* arg )
{
	ProxSensor_Config.BwTh_HSV_S = atoi(arg);
}

void ProxSensor_Console_SetBwTh_HSV_V( char* arg )
{
	ProxSensor_Config.BwTh_HSV_V = atoi(arg);
}

void ProxSensor_Console_SetBwTh_R( char* arg )
{
	ProxSensor_Config.BwTh_R = atoi(arg);
}

void ProxSensor_Console_SetBwTh_G( char* arg )
{
	ProxSensor_Config.BwTh_G = atoi(arg);
}

void ProxSensor_Console_SetBwTh_B( char* arg )
{
	ProxSensor_Config.BwTh_B = atoi(arg);
}

void ProxSensor_Console_SetGrCoeff_R( char* arg )
{
	ProxSensor_Config.Grayscale_coeff_R = atof(arg);
}

void ProxSensor_Console_SetGrCoeff_G( char* arg )
{
	ProxSensor_Config.Grayscale_coeff_G = atof(arg);
}

void ProxSensor_Console_SetGrCoeff_B( char* arg )
{
	ProxSensor_Config.Grayscale_coeff_B = atof(arg);
}

void ProxSensor_Console_SetNoOfPixels_R( char* arg )
{
	ProxSensor_Config.minNumberOfPixels_R = atoi(arg);
}

void ProxSensor_Console_SetNoOfPixels_G( char* arg )
{
	ProxSensor_Config.minNumberOfPixels_G = atoi(arg);
}

void ProxSensor_Console_SetNoOfPixels_B( char* arg )
{
	ProxSensor_Config.minNumberOfPixels_B = atoi(arg);
}

void ProxSensor_Console_SetBrightness( char* arg )
{
	int8_t val = atoi(arg);
	CAMERA_SetBrightnessLevel( val );
}

void ProxSensor_Console_SetContrast( char* arg )
{
	int8_t val = atoi(arg);
	/* Contrast levels in camera driver starts from 0x05 so
	 * value of CAMERA_CONTRAST_LEVEL0 (0x05) is added to
	 * value given by user. */
	CAMERA_SetContrastLevel( val + CAMERA_CONTRAST_LEVEL0 );
}

void ProxSensor_Console_ToggleAlgo( char* arg )
{
	ProxSensor_Config.algoActive ^= 1;
}

void ProxSensor_Console_ToggleHalfScreen( char* arg )
{
	ProxSensor_Config.halfScreenMode ^= 1;
}

void ProxSensor_Console_ToggleLabeling( char* arg )
{
	ProxSensor_Config.labelingActive ^= 1;
}

void ProxSensor_Console_ToggleRemovingSmallObjects( char* arg )
{
	ProxSensor_Config.removingSmallObjectsActive ^= 1;
}

void ProxSensor_Console_SetDetectedColor( char* arg )
{
	uint8_t color = atoi(arg);

	if( color >= ProxSensor_Color_R && color <= ProxSensor_Color_RGB )
	{
		ProxSensor_Config.detectedColor = color;
	}
}

void ProxSensor_Console_CurrParams( char* arg )
{
	memset(commandResponseBuff, 0, RESPONSE_BUFF_SIZE);

	sprintf(commandResponseBuff, "Current parameters of algorithm: %s", lineSeparator );

	sprintf(commandResponseBuff, "%s Pixels  R: %d    G: %d    B: %d %s", commandResponseBuff,
                                                                       ProxSensor_Config.minNumberOfPixels_R,
																       ProxSensor_Config.minNumberOfPixels_G,
																       ProxSensor_Config.minNumberOfPixels_B,
																       lineSeparator );

	sprintf(commandResponseBuff, "%s BWTh    R: %d    G: %d    B: %d %s", commandResponseBuff,
                                                                     ProxSensor_Config.BwTh_R,
                                                                     ProxSensor_Config.BwTh_G,
                                                                     ProxSensor_Config.BwTh_B,
																	 lineSeparator );

	sprintf(commandResponseBuff, "%s GrCoef  R: %.2f  G: %.2f  B: %.2f  %s", commandResponseBuff,
                                                                                 ProxSensor_Config.Grayscale_coeff_R,
																				 ProxSensor_Config.Grayscale_coeff_G,
																				 ProxSensor_Config.Grayscale_coeff_B,
																				 lineSeparator );

	strcat(commandResponseBuff, lineSeparator);

	sprintf(commandResponseBuff, "%s Brigh: %d %s", commandResponseBuff, Camera_Config.brightnessLevel, lineSeparator );
	sprintf(commandResponseBuff, "%s Contr: %d %s", commandResponseBuff, Camera_Config.contrastLevel-CAMERA_CONTRAST_LEVEL0, lineSeparator );

	strcat(commandResponseBuff, lineSeparator);

	sprintf(commandResponseBuff, "%s Detected color: %s %s", commandResponseBuff, detectedColorString[ProxSensor_Config.detectedColor], lineSeparator );
	sprintf(commandResponseBuff, "%s algoActive : %d %s", commandResponseBuff, ProxSensor_Config.algoActive, lineSeparator );
	sprintf(commandResponseBuff, "%s labelingActive : %d %s", commandResponseBuff, ProxSensor_Config.labelingActive, lineSeparator );

	strcat(commandResponseBuff, lineSeparator);

	sendStringToDiagTerminal( commandResponseBuff, strlen(commandResponseBuff) );
}

void ProxSensor_Console_PrintDebugDataOnUSB( char* arg )
{
	memset(commandResponseBuff, 0, RESPONSE_BUFF_SIZE);

	for(uint16_t y = 0; y < CAM_IMG_HEIGHT; ++y)
	{
		for(uint16_t x = 0; x < CAM_IMG_WIDTH; ++x)
		{
			sprintf(commandResponseBuff, " %d;", labelsArray[y][x]);
			sendStringOnUSBPort( commandResponseBuff, strlen(commandResponseBuff) );
		}

		strcat(commandResponseBuff, lineSeparator);
		sendStringOnUSBPort( commandResponseBuff, strlen(commandResponseBuff) );
	}

	sprintf(commandResponseBuff, "%s", lineSeparator);
	sendStringOnUSBPort( commandResponseBuff, strlen(commandResponseBuff) );

	sendStringOnUSBPort(imgFrameStartSeq, strlen(imgFrameStartSeq));

	for(uint16_t y = 0; y < CAM_IMG_HEIGHT; ++y)
	{
		for(uint16_t x = 0; x < CAM_IMG_WIDTH; ++x)
		{
			sprintf(commandResponseBuff, " %d;", ImgPtr[y][x]);
			sendStringOnUSBPort( commandResponseBuff, strlen(commandResponseBuff) );
		}
		strcat(commandResponseBuff, lineSeparator);
		sendStringOnUSBPort( commandResponseBuff, strlen(commandResponseBuff) );

	}

	sendStringOnUSBPort(imgFrameStopSeq, strlen(imgFrameStopSeq));
}

void ProxSensor_Console_ShowHelp( char* arg )
{
	static char tmpBuf[ PROX_SENSOR_MENU_ITEM_DESC_LENGTH + 5 ] = "\0";

	uint8_t i = 0;

	memset(commandResponseBuff, 0, PROX_SENSOR_MENU_BUF_SIZE);
	memset(tmpBuf, 0, PROX_SENSOR_MENU_ITEM_DESC_LENGTH + 5);

	sprintf( commandResponseBuff, "Diag mode menu: \n\r" );

	for( i = 0; i < PROX_SENSOR_NO_OF_OPTIONS; ++i )
	{
		sprintf(tmpBuf, "%c %s%s",
				ProxSensor_consoleOptions[i].index,
				ProxSensor_consoleOptions[i].desc,
				"\n\r"
				);
		strcat( commandResponseBuff, tmpBuf );
		memset( tmpBuf, 0, PROX_SENSOR_MENU_ITEM_DESC_LENGTH + 5 );
	}

	strcat( commandResponseBuff, "\n\r" );

	sendStringToDiagTerminal( commandResponseBuff, strlen(commandResponseBuff) );
}

void ProxSensor_Console_ToggleLiveMode( char* arg )
{
	ProxSensor_ConsoleConfig.liveOutputEnabled ^= 1;
}

/** @brief Restarts uC.
 * Note that USB connection will be lost after this action!
 *
 * @param[in] arg argument for execution
 * @return void
 */
void ProxSensor_Console_RestartuC( char* arg )
{
	NVIC_SystemReset();
}

/** @brief Activates sending image data via USB interface.
 *
 * @param[in] arg argument for execution
 * @return void
 */
void ProxSensor_Console_EnableOutputUSB( char* arg )
{
	ProxSensor_Config.usbOutOn ^= 1;
}

void ProxSensor_Console_SendImgUSB( char* arg )
{
	sendStringOnUSBPort(imgFrameStartSeq, strlen(imgFrameStartSeq));

	sendStringOnUSBPort((const char *) FRAME_BUFFER, 0xffff);
	sendStringOnUSBPort((const char *) (FRAME_BUFFER + 1 * 0xffff), 0xffff);
	sendStringOnUSBPort((const char *) (FRAME_BUFFER + 2 * 0xffff), 22528);

	sendStringOnUSBPort(imgFrameStopSeq, strlen(imgFrameStopSeq));
}

/** @brief Function sends buffer to diagnostic terminal.
 *
 * It is used to send buffer when uC is in diag mode.
 * If output method for diag mode needs to be changed it
 * should be done in this function.
 *
 * @param[in] buffer data to be sent
 * @param[in] size size of the data to be sent
 *
 * @return void
 */
void sendStringToDiagTerminal( char* buffer, size_t size )
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*) buffer, size);
}

static void sendStringOnUSBPort( const char* buffer, size_t size )
{
	while( USBD_OK != CDC_Transmit_FS( (uint8_t*) buffer, size ));
}

/** @brief Function invokes proper callback for given command.
 *
 * @param[in] command Command for which action should be performed
 * @return void
 */
void invokeCallbackForCommand( char* command )
{
	uint8_t i = 0;
	/* Buffer for args that are passed to command handler */
	char commandArgs[ MAX_COMMAND_SIZE ];

	for( i = 0; i < PROX_SENSOR_NO_OF_OPTIONS; ++i )
	{
		if( command[0] == ProxSensor_consoleOptions[i].index && NULL != ProxSensor_consoleOptions[i].optionCallback )
		{
			/* Once the option is recognized arguments should be retrieved from received command */
			getArgFromCommandString(command, commandArgs);
			ProxSensor_consoleOptions[i].optionCallback(commandArgs);
			break;
		}
	}
}

/** @brief Function retrieves argument from command string
 *
 * @param[in] command whole command string received on terminal
 * @param[out] args argument that should be passed to option's callback function
 * @return void
 */
void getArgFromCommandString( char* command, char* arg )
{
	memset(arg, 0, MAX_COMMAND_SIZE);

	if( '=' == command[1] )
	{
		/* Buffer pointer is moved 2 places because of command char and '=' char */
		memcpy(arg, command+2, MAX_COMMAND_SIZE-2);
	}
}
