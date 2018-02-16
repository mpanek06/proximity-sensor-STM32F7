/*
 * prox_sensor_console.c
 *
 *  Created on: Feb 16, 2018
 *      Author: Marcin Panek
 */
#include <ctype.h>

#include "prox_sensor_console.h"
#include "prox_sensor.h"

#include "usbd_cdc_if.h"

#define PROX_SENSOR_MENU_ITEM_DESC_LENGTH     40
#define PROX_SENSOR_NO_OF_OPTIONS             10
#define PROX_SENSOR_MENU_BUF_SIZE             500
#define MAX_COMMAND_SIZE                      10
#define RESPONSE_BUFF_SIZE                    100

typedef struct {
	const char index;
	const char desc[ PROX_SENSOR_MENU_ITEM_DESC_LENGTH ];
	void (*optionCallback)( char* );
}ProxSensor_CommandEntry_T;

void ProxSensor_Console_CurrParams( char* arg );
void ProxSensor_Console_ShowHelp( char* arg );
void ProxSensor_Console_RestartuC( char* arg );
void ProxSensor_Console_EnableOutputUSB( char* arg );

ProxSensor_CommandEntry_T ProxSensor_consoleOptions[ PROX_SENSOR_NO_OF_OPTIONS ] =
{
		{ 'a', "Sample option",	                                        NULL },
		{ 'd', "Display current parameters",   ProxSensor_Console_CurrParams },
		{ 'h', "Display this menu",              ProxSensor_Console_ShowHelp },
		{ 'r', "Restart STM32 uC",              ProxSensor_Console_RestartuC },
		{ 'o', "Enable output on USB",    ProxSensor_Console_EnableOutputUSB },
};

static void sendStringToDiagTerminal( char* buffer, size_t size );
static void invokeCallbackForCommand( char* command );
static void getArgFromCommandString( char* command, char* arg );

/* Line separator used in data sent to terminal */
static const char lineSeparator[] = "\r\n";
/* Flag set to true when data has been received on diag port (USB) */
uint8_t RxClbkFlag = 0;
/* Buffer for received command */
static char command[ MAX_COMMAND_SIZE ];
/* Buffer for response sent to terminal by command handler */
static char commandResponseBuff[ RESPONSE_BUFF_SIZE ];
/* Buffer used by USB driver */
extern uint8_t UserRxBufferFS[ MAX_COMMAND_SIZE ];

extern ProxSensor_Config_T ProxSensor_Config;

void ProxSensor_Console_Perform()
{
	if(RxClbkFlag)
	{
		if( isalpha( UserRxBufferFS[0] ) )
		{
			memset( command, 0, MAX_COMMAND_SIZE );
			memcpy( command, UserRxBufferFS, MAX_COMMAND_SIZE );
			invokeCallbackForCommand(command);
		}

		RxClbkFlag = 0;
	}
}

void ProxSensor_Console_CurrParams( char* arg )
{
	memset(commandResponseBuff, 0, PROX_SENSOR_MENU_BUF_SIZE);

	sprintf(commandResponseBuff, "Current parameters of algorithm: %s", lineSeparator );

	sprintf(commandResponseBuff, "%s Detected color: %d %s", commandResponseBuff, ProxSensor_Config.detectedColor, lineSeparator );

	sprintf(commandResponseBuff, "%s Pixels R: %d %s", commandResponseBuff, ProxSensor_Config.numberOfPixels_R, lineSeparator );
	sprintf(commandResponseBuff, "%s Pixels G: %d %s", commandResponseBuff, ProxSensor_Config.numberOfPixels_G, lineSeparator );
	sprintf(commandResponseBuff, "%s Pixels B: %d %s", commandResponseBuff, ProxSensor_Config.numberOfPixels_B, lineSeparator );

	sprintf(commandResponseBuff, "%s BWTh R: %d %s", commandResponseBuff, ProxSensor_Config.BwTh_R, lineSeparator );
	sprintf(commandResponseBuff, "%s BWTh G: %d %s", commandResponseBuff, ProxSensor_Config.BwTh_G, lineSeparator );
	sprintf(commandResponseBuff, "%s BWTh B: %d %s", commandResponseBuff, ProxSensor_Config.BwTh_B, lineSeparator );

	sprintf(commandResponseBuff, "%s Grayscale coeff R: %d %s", commandResponseBuff, ProxSensor_Config.Grayscale_coeff_R, lineSeparator );
	sprintf(commandResponseBuff, "%s Grayscale coeff G: %d %s", commandResponseBuff, ProxSensor_Config.Grayscale_coeff_G, lineSeparator );
	sprintf(commandResponseBuff, "%s Grayscale coeff B: %d %s", commandResponseBuff, ProxSensor_Config.Grayscale_coeff_B, lineSeparator );

	strcat(commandResponseBuff, lineSeparator);

	sendStringToDiagTerminal( commandResponseBuff, strlen(commandResponseBuff) );
}

void ProxSensor_Console_ShowHelp( char* arg )
{
	static char bufferMenu[ PROX_SENSOR_MENU_BUF_SIZE ]    = "\0";
	static char tmpBuf[ PROX_SENSOR_MENU_ITEM_DESC_LENGTH + 5 ] = "\0";

	uint8_t i = 0;

	memset(bufferMenu, 0, PROX_SENSOR_MENU_BUF_SIZE);
	memset(tmpBuf, 0, PROX_SENSOR_MENU_ITEM_DESC_LENGTH + 5);

	sprintf( bufferMenu, "Diag mode menu: \n\r" );

	for( i = 0; i < PROX_SENSOR_NO_OF_OPTIONS; ++i )
	{
		sprintf(tmpBuf, "%c %s%s",
				ProxSensor_consoleOptions[i].index,
				ProxSensor_consoleOptions[i].desc,
				"\n\r"
				);
		strcat( bufferMenu, tmpBuf );
		memset( tmpBuf, 0, PROX_SENSOR_MENU_ITEM_DESC_LENGTH + 5 );
	}

	strcat( bufferMenu, "\n\r" );

	sendStringToDiagTerminal( bufferMenu, strlen(bufferMenu) );
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

/** @brief Activates sending frame buffer via USB interface.
 *
 * @param[in] arg argument for execution
 * @return void
 */
void ProxSensor_Console_EnableOutputUSB( char* arg )
{

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
	CDC_Transmit_FS( (uint8_t*) buffer, size );
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
