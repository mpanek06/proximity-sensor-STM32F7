/*
 * config.h
 *
 *  Created on: Feb 15, 2018
 *      Author: Marcin Panek
 */

#ifndef CONFIG_H_
#define CONFIG_H_

//#define CAM_R_VGA
#define CAM_R_QVGA
//#define CAM_R_QQVGA

#ifdef CAM_R_QQVGA
	#define CAM_IMG_WIDTH  ((uint16_t)160)
	#define CAM_IMG_HEIGHT ((uint16_t)120)
#elif defined CAM_R_QVGA
	#define CAM_IMG_WIDTH  ((uint16_t)320)
	#define CAM_IMG_HEIGHT ((uint16_t)240)
#elif defined CAM_R_VGA
	#define CAM_IMG_WIDTH  ((uint16_t)480)
	#define CAM_IMG_HEIGHT ((uint16_t)272)
#endif

#define CAM_IMG_SIZE   (CAM_IMG_WIDTH * CAM_IMG_HEIGHT)
/* Image size in byte, each pixel in RGB565 takes 2 bytes*/
#define CAM_IMG_SIZE_B (CAM_IMG_WIDTH * CAM_IMG_HEIGHT * 2)

#define LCD_WIDTH      ((uint16_t)480)
#define LCD_HEIGHT     ((uint16_t)272)

#define SET_DEBUG_PIN4    ( GPIOG->ODR |= ARDUINO_D4_Pin )
#define RESET_DEBUG_PIN4  ( GPIOG->ODR &= ~(ARDUINO_D4_Pin) )

#define SET_DEBUG_PIN2    ( GPIOG->ODR |= ARDUINO_D2_Pin )
#define RESET_DEBUG_PIN2  ( GPIOG->ODR &= ~(ARDUINO_D2_Pin) )

#define MAX_U8_VAL     ((uint8_t)  0xff)
#define MAX_U16_VAL    ((uint16_t) 0xffff)
#define MAX_U32_VAL    ((uint32_t) 0xffffffff)

#define OBJ_SIZE_AT_20_DIST     4700
#define OBJ_SIZE_AT_30_DIST     2000

#define DIST_CALC_D            200
#define DIST_CALC_P            660
#define DIST_CALC_W            40
#define DIST_CALC_F            ( ( DIST_CALC_P * DIST_CALC_D ) / DIST_CALC_W )
#define PROX_PIXEL_TH          1

#define GETTING_CLOSER_MARK    "<-"
#define GETTING_FURTHER_MARK   "->"

#endif /* CONFIG_H_ */
