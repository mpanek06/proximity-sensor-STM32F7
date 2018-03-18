/*
 * config.h
 *
 *  Created on: Feb 15, 2018
 *      Author: Marcin Panek
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define CAM_IMG_WIDTH  320
#define CAM_IMG_HEIGHT 240
#define CAM_IMG_SIZE   (CAM_IMG_WIDTH * CAM_IMG_HEIGHT)
/* Image size in byte, each pixel in RGB565 takes 2 bytes*/
#define CAM_IMG_SIZE_B (CAM_IMG_WIDTH * CAM_IMG_HEIGHT * 2)


#define SET_DEBUG_PIN4    ( GPIOG->ODR |= ARDUINO_D4_Pin )
#define RESET_DEBUG_PIN4  ( GPIOG->ODR &= ~(ARDUINO_D4_Pin) )

#define SET_DEBUG_PIN2    ( GPIOG->ODR |= ARDUINO_D2_Pin )
#define RESET_DEBUG_PIN2  ( GPIOG->ODR &= ~(ARDUINO_D2_Pin) )

#endif /* CONFIG_H_ */
