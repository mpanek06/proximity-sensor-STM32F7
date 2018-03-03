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


#endif /* CONFIG_H_ */
