/*
 * pca9538.h
 *
 *  Created on: Nov 17, 2024
 *      Author: andryssssss
 */

#ifndef INC_PCA9538_H_
#define INC_PCA9538_H_

#include <inttypes.h>
#include "stm32f4xx_hal.h"

typedef enum {
	INPUT_PORT = 0x00,
	OUTPUT_PORT = 0x01,
	POLARITY_INVERSION = 0x02,
	CONFIG = 0x03
} pca9538_regs_t;

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef PCA9538_Read_Register(uint16_t addr, pca9538_regs_t reg, uint8_t* buf);
HAL_StatusTypeDef PCA9538_Read_Inputs(uint16_t addr, uint8_t* buf);
HAL_StatusTypeDef PCA9538_Read_Config(uint16_t addr, uint8_t* buf);
HAL_StatusTypeDef PCA9538_Write_Register(uint16_t addr, pca9538_regs_t reg, uint8_t* buf);
HAL_StatusTypeDef PCA9538_SetDefaultConfig(uint16_t addr);

#endif /* INC_PCA9538_H_ */
