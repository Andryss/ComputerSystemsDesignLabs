/*
 * pca9538.c
 *
 *  Created on: Nov 17, 2024
 *      Author: andryssssss
 */

#include "pca9538.h"

HAL_StatusTypeDef PCA9538_Read_Register(uint16_t addr, pca9538_regs_t reg, uint8_t* buf) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, addr | 1, reg, 1, buf, 1, 100);
	if (status != HAL_OK){
		// UART_Transmit("Can't read from PCA9538 register\n");
	}
	return status;
}

HAL_StatusTypeDef PCA9538_Read_Inputs(uint16_t addr, uint8_t* buf) {
	HAL_StatusTypeDef status = PCA9538_Read_Register(addr, INPUT_PORT, buf);
	if (status != HAL_OK){
		// UART_Transmit("Can't read from PCA9538 input port\n");
	}
	return status;
}

HAL_StatusTypeDef PCA9538_Read_Config(uint16_t addr, uint8_t* buf) {
	HAL_StatusTypeDef status = PCA9538_Read_Register(addr, CONFIG, buf);
	if (status != HAL_OK){
		// UART_Transmit("Can't read from PCA9538 config\n");
	}
	return status;
}


HAL_StatusTypeDef PCA9538_Write_Register(uint16_t addr, pca9538_regs_t reg, uint8_t* buf) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, addr & 0xFFFE, reg, 1, buf, 1, 100);
	if (status != HAL_OK){
		// UART_Transmit("Can't write to PCA9538 register\n");
	}
	return status;
}

static uint8_t OUTPUT_PORT_DEFAULT = 0xFF;
static uint8_t POLARITY_INVERSION_DEFAULT = 0x00;
static uint8_t CONFIG_DEFAULT = 0xF0;

HAL_StatusTypeDef PCA9538_SetDefaultConfig(uint16_t addr) {
	HAL_StatusTypeDef status;

	status = PCA9538_Write_Register(addr, OUTPUT_PORT, &OUTPUT_PORT_DEFAULT);
	if (status != HAL_OK) {
		// UART_Transmit("Can't write to PCA9538 output port\n");
		return status;
	}

	status = PCA9538_Write_Register(addr, POLARITY_INVERSION, &POLARITY_INVERSION_DEFAULT);
	if (status != HAL_OK) {
		// UART_Transmit("Can't write to PCA9538 polarity inversion\n");
		return status;
	}

	status = PCA9538_Write_Register(addr, CONFIG, &CONFIG_DEFAULT);
	if (status != HAL_OK) {
		// UART_Transmit("Can't write to PCA9538 config\n");
	}
	return status;
}
