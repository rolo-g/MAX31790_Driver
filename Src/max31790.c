/*
 * max31790.c
 *
 *  Created on: Aug 2, 2024
 *      Author: rolo
 */

/********************************** Includes **********************************/

#include "stm32f0xx_hal.h"
#include <stdbool.h>
#include "max31790.h"

/********************************* Functions **********************************/

/* Resets controller to its default state and initializes desired settings */
void Init_Controller(Fan_Controller *ctrl)
{
	/* TODO Replace the magic numbers */

	uint8_t data = 0b01100000;
	HAL_I2C_Mem_Write(ctrl->hi2cx, ctrl->i2c_addr,
					GLOBAL_CONF_REG, I2C_MEMADD_SIZE_8BIT, &data,
					sizeof(data), HAL_MAX_DELAY);

	data = 0b00000000;
	HAL_I2C_Mem_Write(ctrl->hi2cx, ctrl->i2c_addr,
					FAN1_CONF_REG, I2C_MEMADD_SIZE_8BIT, &data,
					sizeof(data), HAL_MAX_DELAY);

	data = 0b00101100;
	HAL_I2C_Mem_Write(ctrl->hi2cx, ctrl->i2c_addr,
					FAN1_DYNA_REG, I2C_MEMADD_SIZE_8BIT, &data,
						sizeof(data), HAL_MAX_DELAY);
}

/* Sets the PWM Duty Cycle of a fan, given its controller and channel */
void Set_Fan_PWM(Fan_Controller *ctrl, Channel ch, uint16_t dc)
{
	/* Flag for checking valid values */
	bool isValid = false;

	/* Checks to see if the duty cycle is within range */
	if ((dc >= DC_MIN) && (dc <= DC_MAX))
	{
		/* Checks to see if channel is valid */
		if ((ch >= CH1) && (ch <= CH6))
		{
			/* Will only reach here if all conditions are met */
			isValid = true;
		}
	}

	/* Will begin to send the I2C data if conditions were met */
	if (isValid)
	{
		/* Builds the data to be stored in the PWMOUT register */
		uint8_t data[2] = {
				(dc >> PWM_MSB_SHIFT),
				((dc << PWM_LSB_SHIFT) & PWM_LSB_BIT_MASK)
		};

		/* Writes the data to the appropriate register */
		HAL_I2C_Mem_Write(ctrl->hi2cx, ctrl->i2c_addr,
				(PWMOUT1_TGT_DC_MSB_REG + ch), I2C_MEMADD_SIZE_8BIT, data,
				sizeof(data), HAL_MAX_DELAY);
	}
	else
	{
		/* TODO Potentially add a way to inform user of error */
	}
}

/* Gets the RPM value of the fan from the Tachometer input */
uint16_t Get_Fan_RPM(Fan_Controller *ctrl, Channel ch)
{
	/* Value to store register data */
	uint8_t data[2];

	/* Flag for checking valid values */
	bool isValid = false;

	/* Checks to see if channel is valid */
	if ((ch >= CH1) && (ch <= CH6))
	{
		/* Will only reach here if all conditions are met */
		isValid = true;
	}

	/* Will begin to send the I2C data if conditions were met */
	if (isValid)
	{
		/* Reads the data from the appropriate register */
		HAL_I2C_Mem_Read(ctrl->hi2cx, ctrl->i2c_addr,
				(TACH1_COUNT_MSB_REG + ch), I2C_MEMADD_SIZE_8BIT, data,
				sizeof(data), HAL_MAX_DELAY);
	}
	else
	{
		/* TODO Potentially add a way to inform user of error */
	}

	/* Returns RPM by getting the data values and shifting to correct values */
	return ((data[0] << TACH_MSB_SHIFT) | (data[1] >> TACH_LSB_SHIFT));
}

