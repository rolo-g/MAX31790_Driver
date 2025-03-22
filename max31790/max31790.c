/*
 * max31790.c
 *
 *  Created on: Aug 2, 2024
 *      Author: rolo
 */

/* Includes *******************************************************************/

#include "stm32f0xx_hal.h"
#include <stdbool.h>
#include "max31790.h"

/* Functions ******************************************************************/

/* Resets controller to its default state and initializes desired settings */
void initMAX31790(fanController *ctrl)
{
	/* TODO Replace the magic numbers */

	/* Resets the controller */
	uint8_t data = (1 << 6);
	HAL_I2C_Mem_Write(ctrl->hi2cx, ctrl->i2c_addr,
					GLOBAL_CONF_REG, I2C_MEMADD_SIZE_8BIT, &data,
					sizeof(data), HAL_MAX_DELAY);

	HAL_Delay(1);

	/* TODO Move this to a fan config function */
	/* Enables TACH Input bit in Fan 1 Config. Register */
	data = (1 << 3);
	HAL_I2C_Mem_Write(ctrl->hi2cx, ctrl->i2c_addr,
					FAN1_CONF_REG, I2C_MEMADD_SIZE_8BIT, &data,
					sizeof(data), HAL_MAX_DELAY);
}

/* Sets the PWM Duty Cycle of a fan, given its controller and channel */
void setFanPWM(fanController *ctrl, channel ch, uint16_t dc)
{
	bool isValid = false;	/* Flag for checking valid values */

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
				(PWMOUT1_TGT_DC_MSB_REG + (ch * 2)), I2C_MEMADD_SIZE_8BIT, data,
				sizeof(data), HAL_MAX_DELAY);
	}
	else
	{
		/* TODO Potentially add a way to inform user of error */
	}
}

/* Gets the RPM value of the fan from the Tachometer input */
uint16_t getFanRPM(fanController *ctrl, channel ch)
{
	uint8_t data[2] = {0, 0};	/* Value to store register data */
	uint16_t rpm = 0;			/* Return value for RPM */
	bool isValid = false;		/* Flag for checking valid values */

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
				(TACH1_COUNT_MSB_REG + (ch * 2)), I2C_MEMADD_SIZE_8BIT, data,
				sizeof(data), HAL_MAX_DELAY);

		/* Calculates RPM using the formula in page 11 of the data sheet */
		rpm = (60 * SR * (8192)) / (NP *
				(data[0] << TACH_MSB_SHIFT) | (data[1] >> TACH_LSB_SHIFT));

		/* Sets the RPM to zero if it has hit the minimum RPM_MIN */
		if (rpm <= RPM_MIN)
		{
			rpm = 0;
		}
	}
	else
	{
		/* TODO Potentially add a way to inform user of error */
	}

	/* Returns calculated RPM */
	return rpm;
}

