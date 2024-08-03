/*
 * max31790.h
 *
 *  Created on: Aug 2, 2024
 *      Author: rolo
 */

#ifndef INC_MAX31790_H_
#define INC_MAX31790_H_

/********************************** Includes **********************************/

#include "stm32f0xx_hal.h"
#include <stdbool.h>

/*********************************** Macros ***********************************/

/* Value Limits */

/* Duty Cycle */
#define DC_MIN		0
#define DC_MAX		511

/* Bit Manipulation */

/* Bit Shifts */
#define TACH_MSB_SHIFT		3
#define TACH_LSB_SHIFT		5
#define PWM_MSB_SHIFT		1
#define PWM_LSB_SHIFT		7

/* Bit Masks */
#define TACH_LSB_BIT_MASK		0xE0
#define PWM_LSB_BIT_MASK		0x80

/* MAX31790 Addresses and Registers */

/* I2C Slave Address Table */
#define GND_GND_ADDR	0x40
#define GND_SCL_ADDR	0x42
#define GND_SDA_ADDR	0x44
#define GND_VCC_ADDR	0x44
#define SCL_GND_ADDR	0x48
#define	SCL_SCL_ADDR	0x4A
#define	SCL_SDA_ADDR	0x4C
#define	SCL_VCC_ADDR	0X4E
#define SDA_GND_ADDR	0x50
#define	SDA_SCL_ADDR	0x52
#define	SDA_SDA_ADDR	0x54
#define	SDA_VCC_ADDR	0x56
#define VCC_GND_ADDR	0x58
#define	VCC_SCL_ADDR	0x5A
#define	VCC_SDA_ADDR	0x5C
#define	VCC_VCC_ADDR    0X5E

/* General Configuration */
#define GLOBAL_CONF_REG		0x00
#define PWM_FREQ_REG		0x01

/* Fan Configuration Registers */
#define	FAN1_CONF_REG		0x02
#define FAN2_CONF_REG		0x03
#define FAN3_CONF_REG		0x04
#define	FAN4_CONF_REG		0x05
#define FAN5_CONF_REG		0x06
#define FAN6_CONF_REG		0x07

/* Fan Dynamics Registers */
#define FAN1_DYNA_REG		0x08
#define FAN2_DYNA_REG		0x09
#define FAN3_DYNA_REG		0x0A
#define FAN4_DYNA_REG		0x0B
#define FAN5_DYNA_REG		0x0C
#define	FAN6_DYNA_REG		0x0D

/* Fan Fault Registers */
#define	FAN_FLT_STAT2_REG   		0x10
#define FAN_FLT_STAT1_REG			0x11
#define FAN_FLT_MASK2_REG			0x12
#define FAN_FLT_MASK1_REG			0x13
#define	FAIL_FAN_SEQ_STRT_REG		0x14

/* Tachometer Count Registers */
#define TACH1_COUNT_MSB_REG	    0x18
#define TACH1_COUNT_LSB_REG	    0x19
#define TACH2_COUNT_MSB_REG	    0x1A
#define TACH2_COUNT_LSB_REG	    0x1B
#define TACH3_COUNT_MSB_REG	    0x1C
#define TACH3_COUNT_LSB_REG	    0x1D
#define TACH4_COUNT_MSB_REG	    0x1E
#define TACH4_COUNT_LSB_REG	    0x1F
#define TACH5_COUNT_MSB_REG	    0x20
#define TACH5_COUNT_LSB_REG	    0x21
#define TACH6_COUNT_MSB_REG	    0x22
#define TACH6_COUNT_LSB_REG	    0x23
#define TACH7_COUNT_MSB_REG		0x24
#define TACH7_COUNT_LSB_REG	    0x25
#define TACH8_COUNT_MSB_REG    	0x26
#define TACH8_COUNT_LSB_REG    	0x27
#define TACH9_COUNT_MSB_REG    	0x28
#define TACH9_COUNT_LSB_REG    	0x29
#define TACH10_COUNT_MSB_REG	0x2A
#define TACH10_COUNT_LSB_REG   	0x2B
#define TACH11_COUNT_MSB_REG   	0x2C
#define TACH11_COUNT_LSB_REG   	0x2D
#define TACH12_COUNT_MSB_REG   	0x2E
#define TACH12_COUNT_LSB_REG    0x2F

/* PWM Output Duty Cycle */
#define PWMOUT1_DC_MSB_REG    	0x30
#define PWMOUT1_DC_LSB_REG    	0x31
#define PWMOUT2_DC_MSB_REG    	0x32
#define PWMOUT2_DC_LSB_REG    	0x33
#define PWMOUT3_DC_MSB_REG    	0x34
#define PWMOUT3_DC_LSB_REG    	0x35
#define PWMOUT4_DC_MSB_REG    	0x36
#define PWMOUT4_DC_LSB_REG    	0x37
#define PWMOUT5_DC_MSB_REG		0x38
#define PWMOUT5_DC_LSB_REG    	0x39
#define PWMOUT6_DC_MSB_REG    	0x3A
#define PWMOUT6_DC_LSB_REG    	0x3B

/* PWM Output Target Duty Cycle */
#define PWMOUT1_TGT_DC_MSB_REG		0x40
#define PWMOUT1_TGT_DC_LSB_REG	    0x41
#define PWMOUT2_TGT_DC_MSB_REG		0x42
#define PWMOUT2_TGT_DC_LSB_REG    	0x43
#define PWMOUT3_TGT_DC_MSB_REG    	0x44
#define PWMOUT3_TGT_DC_LSB_REG    	0x45
#define PWMOUT4_TGT_DC_MSB_REG    	0x46
#define PWMOUT4_TGT_DC_LSB_REG    	0x47
#define PWMOUT5_TGT_DC_MSB_REG    	0x48
#define PWMOUT5_TGT_DC_LSB_REG    	0x49
#define PWMOUT6_TGT_DC_MSB_REG    	0x4A
#define PWMOUT6_TGT_DC_LSB_REG    	0x4B

/* Tachometer Output Count */
#define TACH1_TGT_COUNT_MSB_REG		0x50
#define TACH1_TGT_COUNT_LSB_REG   	0x51
#define TACH2_TGT_COUNT_MSB_REG   	0x52
#define TACH2_TGT_COUNT_LSB_REG   	0x53
#define TACH3_TGT_COUNT_MSB_REG   	0x54
#define TACH3_TGT_COUNT_LSB_REG   	0x55
#define TACH4_TGT_COUNT_MSB_REG   	0x56
#define TACH4_TGT_COUNT_LSB_REG   	0x57
#define TACH5_TGT_COUNT_MSB_REG   	0x58
#define TACH5_TGT_COUNT_LSB_REG   	0x59
#define TACH6_TGT_COUNT_MSB_REG   	0x5A
#define TACH6_TGT_COUNT_LSB_REG   	0x5B

/* Window */
#define WINDOW1_REG		0x60
#define WINDOW2_REG		0x62
#define WINDOW3_REG		0x62
#define WINDOW4_REG		0x63
#define	WINDOW5_REG		0x64
#define	WINDOW6_REG		0x65

/****************************** Type Definitions ******************************/

/* Contains information about a specific fan controller */
typedef struct
{
	I2C_HandleTypeDef *hi2cx;		/* I2C handler of the controller */
	uint8_t i2c_addr;				/* The I2C address of the controller*/
} Fan_Controller;

/* Allows for easy reference of each individual channel */
typedef enum
{
	CH1,
	CH2,
	CH3,
	CH4,
	CH5,
	CH6
} Channel;

/********************************* Functions **********************************/

void Init_Controller(Fan_Controller *ctrl);
void Set_Fan_PWM(Fan_Controller *ctrl, Channel ch, uint16_t dc);
uint16_t Get_Fan_RPM(Fan_Controller *ctrl, Channel ch);

#endif /* INC_MAX31790_H_ */
