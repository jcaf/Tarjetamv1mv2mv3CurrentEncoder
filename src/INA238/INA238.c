/*
 * INA238.c
 *
 *  Created on: May 31, 2021
 *      Author: jcaf
 */
#include "../system.h"
#include "../types.h"
#include "../i2c/I2C.h"
#include "../i2c/I2CCommonFx.h"
#include "INA238.h"
/*
 * Register bytes are sent most-significant byte first, followed by the least significant byte.
 *
 */
void INA238_init(void)
{
	uint16_t ina238_reg;
	uint8_t reg[2];

	//INA238_REG_CONFIG: 1h = ± 40.96 mV --> INA238_1_LSB_STEPSIZE_ADCRANGE_40p96mV 1.25E-6 para todos los calculos
	ina238_reg = (IN238_CONFIG_ADCRANGE_40p96mV<<INA238_CONFIG_BIT_ADCRANGE);
	reg[0] = (uint8_t)(ina238_reg >> 8);//MSB
	reg[1] = (uint8_t)ina238_reg;	//LSB
	I2Ccfx_WriteArray(INA238_I2C_ADDR_A1_GND_A2_GND, INA238_REG_CONFIG, &reg[0], 2);

	////////////////////////////////////
	ina238_reg = 	(INA238_ADC_CONFIGMODE_CONTINUOUS_SHUNT_ONLY<<INA238_ADC_CONFIG_BIT_MODE) | \
		 	(INA238_ADC_CONFIGMODE_CT_150uS<<INA238_ADC_CONFIG_BIT_VSHCT) | \
			(INA238_ADC_CONFIGMODE_AVG_SAMPLE_1024 << INA238_ADC_CONFIG_BIT_AVG);
	reg[0] = (uint8_t)(ina238_reg >> 8);//MSB
	reg[1] = (uint8_t)(ina238_reg);	//LSB
	I2Ccfx_WriteArray(INA238_I2C_ADDR_A1_GND_A2_GND, INA238_REG_ADC_CONFIG, &reg[0], 2);

	/////////////////////////////////////
	reg[0] = (uint8_t)( ((uint16_t)INA238_SHUNT_CAL_ADCRANGE_40p96mV)>>8);
	reg[1] = (uint8_t)(INA238_SHUNT_CAL_ADCRANGE_40p96mV);
	I2Ccfx_WriteArray(INA238_I2C_ADDR_A1_GND_A2_GND, INA238_REG_SHUNT_CAL, &reg[0], 2);
}
int16_t INA238_read_shuntvoltage_register(void)
{
	uint8_t reg[2];
	I2Ccfx_ReadRegistersAtAddress(INA238_I2C_ADDR_A1_GND_A2_GND, INA238_REG_VSHUNT, &reg[0], 2);
	int16_t shuntvoltage = (reg[0]<<8) | reg[1];
	return shuntvoltage;
}
int16_t INA238_read_current_register(void)
{
	uint8_t reg[2];
	I2Ccfx_ReadRegistersAtAddress(INA238_I2C_ADDR_A1_GND_A2_GND, INA238_REG_CURRENT, &reg[0], 2);
	int16_t current = (reg[0]<<8) | reg[1];
	return current;

}
