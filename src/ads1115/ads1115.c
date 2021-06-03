#include "../system.h"
#include "../types.h"
#include "../i2c/I2C.h"
#include "../i2c/I2CCommonFx.h"
#include "ads1115.h"
/*
 * En vez de leer el valor actual, llevo en la RAM una "copia" en una variable global. Hace uso de variable de 16bits
 * Si solo se usa en un solo modo durante todo el programa, entonces se puede ahorrar en RAM y FLASH deshabilitando
 * todas las funciones y variable a traves de la directiva ADS1115_CONTROL_BITS
 */

void ADS1115_write_data16bits(uint8_t ADS1115_I2Caddr, uint8_t ADS1115register, uint16_t data16bits)
{
	uint8_t reg[2];
	reg[0] = (uint8_t)(data16bits>>8);
	reg[1] = (uint8_t)(data16bits);
	I2Ccfx_WriteArray(ADS1115_I2Caddr, ADS1115register, &reg[0], 2);

}
#ifdef ADS1115_CONFIG_REG_UPDATE_IN_RUNTIME
	uint16_t ADS1115_ConfigRegister = 0x0000;
#endif

	void ADS1115_init(void)
	{
		#ifndef ADS1115_CONTROL_BITS
		uint16_t
		#endif
		ADS1115_ConfigRegister = (0<<OS_BIT) | (MUX_AIN0_AIN3<<MUX_BIT) | (PGA_2p048V<<PGA_BIT) | (SINGLESHOT_POWERDOWN_CONV<<MODE_BIT) | (DR_128SPS<<DR_BIT);
		ADS1115_write_data16bits(ADS115_ADR_GND, ADS1115_CONFIG_REG, ADS1115_ConfigRegister);

	}
#ifdef ADS1115_CONFIG_REG_UPDATE_IN_RUNTIME
	void ADS1115_setOS(uint8_t OS)
	{
		ADS1115_ConfigRegister = (ADS1115_ConfigRegister & 0x7FFF) | ((uint16_t)OS<<OS_BIT);
		ADS1115_write_data16bits(ADS115_ADR_GND, ADS1115_CONFIG_REG, ADS1115_ConfigRegister);
	}
	/*
	 *
	 */
	void ADS1115_setMuxChannel(uint8_t MUX)
	{
		ADS1115_ConfigRegister = (ADS1115_ConfigRegister & 0x8FFF) | ((uint16_t)MUX<<MUX_BIT);
		ADS1115_write_data16bits(ADS115_ADR_GND, ADS1115_CONFIG_REG, ADS1115_ConfigRegister);
	}
	/*
	 *
	 */
	void ADS1115_setPGA(uint8_t PGA)
	{
		ADS1115_ConfigRegister = (ADS1115_ConfigRegister & 0xF1FF) | ((uint16_t)PGA<<PGA_BIT);
		ADS1115_write_data16bits(ADS115_ADR_GND, ADS1115_CONFIG_REG, ADS1115_ConfigRegister);
	}
	/*
	 *
	 */
	void ADS1115_setOperatingMode(uint8_t MODE)
	{
		ADS1115_ConfigRegister = (ADS1115_ConfigRegister & 0xFEFF) | ((uint16_t)MODE<<MODE_BIT);
		ADS1115_write_data16bits(ADS115_ADR_GND, ADS1115_CONFIG_REG, ADS1115_ConfigRegister);
	}
	/*
	 *
	 */
	void ADS1115_setDataRate(uint8_t DR)
	{
		ADS1115_ConfigRegister = (ADS1115_ConfigRegister & 0xFF1F) | ((uint16_t)DR<<DR_BIT);
		ADS1115_write_data16bits(ADS115_ADR_GND, ADS1115_CONFIG_REG, ADS1115_ConfigRegister);
	}

#endif
