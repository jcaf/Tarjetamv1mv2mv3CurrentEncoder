/*
 * main.c
 *
 *  Created on: May 22, 2021
 *      Author: jcaf
 */


#include <avr/io.h>
#include "src/types.h"
#include "src/system.h"
#include "src/i2c/I2C.h"
#include "src/i2c/I2CCommonFx.h"
#include "src/ads1115/ads1115.h"
#include "src/usart/usart.h"
#include "src/INA238/INA238.h"
#include "main.h"

volatile struct _isr_flag
{
    unsigned f10ms :1;
    unsigned __a :7;
} isr_flag = { 0,0 };

struct _main_flag
{
    unsigned f10ms :1;
    unsigned __a:7;

}main_flag = { 0,0 };

/*
 * #define MUX_AIN0_AIN3   0x01
    #define MUX_AIN1_AIN3   0x02
    #define MUX_AIN2_AIN3   0x03
 */
void ads1115(void)
{
    int16_t ib16;
    uint8_t reg[2];

    //
    char str[20];
    float v=0;
	while (1)
	{
		I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
		ib16 = (reg[0]<<8) + reg[1];
		itoa(ib16,  str,  10);
		strcat(str,"\n");
		//usart_println_string(str);
		//
		v = (ib16*2.048f/32768);
		dtostrf(v, 0, 3, str);
		strcat(str,"\n");
		usart_println_string(str);
		//

	}

//    //current = (ib16 * P_GAIN) + CURRENT_DEVIATION;//0.0101;
//    current = (ib16 * P_GAIN);
//    if (current > 0.005f)
//    {
//    	 current+= CURRENT_DEVIATION;//aplica +desviacion
//    }
//
//    current/=(RSHUNT*GM*RL);
//	return current;
}

char write_datax(unsigned char* indata, char bytes)
{
	unsigned char index, ack = 0;

	if(!twi_start_cond())
		return 0;
	if(!send_slave_address(WRITE))
		return 0;

	for(index = 0; index < bytes; index++)
	{
		 ack = i2c_write_byte(indata[index]);
		 if(!ack)
			break;
	}
	//put stop here
	write_scl(1);
	__delay_cycles(SCL_SDA_DELAY);
	write_sda(1);
	return ack;

}

void BB(void)
{
	int16_t ib16;
	uint8_t reg[3];
	char str[20];
	//

	twi_init();

	//1- config ADS115
	reg[0] = ADS1115_CONFIG_REG;
	reg[1] = (1<<OS_BIT) | (MUX_AIN1_AIN3<<MUX_BIT) | (PGA_2p048V<<PGA_BIT) | (CONTINUOUS_CONV<<MODE_BIT);
	reg[2] = (DR_8SPS<<DR_BIT);//Menor ruido
	write_data(reg,3);

	//2 - read
	reg[0] = ADS1115_CONVRS_REG;
	write_data(reg,1);

	while (1)
	{
		read_bytes(reg,2);
		ib16 = (reg[0]<<8) + reg[1];
		//
		itoa(ib16,  str,  10);
		strcat(str,"\n");
		usart_println_string(str);
	}

}

/*
 * A1=A2=Gnd
 */

//Secondary Device Address
#define INA238_ADDR_A1_A0_GND 0x40	//address

//Registers
#define INA238_CONFIG0 0
#define INA238_CONFIG1 1

void INA238(void)
{

	uint8_t reg[2];

	USART_Init ( (int)MYUBRR );


	////////////////////////////////////////////
	I2C_unimaster_init(100E3);//100KHz

	__delay_ms(10);
	reg[0] = 0;
	reg[1] = 1<<4;	//40.96mV

	I2Ccfx_WriteArray(0x80, INA238_CONFIG1, &reg[0], 2);

	while (1)
	{

		usart_println_string("INA238");
		__delay_ms(100);
	}
}
void xxx(void);
void ina238_test(void);
void ina238_ads1115_test(void);


//+- Encoder
uint16_t ENCODER_PPR = 500;    			//500 Pulses Per Revolution
float ENCODER_1REV_INMETERS = 0.5f;    	//1revol = X meters
volatile float ADQ_KMETERS = 0.15f;		//Adquirir cada "x metros"
float ENC_RESOL = (float)ENCODER_1REV_INMETERS/ENCODER_PPR;
//
typedef int64_t ROTARYCOUNT_T;
volatile ROTARYCOUNT_T rotaryCount = 0;
volatile ROTARYCOUNT_T rotaryCount_last = 0;	//toma el valor de rotaryCount para ser el nuevo punto de referencia de inicio
//Las sgtes. variables no necesitan ser de 64bits
int32_t numPulsesIn_ADQ_KMETERS = (ADQ_KMETERS * ENCODER_PPR) / ENCODER_1REV_INMETERS;//truncar
int32_t numPulses_diff = 0;
//
volatile float meters = 0.0f;
volatile int8_t captureData;

void interrupt(void)
{
	rotaryCount--;

	//-------------------------
	numPulses_diff = rotaryCount - rotaryCount_last;
	if (numPulses_diff >= numPulsesIn_ADQ_KMETERS)
	{
		rotaryCount_last = rotaryCount;
		meters = rotaryCount * ENC_RESOL;
		captureData = 1;
	}
}

int8_t ADS1115_capture_mvx(void)
{
	uint16_t ib16;
	float mVx=0;
	uint8_t reg[2];

	I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
	ib16 = (reg[0]<<8) + reg[1];
	ib16*=-1;

	mVx = (ib16*2.048f/32768);
	//
	return 1;

}

int main(void)
{
	char str[30];
	char buff[20];
	int16_t ib16;
	float mVx = 0;
	float current = 0;
	//
	int8_t sm0 = 0;
	uint16_t counter0;

	PinTo1(PORTWxOUT1,PINxOUT1);
	ConfigOutputPin(CONFIGIOxOUT1, PINxOUT1);

	PinTo0(PORTWxOUT2,PINxOUT2);
	ConfigOutputPin(CONFIGIOxOUT2, PINxOUT2);

	PinTo0(PORTWxBUZZER,PINxBUZZER);
	ConfigOutputPin(CONFIGIOxBUZZER, PINxBUZZER);


	USART_Init ( (int)MYUBRR );
	I2C_unimaster_init(400E3);//100KHz
	ADS1115_init();//ADS1115 in powerdown state
	INA238_init();//INA238_REG_CONFI to ± 40.96 mV --> INA238_1_LSB_STEPSIZE_ADCRANGE_40p96mV 1.25E-6 para todos los calculos

	while (1)
	{
		if (isr_flag.f10ms)
		{
			isr_flag.f10ms = 0;
			main_flag.f10ms = 1;
		}
		//----------------------

		if (sm0 == 0)
		{
			if (captureData)
			{
				PinTo0(PORTWxOUT1,PINxOUT1);

				captureData = 0;
				//
				counter0 = 0x0000;
				sm0++;
			}
		}
		else if (sm0 == 1)
		{
			if (main_flag.f10ms)
			{
				if (++counter0 >= 20)//20*10ms = 200ms
				{
					counter0 = 0;

					ADS1115_setMuxChannel(MUX_AIN0_AIN3);//mv1
					ADS1115_setOperatingMode(CONTINUOUS_CONV);
					sm0++;
				}
			}
		}
		else if (sm0 == 2)
		{
			if (main_flag.f10ms)
			{
				if (++counter0 >= 1)//setup Time new channel ADS1115
				{
					counter0 = 0;
					sm0++;
				}
			}
		}
		else if (sm0 == 3)
		{
			if (ADS1115_capture_mvx())
			{
				ADS1115_setOperatingMode(SINGLESHOT_POWERDOWN_CONV);//ADS1115 powerdown

				//enviar al host mv1 + recorrido
				sm0++;
			}
		}
		else if (sm0 == 4)
		{
			if (main_flag.f10ms)
			{
				if (++counter0 >= 10)//10*10ms = 100ms
				{
					PinTo1(PORTWxOUT2, PINxOUT2);

					counter0 = 0;
					sm0++;
				}
			}
		}
		else if (sm0 == 5)
		{
			if (main_flag.f10ms)
			{
				if (++counter0 >= 20)//20*10ms = 200ms
				{
					counter0 = 0;

					ADS1115_setMuxChannel(MUX_AIN1_AIN3);//mv2
					ADS1115_setOperatingMode(CONTINUOUS_CONV);

					sm0++;
				}
			}
		}
		else if (sm0 == 6)
		{
			if (main_flag.f10ms)
			{
				if (++counter0 >= 1)//setup Time new channel ADS1115
				{
					counter0 = 0;
					sm0++;
				}
			}
		}
		else if (sm0 == 7)
		{
			if (ADS1115_capture_mvx())
			{
				ADS1115_setOperatingMode(SINGLESHOT_POWERDOWN_CONV);//ADS1115 powerdown
				//
				current = INA238_read_current_register() * INA238_CURRENT_LSB;
				if (current > 0.0001f)
				{
					current +=1.7e-3;
				}

				//enviar al host mv2 + corriente actual
				sm0++;
			}
		}
		else if (sm0 == 8)
		{
			if (main_flag.f10ms)
			{
				if (++counter0 >= 20)//20*10ms = 200ms
				{
					counter0 = 0;

					ADS1115_setMuxChannel(MUX_AIN2_AIN3);//mv3
					ADS1115_setOperatingMode(CONTINUOUS_CONV);

					sm0++;
				}
			}
		}

		else if (sm0 == 9)
		{
			if (main_flag.f10ms)
			{
				if (++counter0 >= 1)//setup Time new channel ADS1115
				{
					counter0 = 0;
					sm0++;
				}
			}
		}
		else if (sm0 == 10)
		{
			if (ADS1115_capture_mvx())
			{
				ADS1115_setOperatingMode(SINGLESHOT_POWERDOWN_CONV);//ADS1115 powerdown
				//
				current = INA238_read_current_register() * INA238_CURRENT_LSB;
				if (current > 0.0001f)
				{
					current +=1.7e-3;
				}

				//enviar al host mv3 + corriente actual
				sm0++;
			}
		}
		else if (sm0 == 11)
		{
			if (main_flag.f10ms)
			{
				if (++counter0 >= 10)//10*10ms = 100ms
				{
					PinTo0(PORTWxOUT2, PINxOUT2);

					counter0 = 0;
					sm0++;
				}
			}
		}
		else if (sm0 == 12)
		{
			if (main_flag.f10ms)
			{
				if (++counter0 >= 10)//10*10ms = 100ms
				{
					PinTo1(PORTWxOUT1, PINxOUT1);

					counter0 = 0;
					sm0 = 0x0000;
				}
			}
		}

		//clear flags
		main_flag.f10ms = 0;
	}
}


void xxx(void)
{
	uint8_t reg[2];
	int16_t ib16;
    char str[20];
    float v=0;

	USART_Init ( (int)MYUBRR );
	I2C_unimaster_init(400E3);//100KHz

	//ADS115 setup
	reg[0] = (1<<OS_BIT) | (MUX_AIN1_AIN3<<MUX_BIT) | (PGA_2p048V<<PGA_BIT) | (CONTINUOUS_CONV<<MODE_BIT);
	reg[1] = (DR_8SPS<<DR_BIT);//Menor ruido
	I2Ccfx_WriteArray(ADS115_ADR_GND, ADS1115_CONFIG_REG, &reg[0], 2);

	while (1)
	{
		__delay_ms(10);
		reg[0] = 0;
		reg[1] = 1<<4;	//40.96mV
		I2Ccfx_WriteArray(0x80, INA238_CONFIG1, &reg[0], 2);


		//////////////////
//		reg[0] = (1<<OS_BIT) | (MUX_AIN1_AIN3<<MUX_BIT) | (PGA_2p048V<<PGA_BIT) | (CONTINUOUS_CONV<<MODE_BIT);
//		reg[1] = (DR_8SPS<<DR_BIT);//Menor ruido
//		I2Ccfx_WriteArray(ADS115_ADR_GND, ADS1115_CONFIG_REG, &reg[0], 2);

		//
		I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
		ib16 = (reg[0]<<8) + reg[1];
		itoa(ib16,  str,  10);
		strcat(str,"\n");
		//usart_println_string(str);
		//
		v = (ib16*2.048f/32768);
		dtostrf(v, 0, 3, str);
		strcat(str,"\n");
		usart_println_string(str);
		//
	}
	////////////

}


void ina238_test(void)
{
	char str[30];
	char buff[20];
	int16_t curr_reg = 0;
	float current = 0;

	USART_Init ( (int)MYUBRR );
	I2C_unimaster_init(400E3);//100KHz


	INA238_init();//INA238_REG_CONFI to ± 40.96 mV --> INA238_1_LSB_STEPSIZE_ADCRANGE_40p96mV 1.25E-6 para todos los calculos
	while (1)
	{
		float shuntvoltage = INA238_read_shuntvoltage_register() * INA238_1_LSB_STEPSIZE_ADCRANGE_40p96mV;

		curr_reg = INA238_read_current_register();
	    current = curr_reg * INA238_CURRENT_LSB;


	    if (current > 0.0001f)
		{
			current +=1.7e-3;
		}

	    /*
		strcpy(str, "shuntvoltage: ");
		dtostrf(shuntvoltage, 0, 4, buff);
		strcat(str,buff);
		strcat(str,"\n");
		usart_println_string(str);
		*/
		//

		strcpy(str, "INA238 current: ");
		dtostrf(current, 0, 4, buff);
		strcat(str,buff);
		strcat(str,"\n");
		usart_println_string(str);
		//


	}
}

void ina238_ads1115_test(void)
{
	char str[30];
	char buff[20];
	int16_t ib16;
	float mVx = 0;
	float current = 0;
	//float shuntvoltage = 0;

	USART_Init ( (int)MYUBRR );
	I2C_unimaster_init(400E3);//100KHz

	uint8_t reg[2];
	//ADS115 setup
	reg[0] = (1<<OS_BIT) | (MUX_AIN1_AIN3<<MUX_BIT) | (PGA_2p048V<<PGA_BIT) | (CONTINUOUS_CONV<<MODE_BIT);
	reg[1] = (DR_8SPS<<DR_BIT);//Menor ruido
	I2Ccfx_WriteArray(ADS115_ADR_GND, ADS1115_CONFIG_REG, &reg[0], 2);

	//INA238 setup
	INA238_init();//INA238_REG_CONFI to ± 40.96 mV --> INA238_1_LSB_STEPSIZE_ADCRANGE_40p96mV 1.25E-6 para todos los calculos

	while (1)
	{
		//shuntvoltage = INA238_read_shuntvoltage_register() * INA238_1_LSB_STEPSIZE_ADCRANGE_40p96mV;
	    current = INA238_read_current_register() * INA238_CURRENT_LSB;
	    if (current > 0.0001f)
		{
			current +=1.7e-3;
		}
	    /*
		strcpy(str, "shuntvoltage: ");
		dtostrf(shuntvoltage, 0, 4, buff);
		strcat(str,buff);
		strcat(str,"\n");
		usart_println_string(str);
		*/
		//

		strcpy(str, "INA238 current: ");
		dtostrf(current, 0, 4, buff);
		strcat(str,buff);
		strcat(str,"\n");
		usart_println_string(str);

		//
		//
		I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
		ib16 = (reg[0]<<8) + reg[1];
		ib16*=-1;
		mVx = (ib16*2.048f/32768);
		dtostrf(mVx, 0, 4, buff);
		//
		strcpy(str, "ADS1115 mVx: ");
		strcat(str,buff);
		strcat(str,"\n");
		usart_println_string(str);
		//

	}
}
//0......1m.......2....
/*
 * CUANDO ARRANCA EL SISTEMA ACTIVA OUT1, DESACTIVA OUT2
 *
 * EL BUZZER HACE UN TICK EN CADA CAPTURA, Y EL SOFTWARE TAMBIEN
 *
 * KTE DE TIEMPO VARIA HASTA 10 SEG....
 *
 * EL CUADRO POSICION SIEMPRE MUESTRA LA POSICION ACTUAL
 *
 * AL EXPOTAR, GENERAR EL SGTE FORMATO

 // TITULO DE CABACERA
 * [MV1,DESP],[MV2,CORRIENTE,DESP],[MV3,CORRIENTE,DESP]
 */
void seqfunc(void)
{
	while (1)
	{
		/*
		 En todos los casos capturo desplazaimento


		intervalo = 1 m
		desactivo OUT1
		transcurre un tiempo de 200ms,
		capturo mv1 +
		transcurre un tiempo de 100ms,
		luego activa OUT2,
		LUEGO DE 200ms capturo [mv2 + corriente]
		despues de 200ms capturo [mv3+corriente]
		transcurre un tiempo de 100ms,desactivo OUT2
		transcurre un tiempo de 100ms, y de alli activo OUT1
		y asi secuencialmente

		 */
		;

	}
	//////////////////////
}
