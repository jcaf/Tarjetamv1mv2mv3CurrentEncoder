/*
 * main.c
 *
 *  Created on: May 22, 2021
 *      Author: jcaf
 *
 *      0......1m.......2....
 CUANDO ARRANCA EL SISTEMA ACTIVA OUT1, DESACTIVA OUT2
 EL BUZZER HACE UN TICK EN CADA CAPTURA, Y EL SOFTWARE TAMBIEN
 KTE DE TIEMPO VARIA HASTA 10 SEG....
 EL CUADRO POSICION SIEMPRE MUESTRA LA POSICION ACTUAL
 AL EXPOTAR, GENERAR EL SGTE FORMATO

 TITULO DE CABACERA
 * [MV1,DESP],[MV2,CORRIENTE,DESP],[MV3,CORRIENTE,DESP]

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

 *
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

float mv1;
float mv2;
float mv3;
float current;
float position;

//+- Encoder
uint16_t ENCODER_PPR = 500;    			//500 Pulses Per Revolution
float ENCODER_1REV_INMETERS = 0.5f;    	//1revol = X meters
volatile float ADQ_KMETERS = 0.15f;		//Adquirir cada "x metros"
float ENC_RESOL = 0;// = (float)ENCODER_1REV_INMETERS/ENCODER_PPR;
//
typedef int64_t ROTARYCOUNT_T;
volatile ROTARYCOUNT_T rotaryCount = 0;
volatile ROTARYCOUNT_T rotaryCount_last = 0;	//toma el valor de rotaryCount para ser el nuevo punto de referencia de inicio
//Las sgtes. variables no necesitan ser de 64bits
int32_t numPulsesIn_ADQ_KMETERS = 0; //(ADQ_KMETERS * ENCODER_PPR) / ENCODER_1REV_INMETERS;//truncar
int32_t numPulses_diff = 0;
//
volatile float recorrido = 0.0f;
volatile int8_t captureData;
volatile int8_t sendRecorrido;

struct _job
{
	int8_t sm0;//x jobs
	//int8_t key_sm0;//x keys
	uint16_t counter;
	//int8_t mode;

	struct _job_f
	{
		unsigned enable:1;
		unsigned job:1;
		unsigned lock:1;
		unsigned __a:5;
	}f;
};

#define SMOOTHALG_MAXSIZE 10L

struct _job emptyJob;
struct _job smoothAlgJob;
struct _job capturemvx;
struct _job sequencemain;
struct _job buzzer;

#define BUZZER_KTIME 30//10e-3
void buzzer_job(void);

int8_t smoothAlg_nonblock(int16_t *buffer, float *Answer);

int8_t ADS1115_capture_mvx(float *mvx)
{
	int16_t ib16;
	float smoothAnswer;
	uint8_t reg[2];
	//
	static int16_t smoothVector[SMOOTHALG_MAXSIZE];

	if (capturemvx.sm0 == 0)
	{
		I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
		ib16 = (reg[0]<<8) + reg[1];
		smoothVector[capturemvx.counter] = ib16;
		if (++capturemvx.counter >= SMOOTHALG_MAXSIZE)
		{
			capturemvx.counter = 0x00;
			capturemvx.sm0++;
		}
	}
	else if (capturemvx.sm0 == 1)
	{
		if (smoothAlg_nonblock(smoothVector, &smoothAnswer))
		{
			smoothAnswer*=-1;
			*mvx = (smoothAnswer*2.048f/32768);

			capturemvx.sm0 = 0x0;
			return 1;
		}
	}
	//
	return 0;

}
/*
 *
 */
#define USB_DATACODE_MV1 'X'
#define USB_DATACODE_MV2 'Y'
#define USB_DATACODE_MV3 'Z'
#define USB_DATACODE_CURRENT 'C'
#define USB_DATACODE_POSITION 'P'

void USB_send_data(char datacode, float payload0)
{
	char str[30];
	char buff[30];
	strcpy(str,"@");
	buff[0] = datacode;
	buff[1] = '\0';
	strcat(str,buff);
	dtostrf(payload0, 0, 4, buff);
	strcat(str,buff);
	strcat(str,"\r");
	usart_println_string(str);
}

int main(void)
{

	char str[30];
	char buff[20];
	int16_t ib16;
	float mVx = 0;
	float current = 0;
	//
	//int8_t sm0 = 0;
	//uint16_t counter0;

	PinTo1(PORTWxOUT1,PINxOUT1);
	ConfigOutputPin(CONFIGIOxOUT1, PINxOUT1);

	PinTo0(PORTWxOUT2,PINxOUT2);
	ConfigOutputPin(CONFIGIOxOUT2, PINxOUT2);

	PinTo0(PORTWxBUZZER,PINxBUZZER);
	ConfigOutputPin(CONFIGIOxBUZZER, PINxBUZZER);

	//
	ENC_RESOL = (float)ENCODER_1REV_INMETERS/ENCODER_PPR;
	numPulsesIn_ADQ_KMETERS = (ADQ_KMETERS * ENCODER_PPR) / ENCODER_1REV_INMETERS;//truncar
	//

	USART_Init ( (int)MYUBRR );

	////Encoder setup Atmega328P, external Pull-ups 1K
	////channel A = PD2 INT0
	////channel B = PD3 INT1
	//EICRA = 0x0F;//rising edge on INT1, INT0
	//EIMSK = 0x03;//Enable interrupt on INT1, INT0
	//sei();
	//while (1);
	////


	I2C_unimaster_init(400E3);//100KHz
	ADS1115_init();//ADS1115 in powerdown state
	INA238_init();//INA238_REG_CONFI to ± 40.96 mV --> INA238_1_LSB_STEPSIZE_ADCRANGE_40p96mV 1.25E-6 para todos los calculos

	//Atmega328P TCNT0 CTC mode
    TCNT0 = 0x0000;
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS02) | (0 << CS01) | (0 << CS00); //CTC PRES=256
    OCR0A = CTC_SET_OCR_BYTIME(10E-3,256);
    TIMSK0 |= (1 << OCIE0A);
    sei();


	while (1)
	{
		if (isr_flag.f10ms)
		{
			isr_flag.f10ms = 0;
			main_flag.f10ms = 1;
		}
		//----------------------
		if (sendRecorrido)
		{
			sendRecorrido = 0;
			USB_send_data(USB_DATACODE_POSITION, recorrido);
		}

		if (sequencemain.sm0 == 0)
		{
			if (captureData)
			{
				PinTo0(PORTWxOUT1,PINxOUT1);

				captureData = 0;
				//
				sequencemain.counter = 0x0000;
				sequencemain.sm0++;
			}
		}
		else if (sequencemain.sm0 == 1)
		{
			if (main_flag.f10ms)
			{
				if (++sequencemain.counter >= 20)//20*10ms = 200ms
				{
					sequencemain.counter = 0;

					ADS1115_setMuxChannel(MUX_AIN0_AIN3);//mv1
					ADS1115_setOperatingMode(CONTINUOUS_CONV);
					sequencemain.sm0++;
				}
			}
		}
		else if (sequencemain.sm0 == 2)
		{
			if (main_flag.f10ms)
			{
				if (++sequencemain.counter >= 1)//setup Time new channel ADS1115
				{
					sequencemain.counter = 0;
					sequencemain.sm0++;
				}
			}
		}
		else if (sequencemain.sm0 == 3)
		{
			if (ADS1115_capture_mvx(&mv1))
			{
				ADS1115_setOperatingMode(SINGLESHOT_POWERDOWN_CONV);//ADS1115 powerdown

				USB_send_data(USB_DATACODE_MV1, mv1);//enviar al host mv1 + recorrido

				sequencemain.sm0++;
			}
		}
		else if (sequencemain.sm0 == 4)
		{
			if (main_flag.f10ms)
			{
				if (++sequencemain.counter >= 10)//10*10ms = 100ms
				{
					PinTo1(PORTWxOUT2, PINxOUT2);

					sequencemain.counter = 0;
					sequencemain.sm0++;
				}
			}
		}
		else if (sequencemain.sm0 == 5)
		{
			if (main_flag.f10ms)
			{
				if (++sequencemain.counter >= 20)//20*10ms = 200ms
				{
					sequencemain.counter = 0;

					ADS1115_setMuxChannel(MUX_AIN1_AIN3);//mv2
					ADS1115_setOperatingMode(CONTINUOUS_CONV);

					sequencemain.sm0++;
				}
			}
		}
		else if (sequencemain.sm0 == 6)
		{
			if (main_flag.f10ms)
			{
				if (++sequencemain.counter >= 1)//setup Time new channel ADS1115
				{
					sequencemain.counter = 0;
					sequencemain.sm0++;
				}
			}
		}
		else if (sequencemain.sm0 == 7)
		{
			if (ADS1115_capture_mvx(&mv2))
			{
				ADS1115_setOperatingMode(SINGLESHOT_POWERDOWN_CONV);//ADS1115 powerdown
				//
				current = INA238_read_current_register() * INA238_CURRENT_LSB;
				if (current > 0.0001f)
				{
					current +=1.7e-3;
				}

				//enviar al host mv2 + corriente actual

				USB_send_data(USB_DATACODE_MV2, mv2);
				USB_send_data(USB_DATACODE_CURRENT, current);

				sequencemain.sm0++;
			}
		}
		else if (sequencemain.sm0 == 8)
		{
			if (main_flag.f10ms)
			{
				if (++sequencemain.counter >= 20)//20*10ms = 200ms
				{
					sequencemain.counter = 0;

					ADS1115_setMuxChannel(MUX_AIN2_AIN3);//mv3
					ADS1115_setOperatingMode(CONTINUOUS_CONV);

					sequencemain.sm0++;
				}
			}
		}

		else if (sequencemain.sm0 == 9)
		{
			if (main_flag.f10ms)
			{
				if (++sequencemain.counter >= 1)//setup Time new channel ADS1115
				{
					sequencemain.counter = 0;
					sequencemain.sm0++;
				}
			}
		}
		else if (sequencemain.sm0 == 10)
		{
			if (ADS1115_capture_mvx(&mv3))
			{
				ADS1115_setOperatingMode(SINGLESHOT_POWERDOWN_CONV);//ADS1115 powerdown
				//
				current = INA238_read_current_register() * INA238_CURRENT_LSB;
				if (current > 0.0001f)
				{
					current +=1.7e-3;
				}

				//enviar al host mv3 + corriente actual
				USB_send_data(USB_DATACODE_MV3, mv3);
				USB_send_data(USB_DATACODE_CURRENT, current);

				sequencemain.sm0++;
			}
		}
		else if (sequencemain.sm0 == 11)
		{
			if (main_flag.f10ms)
			{
				if (++sequencemain.counter >= 10)//10*10ms = 100ms
				{
					PinTo0(PORTWxOUT2, PINxOUT2);

					sequencemain.counter = 0;
					sequencemain.sm0++;
				}
			}
		}
		else if (sequencemain.sm0 == 12)
		{
			if (main_flag.f10ms)
			{
				if (++sequencemain.counter >= 10)//10*10ms = 100ms
				{
					PinTo1(PORTWxOUT1, PINxOUT1);

					sequencemain.counter = 0;
					sequencemain.sm0 = 0x0000;
				}
			}
		}
		//
		buzzer_job();

		//clear flags
		main_flag.f10ms = 0;
	}
}
ISR(TIMER0_COMPA_vect)
{
    isr_flag.f10ms = 1;
}

void buzzer_job(void)
{
	//Buzzer
	if (buzzer.f.job)
	{
		if (buzzer.sm0 == 0)
		{
			PinTo1(PORTWxBUZZER, PINxBUZZER);
			buzzer.counter = 0;
			buzzer.sm0++;
		}
		else if (buzzer.sm0 == 1)
		{
			if (main_flag.f10ms)
			{
				if (++buzzer.counter >= BUZZER_KTIME)
				{
					PinTo0(PORTWxBUZZER, PINxBUZZER);
					buzzer.counter = 0;
					buzzer.sm0 = 0x0;
					buzzer.f.job = 0;
				}
			}
		}
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



void encoder_numpulse(void)
{
	char str[20];
	numPulses_diff = rotaryCount - rotaryCount_last;
	recorrido = rotaryCount * ENC_RESOL;//-->meters tendria que siempre ser calculado
	//por cada nuevo cambio en el desplazamiento... enviar al host
	if (numPulses_diff >= numPulsesIn_ADQ_KMETERS)
	{
		rotaryCount_last = rotaryCount;
		//meters = rotaryCount * ENC_RESOL;//-->meters tendria que siempre ser calculado
		captureData = 1;
	}
	itoa(rotaryCount,  str,  10);
	strcat(str,"\n");
	usart_println_string(str);
}

ISR(INT0_vect)//channel A = PD2 INT0
{
	if (ReadPin(PORTRxENC_CHB,PINxENC_CHB) == 1)
	{//reverse
		rotaryCount--;
	}
	else//forward
	{
		rotaryCount++;
	}
	encoder_numpulse();
}
ISR(INT1_vect)//channel B = PD3 INT1
{
	if (ReadPin(PORTRxENC_CHA,PINxENC_CHA) == 1)
	{
		rotaryCount++;
	}
	else
	{
		rotaryCount--;
	}
	encoder_numpulse();

}

/*
 *OK, pero ahora probando con float buffer
 */

int8_t smoothAlg_nonblock(int16_t *buffer, float *Answer)
{
	static float average=0;
	static int16_t Pos;	//# de elementos > que la media
	static int16_t Neg;	//# de elementos > que la media
	static float TD;	//Total Deviation
	//float A;	//Correct answer

	//1- Calculate media
	if (smoothAlgJob.sm0 == 0)
	{
		average = 0;
		smoothAlgJob.counter = 0x0;
		smoothAlgJob.sm0++;
	}
	if (smoothAlgJob.sm0 == 1)
	{
		average +=buffer[smoothAlgJob.counter];

		if (++smoothAlgJob.counter >= SMOOTHALG_MAXSIZE)
		{
			smoothAlgJob.counter = 0x00;//bug fixed

			average /= SMOOTHALG_MAXSIZE;
			//
			Pos = 0;
			Neg = 0;
			TD = 0;
			smoothAlgJob.sm0++;
		}
	}
	//2 - Find Pos and Neg + |Dtotal|
	else if (smoothAlgJob.sm0 == 2)
	{
		if (buffer[smoothAlgJob.counter] > average)
		{
			Pos++;
			TD += ( ((float)(buffer[smoothAlgJob.counter]))-average);//Find |Dtotal|
		}
		if (buffer[smoothAlgJob.counter] < average)
		{
			Neg++;
		}
		//
		if (++smoothAlgJob.counter >= SMOOTHALG_MAXSIZE)
		{
			smoothAlgJob.counter = 0;
			smoothAlgJob.sm0 = 0;
			//bug
			if (TD<0)
			{
				TD *= -1;//convirtiendo a positivo
			}
			//
			*Answer = average + ( ( (Pos-Neg) * TD )/ ( SMOOTHALG_MAXSIZE*SMOOTHALG_MAXSIZE) );
			return 1;
			//
		}
	}
	return 0;
}
