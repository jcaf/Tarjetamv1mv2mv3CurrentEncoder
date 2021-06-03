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



float mv1;
float mv2;
float mv3;
float current;
//float position;
volatile float recorrido = 0.0f;
volatile int8_t captureData;
volatile int8_t sendRecorrido;

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

struct _job
{
	int8_t sm0;//x jobs
	//int8_t key_sm0;//x keys

	uint16_t counter0;
	uint16_t counter1;
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

/*
 * Host-to-device or tact-switch on-board reset the sequence, reset all state machines
 */
void sequence_reset(void)
{
	smoothAlgJob =	capturemvx = sequencemain = buzzer = emptyJob;
}
/*
 *
 */
#define ADS1115_KTIME_CAPTURE_AVERAGE 1	//10e-3

int8_t ADS1115_capture_mvx(float *mvx)
{
	int16_t ib16;
	float smoothAnswer;
	uint8_t reg[2];
	//
	static int16_t smoothVector[SMOOTHALG_MAXSIZE];

////+++++++++
//I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
//ib16 = (reg[0]<<8) + reg[1];
//ib16*=-1;
//*mvx = (ib16*2.048f/32768);
//return 1;
//+++++++++

	if (capturemvx.sm0 == 0)
	{
		I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
		ib16 = (reg[0]<<8) + reg[1];
		smoothVector[capturemvx.counter0] = ib16;

		if (++capturemvx.counter0 >= SMOOTHALG_MAXSIZE)
		{
			capturemvx.counter0 = 0x00;
			capturemvx.sm0 = 2;
		}
		else
		{
			capturemvx.counter1 = 0;
			capturemvx.sm0 = 1;
		}
	}

	else if (capturemvx.sm0 == 1)
	{
		if (main_flag.f10ms)
		{
			if (++capturemvx.counter1 > ADS1115_KTIME_CAPTURE_AVERAGE)
			{
				capturemvx.counter1 = 0;
				capturemvx.sm0--;
			}
		}
	}

	else if (capturemvx.sm0 == 2)
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

uint8_t old_PORTRxENC_CHB;

int main(void)
{
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
	//channel A = PD2 INT0
	//channel B = PD3 INT1
	EICRA = 0x05;//Any logical change on INT1 generates an interrupt request.
	EIMSK = 0x03;//Enable interrupt on INT1, INT0
	sei();
	while (1);
	//


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

int16_t c=0;
int8_t simulate = 1;
uint32_t Rrecorrido = 0;

	while (1)
	{
		if (isr_flag.f10ms)
		{
			isr_flag.f10ms = 0;
			main_flag.f10ms = 1;
		}
		//---SIMULATING ENCODER----------------
		if (simulate == 1)
		{
			if (main_flag.f10ms)
			{
				if (++c >=10)//c/100 ms
				{
					c = 0;
					Rrecorrido++;
					sendRecorrido = 1;
					//
					if ( (Rrecorrido % 10) == 0)
					{
						captureData = 1;
						simulate = 0;
					}
				}
			}
		}


		//----------------------
		//----------------------
		if (sendRecorrido)
		{
			sendRecorrido = 0;
			USB_send_data(USB_DATACODE_POSITION, Rrecorrido);
		}

		if (sequencemain.sm0 == 0)
		{
			if (captureData)
			{
				PinTo0(PORTWxOUT1,PINxOUT1);

				captureData = 0;
				//
				sequencemain.counter0 = 0x0000;
				sequencemain.sm0++;
			}
		}
		else if (sequencemain.sm0 == 1)
		{
			if (main_flag.f10ms)
			{
				if (++sequencemain.counter0 >= 20)//20*10ms = 200ms
				{
					sequencemain.counter0 = 0;

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
				if (++sequencemain.counter0 >= 1)//setup Time new channel ADS1115
				{
					sequencemain.counter0 = 0;

					buzzer.f.job = 1;

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
				if (++sequencemain.counter0 >= 10)//10*10ms = 100ms
				{
					PinTo1(PORTWxOUT2, PINxOUT2);

					sequencemain.counter0 = 0;
					sequencemain.sm0++;
				}
			}
		}
		else if (sequencemain.sm0 == 5)
		{
			if (main_flag.f10ms)
			{
				if (++sequencemain.counter0 >= 20)//20*10ms = 200ms
				{
					sequencemain.counter0 = 0;

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
				if (++sequencemain.counter0 >= 1)//setup Time new channel ADS1115
				{
					sequencemain.counter0 = 0;

					buzzer.f.job = 1;

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
				if (++sequencemain.counter0 >= 20)//20*10ms = 200ms
				{
					sequencemain.counter0 = 0;

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
				if (++sequencemain.counter0 >= 1)//setup Time new channel ADS1115
				{
					sequencemain.counter0 = 0;

					buzzer.f.job = 1;

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
				if (++sequencemain.counter0 >= 10)//10*10ms = 100ms
				{
					PinTo0(PORTWxOUT2, PINxOUT2);

					sequencemain.counter0 = 0;
					sequencemain.sm0++;
				}
			}
		}
		else if (sequencemain.sm0 == 12)
		{
			if (main_flag.f10ms)
			{
				if (++sequencemain.counter0 >= 10)//10*10ms = 100ms
				{
					PinTo1(PORTWxOUT1, PINxOUT1);

					sequencemain.counter0 = 0;
					sequencemain.sm0 = 0x0000;

					//*
					simulate = 1;
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
			buzzer.counter0 = 0;
			buzzer.sm0++;
		}
		else if (buzzer.sm0 == 1)
		{
			if (main_flag.f10ms)
			{
				if (++buzzer.counter0 >= BUZZER_KTIME)
				{
					PinTo0(PORTWxBUZZER, PINxBUZZER);
					buzzer.counter0 = 0;
					buzzer.sm0 = 0x0;
					buzzer.f.job = 0;
				}
			}
		}
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
 * encoder routine

 * 0000BA00
 */

void encoder_xor(void)
{
	volatile static uint8_t old_PORTRxENC_CHB;
	uint8_t xor;

	xor = PORTRxENC_CHA ^ (old_PORTRxENC_CHB>>1);
	old_PORTRxENC_CHB = PORTRxENC_CHB;//save CHB
	if (xor & (1<<PINxENC_CHA))
	{
		rotaryCount++;
	}
	else
	{
		rotaryCount--;
	}
	rotaryCount >>= 2;//div by 4


}



/*
 *
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
		smoothAlgJob.counter0 = 0x0;
		smoothAlgJob.sm0++;
	}
	if (smoothAlgJob.sm0 == 1)
	{
		average +=buffer[smoothAlgJob.counter0];

		if (++smoothAlgJob.counter0 >= SMOOTHALG_MAXSIZE)
		{
			smoothAlgJob.counter0 = 0x00;//bug fixed

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
		if (buffer[smoothAlgJob.counter0] > average)
		{
			Pos++;
			TD += ( ((float)(buffer[smoothAlgJob.counter0]))-average);//Find |Dtotal|
		}
		if (buffer[smoothAlgJob.counter0] < average)
		{
			Neg++;
		}
		//
		if (++smoothAlgJob.counter0 >= SMOOTHALG_MAXSIZE)
		{
			smoothAlgJob.counter0 = 0;
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
