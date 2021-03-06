/*
 * main.c
 *
 *  Created on: May 22, 2021
 *      Author: jcaf
 *
 Optimization: -O3
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
#include "src/serial/serial.h"
#include "main.h"
#include "src/pinGetLevel/pinGetLevel.h"

#define CURRENT_KCORRECTION +2.7E-3	//mA
#define MV1_KCORRECTION +0.35E-3	//mV
#define MV2_KCORRECTION +0.5E-3	//mV
#define MV3_KCORRECTION -0.7E-3	//mV

volatile struct _isr_flag
{
    unsigned f1ms :1;
    unsigned send_posicion :1;
    unsigned posicion_0:1;//solo por EL LO HICD VOLATILE, MEJOR PASARLO A ISR_FLAG
    unsigned __a :5;
} isr_flag = { 0,0 };

//volatile
struct _main_flag
{
    unsigned f1ms :1;
    unsigned send_corriente :1;
    unsigned posicion_0:1;//solo por EL LO HICD VOLATILE, MEJOR PASARLO A ISR_FLAG
    unsigned __a:5;

}main_flag = { 0,0 };

float mv1;
float mv2;
float mv3;
float current;
//float position;
volatile float posicion = 0.0f;

//+- Encoder
void encoder_reset(void);
//typedef int64_t ROTARYCOUNT_T;
typedef int32_t ROTARYCOUNT_T;
struct _encoder
{
	struct _encoder_flag
	{
		unsigned commingFromInc:1;
		unsigned commingFromDec:1;
		unsigned update:1;
		unsigned __a:5;
	}f;
	int8_t count4edges;;
	ROTARYCOUNT_T rotaryCount;
};

volatile struct _encoder encoder;
const struct _encoder encoderReset;


uint16_t ENCODER_PPR = 500;    			//500 Pulses Per Revolution
float ENCODER_1REV_INMETERS = 0.5f;    	//1revol = X meters
volatile float ADQ_KMETERS = 1.0f;		//Adquirir cada "x metros"
float ENC_RESOL = 0;// = (float)ENCODER_1REV_INMETERS/ENCODER_PPR;
//
//Las sgtes. variables no necesitan ser de 64bits
int32_t numPulsesIn_ADQ_KMETERS = 0; //(ADQ_KMETERS * ENCODER_PPR) / ENCODER_1REV_INMETERS;//truncar
int32_t numPulses_diff = 0;
//
volatile uint8_t old_PORTRxENC_CHB;//track last change in quadrature
//

struct _job
{
	int8_t sm0;//x jobs

	uint16_t counter0;
	uint16_t counter1;
	//int8_t mode;

	struct _job_f
	{
		unsigned enable:1;
		unsigned job:1;
		unsigned lock:1;
		unsigned recorridoEnd:1;
		unsigned __a:4;
	}f;
};

struct _smoothAlg
{
	int8_t sm0;//x jobs
	uint16_t counter0;
	float average;
	int16_t Pos;	//# de elementos > que la media
	int16_t Neg;	//# de elementos > que la media
	float TD;	//Total Deviation
	int SMOOTHALG_MAXSIZE;
};


struct _job job_reset;
struct _smoothAlg smoothAlg_reset;
//
struct _job job_capture_mvx;
struct _smoothAlg smoothAlg_mvx;
//
struct _job job_capture_current;
struct _smoothAlg smoothAlg_current;
//
struct _job job_buzzer;
//
struct _job job_captura1;
struct _job job_captura2;

#define BUZZER_KTIME_MS 300
void buzzer_job(void);

//int8_t smoothAlg_nonblock(int16_t *buffer, float *Answer);
//int8_t smoothAlg_nonblock(struct _smoothAlg *smooth, int16_t *buffer, float *Answer);
int8_t smoothAlg_nonblock(struct _smoothAlg *smooth, int16_t *buffer, int SMOOTHALG_MAXSIZE, float *Answer);

#define ADS1115_SMOOTHALG_MAXSIZE 5

#define ADS1115_KTIME_CAPTURE_AVERAGE 8//ADS1115 DATARATE = 128 -> 1/128 = 7.8ms

int8_t ADS1115_capture_mvx(float *mvx)
{
	int16_t ib16;
	float smoothAnswer;
	uint8_t reg[2];
	//
	static int16_t smoothVector[ADS1115_SMOOTHALG_MAXSIZE];

	////+++++++++
	//I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
	//ib16 = (reg[0]<<8) + reg[1];
	//ib16*=-1;
	//*mvx = (ib16*2.048f/32768);
	//return 1;
	//+++++++++

	//-----------------
	if (job_capture_mvx.sm0 == 0)
	{
		if (main_flag.f1ms)
		{
			if (++job_capture_mvx.counter1 >= ADS1115_KTIME_CAPTURE_AVERAGE)
			{
				job_capture_mvx.counter1 = 0;

				//Aqui ya tengo la primera muestra
				I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
				ib16 = (reg[0]<<8) + reg[1];

				smoothVector[job_capture_mvx.counter0] = ib16;
				if (++job_capture_mvx.counter0 >= ADS1115_SMOOTHALG_MAXSIZE)
				{
					job_capture_mvx.counter0 = 0x00;
					//job_capture_mvx.sm0 = 2;//calcular smooth
					job_capture_mvx.sm0++;//calcular smooth
				}
				else
				{
					job_capture_mvx.counter1 = 0;
					//job_capture_mvx.sm0 = 1;//volver a ejercer un cierto intervalo entre cada muestra para el smooth
				}
			}
		}
	}
//	else if (job_capture_mvx.sm0 == 1)
//	{
//		if (main_flag.f1ms)
//		{
//			if (++job_capture_mvx.counter1 >= ADS1115_KTIME_CAPTURE_AVERAGE)
//			{
//				job_capture_mvx.counter1 = 0;
//				job_capture_mvx.sm0--;
//			}
//		}
//	}
	else if (job_capture_mvx.sm0 == 1)
	{
		#if ADS1115_SMOOTHALG_MAXSIZE == 0
				*mvx = (smoothVector[0]*-2048.00f/32768);//expresado en mV
				job_capture_mvx.sm0 = 0x0;
				return 1;
		#else
			if (smoothAlg_nonblock(&smoothAlg_mvx, smoothVector, ADS1115_SMOOTHALG_MAXSIZE, &smoothAnswer))
			{
				//smoothAnswer*=-1;//invirtiendo la senal
				//*mvx = (smoothAnswer*2.048f/32768);//expresados en Voltios..tal como es
				*mvx = (smoothAnswer*-2048.00f/32768);//expresado en mV

				job_capture_mvx.sm0 = 0x0;
				return 1;
			}
		#endif
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
#define USB_DATACODE_POSICION 'P'
//
#define USB_DATACODE_CAPTURA1_ON 'A'
#define USB_DATACODE_CAPTURA1_OFF 'B'
#define USB_DATACODE_CAPTURA2_ON 'D'
#define USB_DATACODE_CAPTURA2_OFF 'E'

#define USB_DATACODE_AMPLIFICARx10_ON 'F'
#define USB_DATACODE_AMPLIFICARx10_OFF 'G'

#define USB_DATACODE_OUT1_ON 'H'
#define USB_DATACODE_OUT1_OFF 'I'
#define USB_DATACODE_OUT2_ON 'J'
#define USB_DATACODE_OUT2_OFF 'K'


#define USB_DATACODE_MV1_CAPTURA1_END 'L'
#define USB_DATACODE_MV2MV3_CAPTURA2_END 'M'

//

#define USB_DATACODE_TOKEN_BEGIN '@'
#define USB_DATACODE_TOKEN_END '\r'

#define INTERVALO_CAPTURA_MVX 500//ms

void USB_send_data(char datacode, float payload0)
{
	char str[30];
	char buff[30];

	str[0] = USB_DATACODE_TOKEN_BEGIN;
	str[1] = datacode;
	str[2] = '\0';

	if (datacode == USB_DATACODE_POSICION)
	{
		dtostrf(payload0, 0, 3, buff);//solo 3 decimales
	}
	else
	{
		dtostrf(payload0, 0, 2, buff);//current in mV + mA
	}

	strcat(str,buff);
	strcat(str,"\r");

	//usart_println_string(str);
	usart_print_string(str);
}


void rx_trama(void);

//float IN238_read_current_mA(void)
//{
//	float current = INA238_read_current_register() * INA238_CURRENT_LSB;
//	if (current > 0.0001f)
//	{
//		current +=CURRENT_KCORRECTION;
//	}
//	current *=1000.0f;	//convert a miliamperios
//	return current;
//}

#define INA238_KTIME_CAPTURE_AVERAGE 5//ms INA238_ADC_CONFIGMODE_CT_150uS *  INA238_ADC_CONFIGMODE_AVG_SAMPLE_1024
#define INA238_SMOOTHALG_MAXSIZE 10

int8_t IN238_capture_current(float *current)
{
	float smoothAnswer;

	//
	static int16_t smoothVector[INA238_SMOOTHALG_MAXSIZE];

	if (job_capture_current.sm0 == 0)
	{
		if (main_flag.f1ms)
		{
			if (++job_capture_current.counter1 >= INA238_KTIME_CAPTURE_AVERAGE)
			{
				job_capture_current.counter1 = 0;

				//Aqui ya tengo la primera muestra
				smoothVector[job_capture_current.counter0] = INA238_read_current_register();
				if (++job_capture_current.counter0 >= INA238_SMOOTHALG_MAXSIZE)
				{
					job_capture_current.counter0 = 0x00;
					job_capture_current.sm0++;//calcular smooth
				}
				else
				{
					job_capture_current.counter1 = 0;
					//volver ejercer un cierto intervalo entre cada muestra para el smooth
					//job_capture_current.sm0 = 1;//volver ejercer un cierto intervalo entre cada muestra para el smooth
				}
			}
		}
	}
	else if (job_capture_current.sm0 == 1)
	{
		#if INA238_SMOOTHALG_MAXSIZE == 0
			*current = smoothVector[0] * INA238_CURRENT_LSB;
			job_capture_current.sm0 = 0x0;
			return 1;
		#else
			if (smoothAlg_nonblock(&smoothAlg_current, smoothVector, INA238_SMOOTHALG_MAXSIZE, &smoothAnswer))
			{
				//++-
				*current = smoothAnswer * INA238_CURRENT_LSB;
				if (*current > 0.0001f)
				{
					*current +=CURRENT_KCORRECTION;
					*current *=1000.0f;	//convert a miliamperios
				}
				//*current *=1000.0f;	//convert a miliamperios
				//++-

				job_capture_current.sm0 = 0x0;
				return 1;
			}
		#endif
	}
	//
	return 0;
}
void test_ads1115(void)
{
	uint8_t reg[2];
	int16_t ib16;
	float mvx;

	ADS1115_setMuxChannel(MUX_AIN0_AIN3);//mv1
	//ADS1115_setMuxChannel(MUX_AIN1_AIN3);//mv2
	//ADS1115_setMuxChannel(MUX_AIN2_AIN3);//mv3
	//
	ADS1115_setOS(1);//wakeup ADS1115
	ADS1115_setOperatingMode(CONTINUOUS_CONV);

	char buff[20];
	while (1)
	{
		//ADS1115_capture_mvx
		I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
		ib16 = (reg[0]<<8) + reg[1];
		ib16*=-1;
		mvx = (ib16*2.048f/32768);
		//
		dtostrf(mvx, 0, 3, buff);//solo 2 decimales
		usart_println_string(buff);
	}
}
void test_ads1115_1(void)
{
	float mvx;

	ADS1115_setMuxChannel(MUX_AIN0_AIN3);//mv1
	//ADS1115_setMuxChannel(MUX_AIN1_AIN3);//mv2
	//ADS1115_setMuxChannel(MUX_AIN2_AIN3);//mv3

	ADS1115_setOS(1);//wakeup ADS1115
	ADS1115_setOperatingMode(CONTINUOUS_CONV);

	char buff[20];
	while (1)
	{
		if (isr_flag.f1ms)
		{
			isr_flag.f1ms = 0;
			main_flag.f1ms = 1;
		}
		//

		if (ADS1115_capture_mvx(&mvx))
		{
			dtostrf(mvx, 0, 3, buff);//solo 2 decimales
			usart_println_string(buff);
		}
		//
		main_flag.f1ms = 0;
	}
}

void testUSB(void)
{
	while (1)
	{
		if (isr_flag.f1ms)
		{
			isr_flag.f1ms = 0;
			main_flag.f1ms = 1;
		}

		for (int i=0; i<10; i++)
		{
			USB_send_data(USB_DATACODE_POSICION, i+1);
			//__delay_ms(USB_KDELAY_BEETWEN_2SENDS);//usar solo centesimas en la posicion

			//current = IN238_read_current_mA();
			USB_send_data(USB_DATACODE_CURRENT, i+2);

			//__delay_ms(USB_KDELAY_BEETWEN_2SENDS);//usar solo centesimas e

		}
		while (1);
		main_flag.f1ms = 0;
	}

}

int main(void)
{
	char str[20];
	int8_t buttonResetEncoder_counter0=0;
	int8_t SW=0;
	//int8_t corriente_counter = 0;

	PinTo1(PORTWxOUT1,PINxOUT1);
	ConfigOutputPin(CONFIGIOxOUT1, PINxOUT1);

	PinTo0(PORTWxOUT2,PINxOUT2);
	ConfigOutputPin(CONFIGIOxOUT2, PINxOUT2);

	PinTo0(PORTWxBUZZER,PINxBUZZER);
	ConfigOutputPin(CONFIGIOxBUZZER, PINxBUZZER);

	pinGetLevel_init();

	//Esto tiene que ser una funcion que se actualice cada vez que se establezca el intervalo desde la PC
	ENC_RESOL = (float)ENCODER_1REV_INMETERS/ENCODER_PPR;
	numPulsesIn_ADQ_KMETERS = (ADQ_KMETERS * ENCODER_PPR) / ENCODER_1REV_INMETERS;//truncar
	//

	USART_Init ( (int)MYUBRR );

	I2C_unimaster_init(400E3);//100KHz
	ADS1115_init();//ADS1115 in powerdown state
	INA238_init();//INA238_REG_CONFI to ?? 40.96 mV --> INA238_1_LSB_STEPSIZE_ADCRANGE_40p96mV 1.25E-6 para todos los calculos

	//Atmega328P TCNT0 CTC mode
    TCNT0 = 0x0000;
    TCCR0A = (1 << WGM01);
    //CTC PRESCALER=256 + 10E-3 -> OCR0A = 624
    //TCCR0B = (1 << CS02) | (0 << CS01) | (0 << CS00); //CTC PRES=256
    //OCR0A = CTC_SET_OCR_BYTIME(1E-3,256);

    //CTC PRESCALER=64 + 1E-3 -> OCR0A = 249
    TCCR0B = (0 << CS02) | (1 << CS01) | (1 << CS00); //CTC PRES = 64
    OCR0A = CTC_SET_OCR_BYTIME(1E-3, 64);
    TIMSK0 |= (1 << OCIE0A);
    //
    //Encoder setup Atmega328P, external Pull-ups 1K
	//channel A = PD2 INT0 / PCINT18
	//channel B = PD3 INT1 / PCINT19
	PCICR 	= 0x04;//PCIE2 PCINT[23:16] Any change on any enabled PCINT[23:16] pin will cause an interrupt.
	PCMSK2 	= 0x0C;//PCINT18 PCINT19
	old_PORTRxENC_CHB = PORTRxENC_CHB;

    sei();

    //test_ads1115();//ok
    //test_ads1115_1();
    //testUSB();
	while (1)
	{
		if (isr_flag.f1ms)
		{
			isr_flag.f1ms = 0;
			main_flag.f1ms = 1;
		}
		if (isr_flag.posicion_0)
		{
			isr_flag.posicion_0 = 0;
			main_flag.posicion_0 = 1;
		}
		//-------------------------------------
		if (isr_flag.send_posicion)
		{
			isr_flag.send_posicion = 0;
			posicion = (encoder.rotaryCount * ENC_RESOL);

			//if (encoder.rotaryCount >= 0 )
			//if (posicion >= 0.000f )
			if (1)
			{
				USB_send_data(USB_DATACODE_POSICION, posicion);
			}


//			if (encoder.rotaryCount <= 0)
//			//if (posicion == 0.000f )
//			{
//				main_flag.posicion_0 = 1;
//			}

		}
		//
		if (main_flag.send_corriente)
		{
			if (IN238_capture_current(&current))
			{
				USB_send_data(USB_DATACODE_CURRENT, current);
			}
		}
		//
		//******************** Captura mv1 ****************************
		if (job_captura1.sm0 > 0)
		{

			//--------------
			if (job_captura1.f.recorridoEnd == 0)
			{
				if (main_flag.posicion_0 == 1)
				{
					job_captura1.f.recorridoEnd = 1;

					job_captura1.sm0 = 1;
				}
			}
			//--------------

			//--------------
			if (job_captura1.sm0 == 1)
			{
				PinTo1(PORTWxOUT1, PINxOUT1);
				ADS1115_setMuxChannel(MUX_AIN0_AIN3);//mv1
				job_captura1.counter0 = 0;
				job_captura1.sm0++;
			}
			else if (job_captura1.sm0 == 2)
			{
				if ( (main_flag.f1ms) || (job_captura1.f.recorridoEnd) )
				{
					if ((++job_captura1.counter0 >= INTERVALO_CAPTURA_MVX) || (job_captura1.f.recorridoEnd))
					{
						job_captura1.counter0 = 0x0000;
						ADS1115_setOS(1);//wakeup ADS1115
						ADS1115_setOperatingMode(CONTINUOUS_CONV);
						job_captura1.sm0++;
						//
						job_buzzer.f.job = 1;
					}
				}
			}
			else if (job_captura1.sm0 == 3)
			{
				if (ADS1115_capture_mvx(&mv1))//finalizo
				{
					//
					if (mv1> 0.0001f)
					{
						mv1+=MV1_KCORRECTION;
					}
					//

					ADS1115_setOperatingMode(SINGLESHOT_POWERDOWN_CONV);
					USB_send_data(USB_DATACODE_MV1, mv1);
					job_captura1.sm0--;

					//++-
					if (job_captura1.f.recorridoEnd)
					{
						job_captura1 = job_capture_mvx = job_reset;
						//
						//OUT1 off
						PinTo0(PORTWxOUT1, PINxOUT1);
						//
						str[0] = USB_DATACODE_TOKEN_BEGIN;
						str[1] = USB_DATACODE_MV1_CAPTURA1_END;
						str[2] = USB_DATACODE_TOKEN_END;
						str[3] = '\0';
						usart_print_string(str);

					}
					//-++
				}
			}
		}

		//******************** Captura mv2 + mv3 ****************************

		if (job_captura2.sm0 > 0)
		{
			//--------------
			if (job_captura2.f.recorridoEnd == 0)
			{
				if (main_flag.posicion_0 == 1)
				{
					job_captura2.f.recorridoEnd = 1;
					//
					job_captura2.sm0 = 1;
				}
			}
			//--------------

			if (job_captura2.sm0 == 1)
			{
				PinTo1(PORTWxOUT1, PINxOUT1);
				PinTo1(PORTWxOUT2, PINxOUT2);
				ADS1115_setMuxChannel(MUX_AIN1_AIN3);//mv2

				job_captura2.counter0 = 0;
				job_captura2.sm0++;
			}
			else if (job_captura2.sm0 == 2)
			{
				if ( (main_flag.f1ms) || (job_captura2.f.recorridoEnd) )
				{
					if ((++job_captura2.counter0 >= INTERVALO_CAPTURA_MVX) || (job_captura2.f.recorridoEnd))
					{
						job_captura2.counter0 = 0x0000;
						ADS1115_setOS(1);//wakeup ADS1115
						ADS1115_setOperatingMode(CONTINUOUS_CONV);
						job_captura2.sm0++;
						//
						job_buzzer.f.job = 1;
					}
				}
			}
			else if (job_captura2.sm0 == 3)
			{
				if (ADS1115_capture_mvx(&mv2))
				{
					//
					if (mv2> 0.0001f)
					{
						mv2+=MV2_KCORRECTION;
					}
					//
					ADS1115_setMuxChannel(MUX_AIN2_AIN3);//mv3

					USB_send_data(USB_DATACODE_MV2, mv2);
					job_captura2.sm0++;
					job_captura2.counter1 = 0;
				}
			}
			else if (job_captura2.sm0 == 4)
			{
				if (main_flag.f1ms)
				{
					//ADS1115 DATARATE = 128 -> 1/128 = 7.8ms
					if (++job_captura2.counter1 > (ADS1115_KTIME_CAPTURE_AVERAGE))//7.8ms el tiempo de 1/SPS actual
					{
						job_captura2.counter1 = 0;
						job_captura2.sm0++;
					}
				}
			}
			else if (job_captura2.sm0 == 5)
			{
				if (ADS1115_capture_mvx(&mv3))
				{
					//
					if (mv3> 0.0001f)
					{
						mv3+=MV3_KCORRECTION;
					}
					//
					ADS1115_setOperatingMode(SINGLESHOT_POWERDOWN_CONV);
					USB_send_data(USB_DATACODE_MV3, mv3);
					job_captura2.sm0 = 1;

					//++-
					if (job_captura2.f.recorridoEnd)
					{
						//
						job_captura2 = job_capture_mvx = job_reset;
						//
						//OUT1 off
						PinTo0(PORTWxOUT1, PINxOUT1);
						PinTo0(PORTWxOUT2, PINxOUT2);
						//
						str[0] = USB_DATACODE_TOKEN_BEGIN;
						str[1] = USB_DATACODE_MV2MV3_CAPTURA2_END;
						str[2] = USB_DATACODE_TOKEN_END;
						str[3] = '\0';
						usart_print_string(str);



					}
					//-++
				}
			}

		}

		//+++++++++++++++++++++++
		if (main_flag.f1ms)
		{
			if (++buttonResetEncoder_counter0 >= 20)    //20ms
			{
				buttonResetEncoder_counter0 = 0;
				pinGetLevel_job();
				if (pinGetLevel_hasChanged(PGLEVEL_LYOUT_SW_ENC_RESET))
				{
					SW = !pinGetLevel_level(PGLEVEL_LYOUT_SW_ENC_RESET);
					pinGetLevel_clearChange(PGLEVEL_LYOUT_SW_ENC_RESET);
				}
			}
		}
		if ((SW == 1) ) //|| (char == "s") // -> usar el pulsador mientras
		{
			encoder_reset();// resetear el contador de # de pulsos
			SW = 0x00;
		}
		//+++++++++++++++++++++++
		//
		rx_trama();
		buzzer_job();

		//clear flags
		main_flag.posicion_0 = 0;
		main_flag.f1ms = 0;


	}
}
ISR(TIMER0_COMPA_vect)
{
    isr_flag.f1ms = 1;
}

void buzzer_job(void)
{
	//Buzzer
	if (job_buzzer.f.job)
	{
		if (job_buzzer.sm0 == 0)
		{
			PinTo1(PORTWxBUZZER, PINxBUZZER);
			job_buzzer.counter0 = 0;
			job_buzzer.sm0++;
		}
		else if (job_buzzer.sm0 == 1)
		{
			if (main_flag.f1ms)
			{
				if (++job_buzzer.counter0 >= BUZZER_KTIME_MS)
				{
					PinTo0(PORTWxBUZZER, PINxBUZZER);
					job_buzzer.counter0 = 0;
					job_buzzer.sm0 = 0x0;
					job_buzzer.f.job = 0;
				}
			}
		}
	}
}

void encoder_reset(void)
{
	if ( (job_captura1.sm0 == 0) && (job_captura2.sm0 == 0) )
	{
		//
		PCICR 	= 0x00;//disable PCIE2 PCINT[23:16]
		//encoder.rotaryCount = 0x0000;
		encoder = encoderReset;//clear struct
		old_PORTRxENC_CHB = PORTRxENC_CHB;
		isr_flag.send_posicion = 1;

		PCIFR 	= 0x04;//PCINT18 PCINT19 clear flags
		PCICR 	= 0x04;//PCIE2 PCINT[23:16] Any change on any enabled PCINT[23:16] pin will cause an interrupt.
	}
}
ISR(PCINT2_vect)//void encoder_xor(void)
{
	uint8_t direction;
	//volatile static int8_t count4edges = 0;
	//
	direction = (PORTRxENC_CHA ^ (old_PORTRxENC_CHB>>1) ) & (1<<PINxENC_CHA);
	old_PORTRxENC_CHB = PORTRxENC_CHB;//save CHB
	//
	if (direction != 0 )
	{
		encoder.f.commingFromInc = 1;
		if (encoder.count4edges>=4)
		{
			encoder.count4edges = 0;
		}
		encoder.count4edges++;
		if (encoder.count4edges == 4)
		{
			if (encoder.f.commingFromDec == 1)
			{
				encoder.f.commingFromDec = 0;
			}
			else
			{
				//encoder.f.update = 1;

				//++-Solo para hacer +rapido el ISR
				encoder.rotaryCount++;
				isr_flag.send_posicion = 1;

				//
				if (encoder.rotaryCount == 0)
				{
					isr_flag.posicion_0 = 1;
				}

				//-++
			}
		}
	}
	else
	{
		encoder.f.commingFromDec = 1;
		if (encoder.count4edges<=0)
		{
			encoder.count4edges = 4;
		}
		encoder.count4edges--;
		if (encoder.count4edges == 0)
		{
			if (encoder.f.commingFromInc == 1)
			{
				encoder.f.commingFromInc = 0;
			}
			else
			{
				//encoder.f.update = 1;

				//++-Solo para hacer +rapido el ISR
				encoder.rotaryCount--;
				isr_flag.send_posicion = 1;
				//-++
				//
				if (encoder.rotaryCount == 0)
				{
					isr_flag.posicion_0 = 1;
				}
			}
		}
	}

	//Play with encoder//
	//Para ganar +velocidad, muevo el inc/dec del contador dentro de los if()
//	if (encoder.f.update == 1)
//	{
//		encoder.f.update = 0;
//
//		if (direction != 0)
//		{
//			encoder.rotaryCount++;
//		}
//		else
//		{
//			encoder.rotaryCount--;
//		}
//
//		isr_flag.send_posicion = 1;
//	}
}

/*
 *
 */
int8_t smoothAlg_nonblock(struct _smoothAlg *smooth, int16_t *buffer, int SMOOTHALG_MAXSIZE, float *Answer)
{
//	static float average=0;
//	static int16_t Pos;	//# de elementos > que la media
//	static int16_t Neg;	//# de elementos > que la media
//	static float TD;	//Total Deviation
//

	//1- Calculate media
	if (smooth->sm0 == 0)
	{
		smooth->average = 0;
		smooth->counter0 = 0x0;
		smooth->sm0++;
	}
	if (smooth->sm0 == 1)
	{
		smooth->average +=buffer[smooth->counter0];

		if (++smooth->counter0 >= SMOOTHALG_MAXSIZE)
		{
			smooth->counter0 = 0x00;//bug fixed

			smooth->average /= SMOOTHALG_MAXSIZE;
			//
			smooth->Pos = 0;
			smooth->Neg = 0;
			smooth->TD = 0;
			smooth->sm0++;
		}
	}
	//2 - Find Pos and Neg + |Dtotal|
	else if (smooth->sm0 == 2)
	{
		if (buffer[smooth->counter0] > smooth->average)
		{
			smooth->Pos++;
			smooth->TD += ( ((float)(buffer[smooth->counter0]))-smooth->average);//Find |Dtotal|
		}
		if (buffer[smooth->counter0] < smooth->average)
		{
			smooth->Neg++;
		}
		//
		if (++smooth->counter0 >= SMOOTHALG_MAXSIZE)
		{
			smooth->counter0 = 0;
			smooth->sm0 = 0;
			//bug
			if (smooth->TD<0)
			{
				smooth->TD *= -1;//convirtiendo a positivo
			}
			//
			*Answer = smooth->average + ( ( (smooth->Pos-smooth->Neg) * smooth->TD )/ ( SMOOTHALG_MAXSIZE*SMOOTHALG_MAXSIZE) );
			return 1;
			//
		}
	}
	return 0;
}

/*
 * int8_t smoothAlg_nonblock(int16_t *buffer, float *Answer)
{
	static float average=0;
	static int16_t Pos;	//# de elementos > que la media
	static int16_t Neg;	//# de elementos > que la media
	static float TD;	//Total Deviation
	//float A;	//Correct answer

	//1- Calculate media
	if (job_smoothAlg.sm0 == 0)
	{
		average = 0;
		job_smoothAlg.counter0 = 0x0;
		job_smoothAlg.sm0++;
	}
	if (job_smoothAlg.sm0 == 1)
	{
		average +=buffer[job_smoothAlg.counter0];

		if (++job_smoothAlg.counter0 >= SMOOTHALG_MAXSIZE)
		{
			job_smoothAlg.counter0 = 0x00;//bug fixed

			average /= SMOOTHALG_MAXSIZE;
			//
			Pos = 0;
			Neg = 0;
			TD = 0;
			job_smoothAlg.sm0++;
		}
	}
	//2 - Find Pos and Neg + |Dtotal|
	else if (job_smoothAlg.sm0 == 2)
	{
		if (buffer[job_smoothAlg.counter0] > average)
		{
			Pos++;
			TD += ( ((float)(buffer[job_smoothAlg.counter0]))-average);//Find |Dtotal|
		}
		if (buffer[job_smoothAlg.counter0] < average)
		{
			Neg++;
		}
		//
		if (++job_smoothAlg.counter0 >= SMOOTHALG_MAXSIZE)
		{
			job_smoothAlg.counter0 = 0;
			job_smoothAlg.sm0 = 0;
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
 *
 */
/*
 * //Construir payload data + checksum
 @NxxxxFxxxxRxxxxCcc'\r\n'
 1234546789........64

 */


/* recorre todo el array
 * 1: success
 * 0: fail
 */
/*
int8_t str_trimlr(char *str_in, char *str_out, char l, char r)
{
	int8_t counter = 0;
	int8_t idx = 0;
	int8_t length = strlen(str_in);

	for (counter = 0; counter < length; counter ++)
	{
		if (str_in[counter] == l)
		{
			counter ++;//sale dejando apuntando al siguiente byte
			//@N512F
			//copy
			for (;counter < length; counter++)
			{

				if (str_in[counter] == r)
				{
					//ok, hasta aqui nomas, procede
					str_out[idx] = '\0';//fin de cadena trimmed
					return 1;
				}

				str_out[idx++] = str_in[counter];
			}
		}
	}

	return 0;
}
*/

/*
 * la busqueda en el buffer circular es cada "x" ms
 * no puede ser directa porque perderia mucho tiempo hasta que se complete la trama completa
 *
 * octave:7>  dec2hex(sum(int8('@N512F1023R256')))
	ans = 321 -> el resultado esta en HEX, solo me quedo con el byte menor = 0x21
 *
 * 	@N512F1023R256C21
	@N512F1023R257C22
 */
#define RX_CSTR_SIZE 32
struct _job_rx
{
	int8_t sm0;
}rx;
void rx_trama(void)
{
	char sbuff_out_temp[SCIRBUF_BUFF_SIZE];
	//Cstr es siempre static porque recolecta y junta todos los caracteres disponibles del buffer serial
	static char Cstr[RX_CSTR_SIZE];//todos los bytes se inicializan a 0
	uint8_t bytes_available;
	char USB_DATACODE = ' ';
	char USB_payload_char[30];
	int8_t USB_payload_idx=0;
	char c;
	int8_t newData = 0;
	int length;


	//busqueda en buffer circular
	bytes_available = scirbuf_bytes_available();
	if (bytes_available>0)
	{
		scirbuf_read_nbytes((uint8_t*)sbuff_out_temp, bytes_available); //hago la copia desde el buffer circular hacia el de salida temporal
		//
		sbuff_out_temp[bytes_available] = '\0';//convertir en c_str
		strcat(Cstr,sbuff_out_temp);

		//
		length = strlen(Cstr);

//usart_print_string(Cstr);

		rx.sm0 = 0;
		for (int i=0; i< length; i++)
		{
			c =  Cstr[i];
			if (rx.sm0 == 0)
			{
				if ( c == USB_DATACODE_TOKEN_BEGIN)
				{
					USB_payload_idx = 0;
					rx.sm0++;
				}
			}
			else if (rx.sm0 == 1)
			{
				USB_DATACODE = c;
				rx.sm0++;
			}
			else if (rx.sm0 == 2)//storage payload
			{
				if (c == USB_DATACODE_TOKEN_END)
				{
					USB_payload_char[USB_payload_idx] = '\0';
					//
					strcpy(Cstr,"");

					rx.sm0 = 0;
					newData = 1;
					break;
				}
				else
				{
					USB_payload_char[USB_payload_idx++] = c;
				}
			}
		}
		if (newData == 1)
		{
			//float payload_f = atof(USB_payload_char);
			newData = 0;

			switch (USB_DATACODE)
			{
				case USB_DATACODE_OUT2_ON:
			    	//usart_print_string("USB_DATACODE_OUT2_ON");
					main_flag.send_corriente = 1;
				break;

				case USB_DATACODE_OUT2_OFF:
					//usart_print_string("USB_DATACODE_OUT2_OFF");
					main_flag.send_corriente = 0;
				 break;
//				case USB_DATACODE_AMPLIFICARx10_ON:
//					//usart_print_string("USB_DATACODE_AMPLIFICARx10_ON");
//				 break;
//				case USB_DATACODE_AMPLIFICARx10_OFF:
//					//usart_print_string("USB_DATACODE_AMPLIFICARx10_OFF");
//				 break;
				case USB_DATACODE_CAPTURA1_ON:
					//usart_print_string("USB_DATACODE_CAPTURA1_ON");
					job_captura1.sm0 = 1;
				 break;
				case USB_DATACODE_CAPTURA2_ON:
					//usart_print_string("USB_DATACODE_CAPTURA2_ON");
					job_captura2.sm0 = 1;
				 break;
				case USB_DATACODE_CAPTURA1_OFF:
					//usart_print_string("USB_DATACODE_CAPTURA1_OFF");
//					job_captura1 = job_capture_mvx = job_reset;
//					smoothAlg_mvx = smoothAlg_reset;
//					break;

				case USB_DATACODE_CAPTURA2_OFF:
					//usart_print_string("USB_DATACODE_CAPTURA2_OFF");
					job_captura2 = job_capture_mvx = job_reset;
					smoothAlg_mvx = smoothAlg_reset;

				 break;
				default:
					break;
			}
		}

	}


}


