/*
 * INA238.h
 *
 *  Created on: May 30, 2021
 *      Author: jcaf
 */

#ifndef SRC_INA238_INA238_H_
#define SRC_INA238_INA238_H_

/*
 * The device allows for selectable ADC conversion
times from 50 µs to 4.12 ms as well as sample
averaging from 1x to 1024x which further helps
reduce the noise of the measured data.
 */

/*
 * VDIFF Shunt voltage input range
TA = –40 °C to +125 °C, ADCRANGE = 0 –163.84 163.84 mV
TA = –40 °C to +125 °C, ADCRANGE = 1 –40.96 40.96 mV
 */

/*
 * ADC resolution 16 Bits
1 LSB step size
Shunt voltage, ADCRANGE = 0 5 µV
Shunt voltage, ADCRANGE = 1 1.25 µV
Bus voltage 3.125 mV
Temperature 125 m°C
TCT ADC conversion-time(1)
50
µs
84
150
280
540
1052
2074
4120
 */


/*
 * The supported common-mode voltage range at the input pins is –0.3 V to +85 V,
which makes the device well suited for both high-side and low-side current measurements. There are no special
considerations for power-supply sequencing because the common-mode input range and device supply voltage
are independent of each other; therefore, the bus voltage can be present with the supply voltage off, and viceversa without damaging the device.
 */

/*For applications that must synchronize with other components in the system, the INA238 conversion can be
delayed by programming the CONVDLY bits in CONFIG register in the range between 0 (no delay) and 510 ms.
The resolution in programming the conversion delay is 2 ms. T
 *
 */

/*
 * *ADC Configuration (ADC_CONFIG) Register (Address = 1h) [reset = FB68h]
------------------------------------------------------------------------
MODE = TRIGGERED OR CONTINUOUS
VBUSCT = 50us... 4120us (4.120ms) -> x BUS VOLTAGE
BSHCT  = 50us... 4120us (4.120ms) -> x SHUNT VOLTAGE!
VTCT   = 50us... 4120us (4.120ms) -> x TEMPERATURE VOLTAGE!
AVG AVERAGING ...1..1024 SAMPLES

*
Conversion Time @4.120 ms tiene la menor cantidad de ruido
----------------------

TRABAJAR CON UNA TASA Y TC PARA MINIZAR EL RUIDO
OUTPUT AVERAGE = 1024
ADC TIMNE CONVERSION = 150
 *
 */

//DATASHEET
#define INA238_1_LSB_STEPSIZE_ADCRANGE_163p84mV 5E-6//	5uV
#define INA238_1_LSB_STEPSIZE_ADCRANGE_40p96mV 1.25E-6//	1.25uV
#define INA238_1_LSB_STEPSIZE_BUS_VOLTAGE 3.125E-3		//mV
#define INA238_1_LSB_STEPSIZE_TEMPERATURE 125E-6	//mC

//I2C_ADDRESS
#define INA238_I2C_ADDR_A1_GND_A2_GND 0x80

//INA328 Registers
#define INA238_REG_CONFIG		0x00
#define INA238_REG_ADC_CONFIG	0x01
#define INA238_REG_SHUNT_CAL	0x02
#define INA238_REG_VSHUNT		0x04
#define INA238_REG_VBUS			0x05
#define INA238_REG_DIETEMP		0x06
#define INA238_REG_CURRENT		0x07
#define INA238_REG_POWER		0x08
#define INA238_REG_DIAG_ALRT	0x0B
#define INA238_REG_SOVL			0x0C
#define INA238_REG_SUVL			0x0D
#define INA238_REG_BOVL			0x0E
#define INA238_REG_BUVL			0x0F
#define INA238_REG_TEMP_LIMIT	0x10
#define INA238_REG_PWR_LIMIT	0x11
#define INA238_REG_MANUFACTURER_ID	0x3E
#define INA238_REG_DEVICE_ID		0x3F

//
#define IN238_CONFIG_ADCRANGE_163p84mV 0
#define IN238_CONFIG_ADCRANGE_40p96mV 1


//CONFIG REGISTER BITS
//Configuration (CONFIG) Register (Address = 0h) [reset = 0h]
#define INA238_CONFIG_BIT_RST 15
#define INA238_CONFIG_BIT_CONVDLY	6
#define INA238_CONFIG_BIT_ADCRANGE	4

//ADC_CONFIG (Address = 1h) [reset = FB68h]
#define INA238_ADC_CONFIG_BIT_MODE 12
#define INA238_ADC_CONFIG_BIT_VBUSCT 9
#define INA238_ADC_CONFIG_BIT_VSHCT 6
#define INA238_ADC_CONFIG_BIT_VTCT 3
#define INA238_ADC_CONFIG_BIT_AVG 0

//ADC_CONFIG BITS
#define INA238_ADC_CONFIGMODE_SHUTDOWN0 0x00
#define INA238_ADC_CONFIGMODE_TRIGGEREDBUS_SINGLESHOT	0x01
#define INA238_ADC_CONFIGMODE_TRIGGEREDSHUNT_SINGLESHOT	0x02
#define INA238_ADC_CONFIGMODE_TRIGGEREDSHUNT_BUS_SINGLESHOT	0x03
#define INA238_ADC_CONFIGMODE_TRIGGEREDTEMPERATURE_SINGLESHOT 0x04
#define INA238_ADC_CONFIGMODE_TRIGGEREDTEMPERATURE_BUS_SINGLESHOT 0x05
#define INA238_ADC_CONFIGMODE_TRIGGEREDTEMPERATURE_SHUNT_SINGLESHOT 0x06
#define INA238_ADC_CONFIGMODE_TRIGGEREDBUS_SHUNT_TEMPERATURE_SINGLESHOT 0x07
#define INA238_ADC_CONFIGMODE_SHUTDOWN1 0x08
#define INA238_ADC_CONFIGMODE_CONTINUOUS_BUS_ONLY 0x09
#define INA238_ADC_CONFIGMODE_CONTINUOUS_SHUNT_ONLY 0x0A
#define INA238_ADC_CONFIGMODE_CONTINUOUS_SHUNT_BUS_ONLY 0x0B
#define INA238_ADC_CONFIGMODE_CONTINUOUS_TEMPERATURE_ONLY 0x0C
#define INA238_ADC_CONFIGMODE_CONTINUOUS_BUS_TEMPERATURE 0x0D
#define INA238_ADC_CONFIGMODE_CONTINUOUS_TEMPERATURE_SHUNT 0x0E
#define INA238_ADC_CONFIGMODE_CONTINUOUS_BUS_SHUNT_TEMPERATURE 0x0F

//ADC_CONFIG BITS CONVERSION TIME FOR VBUS = VSHUNT = VTEMPERATURE
#define INA238_ADC_CONFIGMODE_CT_50uS	0x00
#define INA238_ADC_CONFIGMODE_CT_84uS	0x01
#define INA238_ADC_CONFIGMODE_CT_150uS	0x02
#define INA238_ADC_CONFIGMODE_CT_280uS	0x03
#define INA238_ADC_CONFIGMODE_CT_540uS	0x04
#define INA238_ADC_CONFIGMODE_CT_1052uS	0x05
#define INA238_ADC_CONFIGMODE_CT_2074uS	0x06
#define INA238_ADC_CONFIGMODE_CT_4120uS	0x07

//ADC_CONFIG BITS AVG SAMPLE AVERAGING COUNT
#define INA238_ADC_CONFIGMODE_AVG_SAMPLE_1		0x00
#define INA238_ADC_CONFIGMODE_AVG_SAMPLE_4		0x01
#define INA238_ADC_CONFIGMODE_AVG_SAMPLE_16		0x02
#define INA238_ADC_CONFIGMODE_AVG_SAMPLE_64		0x03
#define INA238_ADC_CONFIGMODE_AVG_SAMPLE_128	0x04
#define INA238_ADC_CONFIGMODE_AVG_SAMPLE_256	0x05
#define INA238_ADC_CONFIGMODE_AVG_SAMPLE_512	0x06
#define INA238_ADC_CONFIGMODE_AVG_SAMPLE_1024	0x07


//Temperature Measurement (DIETEMP) Register (Address = 6h) [reset = 0h]
#define INA238_DIETEMP_BIT_DIETEMP 4


////////////////////////////////////////////////////////////////
#define IN238_MAXIMUM_EXPECTED_CURRENT 500E-3	//500 miliampers
#define INA238_CURRENT_LSB (IN238_MAXIMUM_EXPECTED_CURRENT/(32768))	//2^15 =
#define INA238_RSHUNT 50E-3	//50 miliOhms
//
//819.2 x 10e6 is an internal fixed value used to ensure scaling is maintained properly.
#define INA238_SHUNT_CAL_ADCRANGE_163p84mV (819.2E6 * INA238_CURRENT_LSB * INA238_RSHUNT)
//the value of SHUNT_CAL must be multiplied by 4 for ADCRANGE = 1
#define INA238_SHUNT_CAL_ADCRANGE_40p96mV INA238_SHUNT_CAL_ADCRANGE_163p84mV*4


///////////////////////////////////////////////////
void INA238_init(void);
int16_t INA238_read_shuntvoltage_register(void);
int16_t INA238_read_current_register(void);

#endif /* SRC_INA238_INA238_H_ */
