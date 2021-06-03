#ifndef ADS_1115_H_
#define ADS_1115_H_

    //Slave address
    #define ADS115_ADR_GND 0x90//1001000
    #define ADS115_ADR_VDD 0x92//1001001
    #define ADS115_ADR_SDA 0x94//1001010
    #define ADS115_ADR_SCL 0x96//1001011

    //Address Pointer Register
    #define ADS1115_CONVRS_REG  0x00//conversion register
    #define ADS1115_CONFIG_REG  0x01//Config register
    #define ADS1115_LO_THRH_REG 0x02//Lo_threshold
    #define ADS1115_HI_THRH_REG 0x03//Hi_threshold

    //Config register AS 2-8Bits
//    #define OS_BIT      7//15
//    #define MUX_BIT     4//12
//    #define PGA_BIT     1//9
//    #define MODE_BIT    0//8

	#define OS_BIT      15
    #define MUX_BIT     12
    #define PGA_BIT     9
    #define MODE_BIT    8
    //
    #define DR_BIT          5
    #define COMP_MODE_BIT   4
    #define COMP_POL_BIT    3
    #define COMP_LAT_BIT    2
    #define COMP_QUE_BIT    0

    //PGA: Set the FSR
    #define PGA_6p144V  0x00
    #define PGA_4p096V  0x01
    #define PGA_2p048V  0x02    //default
    #define PGA_1p024V  0x03
    #define PGA_0p512V  0x04
    #define PGA_0p256V  0x05
    //#define PGA_0p256V  0x06
    //#define PGA_0p256V  0x07

    //Data rate
    #define DR_8SPS     0x00	//menor ruido
    #define DR_16SPS    0x01
    #define DR_32SPS    0x02
    #define DR_64SPS    0x03
    #define DR_128SPS   0x04  //default
    #define DR_250SPS   0x05
    #define DR_475SPS   0x06
    #define DR_860SPS   0x07

    //MUX[2:0]
    #define MUX_AIN0_AIN1   0x00
    #define MUX_AIN0_AIN3   0x01
    #define MUX_AIN1_AIN3   0x02
    #define MUX_AIN2_AIN3   0x03
    #define MUX_AIN0_GND    0x04
    #define MUX_AIN1_GND    0x05
    #define MUX_AIN2_GND    0x06
    #define MUX_AIN3_GND    0x07

    //MODE
    #define CONTINUOUS_CONV 0
    #define SINGLESHOT_POWERDOWN_CONV 1



void ADS1115_write_data16bits(uint8_t ADS1115_I2Caddr, uint8_t ADS1115register, uint16_t data16bits);

/*
 * Si es necesario usar una variable global para llevar la pista en RAM del CONFIG_REG
 */
#define ADS1115_CONFIG_REG_UPDATE_IN_RUNTIME

#ifdef ADS1115_CONFIG_REG_UPDATE_IN_RUNTIME
	void ADS1115_init(void);
	void ADS1115_setOS(uint8_t OS);
	void ADS1115_setMuxChannel(uint8_t MUX);
	void ADS1115_setPGA(uint8_t PGA);
	void ADS1115_setOperatingMode(uint8_t MODE);
	void ADS1115_setDataRate(uint8_t DR);
#endif

#endif // ADS_1115_H_
