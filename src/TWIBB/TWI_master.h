

#ifndef TWI_MASTER_H_
#define TWI_MASTER_H_

//#include "ioavr.h"
//#include "inavr.h"
#include "../ads1115/ads1115.h"

/*! \brief Definition of pin used as SCL. */
#define SCL PC2

/*! \brief Definition of pin used as SDA. */
#define SDA PC3

/*! \brief Definition of PORT used as SCL. */
#define PORT_SCL PORTC
/*! \brief Definition of DDR used as SCL. */
#define DDR_SCL	DDRC
/*! \brief Definition of PIN used as SCL. */
#define PIN_SCL PINC
/*! \brief Definition of PORT used as SDA. */
#define PORT_SDA PORTC
/*! \brief Definition of DDR used as SDA. */
#define DDR_SDA	DDRC
/*! \brief Definition of PIN used as SDA. */
#define PIN_SDA PINC

/*! \brief Slave 8 bit address (shifted). */

#define SLAVE_ADDRESS ADS115_ADR_GND//0xA0

#define READ_SDA() (PIN_SDA & (1 << SDA))
#define SET_SDA_OUT() DDR_SDA |= (1 << SDA)
#define SET_SDA_IN() DDR_SDA &= ~(1 << SDA)
#define READ_SCL() (PIN_SCL & (1 << SCL))?1:0

#define WRITE 0x0
#define READ 0x1

/*! \brief Delay used to generate clock */
#define DELAY 2

/*! \brief Delay used for STOP condition */
#define SCL_SDA_DELAY 1


void twi_disable();
void twi_init();
void toggle_scl();
void write_scl(char x);
char twi_start_cond(void);
char send_slave_address(unsigned char read);
char write_data(unsigned char* data, char bytes);
char i2c_write_byte(unsigned char byte);
char read_bytes(unsigned char* data, char bytes);
char i2c_read_byte(unsigned char* data, unsigned char bytes, unsigned char index);
void write_sda( char x);


#define __delay_cycles(DELAY) __delay_us(5);//100Khz
//#define __delay_cycles(SCL_SDA_DELAY) __delay_us(2);//100Khz


#endif /* TWI_MASTER_H_ */
