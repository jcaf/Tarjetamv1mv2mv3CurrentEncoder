#ifndef I2C_H_
#define I2C_H_

/*******************************************************************************
			UBICACION DE PINS I2C - HARDWARE 0
*******************************************************************************/
#define DDRxHW0_I2C_SCL		    (DDRC)
#define PORTWxHW0_I2C_SCL		(PORTC)
#define PORTRxHW0_I2C_SCL		(PINC)
#define PIN_HW0_I2C_SCL 		(5)
//
#define DDRxHW0_I2C_SDA		    (DDRC)
#define PORTWxHW0_I2C_SDA		(PORTC)
#define PORTRxHW0_I2C_SDA		(PINC)
#define PIN_HW0_I2C_SDA 		(4)

/*******************************************************************************
				I2C STATUS CODES
*******************************************************************************/
#define  I2C_STATUS_START_SUCCESS 					  (0x08)
#define  I2C_STATUS_REPEAT_SUCCESS 					  (0x10)

#define  I2C_STATUS_NO_RELEVANT_STATE_INFORMATION	  (0xF8)
#define  I2C_STATUS_BUS_ERROR_ILLEGAL_START_STOP	  (0x00)

/*******************************************************************************
I2C STATUS CODES x MASTER TRANSMITTER
*******************************************************************************/
#define  I2C_STATUS_SLA_W_SUCCESS_ACK_SUCCESS 		  (0x18)
#define  I2C_STATUS_SLA_W_SUCCESS_ACK_UNSUCCESS 	  (0x20)

#define  I2C_STATUS_DATA_TX_SUCCESS_ACK_SUCCESS 	  (0x28)
#define  I2C_STATUS_DATA_TX_SUCCESS_ACK_UNSUCCESS 	  (0x30)

#define  I2C_STATUS_ARBITRATION_LOST_IN_SLA_W 		  (0x38)

/*******************************************************************************
I2C STATUS CODES x MASTER RECEIVER
*******************************************************************************/
#define  I2C_STATUS_ARBITRATION_LOST_IN_SLA_R 		  (0x38)

#define  I2C_STATUS_SLA_R_SUCCESS_ACK_SUCCESS 		  (0x40)
#define  I2C_STATUS_SLA_R_SUCCESS_ACK_UNSUCCESS 	  (0x48)

#define  I2C_STATUS_DATA_RX_SUCCESS_ACK_SUCCESS 	  (0x50)
#define  I2C_STATUS_DATA_RX_SUCCESS_ACK_UNSUCCESS 	  (0x58)

/*******************************************************************************
Definiciones I2C - ORing
*******************************************************************************/
#define  I2C_MASTER_WRITE 		 	(0x00)
#define  I2C_MASTER_READ 		 	(0x01)
/*******************************************************************************

*******************************************************************************/
//#include "src/types.h"
//void I2C_unimaster_init(void);
void I2C_unimaster_init(uint32_t I2C_FREQ);
int8_t I2C_unimaster_start(void);
int8_t I2C_unimaster_restart(void);
void I2C_unimaster_stop(void);
void I2C_unimaster_tx_byte(byte _Byte_);
int8_t I2C_unimaster_tx_data(byte Data);
int8_t I2C_unimaster_tx_addrslave(byte SlaveAddress_RW);
int8_t I2C_unimaster_rx_data(byte PrepararACK_NACK);

void I2C_unimaster_error_handler(byte _TWSR_ERROR_CODE_);
#endif


