
#include "../types.h"
#include "../system.h"

#include "I2C.h"



void I2C_unimaster_error_handler(byte _TWSR_ERROR_CODE_);
/********************************************************************
I2C modo MASTER (1 MASTER ON THE I2C BUS)
********************************************************************/
//void I2C_unimaster_init(void)
void I2C_unimaster_init(uint32_t I2C_FREQ)
//void I2C_unimaster_init(uint32_t I2C_FREQ, unsigned long F_CPU)
{
    //pull-up  off
    //PinTo0(PORTWxHW0_I2C_SCL,PIN_HW0_I2C_SCL);
    //PinTo0(PORTWxHW0_I2C_SDA,PIN_HW0_I2C_SDA);
    //

    #define PREESCALER_I2C 1
    //#define PREESCALER_I2C  4
    //#define PREESCALER_I2C  16
    //#define PREESCALER_I2C  64	//NO USADO
    #if PREESCALER_I2C == 1
        #define  TWI_PREESCALER_POTENCIA 4			//4^1
    #elif PREESCALER_I2C == 4
        #define  TWI_PREESCALER_POTENCIA 256			//4^4
    #elif PREESCALER_I2C == 16
        #define  TWI_PREESCALER_POTENCIA 4294967296	//4^16
    #endif

    #ifndef F_CPU
        #error "I2C: F_CPU NO DEFINIDO"
    #else
        #define BIT_RATE_G_KTE(I2C_FREQ) (  (uint8_t)( (F_CPU-(16*I2C_FREQ) )/(2*I2C_FREQ*TWI_PREESCALER_POTENCIA))  )
    #endif
    TWSR = 0x00;						//TWPS1=PWPS0=0->Preescaler "1"
    //TWBR = BIT_RATE_G_KTE(100*KHz);
    TWBR = (uint8_t)( (F_CPU-(16*I2C_FREQ) )/(2*I2C_FREQ*TWI_PREESCALER_POTENCIA));
    //100Khz @16Mhz/PREESCALER=1  BR = 18
    //400Khz @16Mhz/PREESCALER=1  BR =  3
    //100Khz @8Mhz/PREESCALER=1  BR = 8
    //400Khz @8Mhz/PREESCALER=1  BR = 0.5
    //if TWSR<PREESCALER> = 00 osea Preescaler 1
}

/********************************************************************************
Despues de enviar Start-> TWSTA debe ser borrado por software al cargar TWCR/TWDR
********************************************************************************/
int8_t I2C_unimaster_start(void)
{
    TWCR = ( 1<<TWINT ) | ( 1<<TWSTA) | (1<<TWEN);
    while ( ! (TWCR & (1<<TWINT)) )
    {
        ;   //Esperar POR TWINT =1 x HARDWARE
    }

    if ( (TWSR & 0xF8) != I2C_STATUS_START_SUCCESS)
    {
        I2C_unimaster_error_handler(TWSR & 0xF8);
        return	0;
    }

    return 1;
}
/********************************************************************************
Despues de enviar Restart-> TWSTA debe ser borrado por software al cargar TWCR/TWDR
********************************************************************************/
int8_t I2C_unimaster_restart(void)
{
    TWCR = ( 1<<TWINT ) | ( 1<<TWSTA) | (1<<TWEN);
    while ( ! (TWCR & (1<<TWINT)) )
    {
        ;   //Esperar POR TWINT =1 x HARDWARE
    }

    if ( (TWSR  & 0xF8) != I2C_STATUS_REPEAT_SUCCESS)
    {
        I2C_unimaster_error_handler(TWSR  & 0xF8);
        return	0;
    }
    return 1;
}
/********************************************************************************
Note that TWINT is NOT set after a STOP condition has been sent.
Solo Esperar por TWSTO q sea borrado automaticamente por el hardware
********************************************************************************/
void I2C_unimaster_stop(void)
{
    TWCR = ( 1<<TWINT ) | ( 1<<TWSTO) | ( 1<<TWEN );
    while ( TWCR & (1<<TWSTO) )
    {
        ;
    }
}
/********************************************************************************
********************************************************************************/
void I2C_unimaster_tx_byte(byte _Byte_)
{
    TWDR = _Byte_;
    TWCR = ( 1<<TWINT ) | (1<<TWEN);    //x data, despues de un direccionamiento correcto
    //TWCR = ( 1<<TWINT ) | ( 0<<TWSTA) | (1<<TWEN);    //x address, despues de START condition
    while ( !(TWCR & (1<<TWINT)) )
    {
        ;   //TransmisiÃ³n en progreso
    }
}
/********************************************************************************
********************************************************************************/
int8_t I2C_unimaster_tx_data(byte Data)
{
    I2C_unimaster_tx_byte(Data);

    if ( (TWSR & 0xF8) != I2C_STATUS_DATA_TX_SUCCESS_ACK_SUCCESS)
    {
        I2C_unimaster_error_handler(TWSR & 0xF8);
        return 0;
    }
    return 1;
}
/********************************************************************************
********************************************************************************/
int8_t I2C_unimaster_tx_addrslave(byte SlaveAddress_RW)
{
    byte I2C_STATUS_CODE_R_W;
    //Inicializar KTE para comparaciÃ³n de ERROR
    if ( (SlaveAddress_RW & 0x01 )== I2C_MASTER_READ )
    {
        I2C_STATUS_CODE_R_W = I2C_STATUS_SLA_R_SUCCESS_ACK_SUCCESS;//CODIGO DE STATUS PARA LA LECTURA...
    }
    else
    {
        I2C_STATUS_CODE_R_W = I2C_STATUS_SLA_W_SUCCESS_ACK_SUCCESS ;//CODIGO DE STATUS PARA LA ESCRITURA...
    }
    I2C_unimaster_tx_byte(SlaveAddress_RW);

    if ( (TWSR & 0xF8) != I2C_STATUS_CODE_R_W)
    {
        I2C_unimaster_error_handler(TWSR & 0xF8);
        return 0;
    }
    return 1;
}

/****************************************************************
#define _NACK_ (0)
#define _ACK_ (1)

output: _TWDR update si Status=1
****************************************************************/
int8_t I2C_unimaster_rx_data(byte PrepararACK_NACK)
{
    byte I2C_StatusCode_R_ACK_NACK;

    if (PrepararACK_NACK)
    {
        I2C_StatusCode_R_ACK_NACK = I2C_STATUS_DATA_RX_SUCCESS_ACK_SUCCESS;//Inicializar codigo de respuesta
        TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) ;	//HEMOS RECIBIDO EL DATO Y HEMOS GENERADO UN "ACK PARA PROSEGUIR CON LA RECEPCION
    }
    else//Finalizar recepciÃ³n
    {
        I2C_StatusCode_R_ACK_NACK = I2C_STATUS_DATA_RX_SUCCESS_ACK_UNSUCCESS;//Inicializar codigo de respuesta para
        TWCR = (1<<TWINT) | (0<<TWEA) | (1<<TWEN) ;	////HEMOS RECIBIDO EL DATO Y HEMOS GENERADO UN "NACK PARA FINALIZAR LA RECEPCION
    }
    while ( ! (TWCR & (1<<TWINT)) )
    {
        ;   //Esperamos por dato desde esclavo. Si TWINT=1->Se lleno TWDR
    }

    if	( (TWSR & 0xF8) != I2C_StatusCode_R_ACK_NACK )
    {
        I2C_unimaster_error_handler(TWSR & 0xF8);
        return 0;
    }
    //_TWDR = TWDR;	//TWDR debe mantener la data recibida de manera integral, pues, si no hubo ningun caos en el bus,
    //el host no finaliza bruscamente con STOP.
    return 1;
}
/********************************************************************************

********************************************************************************/

void I2C_unimaster_error_handler(byte _TWSR_ERROR_CODE_)
{

    I2C_unimaster_stop();



    switch (_TWSR_ERROR_CODE_)
    {

    //I2C STATUS CODES x MASTER TRANSMITTER
    case  I2C_STATUS_SLA_W_SUCCESS_ACK_UNSUCCESS: 	//(0x20)
        break;
    case  I2C_STATUS_DATA_TX_SUCCESS_ACK_UNSUCCESS: //(0x30)
        break;

    //el mismo codigo para ambos W/R
    //case  I2C_STATUS_ARBITRATION_LOST_IN_SLA_W: 	//(0x38)
    //I2C STATUS CODES x MASTER RECEIVER
    //case  I2C_STATUS_ARBITRATION_LOST_IN_SLA_R: 	//(0x38)
    case 0x38: //(0x38)

        break;

    case  I2C_STATUS_SLA_R_SUCCESS_ACK_UNSUCCESS: 	//(0x48)
        break;
    case  I2C_STATUS_DATA_RX_SUCCESS_ACK_UNSUCCESS: //(0x58)
        break;

    //otros
    case  I2C_STATUS_NO_RELEVANT_STATE_INFORMATION:	//(0xF8)
        break;
    case  I2C_STATUS_BUS_ERROR_ILLEGAL_START_STOP:	//(0x00)

        break;
    }

}


