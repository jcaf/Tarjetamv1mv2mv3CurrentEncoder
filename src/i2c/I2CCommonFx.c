#include "I2CCommonFx.h"
#include "I2C.h"
#include "../system.h"
#include "../types.h"


//static void I2Ccfx_InitSeqReading(byte SLAVE_ADDRESS, byte START_ADDRESS);
/********************************************************************************
MEMORY ADDRESS: 1 byte
DATA: 1 byte
********************************************************************************/
void I2Ccfx_Write1Byte(byte SLAVE_ADDRESS, byte START_ADDRESS, byte DATA)
{
    I2C_unimaster_start();
    I2C_unimaster_tx_addrslave((SLAVE_ADDRESS & 0xFE)| I2C_MASTER_WRITE);
    I2C_unimaster_tx_data(START_ADDRESS);
    I2C_unimaster_tx_data(DATA);
    I2C_unimaster_stop();
}
/********************************************************************************
MEMORY ADDRESS: 1 byte
DATA: Array [max 256 elementos]
********************************************************************************/
void I2Ccfx_WriteArray(byte SLAVE_ADDRESS, byte START_ADDRESS, volatile byte *pDATA, byte NUMBYTES_TOWRITE)
{

	I2C_unimaster_start();
    I2C_unimaster_tx_addrslave((SLAVE_ADDRESS & 0xFE)| I2C_MASTER_WRITE);
    I2C_unimaster_tx_data(START_ADDRESS);
    for (volatile byte i=0; i<NUMBYTES_TOWRITE; i++)
    {
        I2C_unimaster_tx_data(pDATA[i]);
    }
    I2C_unimaster_stop();
}
/********************************************************************************
Reading
********************************************************************************/
static void I2Ccfx_InitSeqReading(byte SLAVE_ADDRESS, byte START_ADDRESS)
{
    I2C_unimaster_start();
    I2C_unimaster_tx_addrslave(SLAVE_ADDRESS | I2C_MASTER_WRITE);
    I2C_unimaster_tx_data(START_ADDRESS);	//Direccion interna
    I2C_unimaster_restart();
    I2C_unimaster_tx_addrslave(SLAVE_ADDRESS | I2C_MASTER_READ);
}
/********************************************************************************

********************************************************************************/

#define _NACK_ (0)
#define _ACK_ (1)

void I2Ccfx_ReadRegistersAtAddress(byte SLAVE_ADDRESS, byte START_ADDRESS, volatile byte * pDATA, byte NUMBYTES_TOREAD)
{
    byte Count_NumBytesRead=0;

    I2Ccfx_InitSeqReading((SLAVE_ADDRESS & 0xFE), START_ADDRESS);

    do
    {
        Count_NumBytesRead++;

        //Dejar preparado el ACK/NACK para el Ste. evento
        if (Count_NumBytesRead < NUMBYTES_TOREAD)
        {
            I2C_unimaster_rx_data(_ACK_);
        }
        else
        {
            I2C_unimaster_rx_data(_NACK_);   //Finalizar recepciÃ³n
        }

        //*pDATA = _TWDR;	//I2C_unimaster_rx_data	actualiza variable global _TWDR
        *pDATA = TWDR;
        pDATA++;
    }
    while (Count_NumBytesRead < NUMBYTES_TOREAD);

    I2C_unimaster_stop();
}
