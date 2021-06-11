#include "../../src/serial/serial.h"

#include <avr/io.h>
#include "../usart/usart.h"

volatile struct _scirbuf scirbuf;//volatile por el flag buff_lock

uint8_t scirbuf_bytes_available(void)
{
    uint8_t k, len;

    if (scirbuf.uflags.bf.ov == 0)
    {
        len = scirbuf.iw - scirbuf.ir;
    }
    else//ov=1
    {
        if (scirbuf.uflags.bf.ow == 0)
        {
            k =  scirbuf.ir;
        }
        else//ow=1
        {
            //no interesa si ir es mayor, menor
            //scirbuf.ir = scirbuf.iw;//x lectura es necesario
            k = scirbuf.iw;
        }
        len = (SCIRBUF_BUFF_SIZE - k) + scirbuf.iw;
    }
    return len;
}
////////////////////////////////////////////////////////////////////////////////
void scirbuf_write(uint8_t c)
{
    scirbuf.buffer[scirbuf.iw] = c;

   if (scirbuf.uflags.bf.ov == 1)
   {
      if (scirbuf.iw == scirbuf.ir)
       {
	   scirbuf.uflags.bf.ow = 1;
       }
   }
    
    if (++scirbuf.iw == SCIRBUF_BUFF_SIZE)
    {
        scirbuf.iw = 0;//reset index
        scirbuf.uflags.bf.ov = 1;
    }
}
////////////////////////////////////////////////////////////////////////////////
uint8_t scirbuf_read(void)
{
    //uint8_t ir = scirbuf.ir++ & SCIRBUF_BUFF_SIZE_MASK;
	//uint8_t c = scirbuf.buffer[ir];
	
	//1er metodo	
	/*
	uint8_t c = scirbuf.buffer[scirbuf.ir];

	if (++scirbuf.ir == SCIRBUF_BUFF_SIZE)
		{scirbuf.ir = 0x00;}

    if (scirbuf.ir == scirbuf.iw)
    {
        scirbuf.uflags.bf.ov = 0;
        scirbuf.uflags.bf.ow = 0;
    }
    return c;
    */
    //2do metodo
	uint8_t c = scirbuf.buffer[scirbuf.ir];
	
	scirbuf.ir++;
	scirbuf.ir &= SCIRBUF_BUFF_SIZE_MASK;


    if (scirbuf.ir == scirbuf.iw)
    {
        scirbuf.uflags.bf.ov = 0;
        scirbuf.uflags.bf.ow = 0;
    }
    return c;
    
}
////////////////////////////////////////////////////////////////////////////////
//void scirbuf_set_ir_equal_iw(void)
//{
//   scirbuf.ir = scirbuf.iw;
//}
////////////////////////////////////////////////////////////////////////////////
uint8_t scirbuf_read_nbytes(uint8_t *buff_out, uint8_t n)
{
    uint8_t i,x;
    uint8_t temp, resto;
	x = 0;
	if (n > 0 )
    {
        if (scirbuf.uflags.bf.ov == 0)
        {
            for (i=0; i<n; i++)
            {
                buff_out[x++] = scirbuf_read();
            }
        }
        else//ov=1
        {
            scirbuf.uflags.bf.buff_lock = 1;

            if (scirbuf.uflags.bf.ow == 1)
            {
                scirbuf.ir = scirbuf.iw;
            }

            temp = SCIRBUF_BUFF_SIZE - scirbuf.ir;

            if ( temp >= n)
            {
                for (i=0; i<n; i++)
                {
                    buff_out[x++] = scirbuf_read();
                }
            }
            else//en esta parte leeria todos los bytes, desde los mas antiguos a los mas nuevos
            {
                for (i=0; i<temp; i++)
                {
                    buff_out[x++] = scirbuf_read();
                }

                resto = n - temp; //lo q queda por leer
                for (i=0; i<resto; i++)
                {
                    buff_out[x++] = scirbuf_read();
                }
            }

            scirbuf.uflags.bf.buff_lock = 0;
        }
    }


    return x;//el numero de bytes leidos?
}
////////////////////////////////////////////////////////////////////////////////
uint8_t scirbuf_read_1byte(void)
{
    uint8_t buff_out;
    
    if (scirbuf.uflags.bf.ov == 0)
    {
        buff_out = scirbuf_read();
    }
    else//ov=1
    {
        scirbuf.uflags.bf.buff_lock = 1;

	        if (scirbuf.uflags.bf.ow == 1)
	        {
	            scirbuf.ir = scirbuf.iw;
	        }
	        buff_out = scirbuf_read();

        scirbuf.uflags.bf.buff_lock = 0;
    }
    return buff_out;
}
////////////////////////////////////////////////////////////////////////////////
void rx_handler(void)//call from ISR
{
#define SOFTBUFF_SIZE 4
#define SOFTBUFF_SIZE_MASK (SOFTBUFF_SIZE-1)
    static uint8_t soft_buff[4];
    static int8_t level;
    uint8_t i;

    
	uint8_t char_rx  = UDR0;
	USART_Transmit(char_rx);

    if (scirbuf.uflags.bf.buff_lock == 1)
    {
        soft_buff[level++ & SOFTBUFF_SIZE_MASK] = char_rx;
    }
    else
    {
        //hay bytes almacenados?
        if (level > 0)
        {
            //copy to buffer + add last buffer
            for (i=0; i<level; i++)
            {
                scirbuf_write(soft_buff[i]);
            }
            level = 0x00;
        }
        scirbuf_write(char_rx);
    }
}
