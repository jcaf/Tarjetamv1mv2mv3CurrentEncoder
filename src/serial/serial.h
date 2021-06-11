#ifndef SERIAL_H_
#define SERIAL_H_
    #include "stdint.h"

    #define SCIRBUF_BUFF_SIZE 32//2^5 = 32 //4//power of 2
    #define SCIRBUF_BUFF_SIZE_MASK (SCIRBUF_BUFF_SIZE-1)

struct _scirbuf
{
    uint8_t buffer[SCIRBUF_BUFF_SIZE];
    uint8_t ir;//index read
    uint8_t iw;//index storage

    union _serial_cirbuf_uflags
    {
        struct _serial_cirbuf_uflags_bf
        {
            unsigned ov:1;
            unsigned ow:1;
            unsigned buff_lock:1;//protege el buffer de escritura cuando se lee
            unsigned __a:5;
        }bf;
        uint8_t bitflags;
    }uflags;

};

extern volatile struct _scirbuf scirbuf;
void scirbuf_storage(uint8_t c);


uint8_t scirbuf_bytes_available(void);
void scirbuf_write(uint8_t c);
uint8_t scirbuf_read(void);
void scirbuf_set_ir_equal_iw(void);
uint8_t scirbuf_read_nbytes(uint8_t *buff_out, uint8_t n);
void rx_handler(void);
#endif
