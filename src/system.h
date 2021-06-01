#ifndef SYSTEM_H_
#define SYSTEM_H

//define now is in local:"main.h"
#define F_CPU 16000000UL	//16000000UL		//AVR-GCC
#define _XTAL_FREQ F_CPU	//C PIC

#define OR_BITWISE(REG, KTE)  	do{REG |= KTE;}while(0)
#define AND_BITWISE(REG, KTE)	do{REG &= KTE;}while(0)
#define XOR_BITWISE(REG, KTE) 	do{REG ^= KTE;}while(0)

//Write to generic register
#define BitTo1(REG, BIT) OR_BITWISE(REG, 1<<BIT)
#define BitTo0(REG, BIT) AND_BITWISE(REG, ~(1<<BIT))
#define BitToggle(REG,BIT) XOR_BITWISE(REG, 1<<BIT)

//Write physical port
#define PinTo1(PORTX,PIN) BitTo1(PORTX, PIN)
#define PinTo0(PORTX,PIN) BitTo0(PORTX, PIN)
#define PinToggle(PORTX,PIN) BitToggle(PORTX,PIN)
//Lectura de Pin
#define PinRead(PINX,PIN) ((PINX & (1<<PIN))>>PIN)
#define ReadPin(PINX,PIN) ((PINX & (1<<PIN))>>PIN)//To deprecate
//

#if defined(__XC) || defined(HI_TECH_C) || defined(_MPC_)

	#define INPUT_PORT8BIT  0xFF
	#define OUTPUT_PORT8BIT  0x00

	#define INPUT_PIN  1
	#define OUTPUT_PIN  0

	#define ConfigInputPin(IOREG,PIN)  BitTo1(IOREG, PIN)
	#define ConfigOutputPin(IOREG,PIN) BitTo0(IOREG, PIN)

	#define ConfigInputPort(IOREG,  BITS_INPUT) OR_BITWISE(IOREG, BITS_INPUT)
	#define ConfigOutputPort(IOREG, BITS_INPUT) AND_BITWISE(IOREG, ~(BITS_INPUT))

	//#define __delay_ms(x) //the same name as intrinsic functions for microchip
	//#define __delay_us(x)

	//Timer CPU INSTRUCCTION CYCLE = N Q_CYCLES
	#define Q_CYCLE 4   //PIC ARCH.
	#include <xc.h>

#elif defined(__GNUC__) && defined(__AVR__)

    #define INPUT_PORT8BIT  0x00
	#define OUTPUT_PORT8BIT  0xFF

	#define INPUT_PIN  0
	#define OUTPUT_PIN  1

	#define ConfigInputPin(IOREG,PIN) do{IOREG &= (~(0x01<<PIN));}while(0)
	#define ConfigOutputPin(IOREG,PIN) do{IOREG |= (0x01<<PIN);}while(0)

	#define ConfigInputPort(IOREG,  BITS_INPUT) AND_BITWISE(IOREG, ~(BITS_INPUT))
	#define ConfigOutputPort(IOREG, BITS_INPUT) OR_BITWISE(IOREG, BITS_INPUT)

	#define FLASH_DECLARE(x) x __attribute__((__progmem__))
	#define NOP() do{asm volatile ("nop");}while(0)


	//Timer CPU INSTRUCCTION CYCLE = N Q_CYCLES
	#define Q_CYCLE 1   //AVR ARCH.
	//
	#include <avr/io.h>
	#include <avr/interrupt.h>
    	#include <avr/pgmspace.h>
	#include <avr/eeprom.h>
    	#include <util/delay.h>
	#define __delay_ms(x) _delay_ms(x)
	#define __delay_us(x) _delay_us(x)

	#define CTC_SET_OCR_BYFREQ(CTC_FREQ, CTC_PRESCALER) ( (uint8_t)( (F_CPU/ (2.0*CTC_PRESCALER*CTC_FREQ)) -1) )
	#define CTC_SET_OCR_BYTIME(CTC_TIME, CTC_PRESCALER) ( (uint8_t)( (F_CPU/ (2.0*CTC_PRESCALER*(1/(CTC_TIME*2)))) -1) )

/*
#elif defined(__ICCAVR__) && defined(AVR)

	#define ConfigInputPin(IOREG,PIN) do{IOREG &= (~(0x01<<PIN));}while(0)
	#define ConfigOutputPin(IOREG,PIN) do{IOREG |= (0x01<<PIN);}while(0)

	#define ConfigInputPort(IOREG,  BITS_INPUT) AND_BITWISE(IOREG, ~(BITS_INPUT))
	#define ConfigOutputPort(IOREG, BITS_INPUT) OR_BITWISE(IOREG, BITS_INPUT)

	#define PSTR(s) (s)
	#define sei() do{ __enable_interrupt();}while(0)
	#define cli() do{ __disable_interrupt();}while(0)

	#define PRAGMA(text) _Pragma( #text )
	#define ISR(VECT) PRAGMA(vector=VECT) __interrupt void VECT##_fxInt(void)

	#define FLASH_DECLARE(x) __flash x
	#define NOP() do{__no_operation();}while(0)
*/
#endif


#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define TMR8B_OVF(T_DESIRED, PREESCALER) (uint16_t) ((1UL<<8) - (T_DESIRED*F_CPU/(PREESCALER*Q_CYCLE)) )
#define TMR16B_OVF(T_DESIRED, PREESCALER) (uint16_t)((1UL<<16) - ( T_DESIRED *F_CPU / (PREESCALER*Q_CYCLE)) )

#define MHz 1000000
#define KHz 1000
#define ms 0.001
#define us 0.000001

#define CEIL_INTEGERS(DIVIDENDO,DIVISOR)  ((int)DIVISOR == 0? 0 : ((((int)DIVIDENDO) + ((int)DIVISOR) - 1) / ((int)DIVISOR)))

#endif


