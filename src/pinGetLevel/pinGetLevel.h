/*
 * pinGetLevel.h
 *
 *  Created on: Dec 3, 2020
 *      Author: jcaf
 */

#ifndef SRC_PINGETLEVEL_PINGETLEVEL_H_
#define SRC_PINGETLEVEL_PINGETLEVEL_H_

#define PINGETLEVEL_NUMMAX 1 //# of pines to check

/*
struct _pinGetLevel
{
  	int8_t counterDebounce;
    int8_t sm0;
    int8_t level;
    int8_t level_last;
    //
    PTRFX_retUINT8_T readPinLevel;
};
*/
struct _pinGetLevel
{
  	int8_t counterDebounce;
    struct _pinGetLevel_bf
    {
		unsigned sm0:1;
		unsigned level:1;
		unsigned level_last:1;
		unsigned changed:1;
		unsigned __a:4;
  	}bf;

    //
    PTRFX_retUINT8_T readPinLevel;
};

extern struct _pinGetLevel pinGetLevel[PINGETLEVEL_NUMMAX];

//
#define PINGETLEVEL_INITwCHANGED
void pinGetLevel_init();//by default always changed-flag = 1 at begin
//
void pinGetLevel_job(void);

#define pinGetLevel_hasChanged(i)pinGetLevel[i].bf.changed
#define pinGetLevel_setChange(i) do{pinGetLevel[i].bf.changed = 1;}while(0)
#define pinGetLevel_clearChange(i) do{pinGetLevel[i].bf.changed = 0;}while(0)
#define pinGetLevel_level(i) pinGetLevel[i].bf.level


//REMAPING DEFINITIOS PORTW/R
//PD5
#define PORTWxSW_ENC_RESET  	(PORTD)
#define PORTRxSW_ENC_RESET  	(PIND)
#define CONFIGIOxSW_ENC_RESET 	(DDRD)
#define PIN_SW_ENC_RESET    	(5)


//remapping Switch 0
#define PORTWxGETLEVEL_0 	PORTWxSW_ENC_RESET
#define PORTRxGETLEVEL_0 	PORTRxSW_ENC_RESET
#define CONFIGIOxGETLEVEL_0 	CONFIGIOxSW_ENC_RESET
#define PINxGETLEVEL_0		PIN_SW_ENC_RESET
//
#define PGLEVEL_LYOUT_SW_ENC_RESET 0



#endif /* SRC_PINGETLEVEL_PINGETLEVEL_H_ */
