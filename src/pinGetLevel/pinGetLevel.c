/*
 * pinGetLevel.c
 *
 *  Created on: Dec 3, 2020
 *      Author: jcaf
 */
#include "../system.h"
#include "../types.h"
#include "pinGetLevel.h"

//#include "../src/system.h"
//#include "../src/types.h"

#define PINGETLEVEL_PERIODIC_ACCESS 20//msE-3		//aplication-level

#define PINGETLEVEL_SCAN_DEBOUNCE_MIN 100//ms 	//added 2020
//#include <math.h>
//#define PINGETLEVEL_SCAN_KCOUNT_DEBOUNCE (int)( PINGETLEVEL_PERIODIC_ACCESS < PINGETLEVEL_SCAN_DEBOUNCE_MIN ? ceil((PINGETLEVEL_SCAN_DEBOUNCE_MIN*1.0f)/PINGETLEVEL_PERIODIC_ACCESS): 1)
#define PINGETLEVEL_SCAN_KCOUNT_DEBOUNCE (int)( PINGETLEVEL_PERIODIC_ACCESS < PINGETLEVEL_SCAN_DEBOUNCE_MIN ? CEIL_INTEGERS(PINGETLEVEL_SCAN_DEBOUNCE_MIN, PINGETLEVEL_PERIODIC_ACCESS): 1)


struct _pinGetLevel pinGetLevel[PINGETLEVEL_NUMMAX];


static uint8_t pinGetLevel_0(void)
{
	return ReadPin(PORTRxGETLEVEL_0, PINxGETLEVEL_0);
}
void pinGetLevel_init(void)
{
	PinTo1(PORTWxGETLEVEL_0, PINxGETLEVEL_0);//Pull-up
	ConfigInputPin(CONFIGIOxGETLEVEL_0, PINxGETLEVEL_0);
	pinGetLevel[0].readPinLevel = pinGetLevel_0;

	__delay_ms(1);
	//Set initial level
	for (int i=0; i<PINGETLEVEL_NUMMAX; i++)
	{
		pinGetLevel[i].bf.level = pinGetLevel[i].readPinLevel();
		pinGetLevel[i].bf.level_last = pinGetLevel[i].bf.level;

		#ifdef PINGETLEVEL_INITwCHANGED
		pinGetLevel[i].bf.changed = 1;//provocar un cambio inicial
		#endif
	}
}

void pinGetLevel_job(void)//non-block
{
	int8_t level_temp;

	for (int8_t i = 0; i< PINGETLEVEL_NUMMAX; i++)
	{
		if  (pinGetLevel[i].bf.sm0 == 0)
	    {
	        level_temp = pinGetLevel[i].readPinLevel();

	        if (pinGetLevel[i].bf.level_last != level_temp)
	        {
	            pinGetLevel[i].bf.level_last = level_temp;//save
	            pinGetLevel[i].bf.sm0 = 1;
	        }
	    }
	    else if (pinGetLevel[i].bf.sm0 == 1)
	    {
            if (++pinGetLevel[i].counterDebounce == PINGETLEVEL_SCAN_KCOUNT_DEBOUNCE)//ms
            {
                pinGetLevel[i].counterDebounce = 0x0;

                level_temp = pinGetLevel[i].readPinLevel();

                if (pinGetLevel[i].bf.level_last == level_temp)
                {
                	pinGetLevel[i].bf.level = level_temp;//pinGetLevel[i].bf.level_last;}
                	pinGetLevel[i].bf.changed = 1;//clear in app-level
                }
                pinGetLevel[i].bf.sm0 = 0;
            }
	    }
	}
}
