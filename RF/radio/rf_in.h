#ifndef __RF_IN_H
#define __RF_IN_H

#include "typedefs.h"

void buzzer_init(void);

void buzzer_on(void);
void buzzer_off(void);

void buzzer_onAutoOff(u32 ms);
void RF_IN(void);
void RF315_IN(void); 
void RF330_IN(void); 
void RF433_IN(void); 

void show_jm(void);



void out0();
void out1();
void send_bat(unsigned char dat);
void send_rf(unsigned char dat1,unsigned char dat2,unsigned char dat3,unsigned char dat4,unsigned char wid);//1527±‡¬Î∑¢…‰ 


void RF_Display(unsigned char address1,unsigned char address2,unsigned char address3,unsigned char address4,unsigned char chip_ID,unsigned char key_value,unsigned char zhouqi);
#endif
