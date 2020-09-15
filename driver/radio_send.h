#ifndef __RADIO_SEND_H
#define __RADIO_SEND_H

void out0();
void out1();
void send_bat(unsigned char dat);
void send_rf(unsigned char dat1,unsigned char dat2,unsigned char dat3,unsigned char dat4,unsigned char wid);
#endif
