#include "mt_common.h"

#include "bsp_port.h"
#include "cmt2300a_hal.h"
#include "cmt2300a_hal_rf315.h"
#include "cmt2300a_hal_rf330.h"
#include "cmt2300a_hal_rf433.h"

static unsigned char Rec430_short_k = 60;

#define	Hi(p,i)         {p->BSRR=i;}
#define Lo(p,i)         {p->BRR=i;}
#define Toggle(p,i)     {p->ODR ^=i;}

#define RF1_GPIO_PORT       GPIOC
#define RF1_GPIO_CLK        RCC_APB2Periph_GPIOC
#define RF1_GPIO_PIN        GPIO_Pin_3

#define RF1_TOGGLE      Toggle(RF1_GPIO_PORT,RF1_GPIO_PIN)
#define RF1_ON          Hi(RF1_GPIO_PORT,RF1_GPIO_PIN)
#define RF1_OFF         Lo(RF1_GPIO_PORT,RF1_GPIO_PIN)

unsigned char short_width = 60;

void out0(void)
{
    RF1_ON;
	delay_us(short_width*5);
	RF1_OFF;
	delay_us(short_width*15);
}
void out1(void)
{
    RF1_ON;
    delay_us(short_width*15);
    RF1_OFF;
    delay_us(short_width*5);
} 

void send_bat(unsigned char dat)
{
	unsigned char a;

    for (a=0;a<8;a++)			//????h?????????
	{
    	if((dat>>(7-a))&1==1)	
			out1();	       
		else 
			out0();		
	}
}

void send_rf(unsigned char dat1,unsigned char dat2,unsigned char dat3,unsigned char dat4,unsigned char wid)//1527?????? 
{ 
	unsigned char dd,b; 

	 __set_PRIMASK(1);  // ?????EMI??FAULT???
	short_width = wid; 
    // ---------------------2262??1527????????????-------------------------------------
    for(b=0;b<4;b++){if(((dat1>>(b*2)) & 3)==2) {b=80;break;}}  			//??10???1527
    if(b!=80)for(b=0;b<4;b++){if(((dat2>>(b*2)) & 3)==2) {b=80;break;}}  	//??10???1527
    if(b!=80)for(b=0;b<2;b++){if((((dat3)>>(b*2)) & 3)==2) {b=80;break;}} 	//??10???1527
    if(b==80) dat3=dat3*16+dat4;  	//1527
    else  							//2262
    {
		dd = 0;
        for(b=0;b<4;b++) if((dat4>>b)&1==1)  dd|=3<<(b*2);  				//??????????2262????
        dat3 = dd;
    }       
    //------------------------------------------------------------------------------------     
	for(dd=0;dd<8;dd++)         	//?????????8????
	{	     
        send_bat(dat1);
		send_bat(dat2);
		send_bat(dat3);
		RF1_ON;
		delay_us(short_width*5); 
		RF1_OFF;
		delay_us(short_width*32*5);     //????????????????z?	             
	}	
	 __set_PRIMASK(0);  // ????EMI??FAULT???
}