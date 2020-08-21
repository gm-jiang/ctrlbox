#include "stm32f10x.h"
#include <stdlib.h>
#include <string.h>
#include "bsp_port.h" //use watchdog and led
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "user_task.h"
#include "radio_recv.h"
#include "mt_common.h"
#include "system_init.h"
#include "menu.h"
#include "menuinit.h"

#include "lcd.h"
#include "gui.h"

extern unsigned char rf315_en;
extern unsigned char rf330_en;
extern unsigned char rf433_en;
extern unsigned char rf4xx_en;

extern unsigned char rf315_buf[2][4];
extern unsigned char rf330_buf[2][4];
extern unsigned char rf433_buf[2][4];
extern unsigned char rf4xx_buf[2][4];

void RF_Display(uint8_t address1,uint8_t address2,uint8_t address3,uint8_t address4,uint8_t chip_ID,uint8_t key_value,uint8_t zhouqi);


#if 1
void task_key_detect(void *pvParameters)
{
    BaseType_t ret;
    uint8_t key_value = 0;
    eventMsgType_e eventMsg;
    UIINIT();

	while(1)
	{
		ret = xQueueReceive(KeyEventMsgQueue, &eventMsg, portMAX_DELAY);
		if (ret == pdTRUE) {
            switch (eventMsg)
            {
                case EVENT_MSG_KEY1:
                    //LCD_Clear(BLACK);
                    key_value = 1;
                    break;
                case EVENT_MSG_KEY2:
                    key_value = 2;
                    //LCD_Clear(BLUE);
                    break;
                case EVENT_MSG_KEY3:
                    key_value = 3;
                    //LCD_Clear(RED);
                    break;
                case EVENT_MSG_KEY4:
                    key_value = 4;
                    //LCD_Clear(YELLOW);
                    break;
                default:
                    break;
            }
            keycurrentevent(key_value);
		}
	}
}
#endif

void task_rf315(void *pvParameters)
{
    while(1)
    {
        RF315_IN();
        if (rf315_en == 4) {
            rf315_en = 0;
            LCD_Clear_Rectangle(150,32,150+80,32+16,BLACK);
            Show_Str(70+48+32,30,WHITE,BLACK,"433.092Mhz",16,0);
            RF_Display(rf315_buf[1][0],rf315_buf[1][1],rf315_buf[1][2],rf315_buf[1][3], 0, 0, 0);
        }
    }
}

void task_rf330(void *pvParameters)
{
    while(1)
    {
        RF330_IN();
        if (rf330_en == 4) {
            rf330_en = 0;
            bsp_power_status_led_set(0);
            delay_us(100);
            bsp_power_status_led_set(1);
        }
    }
}

void task_rf433(void *pvParameters)
{
    while(1)
    {
        RF433_IN();
        if (rf433_en == 4) {
            rf433_en = 0;
            bsp_power_status_led_set(0);
            delay_us(100);
            bsp_power_status_led_set(1);
        }
    }
}

void task_rf4xx(void *pvParameters)
{
    while(1)
    {
        RF4XX_IN();
        if (rf4xx_en == 4) {
            rf4xx_en = 0;
            bsp_power_status_led_set(0);
            delay_us(100);
            bsp_power_status_led_set(1);
        }
    }
}

void RF_Display(uint8_t address1,uint8_t address2,uint8_t address3,uint8_t address4,uint8_t chip_ID,uint8_t key_value,uint8_t zhouqi) //???1 ???2 ???? ????1 ????2   j??3??????????????? ????1??2????????uint
{
uint8_t i,flag;
uint8_t f=0;
uint8_t wei1,wei2,wei3,wei4;
uint32_t cycle[2],r;
//uchar key_ID;
flag=1;
	
	wei1=address1;
	wei2=address2;
	wei3=address3;
	wei4=address4;
//---------------------------------------------------------------------------------------------------------------------//

																	//":"
	                              LCD_Clear_Rectangle(70+48+32,50,70+48+32+80,50+16,BLACK);
                                switch(chip_ID)
                                {
																  //case 0:  break;
																	case 0:Show_Str(70+48+32,50,WHITE,BLACK,"xx2262",16,0);	break;
														      case 1:Show_Str(70+48+32,50,WHITE,BLACK,"xx1527",16,0);	break;
															    case 2:Show_Str(70+48+32,50,WHITE,BLACK,"T19-L",16,0);	break;	
																  case 3:Show_Str(70+48+32,50,WHITE,BLACK,"T19-S",16,0);	break;
																	default:break;
																}
//---------------------------------------------------------------------------------------------------------------------//
                             //???ID??
                            	
                              // Show_Str(70,70,WHITE,BLACK,"ID",16,0);	
															// Show_Str(70+16,70,WHITE,BLACK,"    ??",16,0);
															// Show_Str(70+64,70,WHITE,BLACK,":",16,0);
																
																			//////////////////////////////////////////////////////////////
                                      
																			//////////////////////////////////////////////////////////////
																        LCD_Clear_Rectangle(70+64+8,70,70+64+8+80,70+16,BLACK);
                                      //-----------------------data------------------------------------------------------------------------------//
																				for(i=0;i<4;i++)//??8???? ????????3????????4??
																					 {	  				  	
																						 if(i==0)
																						 {
																						  
																												 if((wei1&0xC0)==0x40)   LCD_ShowChar(70+64+8, 70, WHITE, BLACK, 'F', 16, 0); //01????F??15?????F???
																											 else if ((wei1&0xC0)==0xC0)  LCD_ShowChar(70+64+8, 70, WHITE, BLACK, '1', 16, 0); //11 ???0x31????????1
																											 else if ((wei1&0xC0)==0x80)  LCD_ShowChar(70+64+8, 70, WHITE, BLACK, '2', 16, 0); //10 ???0x31????????H
																											 else	                        LCD_ShowChar(70+64+8, 70, WHITE, BLACK, '0', 16, 0); //00 ???0x31????????0
																						 
																						 
																						 
																						 }else  if(i==1)

                                             {
																						         if((wei1&0xC0)==0x40)   LCD_ShowChar(70+64+16, 70, WHITE, BLACK, 'F', 16, 0); //01????F??15?????F???
																											 else if ((wei1&0xC0)==0xC0)  LCD_ShowChar(70+64+16, 70, WHITE, BLACK, '1', 16, 0); //11 ???0x31????????1
																											  else if ((wei1&0xC0)==0x80)  LCD_ShowChar(70+64+16, 70, WHITE, BLACK, '2', 16, 0); //10 ???0x31????????H
																											 else	                        LCD_ShowChar(70+64+16, 70, WHITE, BLACK, '0', 16, 0); //00 ???0x31????????0
																						 
																						 
																						 
																						 }else  if(i==2)	
                                             {
																						         if((wei1&0xC0)==0x40)   LCD_ShowChar(70+64+24, 70, WHITE, BLACK, 'F', 16, 0); //01????F??15?????F???
																											 else if ((wei1&0xC0)==0xC0)  LCD_ShowChar(70+64+24, 70, WHITE, BLACK, '1', 16, 0); //11 ???0x31????????1
																											 else if ((wei1&0xC0)==0x80)  LCD_ShowChar(70+64+24, 70, WHITE, BLACK, '2', 16, 0); //10 ???0x31????????H
																											 else	                        LCD_ShowChar(70+64+24, 70, WHITE, BLACK, '0', 16, 0); //00 ???0x31????????0
																						 
																						 
																						 }else
																							{
																											if((wei1&0xC0)==0x40)   LCD_ShowChar(70+64+32, 70, WHITE, BLACK, 'F', 16, 0); //01????F??15?????F???
																											 else if ((wei1&0xC0)==0xC0)  LCD_ShowChar(70+64+32, 70, WHITE, BLACK, '1', 16, 0); //11 ???0x31????????1
																											 else if ((wei1&0xC0)==0x80)  LCD_ShowChar(70+64+32, 70, WHITE, BLACK, '2', 16, 0); //10 ???0x31????????H
																											 else	                        LCD_ShowChar(70+64+32, 70, WHITE, BLACK, '0', 16, 0); //00 ???0x31????????0


																							}																						 
																						
																					 wei1=wei1<<2;
																					 }

                                        for(i=0;i<4;i++)//??8???? ????????3????????4??
																					 {	  				  	
																														if(i==0)
																												 {
																													
																																		 if((wei2&0xC0)==0x40)   LCD_ShowChar(70+64+8+32, 70, WHITE, BLACK, 'F', 16, 0); //01????F??15?????F???
																																	 else if ((wei2&0xC0)==0xC0)  LCD_ShowChar(70+64+8+32, 70, WHITE, BLACK, '1', 16, 0); //11 ???0x31????????1
																																	 else if ((wei2&0xC0)==0x80)  LCD_ShowChar(70+64+8+32, 70, WHITE, BLACK, 'H', 16, 0); //10 ???0x31????????H
																																	 else	                        LCD_ShowChar(70+64+8+32, 70, WHITE, BLACK, '0', 16, 0); //00 ???0x31????????0
																												 
																												 
																												 
																												 }else  if(i==1)

																												 {
																																 if((wei2&0xC0)==0x40)   LCD_ShowChar(70+64+16+32, 70, WHITE, BLACK, 'F', 16, 0); //01????F??15?????F???
																																	 else if ((wei2&0xC0)==0xC0)  LCD_ShowChar(70+64+16+32, 70, WHITE, BLACK, '1', 16, 0); //11 ???0x31????????1
																																	 else if ((wei2&0xC0)==0x80)  LCD_ShowChar(70+64+16+32, 70, WHITE, BLACK, 'H', 16, 0); //10 ???0x31????????H
																																	 else	                        LCD_ShowChar(70+64+16+32, 70, WHITE, BLACK, '0', 16, 0); //00 ???0x31????????0
																												 
																												 
																												 
																												 }else  if(i==2)	
																												 {
																																 if((wei2&0xC0)==0x40)   LCD_ShowChar(70+64+24+32, 70, WHITE, BLACK, 'F', 16, 0); //01????F??15?????F???
																																	 else if ((wei2&0xC0)==0xC0)  LCD_ShowChar(70+64+24+32, 70, WHITE, BLACK, '1', 16, 0); //11 ???0x31????????1
																																	 else if ((wei2&0xC0)==0x80)  LCD_ShowChar(70+64+24+32, 70, WHITE, BLACK, 'H', 16, 0); //10 ???0x31????????H
																																	 else	                        LCD_ShowChar(70+64+24+32, 70, WHITE, BLACK, '0', 16, 0); //00 ???0x31????????0
																												 
																												 
																												 }else
																													{
																																	if((wei2&0xC0)==0x40)   LCD_ShowChar(70+64+32+32, 70, WHITE, BLACK, 'F', 16, 0); //01????F??15?????F???
																																	 else if ((wei2&0xC0)==0xC0)  LCD_ShowChar(70+64+32+32, 70, WHITE, BLACK, '1', 16, 0); //11 ???0x31????????1
																																	 else if ((wei2&0xC0)==0x80)  LCD_ShowChar(70+64+32+32, 70, WHITE, BLACK, 'H', 16, 0); //10 ???0x31????????H
																																	 else	                        LCD_ShowChar(70+64+32+32, 70, WHITE, BLACK, '0', 16, 0); //00 ???0x31????????0


																													}																						 
																												
																											 wei2=wei2<<2;
																					 }
																			 //-----------------------display------------------------------------------------------------------------------//
																			///////////////////////////////////////////////////////////////////////////////////////////////
																
 //-------------------------------------------------------------------------------------------------------------------//
																//????????? 																
                              
                              // Show_Str(70,90,WHITE,BLACK,"?? ?? ??",16,0);	
															LCD_Clear_Rectangle(70+64+8,90,70+64+8+80,90+16,BLACK);
                               switch(key_value)
															 {
															   case 1: Show_Str(70+64+8,90, WHITE, BLACK, "0001", 16, 0);break; //00 ???0x31????????0break;
																 case 2: Show_Str(70+64+8,90, WHITE, BLACK, "0010", 16, 0);break; //00 ???0x31????????0break;
															   case 4: Show_Str(70+64+8,90, WHITE, BLACK, "0100", 16, 0);break; //00 ???0x31????????0break;
															   case 8: Show_Str(70+64+8,90, WHITE, BLACK, "1000", 16, 0);break; //00 ???0x31????????0break;
																 
																	default:break;
															 }
	 //-------------------------------------------------------------------------------------------------------------------//															
																LCD_Clear_Rectangle(70+64+8,110,70+64+8+80,110+16,BLACK);
																//???????
                            	
                              // Show_Str(70,110,WHITE,BLACK,"??    ??",16,0);	
															 //Show_Str(70+32,110,WHITE,BLACK,"??",16,0);
															//Show_Str(70+64,110,WHITE,BLACK,":",16,0);
															 //Show_Str(70+64+8,110,WHITE,BLACK,"0X",16,0);
															 
															 //cycle=zhouqi;
															 cycle[0] = zhouqi / 16;
														   cycle[1] = zhouqi % 16;
															 ShowNum(70+64+8+16, 110, WHITE, BLACK, cycle[0], 16, 0);
															 ShowNum(70+64+8+16+8, 110, WHITE, BLACK, cycle[1], 16, 0);
 //-------------------------------------------------------------------------------------------------------------------//																
																
                                //???????		

                              // Show_Str(70,130,WHITE,BLACK,"??    ??",16,0);	
															 Show_Str(70+64,130,WHITE,BLACK,":",16,0);
															 
															 	LCD_Clear_Rectangle(70+64+8,130,70+64+8+80,130+16,BLACK);
															 cycle[0] = zhouqi / 16;
														   cycle[1] = zhouqi % 16;
															 ShowNum(70+64+8+16, 130, WHITE, BLACK, cycle[0], 16, 0);
															 ShowNum(70+64+8+16+8, 130, WHITE, BLACK, cycle[1], 16, 0);
															 
															 
															 
															 
 //-------------------------------------------------------------------------------------------------------------------//	





}
