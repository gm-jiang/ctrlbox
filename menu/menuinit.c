#include "menu.h"
#include "menuinit.h"
#include "lcd.h"
#include "gui.h"
#include "pic.h"

#include "radio.h"
#include "radio_rf315.h"
#include "radio_rf330.h"
#include "radio_rf433.h"

#include "mt_common.h"
#include "system_init.h" // for os_task_create/delete

WND g_topdeskwnd;
WND g_mainwnd;
WND g_meanwnd[3];
WND g_whitewnd[2];

unsigned char toBCD(unsigned char shijzhi)
{
    unsigned char bcd;
    bcd=(((shijzhi/10)*16)+shijzhi%10);
    return(bcd);
}

void topdeskload(PWND pwnd)
{
    LCD_Clear(BLACK);
    Show_Str(112,112,WHITE,BLACK,"安全盾",32,0);
}	

void topdeskkey(PWND pwnd,unsigned char key)
{
    if (key == 0x14) {
        setcurrent(&g_mainwnd);
    }
}

#define MENU_ITEM   3
static unsigned char t=0;
static unsigned char n;
void meauitemselet(unsigned char p)
{
    t=t+1;
    if(t>1)
    {
        switch(n)
        {
            case 0:
                LCD_Clear_Rectangle(112,136,112+24*3-1,136+24,BLUE);
                Show_Str(112,136,WHITE,BLACK,"白名单",24,0);
                break;
            case 1:
                LCD_Clear_Rectangle(112,84,112+24*4-1,84+24,BLUE);
                Show_Str(112,84,WHITE,BLACK,"频繁爆破",24,0);
                break;
            case 2:
                //CleanRectangle(32,25+16+16,16*4,16);
                LCD_Clear_Rectangle(112,32,112+24*4-1,32+24,BLUE);
                Show_Str(112,32,WHITE,BLACK,"屏蔽拦截",24,0);  
                break;
	     default:
                break;
        }
    }
    switch(p)
    {
        case 0:
            Show_Str(112,136,BLACK,WHITE,"白名单",24,0);
            n=0;
            break;
        case 1:
            Show_Str(112,84,BLACK,WHITE,"频繁爆破",24,0);
            n=1;
            break;
        case 2:
            Show_Str(112,32,BLACK,WHITE,"屏蔽拦截",24,0); 
            n=2;
            break;	
        default:
            break;
    }
}

void mainwndload(PWND pwnd)
{
    LCD_Clear(BLACK);	
    //ScanFre_flag=0;
    Show_Str(112,136,WHITE,BLACK,"白名单",24,0);  
    Show_Str(112,84,WHITE,BLACK,"频繁爆破",24,0);
    Show_Str(112,32,WHITE,BLACK,"屏蔽拦截",24,0);                 
    meauitemselet(pwnd->buf);
}

void mainwndkey(PWND pwnd,unsigned char key) 
{
    unsigned char s;
    if(key==0x15)//返回键
    {
        pwnd->buf=0;
        exitcurrent();
        return;
    }
    else if(key==0x16)//下移键
    {
        if(pwnd->buf==(MENU_ITEM-1))
        {
            pwnd->buf=0;
            meauitemselet(pwnd->buf);
        }
        else if(pwnd->buf<(MENU_ITEM-1))
        {
            ++(pwnd->buf);
            meauitemselet(pwnd->buf);
        }
    }
    else if(key==0x17)//上移键
    {
        if(pwnd->buf>0)
        {
			 s=pwnd->buf;
            --(pwnd->buf);
            meauitemselet(pwnd->buf);
        }
        else if(pwnd->buf==0)
        {
            pwnd->buf=(MENU_ITEM-1);
            meauitemselet(pwnd->buf);
        }
    }
    else if(key==0x14)//确认键
    { 
        s=pwnd->buf;
        pwnd->buf=0;
        setcurrent(&g_meanwnd[s]);
        return;
    }
}

extern uint8_t g_rf_recv_run;
#include "radio_init.h"
void kuozhanload(PWND pwnd)
{
    radio_init();
    g_rf_recv_run = 1;
    os_rf_recv_task_create();

    LCD_Clear(BLACK);	
    Show_Str(70,30,WHITE,BLACK,"频    率",16,0);
    Show_Str(70+32+32,30,WHITE,BLACK,":",16,0);

    //显示芯片类型
    Show_Str(70,50,WHITE,BLACK,"芯片类型",16,0);
    Show_Str(70+32+32,50,WHITE,BLACK,":",16,0);

    //显示ID码
    Show_Str(70,70,WHITE,BLACK,"ID",16,0);
    Show_Str(70+16,70,WHITE,BLACK,"    码",16,0);
    Show_Str(70+64,70,WHITE,BLACK,":",16,0);

    //显示按键码
    Show_Str(70,90,WHITE,BLACK,"按 键 码",16,0);
    Show_Str(70+64,90,WHITE,BLACK,":",16,0);

    //显示周期
    Show_Str(70,110,WHITE,BLACK,"周    期",16,0);
    Show_Str(70+32,110,WHITE,BLACK,"码",16,0);

    //显示电阻		
    Show_Str(70,130,WHITE,BLACK,"电    阻",16,0);

	//Show_Str(110,100,WHITE,BLACK,"拦截中",24,0);
}	

void kuozhankey(PWND pwnd,unsigned char key)
{
    if (key==0x15)//返回键
    {
        g_rf_recv_run = 0;
        setcurrent(&g_mainwnd);
    }
    else if(key==0x14)//确认键
    {
        //pwnd->buf=0;
        //setcurrent(&g_meanwnd[2]);
        return;
    }
}

extern uint8_t g_rf_send_run;
void pingbiload(PWND pwnd)
{
	//  LCD_Clear_slow(BLACK);	//开机画面逐渐消失
    LCD_Clear(BLACK);
    Show_Str(110,100,WHITE,BLACK,"屏蔽中",24,0);

    RF_Init_TX();
    CMT2300A_GoSleep();
    CMT2300A_GoStby();
    cmt_spi3_write(CMT2300A_CUS_FREQ_OFS,200);
    cmt_spi3_write(CMT2300A_CUS_FREQ_CHNL,0x20);
    //-----------------------------------------------------//
    CMT2300A_ConfigGpio(CMT2300A_GPIO3_SEL_DIN);
    CMT2300A_EnableTxDin(TRUE);
    CMT2300A_ConfigTxDin(CMT2300A_TX_DIN_SEL_GPIO3);
    CMT2300A_EnableTxDinInvert(FALSE);
    CMT2300A_GoTx();

    g_rf_send_run = 1;
    os_rf_send_task_create();
}

void pingbikey(PWND pwnd,unsigned char key)
{
    if (key==0x15)//返回键
    {
        g_rf_send_run = 0;	
        setcurrent(&g_mainwnd);
    }
    else if(key==0x14)//确认键
    {
        pwnd->buf=0;
        setcurrent(&g_meanwnd[1]);
        return;
    }
}

#define whitewnd_ITEM   2
void whitewnd_itemselet(unsigned char p)
{
  
    t=t+1;
    if(t>1)
    {
        switch(n)
        {
            case 0:
                //CleanRectangle(32,20+16,16*4,16);
                LCD_Clear_Rectangle(112,84,112+24*4-1,84+24,BLUE);
                Show_Str(112,84,WHITE,BLACK,"遥控学习",24,0);
                break;
            case 1:
                //CleanRectangle(32,25+16+16,16*4,16);
                LCD_Clear_Rectangle(112,32,112+24*4-1,32+24,BLUE);
                Show_Str(112,32,WHITE,BLACK,"报警监测",24,0);
                break;
            default:
                break;
        }
    }

    switch(p)
    {
        case 0:
            Show_Str(112,84,BLACK,WHITE,"遥控学习",24,0);
            n=0;
            break;
        case 1:
            Show_Str(112,32,BLACK,WHITE,"报警监测",24,0);
            n=1;
            break;
        default:
            break;
    }
}

void whitewindload(PWND pwnd)
{
    LCD_Clear(BLACK);	
    //ScanFre_flag=0;
    // Show_Str(112,136,WHITE,BLACK,"白名单",24,0);
    Show_Str(112,84,WHITE,BLACK,"遥控学习",24,0);
    Show_Str(112,32,WHITE,BLACK,"报警监测",24,0);               
    whitewnd_itemselet(pwnd->buf);
}

void whitewindkey(PWND pwnd,unsigned char key) 
{
    unsigned char s;
    if(key==0x15)//返回键
    {
        pwnd->buf=0;
        exitcurrent();
        return;
    }
    else if(key==0x16)//下移键
    {
        if(pwnd->buf==(whitewnd_ITEM-1))
        {
            pwnd->buf=0;
            whitewnd_itemselet(pwnd->buf);
        }
        else if(pwnd->buf<(whitewnd_ITEM-1))
        {
            ++(pwnd->buf);
            whitewnd_itemselet(pwnd->buf);
        }  
    }
    else if(key==0x17)//上移键
    {
        if(pwnd->buf>0)
        {
            s=pwnd->buf;
            --(pwnd->buf);
            whitewnd_itemselet(pwnd->buf);
        }
        else if(pwnd->buf==0)
        {
            pwnd->buf=(whitewnd_ITEM-1);
            whitewnd_itemselet(pwnd->buf);
        }
    }
    else if(key==0x14)//确认键
    { 
        s=pwnd->buf;
        pwnd->buf=0;
        setcurrent(&g_whitewnd[s]);
        return;
    }
}
//遥控学习

#define remote_code_ITEM   1
void remote_code_itemselet(unsigned char p)
{
    t=t+1;
    if(t>1)
    {
        switch(n)
        {
            case 0:
                //CleanRectangle(32,20+16,16*4,16);
                LCD_Clear_Rectangle(112,200,112+24*2-1,200+24,BLACK);
                Show_Str(112,200,WHITE,BLACK,"保存",24,0);
                break;
            default:
                break;
        }
    }

    switch(p)
    {
        case 0:
            Show_Str(112,200,BLACK,WHITE,"保存",24,0);
            n=0;
            break;
        default:
            break;
    }
}

void remote_codeload(PWND pwnd)
{

    LCD_Clear(BLACK);	
    //ScanFre_flag=0;
    //Show_Str(112,136,WHITE,BLACK,"白名单",24,0);
    //Show_Str(112,84,WHITE,BLACK,"遥控学习",24,0);
    //Show_Str(112,32,WHITE,BLACK,"报警监测",24,0);
    Show_Str(64,84,WHITE,BLACK,"请长按遥控器",24,0);

    delay_ms(3000);

    LCD_Clear(BLACK);
    Show_Str(70,30,WHITE,BLACK,"频    率",16,0);
    Show_Str(70+32+32,30,WHITE,BLACK,":",16,0);

    //显示芯片类型
    Show_Str(70,50,WHITE,BLACK,"芯片类型",16,0);
    Show_Str(70+32+32,50,WHITE,BLACK,":",16,0);

    //显示ID码
    Show_Str(70,70,WHITE,BLACK,"ID",16,0);
    Show_Str(70+16,70,WHITE,BLACK,"    码",16,0);
    Show_Str(70+64,70,WHITE,BLACK,":",16,0);
				
    //显示按键码
    Show_Str(70,90,WHITE,BLACK,"按 键 码",16,0);
    Show_Str(70+64,90,WHITE,BLACK,":",16,0);
				
    //显示周期
    Show_Str(70,110,WHITE,BLACK,"周    期",16,0);
    Show_Str(70+32,110,WHITE,BLACK,"码",16,0);

    //显示电阻
    Show_Str(70,130,WHITE,BLACK,"电    阻",16,0);

    //Show_Str(110,100,WHITE,BLACK,"拦截中",24,0);
    //x_flag=0x01;
    //remote_flag=1;
    remote_code_itemselet(pwnd->buf);
}

void remote_codekey(PWND pwnd,unsigned char key) 
{
    unsigned char s;
    if(key==0x15)//返回键
    {
        pwnd->buf=0;
        //remote_flag=0;
        //exitcurrent();
        setcurrent(&g_meanwnd[0]);
        return;
    }
    else if(key==0x16)//下移键
    {
        if(pwnd->buf==(remote_code_ITEM-1))
        {
            pwnd->buf=0;
            remote_code_itemselet(pwnd->buf);
        }
        else if(pwnd->buf < (remote_code_ITEM-1))
        {
            ++(pwnd->buf);
            remote_code_itemselet(pwnd->buf);
        } 
    }
    else if(key==0x17)//上移键
    {
        if(pwnd->buf>0)
        {
            s=pwnd->buf;
            --(pwnd->buf);
            remote_code_itemselet(pwnd->buf);
        }
        else if(pwnd->buf==0)
        {
            pwnd->buf=(remote_code_ITEM-1);
            remote_code_itemselet(pwnd->buf);
        }
    }
    else if(key==0x14)//确认键
    {
        s=pwnd->buf;
        pwnd->buf=0;
        //remote_flag=1;  //遥控学习标志位
   
        setcurrent(&g_whitewnd[s]);
        return;
    }
}

void alarmload(PWND pwnd)
{
    LCD_Clear(BLACK);
    //ScanFre_flag=0;

    Show_Str(112,32,WHITE,BLACK,"报警监测中",24,0);

    LCD_Clear(BLACK);
    Show_Str(70,30,WHITE,BLACK,"频    率",16,0);
    Show_Str(70+32+32,30,WHITE,BLACK,":",16,0);

	//显示芯片类型
    Show_Str(70,50,WHITE,BLACK,"芯片类型",16,0);
    Show_Str(70+32+32,50,WHITE,BLACK,":",16,0);

    //显示ID码
    Show_Str(70,70,WHITE,BLACK,"ID",16,0);
    Show_Str(70+16,70,WHITE,BLACK,"    码",16,0);
    Show_Str(70+64,70,WHITE,BLACK,":",16,0);
				
    //显示按键码
    Show_Str(70,90,WHITE,BLACK,"按 键 码",16,0);
    Show_Str(70+64,90,WHITE,BLACK,":",16,0);

    //显示周期
    Show_Str(70,110,WHITE,BLACK,"周    期",16,0);
    Show_Str(70+32,110,WHITE,BLACK,"码",16,0);

    //显示电阻
    Show_Str(70,130,WHITE,BLACK,"电    阻",16,0);	

	//Show_Str(110,100,WHITE,BLACK,"拦截中",24,0);
    //x_flag=0x01;	


    //alarm_flag=1; //遥控学习标志位
    //x_flag=0x01;	
    //whitewnd_itemselet(pwnd->buf);
}

void alarmkey(PWND pwnd,unsigned char key) 
{
    unsigned char s;
    if(key==0x15)//返回键
    {
        pwnd->buf=0;
        //alarm_flag=0;
        exitcurrent();
        return;
    }
    else if(key==0x16)//下移键
    {
    }
    else if(key==0x17)//上移键
    {
    }
    else if(key==0x14)//确认键
    {
        s=pwnd->buf;
        pwnd->buf=0;
        setcurrent(&g_whitewnd[s]);
        return;
    }
}


void UIINIT(void)                                   
{
    creatwnd(&g_topdeskwnd,topdeskkey,0,0);         //开机界面
    setcallback(&g_topdeskwnd,topdeskload,0);

    creatwnd(&g_mainwnd,mainwndkey,&g_topdeskwnd,0); //主菜单界面
    setcallback(&g_mainwnd,mainwndload,0);

    creatwnd(&g_meanwnd[2],kuozhankey,&g_mainwnd,0); //破解预警
    setcallback(&g_meanwnd[2],kuozhanload,0);

    creatwnd(&g_meanwnd[1],pingbikey,&g_mainwnd,0); //屏蔽干扰
    setcallback(&g_meanwnd[1],pingbiload,0);

    creatwnd(&g_meanwnd[0],whitewindkey,&g_mainwnd,0); //白名单
    setcallback(&g_meanwnd[0],whitewindload,0);

    creatwnd(&g_whitewnd[0],remote_codekey,&g_meanwnd[0],0); //遥控学习 2020年5月8日
    setcallback(&g_whitewnd[0],remote_codeload,0);

    creatwnd(&g_whitewnd[1],alarmkey,&g_meanwnd[0],0);       //报警监测 2020年5月8日
    setcallback(&g_whitewnd[1],alarmload,0);

    setcurrent(&g_topdeskwnd);
}
