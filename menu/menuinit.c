#include "menu.h"
#include "menuinit.h"
#include "lcd.h"
#include "gui.h"
#include "pic.h"

WND g_topdeskwnd;
WND g_mainwnd;
WND g_meanwnd[3];
WND g_whitewnd[2];

void topdeskload(PWND pwnd)
{
    Gui_Drawbmp16(0, 0, 320, 240, gImage_power_on);
}

void topdeskkey(PWND pwnd,unsigned char key)
{
    if (key == 1) {
        setcurrent(&g_mainwnd);
    }
}

void mainwndload(PWND pwnd)
{

    LCD_Clear(BLACK);	
    Show_Str(112, 136, WHITE,BLACK, "123", 24, 0);  
    Show_Str(112, 84, WHITE,BLACK, "456", 24, 0);
    Show_Str(112, 32, WHITE,BLACK, "789", 24, 0);                 
    //meauitemselet(pwnd->buf);
}

void mainwndkey(PWND pwnd, unsigned char key) 
{
    if (key == 2) {
        pwnd->buf = 0;
        exitcurrent();
        return;
    }
}

void UIINIT(void)                                   
{
    creatwnd(&g_topdeskwnd, topdeskkey, 0, 0);
    setcallback(&g_topdeskwnd, topdeskload, 0);

    creatwnd(&g_mainwnd, mainwndkey, &g_topdeskwnd, 0);
    setcallback(&g_mainwnd, mainwndload, 0);
/*	
	creatwnd(&g_meanwnd[2],kuozhankey,&g_mainwnd,0); //??????
  setcallback(&g_meanwnd[2],kuozhanload,0);
	
	creatwnd(&g_meanwnd[1],pingbikey,&g_mainwnd,0); //???????
  setcallback(&g_meanwnd[1],pingbiload,0);
	
	creatwnd(&g_meanwnd[0],whitewindkey,&g_mainwnd,0); //??????
  setcallback(&g_meanwnd[0],whitewindload,0);
	
  creatwnd(&g_whitewnd[0],remote_codekey,&g_meanwnd[0],0); //????? 2020??5??8??
  setcallback(&g_whitewnd[0],remote_codeload,0);
	
	creatwnd(&g_whitewnd[1],alarmkey,&g_meanwnd[0],0);       //??????? 2020??5??8??
  setcallback(&g_whitewnd[1],alarmload,0);
*/
    setcurrent(&g_topdeskwnd);
}
