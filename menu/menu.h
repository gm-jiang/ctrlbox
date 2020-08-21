#ifndef __MENU_H__
#define __MENU_H__

#include "stm32f10x.h"	 
#include "stdlib.h"

typedef struct tag_WND
{ 
    struct tag_WND *parentwnd;
    unsigned char buf;
    void (*load)(struct tag_WND *pwnd);
    void (*unload)(struct tag_WND *pwnd);
    void (*keyevent)(struct tag_WND *pwnd, unsigned char key);
    // void (*timeevent)(struct tag_WND *pwnd,unsigned char flag);
}WND,*PWND;

void creatwnd(PWND pwnd,void (*kf)(PWND,unsigned char key),PWND parwnd,unsigned char buf_size);
void setcallback(PWND pwnd ,void(*load)(PWND),void(*unload)(PWND));
void setcurrent(PWND pwnd);
void exitcurrent(void);
void keycurrentevent(unsigned char key);
#endif
