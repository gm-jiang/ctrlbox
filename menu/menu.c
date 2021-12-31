#include "menu.h"

static PWND g_currentwnd = 0;

void creatwnd(PWND pwnd, void (*kf)(PWND, unsigned char key), PWND parwnd, unsigned char buf_size)
{
    pwnd->keyevent = kf;
    pwnd->buf = buf_size;
    pwnd->parentwnd = (struct tag_WND *)parwnd;
    pwnd->load = 0;
    pwnd->unload = 0;
}

void setcallback(PWND pwnd, void (*load)(PWND), void (*unload)(PWND))
{
    pwnd->load = load;
    pwnd->unload = unload;
}

void setcurrent(PWND pwnd)
{
    if (g_currentwnd) {
        if (g_currentwnd->unload != 0) {
            g_currentwnd->unload(g_currentwnd);
        }
    }
    g_currentwnd = pwnd;
    if (pwnd->load != 0) {
        pwnd->load(g_currentwnd);
    }
}

void exitcurrent(void)
{
    if (g_currentwnd->parentwnd == 0) {
        return;
    }

    setcurrent(g_currentwnd->parentwnd);
}

void keycurrentevent(unsigned char key)
{
    if (g_currentwnd->keyevent != 0) {
        g_currentwnd->keyevent(g_currentwnd, key);
    }
}
