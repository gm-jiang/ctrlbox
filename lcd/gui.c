#include "lcd.h"
#include "string.h"
#include "font.h"
#include "mt_common.h"
#include "gui.h"

#define ModuleNum 11

uint8_t max_value = 0;
uint8_t m_max_value = 0;
uint8_t last_max_value = 0;

extern const unsigned short int BAR_COLOR[160];
const unsigned int Xpos[15] = { 237, 191, 173, 155, 138, 119, 99, 78, 62, 38, 21, 0, 0, 0, 0 };
extern const unsigned char VALUE[256];
extern uint8_t m_max_value;
//////////////////////////////////////////////////////////////////////////////////
//???????????'?�?d????????????????????????????;
//??????????????STM32F103RBT6,???72M  ????????????3.3V
//QDtech-TFT??????? for STM32 IOg??
//xiao??@ShenZhen QDtech co.,LTD
//??????:www.qdtech.net
//????????http://qdtech.taobao.com
//???????????????�?????????????????
//???(????) :+86 0755-23594567
//???:15989313508??????
//????:QDtech2008@gmail.com
//Skype:QDtech2008
//????????QQ?:324828016
//????????:2013/5/13
//????V1.1
//????????????????
//Copyright(C) ?????????????????????? 2009-2019
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////
//******************************************************************
//????????  GUI_DrawPoint
//?????    xiao??@???????
//?????    2013-02-22
//?????    GUI???h????
//?????????x:??????x????
//        	y:??????y????
//			color:????????
//???????  ??
//??l?�????
//******************************************************************

const unsigned int blocktype[2] = { 60, 123 }; //??????Y??????'????
void GUI_DrawPoint(u16 x, u16 y, u16 color)
{
    LCD_SetCursor(x, y); //???�?????
    LCD_DrawPoint_16Bit(color);
}

//******************************************************************
//????????  LCD_Fill
//?????    xiao??@???????
//?????    2013-02-22
//?????    ?????????????????
//?????????sx:???????'??x????
//        	sy:???????'??y????
//			ex:????????????x????
//			ey:????????????y????
//        	color:????????
//???????  ??
//??l?�????
//******************************************************************
void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 color)
{
    u16 i, j;
    u16 width = ex - sx + 1;                //?�????????
    u16 height = ey - sy + 1;               //???
    LCD_SetWindows(sx, sy, ex - 1, ey - 1); //???????????

#if LCD_USE8BIT_MODEL == 1
    LCD_RS_SET; //?????
    LCD_CS_CLR;
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            DATAOUT(color >> 8);
            LCD_WR_CLR;
            LCD_WR_SET;

            DATAOUT(color);
            LCD_WR_CLR;
            LCD_WR_SET;
        }
    }
    LCD_CS_SET;
#else //16?g?
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++)
            LCD_WR_DATA(color); //???????
    }
#endif
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); //???????????????
}

//******************************************************************
//????????  LCD_DrawLine
//?????    xiao??@???????
//?????    2013-02-22
//?????    GUI????
//?????????x1,y1:???????
//        	x2,y2:???????
//???????  ??
//??l?�????
//******************************************************************
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
    u16 t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;

    delta_x = x2 - x1; //????????????
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0)
        incx = 1; //???�???????
    else if (delta_x == 0)
        incx = 0; //?????
    else {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; //????
    else {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)
        distance = delta_x; //????????????????
    else
        distance = delta_y;
    for (t = 0; t <= distance + 1; t++) //???????
    {
        LCD_DrawPoint(uRow, uCol); //????
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
}

//******************************************************************
//????????  LCD_DrawLine
//?????    xiao??@???????
//?????    2013-02-22
//?????    GUI????
//?????????x1,y1:???????
//        	x2,y2:???????
//???????  ??
//??l?�????????????????
//******************************************************************
void Gui_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color)
{
    u16 t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;

    delta_x = x2 - x1; //????????????
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0)
        incx = 1; //???�???????
    else if (delta_x == 0)
        incx = 0; //?????
    else {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; //????
    else {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)
        distance = delta_x; //????????????????
    else
        distance = delta_y;
    for (t = 0; t <= distance + 1; t++) //???????
    {
        GUI_DrawPoint(uRow, uCol, color); //????
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
}

//******************************************************************
//????????  LCD_DrawRectangle
//?????    xiao??@???????
//?????    2013-02-22
//?????    GUI??????(?????)
//?????????(x1,y1),(x2,y2):???eK??????
//???????  ??
//??l?�????
//******************************************************************
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
    LCD_DrawLine(x1, y1, x2, y1);
    LCD_DrawLine(x1, y1, x1, y2);
    LCD_DrawLine(x1, y2, x2, y2);
    LCD_DrawLine(x2, y1, x2, y2);
}

//******************************************************************
//????????  LCD_DrawFillRectangle
//?????    xiao??@???????
//?????    2013-02-22
//?????    GUI??????(???)
//?????????(x1,y1),(x2,y2):???eK??????
//???????  ??
//??l?�????
//******************************************************************
void LCD_DrawFillRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
    LCD_Fill(x1, y1, x2, y2, POINT_COLOR);
}

//******************************************************************
//????????  _draw_circle_8
//?????    xiao??@???????
//?????    2013-02-22
//?????    8?????????(???????)
//?????????(xc,yc) :?????????
// 			(x,y):??????????j?????
//         	c:???????
//???????  ??
//??l?�????
//******************************************************************
void _draw_circle_8(int xc, int yc, int x, int y, u16 c)
{
    GUI_DrawPoint(xc + x, yc + y, c);

    GUI_DrawPoint(xc - x, yc + y, c);

    GUI_DrawPoint(xc + x, yc - y, c);

    GUI_DrawPoint(xc - x, yc - y, c);

    GUI_DrawPoint(xc + y, yc + x, c);

    GUI_DrawPoint(xc - y, yc + x, c);

    GUI_DrawPoint(xc + y, yc - x, c);

    GUI_DrawPoint(xc - y, yc - x, c);
}

//******************************************************************
//????????  gui_circle
//?????    xiao??@???????
//?????    2013-02-22
//?????    ???????�?h???????????(???)
//?????????(xc,yc) :?????????
//         	c:???????
//		 	r:???
//		 	fill:??????????1-???0-?????
//???????  ??
//??l?�????
//******************************************************************
void gui_circle(int xc, int yc, u16 c, int r, int fill)
{
    int x = 0, y = r, yi, d;

    d = 3 - 2 * r;

    if (fill) {
        // ??????????????
        while (x <= y) {
            for (yi = x; yi <= y; yi++)
                _draw_circle_8(xc, yc, x, yi, c);

            if (d < 0) {
                d = d + 4 * x + 6;
            } else {
                d = d + 4 * (x - y) + 10;
                y--;
            }
            x++;
        }
    } else {
        // ?????????????????
        while (x <= y) {
            _draw_circle_8(xc, yc, x, y, c);
            if (d < 0) {
                d = d + 4 * x + 6;
            } else {
                d = d + 4 * (x - y) + 10;
                y--;
            }
            x++;
        }
    }
}

//******************************************************************
//????????  LCD_ShowChar
//?????    xiao??@???????
//?????    2013-02-22
//?????    ?????????????
//?????????(x,y):???????????'????
//        	fc:j?�??????
//			bc:???????
//			num:?????0-94??
//			size:??????
//			mode:g?  0,???g?;1,????g?
//???????  ??
//??l?�????
//******************************************************************
void LCD_ShowChar(u16 x, u16 y, u16 fc, u16 bc, u8 num, u8 size, u8 mode)
{
    u8 temp;
    u8 pos, t;
    u16 colortemp = POINT_COLOR;

    num = num - ' ';                                      //?�?t?????
    LCD_SetWindows(x, y, x + size / 2 - 1, y + size - 1); //???�??????????????
    if (!mode)                                            //???????
    {
        for (pos = 0; pos < size; pos++) {
            if (size == 12)
                temp = asc2_1206[num][pos]; //????1206????
            else
                temp = asc2_1608[num][pos]; //????1608????
            for (t = 0; t < size / 2; t++) {
                if (temp & 0x01)
                    LCD_DrawPoint_16Bit(fc);
                else
                    LCD_DrawPoint_16Bit(bc);
                temp >>= 1;
            }
        }
    } else //??????
    {
        for (pos = 0; pos < size; pos++) {
            if (size == 12)
                temp = asc2_1206[num][pos]; //????1206????
            else
                temp = asc2_1608[num][pos]; //????1608????
            for (t = 0; t < size / 2; t++) {
                POINT_COLOR = fc;
                if (temp & 0x01)
                    LCD_DrawPoint(x + t, y + pos); //??h????
                temp >>= 1;
            }
        }
    }
    POINT_COLOR = colortemp;
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); //???????????
}
//******************************************************************
//????????  LCD_ShowString
//?????    xiao??@???????
//?????    2013-02-22
//?????    ???????????
//?????????x,y :???????
//			size:??????
//			*p:???????'???
//			mode:g?	0,???g?;1,????g?
//???????  ??
//??l?�????
//******************************************************************
void LCD_ShowString(u16 x, u16 y, u8 size, u8 *p, u8 mode)
{
    while ((*p <= '~') && (*p >= ' ')) //?????????????!
    {
        if (x > (lcddev.width - 1) || y > (lcddev.height - 1))
            return;
        LCD_ShowChar(x, y, POINT_COLOR, BACK_COLOR, *p, size, mode);
        x += size / 2;
        p++;
    }
}

//******************************************************************
//????????  mypow
//?????    xiao??@???????
//?????    2013-02-22
//?????    ??m??n???(gui???????)
//?????????m:????
//	        n:??
//???????  m??n???
//??l?�????
//******************************************************************
u32 mypow(u8 m, u8 n)
{
    u32 result = 1;
    while (n--)
        result *= m;
    return result;
}

//******************************************************************
//????????  LCD_ShowNum
//?????    xiao??@???????
//?????    2013-02-22
//?????    ???????????????
//?????????x,y :???????
//			len :??????????????
//			size:??????(12,16)
//			color:???
//			num:???(0~4294967295)
//???????  ??
//??l?�????
//******************************************************************
void LCD_ShowNum(u16 x, u16 y, u32 num, u8 len, u8 size)
{
    u8 t, temp;
    u8 enshow = 0;
    for (t = 0; t < len; t++) {
        temp = (num / mypow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                LCD_ShowChar(x + (size / 2) * t, y, POINT_COLOR, BACK_COLOR, ' ', size, 0);
                continue;
            } else
                enshow = 1;
        }
        LCD_ShowChar(x + (size / 2) * t, y, POINT_COLOR, BACK_COLOR, temp + '0', size, 0);
    }
}

//******************************************************************
//????????  GUI_DrawFont16
//?????    xiao??@???????
//?????    2013-02-22
//?????    ???????16X16????????
//?????????x,y :???????
//			fc:j?�??????
//			bc:???????
//			s:????????
//			mode:g?	0,???g?;1,????g?
//???????  ??
//??l?�????
//******************************************************************
void GUI_DrawFont16(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u8 mode)
{
    u8 i, j;
    u16 k;
    u16 HZnum;
    u16 x0 = x;
    HZnum = sizeof(tfont16) / sizeof(typFNT_GB16); //????????????

    for (k = 0; k < HZnum; k++) {
        if ((tfont16[k].Index[0] == *(s)) && (tfont16[k].Index[1] == *(s + 1))) {
            LCD_SetWindows(x, y, x + 16 - 1, y + 16 - 1);
            for (i = 0; i < 16 * 2; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode) //???????
                    {
                        if (tfont16[k].Msk[i] & (0x80 >> j))
                            LCD_DrawPoint_16Bit(fc);
                        else
                            LCD_DrawPoint_16Bit(bc);
                    } else {
                        POINT_COLOR = fc;
                        if (tfont16[k].Msk[i] & (0x80 >> j))
                            LCD_DrawPoint(x, y); //??h????
                        x++;
                        if ((x - x0) == 16) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue; //??????????????????????????????????????g???????
    }

    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); //???????????
}

//******************************************************************
//????????  GUI_DrawFont24
//?????    xiao??@???????
//?????    2013-02-22
//?????    ???????24X24????????
//?????????x,y :???????
//			fc:j?�??????
//			bc:???????
//			s:????????
//			mode:g?	0,???g?;1,????g?
//???????  ??
//??l?�????
//******************************************************************
void GUI_DrawFont24(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u8 mode)
{
    u8 i, j;
    u16 k;
    u16 HZnum;
    u16 x0 = x;
    HZnum = sizeof(tfont24) / sizeof(typFNT_GB24); //????????????

    for (k = 0; k < HZnum; k++) {
        if ((tfont24[k].Index[0] == *(s)) && (tfont24[k].Index[1] == *(s + 1))) {
            LCD_SetWindows(x, y, x + 24 - 1, y + 24 - 1);
            for (i = 0; i < 24 * 3; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode) //???????
                    {
                        if (tfont24[k].Msk[i] & (0x80 >> j))
                            LCD_DrawPoint_16Bit(fc);
                        else
                            LCD_DrawPoint_16Bit(bc);
                    } else {
                        POINT_COLOR = fc;
                        if (tfont24[k].Msk[i] & (0x80 >> j))
                            LCD_DrawPoint(x, y); //??h????
                        x++;
                        if ((x - x0) == 24) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue; //??????????????????????????????????????g???????
    }

    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); //???????????
}

//******************************************************************
//????????  GUI_DrawFont32
//?????    xiao??@???????
//?????    2013-02-22
//?????    ???????32X32????????
//?????????x,y :???????
//			fc:j?�??????
//			bc:???????
//			s:????????
//			mode:g?	0,???g?;1,????g?
//???????  ??
//??l?�????
//******************************************************************
void GUI_DrawFont32(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u8 mode)
{
    u8 i, j;
    u16 k;
    u16 HZnum;
    u16 x0 = x;
    HZnum = sizeof(tfont32) / sizeof(typFNT_GB32); //????????????
    for (k = 0; k < HZnum; k++) {
        if ((tfont32[k].Index[0] == *(s)) && (tfont32[k].Index[1] == *(s + 1))) {
            LCD_SetWindows(x, y, x + 32 - 1, y + 32 - 1);
            for (i = 0; i < 32 * 4; i++) {
                for (j = 0; j < 8; j++) {
                    if (!mode) //???????
                    {
                        if (tfont32[k].Msk[i] & (0x80 >> j))
                            LCD_DrawPoint_16Bit(fc);
                        else
                            LCD_DrawPoint_16Bit(bc);
                    } else {
                        POINT_COLOR = fc;
                        if (tfont32[k].Msk[i] & (0x80 >> j))
                            LCD_DrawPoint(x, y); //??h????
                        x++;
                        if ((x - x0) == 32) {
                            x = x0;
                            y++;
                            break;
                        }
                    }
                }
            }
        }
        continue; //??????????????????????????????????????g???????
    }

    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); //???????????
}

//******************************************************************
//????????  Show_Str
//?????    xiao??@???????
//?????    2013-02-22
//?????    ???h???????,????????????
//?????????x,y :???????
// 			fc:j?�??????
//			bc:???????
//			str :?????
//			size:??????
//			mode:g?	0,???g?;1,????g?
//???????  ??
//??l?�????
//******************************************************************
void Show_Str(u16 x, u16 y, u16 fc, u16 bc, u8 *str, u8 size, u8 mode)
{
    u16 x0 = x;
    u8 bHz = 0;       //???????????
    while (*str != 0) //????d????
    {
        if (!bHz) {
            if (x > (lcddev.width - size / 2) || y > (lcddev.height - size))
                return;
            if (*str > 0x80)
                bHz = 1; //????
            else         //???
            {
                if (*str == 0x0D) //???????
                {
                    y += size;
                    x = x0;
                    str++;
                } else {
                    if (size > 16) //?????�?????12X24 16X32?????????,??8X16????
                    {
                        LCD_ShowChar(x, y, fc, bc, *str, 16, mode);
                        x += 8; //???,?????h??
                    } else {
                        LCD_ShowChar(x, y, fc, bc, *str, size, mode);
                        x += size / 2; //???,?????h??
                    }
                }
                str++;
            }
        } else //????
        {
            if (x > (lcddev.width - size) || y > (lcddev.height - size))
                return;
            bHz = 0; //??????
            if (size == 32)
                GUI_DrawFont32(x, y, fc, bc, str, mode);
            else if (size == 24)
                GUI_DrawFont24(x, y, fc, bc, str, mode);
            else
                GUI_DrawFont16(x, y, fc, bc, str, mode);

            str += 2;
            x += size; //??h??????t??
        }
    }
}

//******************************************************************
//????????  Gui_StrCenter
//?????    xiao??@???????
//?????    2013-02-22
//?????    ???????h???????,????????????
//?????????x,y :???????
// 			fc:j?�??????
//			bc:???????
//			str :?????
//			size:??????
//			mode:g?	0,???g?;1,????g?
//???????  ??
//??l?�????
//******************************************************************
void Gui_StrCenter(u16 x, u16 y, u16 fc, u16 bc, u8 *str, u8 size, u8 mode)
{
    u16 len = strlen((const char *)str);
    u16 x1 = (lcddev.width - len * 8) / 2;
    Show_Str(x + x1, y, fc, bc, str, size, mode);
}

//******************************************************************
//????????  Gui_Drawbmp16
//?????    xiao??@???????
//?????    2013-02-22
//?????    ???h??16?BMP???
//?????????x,y :???????
// 			*p :?????????'???
//???????  ??
//??l?�????
//******************************************************************
void Gui_Drawbmp16(u16 x, u16 y, u16 p_x, u8 p_y, const unsigned char *p) //???40*40 QQ??
{
    int i;
    unsigned char picH, picL;
    LCD_SetWindows(x, y, x + p_x - 1, y + p_y - 1); //????????
    for (i = 0; i < p_x * p_y; i++) {
        picL = *(p + i * 2); //????????j
        picH = *(p + i * 2 + 1);
        LCD_DrawPoint_16Bit(picH << 8 | picL);
    }
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); //??????????????
}

void show_explanwod(void)
{
    u8 i;
    u16 sy = 165;
    //	u16 ey=175;
    u16 x_start;
    x_start = 12;
    Show_Str(90, 130, WHITE, BLACK, "???????????", 16, 0);
    //LCD_Fill(2,2,318,23,WHITE);
    Show_Str(109, 6, WHITE, BLACK, "??????????", 16, 0);
    Gui_DrawLine(20, 137, 87, 137, WHITE); //??????
    Gui_DrawLine(185, 137, 251, 137, WHITE);
    Gui_DrawLine(245, 131, 251, 137, WHITE);
    Gui_DrawLine(251, 137, 245, 143, WHITE); //??????
    Show_Str(298, 10, WHITE, BLACK, "??", 16, 0);
    Show_Str(298, 30, WHITE, BLACK, "?", 16, 0);
    Show_Str(298, 50, WHITE, BLACK, "??", 16, 0);
    Show_Str(298, 70, WHITE, BLACK, "??", 16, 0);
    Show_Str(298, 90, WHITE, BLACK, "??", 16, 0);
    Show_Str(298, 110, WHITE, BLACK, "U", 16, 0);
    Show_Str(298, 130, WHITE, BLACK, "??", 16, 0);
    //Show_Str(30,225,WHITE,BLACK,"3724910-78B STM32 CP-01",12,0);

    //----------------------------------------------------------//
    /*Show_Str(x_start,sy,WHITE,BLACK,"1 ???,",16,0);	     
	Show_Str(x_start+64,sy,WHITE,BLACK,"2 ???,",16,0);	     
	Show_Str(x_start+128,sy,WHITE,BLACK,"3 ???",16,0);	
	Show_Str(x_start,sy+30,WHITE,BLACK,"4 ???,",16,0);	 
	Show_Str(x_start+64,sy+30,WHITE,BLACK,"5 ???,",16,0);	 
	Show_Str(x_start+128,sy+30,WHITE,BLACK,"6 ???",16,0);*/

    Show_Str(x_start, sy, WHITE, BLACK, "1???", 16, 0);
    Show_Str(x_start + 48, sy, WHITE, BLACK, "2???", 16, 0);
    Show_Str(x_start + 96, sy, WHITE, BLACK, "3???", 16, 0);
    Show_Str(x_start + 144, sy, WHITE, BLACK, "4???", 16, 0);

    Show_Str(x_start, sy + 28, WHITE, BLACK, "5???", 16, 0);
    Show_Str(x_start + 48, sy + 28, WHITE, BLACK, "6???", 16, 0);
    Show_Str(x_start + 96, sy + 28, WHITE, BLACK, "7???", 16, 0);
    Show_Str(x_start + 144, sy + 28, WHITE, BLACK, "8???", 16, 0);

    Show_Str(x_start, sy + 56, WHITE, BLACK, "9???", 16, 0);
    Show_Str(x_start + 48, sy + 56, WHITE, BLACK, "10???", 16, 0);
    Show_Str(x_start + 104, sy + 56, WHITE, BLACK, "11???", 16, 0);

    //----------------------------------------------------------//

    for (i = 1; i <= ModuleNum; i++) {
        Display_ModuleStatus(i, 5);
    }
    delay_ms(1000);
    for (i = 1; i <= ModuleNum; i++) {
        Display_ModuleStatus(i, 2);
    }
    delay_ms(1000);
    for (i = 1; i <= ModuleNum; i++) {
        Display_ModuleStatus(i, 4);
    }
    delay_ms(1000);
    for (i = 1; i <= ModuleNum; i++) {
        Display_ModuleStatus(i, 0);
    }
}

void show_car(u16 x, u16 y, u16 p_x, u16 p_y, const unsigned char bmp[]) //????????
{
    int i;
    unsigned char j;
    LCD_SetWindows(x, y, x + p_x - 1, y + p_y - 1); //????????
    for (i = 0; i < 2640; i++) {
        for (j = 0; j < 8; j++) {
            if ((bmp[i] & (0x80 >> j)))
                LCD_DrawPoint_16Bit(BLACK);
            else //if((bmp[i]&(0x80>>j)))
                LCD_DrawPoint_16Bit(WHITE);
        }
    }
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); //??????????????
}

void show_bar(u16 x, u16 y, u16 p_x, u16 p_y, const unsigned char bmp[]) //???U????
{
    int i;
    unsigned char j;
    LCD_SetWindows(x, y, x + p_x - 1, y + p_y - 1); //????????
    for (i = 0; i < 450; i++) {
        for (j = 0; j < 8; j++) {
            if ((bmp[i] & (0x80 >> j)))
                LCD_DrawPoint_16Bit(BLACK);
            else //if((bmp[i]&(0x80>>j)))
                LCD_DrawPoint_16Bit(WHITE);
        }
    }
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); //??????????????
}

void Display_ModuleStatus(unsigned char Light_Num, unsigned char Status) //???????????????????????
{
    u16 i, j;
    u16 color;
    u16 width = 18; //?�????????
    u16 height = 9; //???
    u16 sy = 25;
    u16 ey = 33;
    u16 width_x = 14;
    unsigned int X1_pos = Xpos[0], X2_pos = Xpos[1], X3_pos = Xpos[2], X4_pos = Xpos[3], X5_pos = Xpos[4], X6_pos = Xpos[5],
                 X7_pos = Xpos[6], X8_pos = Xpos[7], X9_pos = Xpos[8], X10_pos = Xpos[9], X11_pos = Xpos[10], X12_pos = Xpos[11],
                 X13_pos = Xpos[12], X14_pos = Xpos[13], X15_pos = Xpos[14];
    switch (Light_Num) {
        case 1:
            LCD_SetWindows(X1_pos, sy, width_x + X1_pos, ey); //???????????
            break;
        case 2:
            LCD_SetWindows(X2_pos, sy, width_x + X2_pos, ey); //???????????
            break;
        case 3:
            LCD_SetWindows(X3_pos, sy, width_x + X3_pos, ey); //???????????
            break;
        case 4:
            LCD_SetWindows(X4_pos, sy, width_x + X4_pos, ey); //???????????
            break;
        case 5:
            LCD_SetWindows(X5_pos, sy, width_x + X5_pos, ey); //???????????
            break;
        case 6:
            LCD_SetWindows(X6_pos, sy, width_x + X6_pos, ey); //???????????
            break;
        case 7:
            LCD_SetWindows(X7_pos, sy, width_x + X7_pos, ey); //???????????
            break;
        case 8:
            LCD_SetWindows(X8_pos, sy, width_x + X8_pos, ey); //???????????
            break;
        case 9:
            LCD_SetWindows(X9_pos, sy, width_x + X9_pos, ey); //???????????
            break;
        case 10:
            LCD_SetWindows(X10_pos, sy, width_x + X10_pos, ey); //???????????
            break;
        case 11:
            LCD_SetWindows(X11_pos, sy, width_x + X11_pos, ey); //???????????
            break;
        case 12:
            LCD_SetWindows(X12_pos, sy, width_x + X12_pos, ey); //???????????
            break;
        case 13:
            LCD_SetWindows(X13_pos, sy, width_x + X13_pos, ey); //???????????
            break;
        case 14:
            LCD_SetWindows(X14_pos, sy, width_x + X14_pos, ey); //???????????
            break;
        case 15:
            LCD_SetWindows(X15_pos, sy, width_x + X15_pos, ey); //???????????
            break;

        default:
            __NOP();
            break;
    }

    switch (Status) {
        case 0:
            color = GRAY; //????????�??????g??

            break;
        case 1:
            color = WHITE; //????????g?????

            break;
        case 2:
            color = GREEN; //????????g??????????

            break;
        case 3:
            color = YELLOW; //??????????????

            break;
        case 4:
            color = RED; //????????g????

            break;
        case 5:
            color = BLUE; //????????g????//????????

            break;

        default:
            __NOP();
            break;
    }
#if LCD_USE8BIT_MODEL == 1
    LCD_RS_SET; //?????
    LCD_CS_CLR;
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            DATAOUT(color >> 8);
            LCD_WR_CLR;
            LCD_WR_SET;

            DATAOUT(color);
            LCD_WR_CLR;
            LCD_WR_SET;
        }
    }
    LCD_CS_SET;
#else //16?g?
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++)
            LCD_WR_DATA(color); //???????
    }
#endif
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); //???????????????
}

void Display_Area_RedGlint(unsigned char const bmp[], unsigned char Area, unsigned char original) //???28?????l?????
{
    unsigned char Start_Y = blocktype[0], End_Y = blocktype[1]; //???????Y???????????????
    unsigned char Start_X1 = 244, End_X1 = 251, Start_X2 = 236, End_X2 = 243, Start_X3 = 228, End_X3 = 235, Start_X4 = 220, End_X4 = 227,
                  Start_X5 = 212, End_X5 = 219, Start_X6 = 204, End_X6 = 211, Start_X7 = 196, End_X7 = 203, Start_X8 = 188, End_X8 = 195,
                  Start_X9 = 180, End_X9 = 187, Start_X10 = 172, End_X10 = 179, Start_X11 = 164, End_X11 = 171, Start_X12 = 156,
                  End_X12 = 163, Start_X13 = 148, End_X13 = 155, Start_X14 = 140, End_X14 = 147, Start_X15 = 132, End_X15 = 139,
                  Start_X16 = 124, End_X16 = 131, Start_X17 = 116, End_X17 = 123, Start_X18 = 108, End_X18 = 115, Start_X19 = 100,
                  End_X19 = 107, Start_X20 = 92, End_X20 = 99, Start_X21 = 84, End_X21 = 91, Start_X22 = 76, End_X22 = 83, Start_X23 = 68,
                  End_X23 = 75, Start_X24 = 60, End_X24 = 67, Start_X25 = 52, End_X25 = 59, Start_X26 = 44, End_X26 = 51, Start_X27 = 36,
                  End_X27 = 43, Start_X28 = 28, End_X28 = 35; //28?????X????????????

    u16 color;
    int i;
    unsigned char j;
    if (original == 0) {
        color = RED;
    } else if (original == 1) {
        color = BLACK;
    } else if (original == 2) {
        color = YELLOW;
    }

    switch (Area) {
        case 1:
            LCD_SetWindows(Start_X1, Start_Y, End_X1, End_Y); //???????????
            break;
        case 2:
            LCD_SetWindows(Start_X2, Start_Y, End_X2, End_Y); //???????????
            break;
        case 3:
            LCD_SetWindows(Start_X3, Start_Y, End_X3, End_Y); //???????????
            break;
        case 4:
            LCD_SetWindows(Start_X4, Start_Y, End_X4, End_Y); //???????????
            break;
        case 5:
            LCD_SetWindows(Start_X5, Start_Y, End_X5, End_Y); //???????????
            break;
        case 6:
            LCD_SetWindows(Start_X6, Start_Y, End_X6, End_Y); //???????????
            break;
        case 7:
            LCD_SetWindows(Start_X7, Start_Y, End_X7, End_Y); //???????????
            break;
        case 8:
            LCD_SetWindows(Start_X8, Start_Y, End_X8, End_Y); //???????????
            break;
        case 9:
            LCD_SetWindows(Start_X9, Start_Y, End_X9, End_Y); //???????????
            break;
        case 10:
            LCD_SetWindows(Start_X10, Start_Y, End_X10, End_Y); //???????????
            break;
        case 11:
            LCD_SetWindows(Start_X11, Start_Y, End_X11, End_Y); //???????????
            break;
        case 12:
            LCD_SetWindows(Start_X12, Start_Y, End_X12, End_Y); //???????????
            break;
        case 13:
            LCD_SetWindows(Start_X13, Start_Y, End_X13, End_Y); //???????????
            break;
        case 14:
            LCD_SetWindows(Start_X14, Start_Y, End_X14, End_Y); //???????????
            break;
        case 15:
            LCD_SetWindows(Start_X15, Start_Y, End_X15, End_Y); //???????????
            break;
        case 16:
            LCD_SetWindows(Start_X16, Start_Y, End_X16, End_Y); //???????????
            break;
        case 17:
            LCD_SetWindows(Start_X17, Start_Y, End_X17, End_Y); //???????????
            break;
        case 18:
            LCD_SetWindows(Start_X18, Start_Y, End_X18, End_Y); //???????????
            break;
        case 19:
            LCD_SetWindows(Start_X19, Start_Y, End_X19, End_Y); //???????????
            break;
        case 20:
            LCD_SetWindows(Start_X20, Start_Y, End_X20, End_Y); //???????????
            break;
        case 21:
            LCD_SetWindows(Start_X21, Start_Y, End_X21, End_Y); //???????????
            break;
        case 22:
            LCD_SetWindows(Start_X22, Start_Y, End_X22, End_Y); //???????????
            break;
        case 23:
            LCD_SetWindows(Start_X23, Start_Y, End_X23, End_Y); //???????????
            break;
        case 24:
            LCD_SetWindows(Start_X24, Start_Y, End_X24, End_Y); //???????????
            break;
        case 25:
            LCD_SetWindows(Start_X25, Start_Y, End_X25, End_Y); //???????????
            break;
        case 26:
            LCD_SetWindows(Start_X26, Start_Y, End_X26, End_Y); //???????????
            break;
        case 27:
            LCD_SetWindows(Start_X27, Start_Y, End_X27, End_Y); //???????????
            break;
        case 28:
            LCD_SetWindows(Start_X28, Start_Y, End_X28, End_Y); //???????????
            break;
        default:
            __NOP();
            break;
    }

    for (i = 0; i < 64; i++) {
        for (j = 0; j < 8; j++) {
            if ((bmp[i] & (0x80 >> j)))
                LCD_DrawPoint_16Bit(color);
            else //if((bmp[i]&(0x80>>j)))
                LCD_DrawPoint_16Bit(WHITE);
        }
    }
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); //??????????????
}

void Display_Words_Status(unsigned char Light_Num, unsigned char Status) //??????????????
{
    u16 sy = 165;
    u16 x_start;
    x_start = 12;
    switch (Light_Num) {
        case 1:
            if (Status == 0)
                Show_Str(x_start, sy, WHITE, BLACK, "1???", 16, 0);
            else if (Status == 1)
                Show_Str(x_start, sy, WHITE, BLACK, "1???", 16, 0);
            else if (Status == 2)
                Show_Str(x_start, sy, WHITE, BLACK, "1????", 16, 0);
            else if ((Status == 3) | (Status == 4))
                Show_Str(x_start, sy, RED, BLACK, "1????", 16, 0);
            break;
        case 2:
            if (Status == 0)
                Show_Str(x_start + 48, sy, WHITE, BLACK, "2???", 16, 0);
            else if (Status == 1)
                Show_Str(x_start + 48, sy, WHITE, BLACK, "2???", 16, 0);
            else if (Status == 2)
                Show_Str(x_start + 48, sy, WHITE, BLACK, "2????", 16, 0);
            else if ((Status == 3) | (Status == 4))
                Show_Str(x_start + 48, sy, RED, BLACK, "2????", 16, 0);
            break;
        case 3:
            if (Status == 0)
                Show_Str(x_start + 96, sy, WHITE, BLACK, "3???", 16, 0);
            else if (Status == 1)
                Show_Str(x_start + 96, sy, WHITE, BLACK, "3???", 16, 0);
            else if (Status == 2)
                Show_Str(x_start + 96, sy, WHITE, BLACK, "3????", 16, 0);
            else if ((Status == 3) | (Status == 4))
                Show_Str(x_start + 96, sy, RED, BLACK, "3????", 16, 0);
            break;
        case 4:
            if (Status == 0)
                Show_Str(x_start + 144, sy, WHITE, BLACK, "4???", 16, 0);
            else if (Status == 1)
                Show_Str(x_start + 144, sy, WHITE, BLACK, "4???", 16, 0);
            else if (Status == 2)
                Show_Str(x_start + 144, sy, WHITE, BLACK, "4????", 16, 0);
            else if ((Status == 3) | (Status == 4))
                Show_Str(x_start + 144, sy, RED, BLACK, "4????", 16, 0);
            break;
        case 5:
            if (Status == 0)
                Show_Str(x_start, sy + 28, WHITE, BLACK, "5???", 16, 0);
            else if (Status == 1)
                Show_Str(x_start, sy + 28, WHITE, BLACK, "5???", 16, 0);
            else if (Status == 2)
                Show_Str(x_start, sy + 28, WHITE, BLACK, "5????", 16, 0);
            else if ((Status == 3) | (Status == 4))
                Show_Str(x_start, sy + 28, RED, BLACK, "5????", 16, 0);
            break;

        case 6:
            if (Status == 0)
                Show_Str(x_start + 48, sy + 28, WHITE, BLACK, "6???", 16, 0);
            else if (Status == 1)
                Show_Str(x_start + 48, sy + 28, WHITE, BLACK, "6???", 16, 0);
            else if (Status == 2)
                Show_Str(x_start + 48, sy + 28, WHITE, BLACK, "6????", 16, 0);
            else if ((Status == 3) | (Status == 4))
                Show_Str(x_start + 48, sy + 28, RED, BLACK, "6????", 16, 0);
            break;

        case 7:
            if (Status == 0)
                Show_Str(x_start + 96, sy + 28, WHITE, BLACK, "7???", 16, 0);
            else if (Status == 1)
                Show_Str(x_start + 96, sy + 28, WHITE, BLACK, "7???", 16, 0);
            else if (Status == 2)
                Show_Str(x_start + 96, sy + 28, WHITE, BLACK, "7????", 16, 0);
            else if ((Status == 3) | (Status == 4))
                Show_Str(x_start + 96, sy + 28, RED, BLACK, "7????", 16, 0);
            break;
        case 8:
            if (Status == 0)
                Show_Str(x_start + 144, sy + 28, WHITE, BLACK, "8???", 16, 0);
            else if (Status == 1)
                Show_Str(x_start + 144, sy + 28, WHITE, BLACK, "8???", 16, 0);
            else if (Status == 2)
                Show_Str(x_start + 144, sy + 28, WHITE, BLACK, "8????", 16, 0);
            else if ((Status == 3) | (Status == 4))
                Show_Str(x_start + 144, sy + 28, RED, BLACK, "8????", 16, 0);
            break;
        case 9:
            if (Status == 0)
                Show_Str(x_start, sy + 56, WHITE, BLACK, "9???", 16, 0);
            else if (Status == 1)
                Show_Str(x_start, sy + 56, WHITE, BLACK, "9???", 16, 0);
            else if (Status == 2)
                Show_Str(x_start, sy + 56, WHITE, BLACK, "9????", 16, 0);
            else if ((Status == 3) | (Status == 4))
                Show_Str(x_start, sy + 56, RED, BLACK, "9????", 16, 0);
            break;
        case 10:
            if (Status == 0)
                Show_Str(x_start + 48, sy + 56, WHITE, BLACK, "10???", 16, 0);
            else if (Status == 1)
                Show_Str(x_start + 48, sy + 56, WHITE, BLACK, "10???", 16, 0);
            else if (Status == 2)
                Show_Str(x_start + 48, sy + 56, WHITE, BLACK, "10????", 16, 0);
            else if ((Status == 3) | (Status == 4))
                Show_Str(x_start + 48, sy + 56, RED, BLACK, "10????", 16, 0);
            break;
        case 11:
            if (Status == 0)
                Show_Str(x_start + 104, sy + 56, WHITE, BLACK, "11???", 16, 0);
            else if (Status == 1)
                Show_Str(x_start + 104, sy + 56, WHITE, BLACK, "11???", 16, 0);
            else if (Status == 2)
                Show_Str(x_start + 104, sy + 56, WHITE, BLACK, "11????", 16, 0);
            else if ((Status == 3) | (Status == 4))
                Show_Str(x_start + 104, sy + 56, RED, BLACK, "11????", 16, 0);
            break;

        default:
            __NOP();
            break;
    }
}

void ShowLine() //???U?????
{
    u8 i;
    u16 x1, y1, x2, color;
    x1 = 275, x2 = 295;
    m_max_value = VALUE[max_value]; //????�?U??????????
    if (m_max_value != last_max_value) {
        if ((m_max_value - last_max_value) > 0) {
            y1 = 157;
            for (i = 0; i <= m_max_value; i++) //?????????????
            {
                y1--;
                color = BAR_COLOR[y1];
                Gui_DrawLine(x1, y1, x2, y1, color);
            }
        } else {
            y1 = 27;
            for (i = 0; i <= 130 - m_max_value; i++) //????????????
            {
                y1++;
                Gui_DrawLine(x1, y1, x2, y1, BLACK);
            }
        }
        last_max_value = m_max_value;
    } else
        __NOP();
}

void show_speak(u16 x, u16 y, u16 p_x, u16 p_y, const unsigned char bmp[]) //???????
{
    int i;
    unsigned char j;
    LCD_SetWindows(x, y, x + p_x - 1, y + p_y - 1); //????????
    for (i = 0; i < 180; i++) {
        for (j = 0; j < 8; j++) {
            if ((bmp[i] & (0x80 >> j)))
                LCD_DrawPoint_16Bit(BLACK);
            else //if((bmp[i]&(0x80>>j)))
                LCD_DrawPoint_16Bit(WHITE);
        }
    }
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); //??????????????
}

void show_rectangle(void)
{
    Gui_DrawLine(1, 1, 319, 1, GRAY2); //
    Gui_DrawLine(1, 239, 319, 239, GRAY2);
    Gui_DrawLine(1, 1, 1, 239, GRAY2);       //
    Gui_DrawLine(319, 1, 319, 239, GRAY2);   //
    Gui_DrawLine(318, 1, 318, 239, GRAY2);   //
                                             //	Gui_DrawLine(1,223,200,223,GRAY2);
    Gui_DrawLine(1, 158, 319, 158, GRAY2);   //
    Gui_DrawLine(200, 158, 200, 240, GRAY2); //

    Gui_DrawLine(0, 0, 320, 0, GRAY2);
    Gui_DrawLine(0, 240, 320, 240, GRAY2);
    Gui_DrawLine(0, 238, 320, 238, GRAY2);
    Gui_DrawLine(0, 0, 0, 240, GRAY2);
    Gui_DrawLine(320, 0, 320, 240, GRAY2);
}

void ShowNum(u16 x, u16 y, u16 fc, u16 bc, u8 num, u8 size, u8 mode)
{
    switch (num) {
        case 0:
            LCD_ShowChar(x, y, fc, bc, '0', size, mode);
            break;
        case 1:
            LCD_ShowChar(x, y, fc, bc, '1', size, mode);
            break;
        case 2:
            LCD_ShowChar(x, y, fc, bc, '2', size, mode);
            break;
        case 3:
            LCD_ShowChar(x, y, fc, bc, '3', size, mode);
            break;
        case 4:
            LCD_ShowChar(x, y, fc, bc, '4', size, mode);
            break;
        case 5:
            LCD_ShowChar(x, y, fc, bc, '5', size, mode);
            break;
        case 6:
            LCD_ShowChar(x, y, fc, bc, '6', size, mode);
            break;
        case 7:
            LCD_ShowChar(x, y, fc, bc, '7', size, mode);
            break;
        case 8:
            LCD_ShowChar(x, y, fc, bc, '8', size, mode);
            break;
        case 9:
            LCD_ShowChar(x, y, fc, bc, '9', size, mode);
            break;
        case 10:
            LCD_ShowChar(x, y, fc, bc, 'A', size, mode);
            break;
        case 11:
            LCD_ShowChar(x, y, fc, bc, 'B', size, mode);
            break;
        case 12:
            LCD_ShowChar(x, y, fc, bc, 'C', size, mode);
            break;
        case 13:
            LCD_ShowChar(x, y, fc, bc, 'D', size, mode);
            break;
        case 14:
            LCD_ShowChar(x, y, fc, bc, 'E', size, mode);
            break;
        case 15:
            LCD_ShowChar(x, y, fc, bc, 'F', size, mode);
            break;
            ////				case 0: LCD_Show_2412_char(x,y,fc,bc,'0',mode);
            ////				break;
            ////				case 1: LCD_Show_2412_char(x,y,fc,bc,'1',mode);
            ////				break;
            ////			  case 2: LCD_Show_2412_char(x,y,fc,bc,'2',mode);
            ////				break;
            ////			  case 3: LCD_Show_2412_char(x,y,fc,bc,'3',mode);
            ////				break;
            ////			  case 4: LCD_Show_2412_char(x,y,fc,bc,'4',mode);
            ////				break;
            ////			  case 5: LCD_Show_2412_char(x,y,fc,bc,'5',mode);
            ////				break;
            ////			  case 6: LCD_Show_2412_char(x,y,fc,bc,'6',mode);
            ////				break;
            ////			  case 7: LCD_Show_2412_char(x,y,fc,bc,'7',mode);
            ////				break;
            ////			  case 8: LCD_Show_2412_char(x,y,fc,bc,'8',mode);
            ////				break;
            ////			  case 9: LCD_Show_2412_char(x,y,fc,bc,'9',mode);
            ////				break;
            ////		  	case 10: LCD_Show_2412_char(x,y,fc,bc,'A',mode);
            ////				break;
            ////				case 11: LCD_Show_2412_char(x,y,fc,bc,'B',mode);
            ////				break;
            ////				case 12: LCD_Show_2412_char(x,y,fc,bc,'C',mode);
            ////				break;
            ////				case 13: LCD_Show_2412_char(x,y,fc,bc,'D',mode);
            ////				break;
            ////				case 14: LCD_Show_2412_char(x,y,fc,bc,'E',mode);
            ////				break;
            ////				case 15: LCD_Show_2412_char(x,y,fc,bc,'F',mode);
            ////				break;
            ////
        default:
            break;
    }
}
void LCD_Show_2412_char(u16 x, u16 y, u16 fc, u16 bc, u8 num, u8 mode)
{
    u16 temp;
    u8 pos, t;
    u16 colortemp = POINT_COLOR;
    num = num - ' ';                              //???????
    LCD_SetWindows(x, y, x + 12 - 1, y + 24 - 1); //??????????
    if (!mode)                                    //?????:???????,???????????????
    {
        for (pos = 0; pos < 24; pos++) {
            temp = (asc2_2412[num][pos * 2] << 8) | asc2_2412[num][pos * 2 + 1]; //??2412??,????????
            for (t = 0; t < 12; t++) {
                if (temp & 0x8000) {
                    LCD_DrawPoint_16Bit(fc);
                } else {
                    LCD_DrawPoint_16Bit(bc);
                }
                temp <<= 1;
            }
        }
    } else //????:???????,???????????????
    {
        for (pos = 0; pos < 24; pos++) {
            temp = (asc2_2412[num][pos * 2] << 8) | asc2_2412[num][pos * 2 + 1]; //??2412??,????????
            for (t = 0; t < 12; t++) {
                POINT_COLOR = fc;
                if (temp & 0x8000) {
                    LCD_DrawPoint(x + t, y + pos); //????
                }
                temp <<= 1;
            }
        }
    }
    POINT_COLOR = colortemp;
    LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1); //???????
}
//---20191230
/*	void ShowNum(u16 x,u16 y,u16 fc, u16 bc, u8 num,u8 size,u8 mode) 
{
		switch(num)
		{
			  case 0: LCD_ShowChar(x,y,fc,bc,'0',size,mode);
				break;
				case 1: LCD_ShowChar(x,y,fc,bc,'1',size,mode);
				break;
			  case 2: LCD_ShowChar(x,y,fc,bc,'2',size,mode);
				break;
			  case 3: LCD_ShowChar(x,y,fc,bc,'3',size,mode);
				break;
			  case 4: LCD_ShowChar(x,y,fc,bc,'4',size,mode);
				break;
			  case 5: LCD_ShowChar(x,y,fc,bc,'5',size,mode);
				break;
			  case 6: LCD_ShowChar(x,y,fc,bc,'6',size,mode);
				break;
			  case 7: LCD_ShowChar(x,y,fc,bc,'7',size,mode);
				break;
			  case 8: LCD_ShowChar(x,y,fc,bc,'8',size,mode);
				break;
			  case 9: LCD_ShowChar(x,y,fc,bc,'9',size,mode);
				break;
		  	case 10: LCD_ShowChar(x,y,fc,bc,'A',size,mode);
				break;
				case 11: LCD_ShowChar(x,y,fc,bc,'B',size,mode);
				break;
				case 12: LCD_ShowChar(x,y,fc,bc,'C',size,mode);
				break;
				case 13: LCD_ShowChar(x,y,fc,bc,'D',size,mode);
				break;
				case 14: LCD_ShowChar(x,y,fc,bc,'E',size,mode);
				break;
				case 15: LCD_ShowChar(x,y,fc,bc,'F',size,mode);
				break;
				
			default: break;
		}
		
}*/
