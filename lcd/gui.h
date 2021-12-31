#ifndef __GUI_H__
#define __GUI_H__
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
#include "stm32f10x.h"
//#include "font.h"
extern uint8_t line;      //
extern uint8_t max_value; //
extern uint8_t last_max_value;
void GUI_DrawPoint(u16 x, u16 y, u16 color);
void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 color);
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);
void Draw_Circle(u16 x0, u16 y0, u16 fc, u8 r);
void LCD_ShowChar(u16 x, u16 y, u16 fc, u16 bc, u8 num, u8 size, u8 mode);
void LCD_ShowNum(u16 x, u16 y, u32 num, u8 len, u8 size);
void LCD_Show2Num(u16 x, u16 y, u16 num, u8 len, u8 size, u8 mode);
void LCD_ShowString(u16 x, u16 y, u8 size, u8 *p, u8 mode);
void GUI_DrawFont16(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u8 mode);
void GUI_DrawFont24(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u8 mode);
void GUI_DrawFont32(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u8 mode);
void Show_Str(u16 x, u16 y, u16 fc, u16 bc, u8 *str, u8 size, u8 mode);
void Gui_Drawbmp16(u16 x, u16 y, u16 p_x, u8 p_y, const unsigned char *p); //???40*40 QQ??
void gui_circle(int xc, int yc, u16 c, int r, int fill);
void Gui_StrCenter(u16 x, u16 y, u16 fc, u16 bc, u8 *str, u8 size, u8 mode);
void LCD_DrawFillRectangle(u16 x1, u16 y1, u16 x2, u16 y2);
void show_car(u16 x, u16 y, u16 p_x, u16 p_y, const unsigned char bmp[]);
void Display_ModuleStatus(unsigned char Light_Num, unsigned char Status);
void Display_Area_RedGlint(unsigned char const bmp[], unsigned char Area, unsigned char original);
void Gui_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);
void ShowLine(void); //???U?????
void show_bar(u16 x, u16 y, u16 p_x, u16 p_y, const unsigned char bmp[]);
void show_speak(u16 x, u16 y, u16 p_x, u16 p_y, const unsigned char bmp[]);
void show_rectangle(void);
void LCD_Clear_slow(u16 Color);
void Display_Words_Status(unsigned char Light_Num, unsigned char Status); //???????????????????????
void ShowNum(u16 x, u16 y, u16 fc, u16 bc, u8 num, u8 size, u8 mode);
void LCD_Show_2412_char(u16 x, u16 y, u16 fc, u16 bc, u8 num, u8 mode);

#endif
