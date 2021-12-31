#include <stdlib.h>
#include <stdint.h>
#include "mt_common.h"
#include "cmt2300a_hal.h"
#include "cmt2300a_hal_rf315.h"
#include "cmt2300a_hal_rf330.h"
#include "cmt2300a_hal_rf433.h"

unsigned char rf315_en = 0;
unsigned char rf330_en = 0;
unsigned char rf430_en = 0;
unsigned char rf433_en = 0;

unsigned char rf315_buf[2][4];
unsigned char rf330_buf[2][4];
unsigned char rf430_buf[2][4];
unsigned char rf433_buf[2][4];

#define Delay_us(x) delay_us(x)

unsigned char Rec315_jmnx;
unsigned char Rec315_key_d;
unsigned char Rec315_short_k;

unsigned char Rec330_jmnx;
unsigned char Rec330_key_d;
unsigned char Rec330_short_k;

unsigned char Rec430_jmnx;
unsigned char Rec430_key_d;
unsigned char Rec430_short_k;

unsigned char Rec433_jmnx;
unsigned char Rec433_key_d;
unsigned char Rec433_short_k;

void RF315_IN(void)
{
    unsigned char j = 0;
    unsigned char k = 0;
    unsigned char u, i;
    unsigned int head_k = 0;
    unsigned int short_k = 0;
    unsigned char T19_S;
    unsigned char T19_L;
    unsigned char key_d;
    unsigned char jmnx;
    unsigned char ii = 0;

    if (rf315_en == 0) {
        short_k = 0;
        while (RFIN_RF315 && j < 252) {
            Delay_us(5);
            short_k++;
        }
        while (!RFIN_RF315) {
            Delay_us(5);
            head_k++;
        }
        if (((short_k * 22) < head_k) && (head_k < (short_k * 38))) {
            for (ii = 0; ii < 3; ii++) //3??
            {
                for (k = 0; k < 8; k++) //????8?
                {
                    j = 0;
                    while (RFIN_RF315 && j < 245) {
                        Delay_us(5); //16us(6mhz:2~5)
                        j++;
                    }
                    if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                        rf315_buf[0][ii] &= ~(1 << ((7 - k)));
                    else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                        rf315_buf[0][ii] |= (1 << (7 - k));
                    else //????
                    {
                        rf315_en = 0;
                        return;
                    }
                    head_k = 0;
                    while (!RFIN_RF315 && j < 255) //?????
                    {
                        Delay_us(5);
                        head_k++;
                    }
                    if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                    {
                        //rf315_buf[0][2]|=(1<<(0));
                        T19_S = 1;
                        break;
                    } else {
                        T19_S = 0;
                    }
                }
            }
            if (T19_S == 1) {
                for (ii = 0; ii < 3; ii++) //3??
                {
                    for (k = 0; k < 8; k++) //????8?
                    {
                        j = 0;
                        while (RFIN_RF315 && j < 245) {
                            Delay_us(5); //16us(6mhz:2~5)
                            j++;
                        }
                        if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                            rf315_buf[1][ii] &= ~(1 << ((7 - k)));
                        else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                            rf315_buf[1][ii] |= (1 << (7 - k));
                        else //????
                        {
                            rf315_en = 0;
                            return;
                        }
                        head_k = 0;
                        while (!RFIN_RF315 && j < 255) //?????
                        {
                            Delay_us(5);
                            head_k++;
                        }
                        if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                        {
                            //rf315_buf[1][2]|=(1<<(0));
                            T19_S = 1;
                            rf315_buf[0][3] = 0;
                            rf315_buf[1][3] = 0;
                            break;
                        } else {
                            T19_S = 0;
                        }
                    }
                }
            }
            j = 0;
            while (RFIN_RF315 && (j < 200)) //???????????
            {
                Delay_us(5);
                j++;
                //j1++;
            }
            head_k = 0;
            while (!RFIN_RF315) //???????????
            {
                Delay_us(5);
                head_k++;
            }
            if (((short_k * 22) < head_k) && (head_k < (short_k * 38)) && (T19_S == 0)) //???????????32?
            {
                for (ii = 0; ii < 3; ii++) //3??
                {
                    for (k = 0; k < 8; k++) //????8?
                    {
                        j = 0;
                        while (RFIN_RF315 && j < 245) {
                            Delay_us(5); //16us(6mhz:2~5)
                            j++;
                        }
                        head_k = 0;
                        while (!RFIN_RF315 && j < 255) //?????
                        {
                            Delay_us(5);
                            head_k++;
                        }
                        if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                            rf315_buf[1][ii] &= ~(1 << ((7 - k)));
                        else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                            rf315_buf[1][ii] |= (1 << (7 - k));
                        else //????
                        {
                            rf315_en = 0;
                            return;
                        }
                    }
                }
                rf315_buf[0][3] = 0;
                rf315_buf[1][3] = 0;
            } else if ((j < (short_k * 5)) && (T19_S == 0)) //T19_L
            {
                k = 0;
                if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                    rf315_buf[0][3] &= ~(1 << ((7 - k)));
                else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                    rf315_buf[0][3] |= (1 << (7 - k));
                else //????
                {
                    rf315_en = 0;
                    return;
                }
                for (k = 1; k < 8; k++) //????8?
                {
                    j = 0;
                    while (RFIN_RF315 && j < 245) {
                        Delay_us(5); //16us(6mhz:2~5)
                        j++;
                    }
                    if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                        rf315_buf[0][3] &= ~(1 << ((7 - k)));
                    else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                        rf315_buf[0][3] |= (1 << (7 - k));
                    else //????
                    {
                        rf315_en = 0;
                        return;
                    }
                    head_k = 0;
                    while (!RFIN_RF315 && j < 255) //?????
                    {
                        Delay_us(5);
                        head_k++;
                    }
                    if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                    {
                        //rf315_buf[1][2]|=(1<<(0));
                        T19_L = 1;
                        // rf315_buf[0][3]=0;
                        //rf315_buf[1][3]=0;
                        break;
                    } else {
                        T19_L = 0;
                    }
                }
                if (T19_L == 1) {
                    for (ii = 0; ii < 4; ii++) //4??
                    {
                        for (k = 0; k < 8; k++) //????8?
                        {
                            j = 0;
                            while (RFIN_RF315 && j < 245) {
                                Delay_us(5); //16us(6mhz:2~5)
                                j++;
                            }
                            if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                                rf315_buf[1][ii] &= ~(1 << ((7 - k)));
                            else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                                rf315_buf[1][ii] |= (1 << (7 - k));
                            else //????
                            {
                                rf315_en = 0;
                                return;
                            }
                            head_k = 0;
                            while (!RFIN_RF315 && j < 255) //?????
                            {
                                Delay_us(5);
                                head_k++;
                            }
                            if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                            {
                                //rf315_buf[1][2]|=(1<<(0));
                                T19_L = 1;
                                // rf315_buf[0][3]=0;
                                //rf315_buf[1][3]=0;
                                break;
                            } else {
                                T19_L = 0;
                            }
                        }
                    }
                }
            }

            //+++++++++++++++++++++++++2262?1527??????++++++++++++++++++++++++++++++++++++++++
            if (T19_S == 1) {
                for (i = 0; i < 3; i++) //??2262?T19S
                {
                    for (u = 0; u < 4; u++) {
                        if (((rf315_buf[0][i] >> (u * 2)) & 3) == 2) {
                            i = 80;
                            break;
                        }
                    } //?10??T19S"H"
                    if (i == 80)
                        break;
                }
                if (i == 80) //T19S
                {
                    // key_d=rf315_buf[1][2] & 0x0f;      //??1527????
                    //rf315_buf[0][2]=rf315_buf[1][2]>>4; 	//??1527??4???
                    key_d = rf315_buf[1][2] & 0x0f;         //??1527????
                    rf315_buf[0][2] = rf315_buf[1][2] >> 4; //??1527??4???
                    jmnx = 2;                               //?0??2262 ,1?T19S
                } else {
                    jmnx = 0;
                }
                rf315_en = 4; //????
                //jmnx = 0;
            } else if (T19_L == 1) {
                key_d = rf315_buf[1][2] >> 1; //??1527????
                key_d = key_d & 0x0f;
                key_d = key_d << 4;
                //rf315_buf[0][2]=key_d; 	//??1527??4???

                jmnx = 3;     //T19_L
                rf315_en = 4; //????
            } else if ((rf315_buf[0][0] == rf315_buf[1][0]) && (rf315_buf[0][1] == rf315_buf[1][1])
                       && (rf315_buf[0][2] == rf315_buf[1][2])) //?????????????
            {
                for (i = 0; i < 4; i++) //??2262?1527
                {
                    for (u = 0; u < 4; u++) {
                        if (((rf315_buf[0][i] >> (u * 2)) & 3) == 2) {
                            i = 80;
                            break;
                        }
                    } //?10??1527"H"
                    if (i == 80)
                        break;
                }
                if (i == 80) //1527
                {
                    key_d = rf315_buf[1][2] & 0x0f;         //??1527????
                    rf315_buf[0][2] = rf315_buf[1][2] >> 4; //??1527??4???
                    jmnx = 1;                               //?0??2262 ,1?1527
                } else                                      //2262
                {
                    key_d = 0;
                    for (i = 0; i < 4; i++) //???? 2262?? ?????
                    {
                        if (((rf315_buf[0][2] >> (i * 2)) & 3) == 3)
                            key_d |= 1 << i;
                    }
                    rf315_buf[0][2] = 0; //2262??4???,??0
                    jmnx = 0;            //?0?2262,1?1527
                }
                rf315_en = 4; //????
            }
        }
    }
    Rec315_short_k = short_k;
    Rec315_jmnx = jmnx;
    Rec315_key_d = key_d;
}

void RF330_IN(void)
{
    unsigned char j = 0;
    unsigned char k = 0;
    unsigned char u, i;
    unsigned int head_k = 0;
    unsigned int short_k = 0;
    unsigned char T19_S;
    unsigned char T19_L;
    unsigned char key_d;
    unsigned char jmnx;
    unsigned char ii = 0;

    if (rf330_en == 0) {
        short_k = 0;
        while (RFIN_RF330 && j < 252) {
            Delay_us(5);
            short_k++;
        }
        while (!RFIN_RF330) {
            Delay_us(5);
            head_k++;
        }
        if (((short_k * 22) < head_k) && (head_k < (short_k * 38))) {
            for (ii = 0; ii < 3; ii++) //3??
            {
                for (k = 0; k < 8; k++) //????8?
                {
                    j = 0;
                    while (RFIN_RF330 && j < 245) {
                        Delay_us(5); //16us(6mhz:2~5)
                        j++;
                    }
                    if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                        rf330_buf[0][ii] &= ~(1 << ((7 - k)));
                    else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                        rf330_buf[0][ii] |= (1 << (7 - k));
                    else //????
                    {
                        rf330_en = 0;
                        return;
                    }
                    head_k = 0;
                    while (!RFIN_RF330 && j < 255) //?????
                    {
                        Delay_us(5);
                        head_k++;
                    }
                    if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                    {
                        //rf330_buf[0][2]|=(1<<(0));
                        T19_S = 1;
                        break;
                    } else {
                        T19_S = 0;
                    }
                }
            }
            if (T19_S == 1) {
                for (ii = 0; ii < 3; ii++) //3??
                {
                    for (k = 0; k < 8; k++) //????8?
                    {
                        j = 0;
                        while (RFIN_RF330 && j < 245) {
                            Delay_us(5); //16us(6mhz:2~5)
                            j++;
                        }
                        if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                            rf330_buf[1][ii] &= ~(1 << ((7 - k)));
                        else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                            rf330_buf[1][ii] |= (1 << (7 - k));
                        else //????
                        {
                            rf330_en = 0;
                            return;
                        }
                        head_k = 0;
                        while (!RFIN_RF330 && j < 255) //?????
                        {
                            Delay_us(5);
                            head_k++;
                        }
                        if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                        {
                            //rf330_buf[1][2]|=(1<<(0));
                            T19_S = 1;
                            rf330_buf[0][3] = 0;
                            rf330_buf[1][3] = 0;
                            break;
                        } else {
                            T19_S = 0;
                        }
                    }
                }
            }
            j = 0;
            while (RFIN_RF330 && (j < 200)) //???????????
            {
                Delay_us(5);
                j++;
                //j1++;
            }
            head_k = 0;
            while (!RFIN_RF330) //???????????
            {
                Delay_us(5);
                head_k++;
            }
            if (((short_k * 22) < head_k) && (head_k < (short_k * 38)) && (T19_S == 0)) //???????????32?
            {
                for (ii = 0; ii < 3; ii++) //3??
                {
                    for (k = 0; k < 8; k++) //????8?
                    {
                        j = 0;
                        while (RFIN_RF330 && j < 245) {
                            Delay_us(5); //16us(6mhz:2~5)
                            j++;
                        }
                        head_k = 0;
                        while (!RFIN_RF330 && j < 255) //?????
                        {
                            Delay_us(5);
                            head_k++;
                        }
                        if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                            rf330_buf[1][ii] &= ~(1 << ((7 - k)));
                        else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                            rf330_buf[1][ii] |= (1 << (7 - k));
                        else //????
                        {
                            rf330_en = 0;
                            return;
                        }
                    }
                }
                rf330_buf[0][3] = 0;
                rf330_buf[1][3] = 0;
            } else if ((j < (short_k * 5)) && (T19_S == 0)) //T19_L
            {
                k = 0;
                if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                    rf330_buf[0][3] &= ~(1 << ((7 - k)));
                else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                    rf330_buf[0][3] |= (1 << (7 - k));
                else //????
                {
                    rf330_en = 0;
                    return;
                }
                for (k = 1; k < 8; k++) //????8?
                {
                    j = 0;
                    while (RFIN_RF330 && j < 245) {
                        Delay_us(5); //16us(6mhz:2~5)
                        j++;
                    }
                    if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                        rf330_buf[0][3] &= ~(1 << ((7 - k)));
                    else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                        rf330_buf[0][3] |= (1 << (7 - k));
                    else //????
                    {
                        rf330_en = 0;
                        return;
                    }
                    head_k = 0;
                    while (!RFIN_RF330 && j < 255) //?????
                    {
                        Delay_us(5);
                        head_k++;
                    }
                    if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                    {
                        //rf330_buf[1][2]|=(1<<(0));
                        T19_L = 1;
                        // rf330_buf[0][3]=0;
                        //rf330_buf[1][3]=0;
                        break;
                    } else {
                        T19_L = 0;
                    }
                }
                if (T19_L == 1) {
                    for (ii = 0; ii < 4; ii++) //4??
                    {
                        for (k = 0; k < 8; k++) //????8?
                        {
                            j = 0;
                            while (RFIN_RF330 && j < 245) {
                                Delay_us(5); //16us(6mhz:2~5)
                                j++;
                            }
                            if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                                rf330_buf[1][ii] &= ~(1 << ((7 - k)));
                            else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                                rf330_buf[1][ii] |= (1 << (7 - k));
                            else //????
                            {
                                rf330_en = 0;
                                return;
                            }
                            head_k = 0;
                            while (!RFIN_RF330 && j < 255) //?????
                            {
                                Delay_us(5);
                                head_k++;
                            }
                            if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                            {
                                //rf330_buf[1][2]|=(1<<(0));
                                T19_L = 1;
                                // rf330_buf[0][3]=0;
                                //rf330_buf[1][3]=0;
                                break;
                            } else {
                                T19_L = 0;
                            }
                        }
                    }
                }
            }

            //+++++++++++++++++++++++++2262?1527??????++++++++++++++++++++++++++++++++++++++++
            if (T19_S == 1) {
                for (i = 0; i < 3; i++) //??2262?T19S
                {
                    for (u = 0; u < 4; u++) {
                        if (((rf330_buf[0][i] >> (u * 2)) & 3) == 2) {
                            i = 80;
                            break;
                        }
                    } //?10??T19S"H"
                    if (i == 80)
                        break;
                }
                if (i == 80) //T19S
                {
                    // key_d=rf330_buf[1][2] & 0x0f;      //??1527????
                    //rf330_buf[0][2]=rf330_buf[1][2]>>4; 	//??1527??4???
                    key_d = rf330_buf[1][2] & 0x0f;         //??1527????
                    rf330_buf[0][2] = rf330_buf[1][2] >> 4; //??1527??4???
                    jmnx = 2;                               //?0??2262 ,1?T19S
                } else {
                    jmnx = 0;
                }
                rf330_en = 4; //????
                //jmnx = 0;
            } else if (T19_L == 1) {
                key_d = rf330_buf[1][2] >> 1; //??1527????
                key_d = key_d & 0x0f;
                key_d = key_d << 4;
                //rf330_buf[0][2]=key_d; 	//??1527??4???

                jmnx = 3;     //T19_L
                rf330_en = 4; //????
            } else if ((rf330_buf[0][0] == rf330_buf[1][0]) && (rf330_buf[0][1] == rf330_buf[1][1])
                       && (rf330_buf[0][2] == rf330_buf[1][2])) //?????????????
            {
                for (i = 0; i < 4; i++) //??2262?1527
                {
                    for (u = 0; u < 4; u++) {
                        if (((rf330_buf[0][i] >> (u * 2)) & 3) == 2) {
                            i = 80;
                            break;
                        }
                    } //?10??1527"H"
                    if (i == 80)
                        break;
                }
                if (i == 80) //1527
                {
                    key_d = rf330_buf[1][2] & 0x0f;         //??1527????
                    rf330_buf[0][2] = rf330_buf[1][2] >> 4; //??1527??4???
                    jmnx = 1;                               //?0??2262 ,1?1527
                } else                                      //2262
                {
                    key_d = 0;
                    for (i = 0; i < 4; i++) //???? 2262?? ?????
                    {
                        if (((rf330_buf[0][2] >> (i * 2)) & 3) == 3)
                            key_d |= 1 << i;
                    }
                    rf330_buf[0][2] = 0; //2262??4???,??0
                    jmnx = 0;            //?0?2262,1?1527
                }
                rf330_en = 4; //????
            }
        }
    }
    Rec330_short_k = short_k;
    Rec330_jmnx = jmnx;
    Rec330_key_d = key_d;
}

void RF433_IN(void)
{
    unsigned char j = 0;
    unsigned char k = 0;
    unsigned char u, i;
    unsigned int head_k = 0;
    unsigned int short_k = 0;
    unsigned char T19_S;
    unsigned char T19_L;
    unsigned char key_d;
    unsigned char jmnx;
    unsigned char ii = 0;

    if (rf433_en == 0) {
        short_k = 0;
        while (RFIN_RF433 && j < 252) {
            Delay_us(5);
            short_k++;
        }
        while (!RFIN_RF433) {
            Delay_us(5);
            head_k++;
        }
        if (((short_k * 22) < head_k) && (head_k < (short_k * 38))) {
            for (ii = 0; ii < 3; ii++) //3??
            {
                for (k = 0; k < 8; k++) //????8?
                {
                    j = 0;
                    while (RFIN_RF433 && j < 245) {
                        Delay_us(5); //16us(6mhz:2~5)
                        j++;
                    }
                    if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                        rf433_buf[0][ii] &= ~(1 << ((7 - k)));
                    else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                        rf433_buf[0][ii] |= (1 << (7 - k));
                    else //????
                    {
                        rf433_en = 0;
                        return;
                    }
                    head_k = 0;
                    while (!RFIN_RF433 && j < 255) //?????
                    {
                        Delay_us(5);
                        head_k++;
                    }
                    if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                    {
                        //rf433_buf[0][2]|=(1<<(0));
                        T19_S = 1;
                        break;
                    } else {
                        T19_S = 0;
                    }
                }
            }
            if (T19_S == 1) {
                for (ii = 0; ii < 3; ii++) //3??
                {
                    for (k = 0; k < 8; k++) //????8?
                    {
                        j = 0;
                        while (RFIN_RF433 && j < 245) {
                            Delay_us(5); //16us(6mhz:2~5)
                            j++;
                        }
                        if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                            rf433_buf[1][ii] &= ~(1 << ((7 - k)));
                        else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                            rf433_buf[1][ii] |= (1 << (7 - k));
                        else //????
                        {
                            rf433_en = 0;
                            return;
                        }
                        head_k = 0;
                        while (!RFIN_RF433 && j < 255) //?????
                        {
                            Delay_us(5);
                            head_k++;
                        }
                        if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                        {
                            //rf433_buf[1][2]|=(1<<(0));
                            T19_S = 1;
                            rf433_buf[0][3] = 0;
                            rf433_buf[1][3] = 0;
                            break;
                        } else {
                            T19_S = 0;
                        }
                    }
                }
            }
            j = 0;
            while (RFIN_RF433 && (j < 200)) //???????????
            {
                Delay_us(5);
                j++;
                //j1++;
            }
            head_k = 0;
            while (!RFIN_RF433) //???????????
            {
                Delay_us(5);
                head_k++;
            }
            if (((short_k * 22) < head_k) && (head_k < (short_k * 38)) && (T19_S == 0)) //???????????32?
            {
                for (ii = 0; ii < 3; ii++) //3??
                {
                    for (k = 0; k < 8; k++) //????8?
                    {
                        j = 0;
                        while (RFIN_RF433 && j < 245) {
                            Delay_us(5); //16us(6mhz:2~5)
                            j++;
                        }
                        head_k = 0;
                        while (!RFIN_RF433 && j < 255) //?????
                        {
                            Delay_us(5);
                            head_k++;
                        }
                        if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                            rf433_buf[1][ii] &= ~(1 << ((7 - k)));
                        else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                            rf433_buf[1][ii] |= (1 << (7 - k));
                        else //????
                        {
                            rf433_en = 0;
                            return;
                        }
                    }
                }
                rf433_buf[0][3] = 0;
                rf433_buf[1][3] = 0;
            } else if ((j < (short_k * 5)) && (T19_S == 0)) //T19_L
            {
                k = 0;
                if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                    rf433_buf[0][3] &= ~(1 << ((7 - k)));
                else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                    rf433_buf[0][3] |= (1 << (7 - k));
                else //????
                {
                    rf433_en = 0;
                    return;
                }
                for (k = 1; k < 8; k++) //????8?
                {
                    j = 0;
                    while (RFIN_RF433 && j < 245) {
                        Delay_us(5); //16us(6mhz:2~5)
                        j++;
                    }
                    if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                        rf433_buf[0][3] &= ~(1 << ((7 - k)));
                    else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                        rf433_buf[0][3] |= (1 << (7 - k));
                    else //????
                    {
                        rf433_en = 0;
                        return;
                    }
                    head_k = 0;
                    while (!RFIN_RF433 && j < 255) //?????
                    {
                        Delay_us(5);
                        head_k++;
                    }
                    if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                    {
                        //rf433_buf[1][2]|=(1<<(0));
                        T19_L = 1;
                        // rf433_buf[0][3]=0;
                        //rf433_buf[1][3]=0;
                        break;
                    } else {
                        T19_L = 0;
                    }
                }
                if (T19_L == 1) {
                    for (ii = 0; ii < 4; ii++) //4??
                    {
                        for (k = 0; k < 8; k++) //????8?
                        {
                            j = 0;
                            while (RFIN_RF433 && j < 245) {
                                Delay_us(5); //16us(6mhz:2~5)
                                j++;
                            }
                            if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                                rf433_buf[1][ii] &= ~(1 << ((7 - k)));
                            else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                                rf433_buf[1][ii] |= (1 << (7 - k));
                            else //????
                            {
                                rf433_en = 0;
                                return;
                            }
                            head_k = 0;
                            while (!RFIN_RF433 && j < 255) //?????
                            {
                                Delay_us(5);
                                head_k++;
                            }
                            if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                            {
                                //rf433_buf[1][2]|=(1<<(0));
                                T19_L = 1;
                                // rf433_buf[0][3]=0;
                                //rf433_buf[1][3]=0;
                                break;
                            } else {
                                T19_L = 0;
                            }
                        }
                    }
                }
            }

            //+++++++++++++++++++++++++2262?1527??????++++++++++++++++++++++++++++++++++++++++
            if (T19_S == 1) {
                for (i = 0; i < 3; i++) //??2262?T19S
                {
                    for (u = 0; u < 4; u++) {
                        if (((rf433_buf[0][i] >> (u * 2)) & 3) == 2) {
                            i = 80;
                            break;
                        }
                    } //?10??T19S"H"
                    if (i == 80)
                        break;
                }
                if (i == 80) //T19S
                {
                    // key_d=rf433_buf[1][2] & 0x0f;      //??1527????
                    //rf433_buf[0][2]=rf433_buf[1][2]>>4; 	//??1527??4???
                    key_d = rf433_buf[1][2] & 0x0f;         //??1527????
                    rf433_buf[0][2] = rf433_buf[1][2] >> 4; //??1527??4???
                    jmnx = 2;                               //?0??2262 ,1?T19S
                } else {
                    jmnx = 0;
                }
                rf433_en = 4; //????
                //jmnx = 0;
            } else if (T19_L == 1) {
                key_d = rf433_buf[1][2] >> 1; //??1527????
                key_d = key_d & 0x0f;
                key_d = key_d << 4;
                //rf433_buf[0][2]=key_d; 	//??1527??4???

                jmnx = 3;     //T19_L
                rf433_en = 4; //????
            } else if ((rf433_buf[0][0] == rf433_buf[1][0]) && (rf433_buf[0][1] == rf433_buf[1][1])
                       && (rf433_buf[0][2] == rf433_buf[1][2])) //?????????????
            {
                for (i = 0; i < 4; i++) //??2262?1527
                {
                    for (u = 0; u < 4; u++) {
                        if (((rf433_buf[0][i] >> (u * 2)) & 3) == 2) {
                            i = 80;
                            break;
                        }
                    } //?10??1527"H"
                    if (i == 80)
                        break;
                }
                if (i == 80) //1527
                {
                    key_d = rf433_buf[1][2] & 0x0f;         //??1527????
                    rf433_buf[0][2] = rf433_buf[1][2] >> 4; //??1527??4???
                    jmnx = 1;                               //?0??2262 ,1?1527
                } else                                      //2262
                {
                    key_d = 0;
                    for (i = 0; i < 4; i++) //???? 2262?? ?????
                    {
                        if (((rf433_buf[0][2] >> (i * 2)) & 3) == 3)
                            key_d |= 1 << i;
                    }
                    rf433_buf[0][2] = 0; //2262??4???,??0
                    jmnx = 0;            //?0?2262,1?1527
                }
                rf433_en = 4; //????
            }
        }
    }
    Rec433_short_k = short_k;
    Rec433_jmnx = jmnx;
    Rec433_key_d = key_d;
}

void RF430_IN(void)
{
    unsigned char j = 0;
    unsigned char k = 0;
    unsigned char u, i;
    unsigned int head_k = 0;
    unsigned int short_k = 0;
    unsigned char T19_S;
    unsigned char T19_L;
    unsigned char key_d;
    unsigned char jmnx;
    unsigned char ii = 0;

    if (rf430_en == 0) {
        short_k = 0;
        while (RFIN_RF430 && j < 252) {
            Delay_us(5);
            short_k++;
        }
        while (!RFIN_RF430) {
            Delay_us(5);
            head_k++;
        }
        if (((short_k * 22) < head_k) && (head_k < (short_k * 38))) {
            for (ii = 0; ii < 3; ii++) //3??
            {
                for (k = 0; k < 8; k++) //????8?
                {
                    j = 0;
                    while (RFIN_RF430 && j < 245) {
                        Delay_us(5); //16us(6mhz:2~5)
                        j++;
                    }
                    if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                        rf430_buf[0][ii] &= ~(1 << ((7 - k)));
                    else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                        rf430_buf[0][ii] |= (1 << (7 - k));
                    else //????
                    {
                        rf315_en = 0;
                        return;
                    }
                    head_k = 0;
                    while (!RFIN_RF430 && j < 255) //?????
                    {
                        Delay_us(5);
                        head_k++;
                    }
                    if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                    {
                        //rf4xx_buf[0][2]|=(1<<(0));
                        T19_S = 1;
                        break;
                    } else {
                        T19_S = 0;
                    }
                }
            }
            if (T19_S == 1) {
                for (ii = 0; ii < 3; ii++) //3??
                {
                    for (k = 0; k < 8; k++) //????8?
                    {
                        j = 0;
                        while (RFIN_RF430 && j < 245) {
                            Delay_us(5); //16us(6mhz:2~5)
                            j++;
                        }
                        if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                            rf430_buf[1][ii] &= ~(1 << ((7 - k)));
                        else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                            rf430_buf[1][ii] |= (1 << (7 - k));
                        else //????
                        {
                            rf430_en = 0;
                            return;
                        }
                        head_k = 0;
                        while (!RFIN_RF430 && j < 255) //?????
                        {
                            Delay_us(5);
                            head_k++;
                        }
                        if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                        {
                            //rf4xx_buf[1][2]|=(1<<(0));
                            T19_S = 1;
                            rf430_buf[0][3] = 0;
                            rf430_buf[1][3] = 0;
                            break;
                        } else {
                            T19_S = 0;
                        }
                    }
                }
            }
            j = 0;
            while (RFIN_RF430 && (j < 200)) //???????????
            {
                Delay_us(5);
                j++;
                //j1++;
            }
            head_k = 0;
            while (!RFIN_RF430) //???????????
            {
                Delay_us(5);
                head_k++;
            }
            if (((short_k * 22) < head_k) && (head_k < (short_k * 38)) && (T19_S == 0)) //???????????32?
            {
                for (ii = 0; ii < 3; ii++) //3??
                {
                    for (k = 0; k < 8; k++) //????8?
                    {
                        j = 0;
                        while (RFIN_RF430 && j < 245) {
                            Delay_us(5); //16us(6mhz:2~5)
                            j++;
                        }
                        head_k = 0;
                        while (!RFIN_RF430 && j < 255) //?????
                        {
                            Delay_us(5);
                            head_k++;
                        }
                        if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                            rf430_buf[1][ii] &= ~(1 << ((7 - k)));
                        else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                            rf430_buf[1][ii] |= (1 << (7 - k));
                        else //????
                        {
                            rf430_en = 0;
                            return;
                        }
                    }
                }
                rf430_buf[0][3] = 0;
                rf430_buf[1][3] = 0;
            } else if ((j < (short_k * 5)) && (T19_S == 0)) //T19_L
            {
                k = 0;
                if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                    rf430_buf[0][3] &= ~(1 << ((7 - k)));
                else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                    rf430_buf[0][3] |= (1 << (7 - k));
                else //????
                {
                    rf430_en = 0;
                    return;
                }
                for (k = 1; k < 8; k++) //????8?
                {
                    j = 0;
                    while (RFIN_RF430 && j < 245) {
                        Delay_us(5); //16us(6mhz:2~5)
                        j++;
                    }
                    if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                        rf430_buf[0][3] &= ~(1 << ((7 - k)));
                    else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                        rf430_buf[0][3] |= (1 << (7 - k));
                    else //????
                    {
                        rf430_en = 0;
                        return;
                    }
                    head_k = 0;
                    while (!RFIN_RF430 && j < 255) //?????
                    {
                        Delay_us(5);
                        head_k++;
                    }
                    if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                    {
                        //rf430_buf[1][2]|=(1<<(0));
                        T19_L = 1;
                        // rf430_buf[0][3]=0;
                        //rf430_buf[1][3]=0;
                        break;
                    } else {
                        T19_L = 0;
                    }
                }
                if (T19_L == 1) {
                    for (ii = 0; ii < 4; ii++) //4??
                    {
                        for (k = 0; k < 8; k++) //????8?
                        {
                            j = 0;
                            while (RFIN_RF430 && j < 245) {
                                Delay_us(5); //16us(6mhz:2~5)
                                j++;
                            }
                            if (j > (short_k - short_k / 2 - short_k / 3) && j < (short_k * 1.96))
                                rf430_buf[1][ii] &= ~(1 << ((7 - k)));
                            else if (j > (short_k * 1.96) && j < (short_k * 5)) //%25 ?????????3?
                                rf430_buf[1][ii] |= (1 << (7 - k));
                            else //????
                            {
                                rf430_en = 0;
                                return;
                            }
                            head_k = 0;
                            while (!RFIN_RF430 && j < 255) //?????
                            {
                                Delay_us(5);
                                head_k++;
                            }
                            if ((head_k > (short_k * 22)) && (head_k < (short_k * 38))) //23?,T19S
                            {
                                //rf4xx_buf[1][2]|=(1<<(0));
                                T19_L = 1;
                                // rf4xx_buf[0][3]=0;
                                //rf4xx_buf[1][3]=0;
                                break;
                            } else {
                                T19_L = 0;
                            }
                        }
                    }
                }
            }

            //+++++++++++++++++++++++++2262?1527??????++++++++++++++++++++++++++++++++++++++++
            if (T19_S == 1) {
                for (i = 0; i < 3; i++) //??2262?T19S
                {
                    for (u = 0; u < 4; u++) {
                        if (((rf430_buf[0][i] >> (u * 2)) & 3) == 2) {
                            i = 80;
                            break;
                        }
                    } //?10??T19S"H"
                    if (i == 80)
                        break;
                }
                if (i == 80) //T19S
                {
                    // key_d=rf4xx_buf[1][2] & 0x0f;      //??1527????
                    //rf4xx_buf[0][2]=rf4xx_buf[1][2]>>4; 	//??1527??4???
                    key_d = rf430_buf[1][2] & 0x0f;         //??1527????
                    rf430_buf[0][2] = rf430_buf[1][2] >> 4; //??1527??4???
                    jmnx = 2;                               //?0??2262 ,1?T19S
                } else {
                    jmnx = 0;
                }
                rf430_en = 4; //????
                //jmnx = 0;
            } else if (T19_L == 1) {
                key_d = rf430_buf[1][2] >> 1; //??1527????
                key_d = key_d & 0x0f;
                key_d = key_d << 4;
                //rf4xx_buf[0][2]=key_d; 	//??1527??4???
                jmnx = 3;     //T19_L
                rf430_en = 4; //????
            } else if ((rf430_buf[0][0] == rf430_buf[1][0]) && (rf430_buf[0][1] == rf430_buf[1][1])
                       && (rf430_buf[0][2] == rf430_buf[1][2])) //?????????????
            {
                for (i = 0; i < 4; i++) //??2262?1527
                {
                    for (u = 0; u < 4; u++) {
                        if (((rf430_buf[0][i] >> (u * 2)) & 3) == 2) {
                            i = 80;
                            break;
                        }
                    } //?10??1527"H"
                    if (i == 80)
                        break;
                }
                if (i == 80) //1527
                {
                    key_d = rf430_buf[1][2] & 0x0f;         //??1527????
                    rf430_buf[0][2] = rf430_buf[1][2] >> 4; //??1527??4???
                    jmnx = 1;                               //?0??2262 ,1?1527
                } else                                      //2262
                {
                    key_d = 0;
                    for (i = 0; i < 4; i++) //???? 2262?? ?????
                    {
                        if (((rf430_buf[0][2] >> (i * 2)) & 3) == 3)
                            key_d |= 1 << i;
                    }
                    rf430_buf[0][2] = 0; //2262??4???,??0
                    jmnx = 0;            //?0?2262,1?1527
                }
                rf430_en = 4;
            }
        }
    }
    Rec430_short_k = short_k;
    Rec430_jmnx = jmnx;
    Rec430_key_d = key_d;
}
