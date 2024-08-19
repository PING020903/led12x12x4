#include "STC8C.h"
#include "GPIO_init.h"
#include "ShowStrings.h"
#define STR_TEST 1

/*
P0  -->P3.3    |P0.0~P0.3   0x0f    0000 1111
P1  -->P0.0    |P1.0~P1.7   0xff    1111 1111
P2  -->P0.1    |P2.0~P2.7   0xff    1111 1111
P3  -->P0.2    |P3.3~P3.7   0xf8    1111 1000
P4  -->P0.3
P5  -->P3.4
P6  -->P3.5
P7  -->P3.6
P8  -->P3.7
P9  -->P2.0
P10 -->P2.1
P11 -->P2.2
P12 -->P2.3
P13 -->P1.0
P14 -->P1.1
P15 -->P1.2
P16 -->P1.3
P17 -->P1.4
P18 -->P1.5
P19 -->P1.6
P20 -->P1.7
P21 -->P2.4
P22 -->P2.5
P23 -->P2.6
P24 -->P2.7
*/
#if 0
{0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, },
{0x40,0x00,0x40,0x00,0x40,0x00,0x40,0x00,},/*"一",0*/


{0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00,},
{0x40,0x00,0x40,0x00,0x40,0x00,0x40,0x00,},/*"一",0*/
{0x80,0x08,0xFE,0x04,0xAB,0x02,0xAA,0x01,0xFE,0x0F,0x00,0x00,0xF9,0x0F,0x0A,0x00,},
{0xE8,0x03,0x2F,0x02,0xE8,0x03,0x0A,0x00,},/*"躺",1*/
{0x91,0x08,0x99,0x08,0x95,0x08,0xF3,0x0F,0x91,0x04,0x95,0x04,0xB9,0x04,0x00,0x00,},
{0xFC,0x01,0x00,0x00,0x00,0x00,0xFF,0x0F,},/*"到",2*/
{0x00,0x00,0xFE,0x0F,0x02,0x00,0xF2,0x0F,0x92,0x08,0x92,0x04,0x92,0x00,0x93,0x08,},
{0xF2,0x01,0x8A,0x06,0x8A,0x08,0x8A,0x00,},/*"底",3*/
#endif
#if !STR_TEST
static unsigned char hello_cn[4][12] = {
    {0xBB, 0xFF, 0x1B, 0xF0, 0xD9, 0xF7, 0xC8, 0xF6, 0xEA, 0xFE, 0xFB, 0xFE,},
    {0xBB, 0xF2, 0xDB, 0xF6, 0xCB, 0xF6, 0xEB, 0xFE, 0xFB, 0xFE, 0x3B, 0xFE,},/*"你",0*/
    {0x3B, 0xF0, 0xFB, 0xF7, 0xC0, 0xFB, 0xC9, 0xFD, 0xED, 0xFD, 0x2D, 0xF0,},
    {0xED, 0xFD, 0xF1, 0xFD, 0xF7, 0xFD, 0xEB, 0xFD, 0xDD, 0xFD, 0x7E, 0xFC,}/*"好",1*/
};
#else
static unsigned char hello_cn[][12] = {
{0x20, 0x00, 0x27, 0xc0, 0x20, 0x40, 0xf8, 0x80, 0x49, 0x00, 0x49, 0x00},
{0x4f, 0xe0, 0x91, 0x00, 0x51, 0x00, 0x21, 0x00, 0x51, 0x00, 0x8b, 0x00}
};
#endif // STR_TEST


#if 0
/// @brief 更改IO状态
/// @param strData 被解析的字符点阵行数据
/// @param electricityIO 供电的第几个IO( 0~24 )
/// @note 调用时无需考虑IO模式转换, 但要记得传入的 strData 必须要处理供电Pin(x)的状态
void changeIO_status(LED_un strData, const unsigned char electricityIO)
{
    /* 先取消供电 */
    P0 = ~USE_P0;
    P1 = ~USE_P1;
    P2 = ~USE_P2;
    P3 = ~USE_P3;

    switch ( electricityIO )
    {
        case 0:
            GPIO_setOpenDrainOutput(3, 3);//  P0
            GPIO_setPushPullupOutput(0, 0);// P1
            break;
        case 1:
            GPIO_setOpenDrainOutput(0, 0);//  P1
            GPIO_setPushPullupOutput(0, 1);// P2
            break;
        case 2:
            GPIO_setOpenDrainOutput(0, 1);//  P2
            GPIO_setPushPullupOutput(0, 2);// P3
            break;
        case 3:
            GPIO_setOpenDrainOutput(0, 2);//  P3
            GPIO_setPushPullupOutput(0, 3);// P4
            break;
        case 4:
            GPIO_setOpenDrainOutput(0, 3);//  P4
            GPIO_setPushPullupOutput(3, 4);// P5
            break;
        case 5:
            GPIO_setOpenDrainOutput(3, 4);//  P5
            GPIO_setPushPullupOutput(3, 5);// P6
            break;
        case 6:
            GPIO_setOpenDrainOutput(3, 5);//  P6
            GPIO_setPushPullupOutput(3, 6);// P7
            break;
        case 7:
            GPIO_setOpenDrainOutput(3, 6);//  P7
            GPIO_setPushPullupOutput(3, 7);// P8
            break;
        case 8:
            GPIO_setOpenDrainOutput(3, 7);//  P8
            GPIO_setPushPullupOutput(2, 0);// P9
            break;
        case 9:
            GPIO_setOpenDrainOutput(2, 0);//  P9
            GPIO_setPushPullupOutput(2, 1);//P10
            break;
        case 10:
            GPIO_setOpenDrainOutput(2, 1);// P10
            GPIO_setPushPullupOutput(2, 2);//P11
            break;
        case 11:
            GPIO_setOpenDrainOutput(2, 2);// P11
            GPIO_setPushPullupOutput(2, 3);//P12
            break;
        case 12:
            GPIO_setOpenDrainOutput(2, 3);// P12
            GPIO_setPushPullupOutput(1, 0);//P13
            break;
        case 13:
            GPIO_setOpenDrainOutput(1, 0);// P13
            GPIO_setPushPullupOutput(1, 1);//P14
            break;
        case 14:
            GPIO_setOpenDrainOutput(1, 1);// P14
            GPIO_setPushPullupOutput(1, 2);//P15
            break;
        case 15:
            GPIO_setOpenDrainOutput(1, 2);// P15
            GPIO_setPushPullupOutput(1, 3);//P16
            break;
        case 16:
            GPIO_setOpenDrainOutput(1, 3);// P16
            GPIO_setPushPullupOutput(1, 4);//P17
            break;
        case 17:
            GPIO_setOpenDrainOutput(1, 4);// P17
            GPIO_setPushPullupOutput(1, 5);//P18
            break;
        case 18:
            GPIO_setOpenDrainOutput(1, 5);// P18
            GPIO_setPushPullupOutput(1, 6);//P19
            break;
        case 19:
            GPIO_setOpenDrainOutput(1, 6);// P19
            GPIO_setPushPullupOutput(1, 7);//P20
            break;
        case 20:
            GPIO_setOpenDrainOutput(1, 7);// P20
            GPIO_setPushPullupOutput(2, 4);//P21
            break;
        case 21:
            GPIO_setOpenDrainOutput(2, 4);// P21
            GPIO_setPushPullupOutput(2, 5);//P22
            break;
        case 22:
            GPIO_setOpenDrainOutput(2, 5);// P22
            GPIO_setPushPullupOutput(2, 6);//P23
            break;
        case 23:
            GPIO_setOpenDrainOutput(2, 6);// P23
            GPIO_setPushPullupOutput(2, 7);//P24
            break;
        case 24:
            GPIO_setOpenDrainOutput(2, 7);// P24
            GPIO_setPushPullupOutput(3, 3);// P0
            break;
        default:
            return;
    }
    P33 = strData.PortPin.P0;

    P00 = strData.PortPin.P1;
    P01 = strData.PortPin.P2;
    P02 = strData.PortPin.P3;
    P03 = strData.PortPin.P4;

    P34 = strData.PortPin.P5;
    P35 = strData.PortPin.P6;
    P36 = strData.PortPin.P7;
    P37 = strData.PortPin.P8;

    P20 = strData.PortPin.P9;
    P21 = strData.PortPin.P10;
    P22 = strData.PortPin.P11;
    P23 = strData.PortPin.P12;

    P10 = strData.PortPin.P13;
    P11 = strData.PortPin.P14;
    P12 = strData.PortPin.P15;
    P13 = strData.PortPin.P16;
    P14 = strData.PortPin.P17;
    P15 = strData.PortPin.P18;
    P16 = strData.PortPin.P19;
    P17 = strData.PortPin.P20;

    P24 = strData.PortPin.P21;
    P25 = strData.PortPin.P22;
    P26 = strData.PortPin.P23;
    P27 = strData.PortPin.P24;

    return;
}
#endif // 0



char** hello_cn_str(void)
{
    return hello_cn;
}

#if 0
{
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xAB, 0xAB, 0xAB, 0xD7, 0xD7, 0xD7, 0xFF
},/*"w",0*/
1111 1111
1111 1111
1111 1111
1111 1111
1111 1111
0 0  0
0 0  0
0 0  0
0  0
0  0
0  0
1111 1111

{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBB, 0xBB, 0xD7, 0xD7, 0xEF, 0xEF, 0x9F},/*"y",1*/

{0xFF,0xDF,0xAF,0xAF,0xDF,0xDF,0xAB,0xAB,0xB7,0xB7,0xCB,0xFF},/*"&",2*/

{0xFF,0xFF,0xC7,0xBB,0xBB,0xBB,0xBB,0xBB,0xAB,0xB7,0xCB,0xFF},/*"Q",3*/

{0xFF,0xFF,0xC7,0xBB,0xBB,0xBB,0xBB,0xBB,0xAB,0xB7,0xCB,0xFF},/*"Q",4*/

{0xBB,0xFF,0x1B,0xF0,0xD9,0xF7,0xC8,0xF6,0xEA,0xFE,0xFB,0xFE,},
{0xBB,0xF2,0xDB,0xF6,0xCB,0xF6,0xEB,0xFE,0xFB,0xFE,0x3B,0xFE,},/*"你",0*/
{0x3B,0xF0,0xFB,0xF7,0xC0,0xFB,0xC9,0xFD,0xED,0xFD,0x2D,0xF0,},
{0xED,0xFD,0xF1,0xFD,0xF7,0xFD,0xEB,0xFD,0xDD,0xFD,0x7E,0xFC,},/*"好",1*/
#endif
