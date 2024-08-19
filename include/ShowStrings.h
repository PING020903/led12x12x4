#ifndef _SHOWSTRINGS_H_
#define _SHOWSTRINGS_H_

#define MAX_BYTES (4)
#define MAX_ARR_LEN (12)

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
// 由于更改了Pin的排序, 故此选择更改结构体部分结构
typedef struct
{
    unsigned char L0_P33 : 1;
    unsigned char L1_P00 : 1; // 按位操作,低位
    unsigned char L2_P01 : 1; // P2占1bit
    unsigned char L3_P02 : 1;
    unsigned char L4_P03 : 1;
    unsigned char L5_P34 : 1;
    unsigned char L6_P35 : 1;
    unsigned char L7_P36 : 1;
    unsigned char L8_P37 : 1;
    unsigned char L9_P20 : 1;
    unsigned char L10_P21 : 1;
    unsigned char L11_P22 : 1;
    unsigned char L12_P23 : 1;
    unsigned char L13_P10 : 1;
    unsigned char L14_P11 : 1;
    unsigned char L15_P12 : 1;
    unsigned char L16_P13 : 1;
    unsigned char L17_P14 : 1;
    unsigned char L18_P15 : 1;
    unsigned char L19_P16 : 1;
    unsigned char L20_P17 : 1;
    unsigned char L21_P24 : 1;
    unsigned char L22_P25 : 1;
    unsigned char L23_P26 : 1;
    unsigned char L24_P27 : 1;
    unsigned char NullPin2 : 7; // <--( 更改了此处占位 )空位不用到, 到此为止係32bit, 4bytes大小
} PORT_status;
///////////
typedef union
{
    PORT_status PortPin;// 共占32bit, 即为4bytes
    unsigned long seg;  // 整体操作, long 占4bytes, 与PortPin长度对齐
} LED_un;

void changeIO_status(LED_un strData, const unsigned char electricityIO);

char** hello_cn_str(void);

#endif // !_SHOWSTRINGS_H_
