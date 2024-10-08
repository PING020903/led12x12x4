#ifndef     __STC8C_H__
#define     __STC8C_H__

/////////////////////////////////////////////////

//"REG51.H"

sfr         P0          =           0x80;       // PORT 0
sbit        P00         =           P0^0;       // PORT 0 Pin 0
sbit        P01         =           P0^1;       // PORT 0 Pin 1
sbit        P02         =           P0^2;       // PORT 0 Pin 2
sbit        P03         =           P0^3;       // PORT 0 Pin 3
sbit        P04         =           P0^4;       // PORT 0 Pin 4
sbit        P05         =           P0^5;       // PORT 0 Pin 5
sbit        P06         =           P0^6;       // PORT 0 Pin 6
sbit        P07         =           P0^7;       // PORT 0 Pin 7
sfr         SP          =           0x81;
sfr         DPL         =           0x82;
sfr         DPH         =           0x83;
sfr         PCON        =           0x87;       // 电源控制寄存器
sfr         TCON        =           0x88;       // 定时器控制寄存器
sbit        TF1         =           TCON^7;
sbit        TR1         =           TCON^6;
sbit        TF0         =           TCON^5;
sbit        TR0         =           TCON^4;
sbit        IE1         =           TCON^3;
sbit        IT1         =           TCON^2;
sbit        IE0         =           TCON^1;
sbit        IT0         =           TCON^0;
sfr         TMOD        =           0x89;       // 定时器模式寄存器
sfr         TL0         =           0x8A;       // 定时器0(low 8bit)
sfr         TL1         =           0x8B;       // 定时器1(low 8bit)
sfr         TH0         =           0x8C;       // 定时器0(high 8bit)
sfr         TH1         =           0x8D;       // 定时器1(high 8bit)
sfr         AUXR        =           0x8E;       // 辅助寄存器1
sfr         INTCLKO     =           0x8F;       // 中断与时钟输出控制寄存器
sfr         P1          =           0x90;       // PORT 1
sbit        P10         =           P1^0;       // PORT 1 Pin 0
sbit        P11         =           P1^1;       // PORT 1 Pin 1
sbit        P12         =           P1^2;       // PORT 1 Pin 2
sbit        P13         =           P1^3;       // PORT 1 Pin 3
sbit        P14         =           P1^4;       // PORT 1 Pin 4
sbit        P15         =           P1^5;       // PORT 1 Pin 5
sbit        P16         =           P1^6;       // PORT 1 Pin 6
sbit        P17         =           P1^7;       // PORT 1 Pin 7
sfr         P1M1        =           0x91;       // P1配置寄存器1
sfr         P1M0        =           0x92;       // P1配置寄存器0
sfr         P0M1        =           0x93;       // P0配置寄存器1
sfr         P0M0        =           0x94;       // P0配置寄存器0
sfr         P2M1        =           0x95;       // P2配置寄存器1
sfr         P2M0        =           0x96;       // P2配置寄存器0
sfr         SCON        =           0x98;       // 串口1控制寄存器
sbit        SM0         =           SCON^7;
sbit        SM1         =           SCON^6;
sbit        SM2         =           SCON^5;
sbit        REN         =           SCON^4;     // 串口1接收使能
sbit        TB8         =           SCON^3;
sbit        RB8         =           SCON^2;
sbit        TI          =           SCON^1;     // 串口1发送中断请求标志
sbit        RI          =           SCON^0;     // 串口1接收中断请求标志
sfr         SBUF        =           0x99;       // 串口1数据寄存器
sfr         S2CON       =           0x9A;       // 串口2控制寄存器
sfr         S2BUF       =           0x9B;       // 串口2数据寄存器
sfr         IRCBAND     =           0x9D;
sfr         LIRTRIM     =           0x9E;
sfr         IRTRIM      =           0x9F;
sfr         P2          =           0xA0;       // PORT 2
sbit        P20         =           P2^0;       // PORT 2 Pin 0
sbit        P21         =           P2^1;       // PORT 2 Pin 1
sbit        P22         =           P2^2;       // PORT 2 Pin 2
sbit        P23         =           P2^3;       // PORT 2 Pin 3
sbit        P24         =           P2^4;       // PORT 2 Pin 4
sbit        P25         =           P2^5;       // PORT 2 Pin 5
sbit        P26         =           P2^6;       // PORT 2 Pin 6
sbit        P27         =           P2^7;       // PORT 2 Pin 7
sfr         BUS_SPEED   =           0xA1;
sfr         P_SW1       =           0xA2;
sfr         IE          =           0xA8;       // 中断允许寄存器
sbit        EA          =           IE^7;       // 总中断允许控制位
sbit        ELVD        =           IE^6;       // 低电压检测中断允许位
sbit        EADC        =           IE^5;       
sbit        ES          =           IE^4;       // UART1中断允许位
sbit        ET1         =           IE^3;       // timer1中断允许位
sbit        EX1         =           IE^2;       // 外部中断1中断允许位
sbit        ET0         =           IE^1;       // timer0中断允许位
sbit        EX0         =           IE^0;       // 外部中断0中断允许位
sfr         SADDR       =           0xA9;       // 串口1从机地址寄存器
sfr         WKTCL       =           0xAA;       // 掉电唤醒定时器(low 8bit)
sfr         WKTCH       =           0xAB;       // 掉电唤醒定时器(high 7bit)
sfr         TA          =           0xAE;
sfr         IE2         =           0xAF;       // 中断允许寄存器2
sfr         P3          =           0xB0;       // PORT 3
sbit        P30         =           P3^0;       // PORT 3 Pin 0
sbit        P31         =           P3^1;       // PORT 3 Pin 1
sbit        P32         =           P3^2;       // PORT 3 Pin 2
sbit        P33         =           P3^3;       // PORT 3 Pin 3
sbit        P34         =           P3^4;       // PORT 3 Pin 4
sbit        P35         =           P3^5;       // PORT 3 Pin 5
sbit        P36         =           P3^6;       // PORT 3 Pin 6
sbit        P37         =           P3^7;       // PORT 3 Pin 7
sfr         P3M1        =           0xB1;       // P3配置寄存器1
sfr         P3M0        =           0xB2;       // P3配置寄存器0
sfr         P4M1        =           0xB3;       // P4配置寄存器1
sfr         P4M0        =           0xB4;       // P4配置寄存器0
sfr         IP2         =           0xB5;
sfr         IP2H        =           0xB6;
sfr         IPH         =           0xB7;
sfr         IP          =           0xB8;
sbit        PLVD        =           IP^6;
sbit        PS          =           IP^4;
sbit        PT1         =           IP^3;
sbit        PX1         =           IP^2;
sbit        PT0         =           IP^1;
sbit        PX0         =           IP^0;
sfr         SADEN       =           0xB9;       // 串口1从机地址屏蔽寄存器
sfr         P_SW2       =           0xBA;
sfr         P4          =           0xC0;       // PORT 4
sbit        P40         =           P4^0;       // PORT 4 Pin 0
sbit        P41         =           P4^1;       // PORT 4 Pin 1
sbit        P42         =           P4^2;       // PORT 4 Pin 2
sbit        P43         =           P4^3;       // PORT 4 Pin 3
sbit        P44         =           P4^4;       // PORT 4 Pin 4
sbit        P45         =           P4^5;       // PORT 4 Pin 5
sbit        P46         =           P4^6;       // PORT 4 Pin 6
sbit        P47         =           P4^7;       // PORT 4 Pin 7
sfr         WDT_CONTR   =           0xC1;
sfr         IAP_DATA    =           0xC2;
sfr         IAP_ADDRH   =           0xC3;
sfr         IAP_ADDRL   =           0xC4;
sfr         IAP_CMD     =           0xC5;
sfr         IAP_TRIG    =           0xC6;
sfr         IAP_CONTR   =           0xC7;
sfr         P5          =           0xC8;       // PORT 5
sbit        P50         =           P5^0;       // PORT 5 Pin 0
sbit        P51         =           P5^1;       // PORT 5 Pin 1
sbit        P52         =           P5^2;       // PORT 5 Pin 2
sbit        P53         =           P5^3;       // PORT 5 Pin 3
sbit        P54         =           P5^4;       // PORT 5 Pin 4
sbit        P55         =           P5^5;       // PORT 5 Pin 5
sbit        P56         =           P5^6;       // PORT 5 Pin 6
sbit        P57         =           P5^7;       // PORT 5 Pin 7
sfr         P5M1        =           0xC9;
sfr         P5M0        =           0xCA;
sfr         SPSTAT      =           0xCD;
sfr         SPCTL       =           0xCE;
sfr         SPDAT       =           0xCF;
sfr         PSW         =           0xD0;
sbit        CY          =           PSW^7;
sbit        AC          =           PSW^6;
sbit        F0          =           PSW^5;
sbit        RS1         =           PSW^4;
sbit        RS0         =           PSW^3;
sbit        OV          =           PSW^2;
sbit        F1          =           PSW^1;
sbit        P           =           PSW^0;
sfr         T4T3M       =           0xD1;
sfr         T4H         =           0xD2;
sfr         T4L         =           0xD3;
sfr         T3H         =           0xD4;
sfr         T3L         =           0xD5;
sfr         T2H         =           0xD6;
sfr         T2L         =           0xD7;
sfr         IP3         =           0xDF;
sfr         ACC         =           0xE0;
sfr         DPS         =           0xE3;
sfr         DPL1        =           0xE4;
sfr         DPH1        =           0xE5;
sfr         CMPCR1      =           0xE6;
sfr         CMPCR2      =           0xE7;
sfr         IP3H        =           0xEE;
sfr         AUXINTIF    =           0xEF;
sfr         B           =           0xF0;
sfr         IAP_TPS     =           0xF5;
sfr         RSTCFG      =           0xFF;




//ÈçÏÂÌØÊ⹦ÄܼĴæÆ÷λÓÚ)չRAMÇøÓò
//·ÃÎÊÕâЩ¼ĴæÆ÷,ÐèÏȽ«P_SW2µÄBIT7ÉèÖÃΪ1,²ſÉÕ?Áд

/////////////////////////////////////////////////
//FF00H-FFFFH
/////////////////////////////////////////////////


/////////////////////////////////////////////////
//FE00H-FEFFH
/////////////////////////////////////////////////


#define     CLKSEL      (*(unsigned char volatile xdata *)0xfe00)   // 时钟选择寄存器
#define     CLKDIV      (*(unsigned char volatile xdata *)0xfe01)   // 时钟分频寄存器
#define     HIRCCR      (*(unsigned char volatile xdata *)0xfe02)   // 高速晶体振荡器校准寄存器
#define     XOSCCR      (*(unsigned char volatile xdata *)0xfe03)   // 外部振荡器校准寄存器
#define     IRC32KCR    (*(unsigned char volatile xdata *)0xfe04)   // IRC 32KHz 振荡器校准寄存器
#define     MCLKOCR     (*(unsigned char volatile xdata *)0xfe05)   // 主时钟输出控制寄存器
#define     IRCDB       (*(unsigned char volatile xdata *)0xfe06)   // IRC 调节寄存器

// GPIO 端口上拉电阻配置寄存器
#define     P0PU        (*(unsigned char volatile xdata *)0xfe10)
#define     P1PU        (*(unsigned char volatile xdata *)0xfe11)
#define     P2PU        (*(unsigned char volatile xdata *)0xfe12)
#define     P3PU        (*(unsigned char volatile xdata *)0xfe13)
#define     P4PU        (*(unsigned char volatile xdata *)0xfe14)
#define     P5PU        (*(unsigned char volatile xdata *)0xfe15)

// GPIO 端口施密特触发控制寄存器
#define     P0NCS       (*(unsigned char volatile xdata *)0xfe18)
#define     P1NCS       (*(unsigned char volatile xdata *)0xfe19)
#define     P2NCS       (*(unsigned char volatile xdata *)0xfe1a)
#define     P3NCS       (*(unsigned char volatile xdata *)0xfe1b)
#define     P4NCS       (*(unsigned char volatile xdata *)0xfe1c)
#define     P5NCS       (*(unsigned char volatile xdata *)0xfe1d)

// GPIO 端口电平转换速度控制寄存器
#define     P0SR        (*(unsigned char volatile xdata *)0xfe20)
#define     P1SR        (*(unsigned char volatile xdata *)0xfe21)
#define     P2SR        (*(unsigned char volatile xdata *)0xfe22)
#define     P3SR        (*(unsigned char volatile xdata *)0xfe23)
#define     P4SR        (*(unsigned char volatile xdata *)0xfe24)
#define     P5SR        (*(unsigned char volatile xdata *)0xfe25)

// GPIO 端口电流控制寄存器
#define     P0DR        (*(unsigned char volatile xdata *)0xfe28)
#define     P1DR        (*(unsigned char volatile xdata *)0xfe29)
#define     P2DR        (*(unsigned char volatile xdata *)0xfe2a)
#define     P3DR        (*(unsigned char volatile xdata *)0xfe2b)
#define     P4DR        (*(unsigned char volatile xdata *)0xfe2c)
#define     P5DR        (*(unsigned char volatile xdata *)0xfe2d)

// GPIO 端口中断使能寄存器
#define     P0IE        (*(unsigned char volatile xdata *)0xfe30)
#define     P1IE        (*(unsigned char volatile xdata *)0xfe31)

// I2C 配置和控制寄存器
#define     I2CCFG      (*(unsigned char volatile xdata *)0xfe80)
#define     I2CMSCR     (*(unsigned char volatile xdata *)0xfe81)
#define     I2CMSST     (*(unsigned char volatile xdata *)0xfe82)
#define     I2CSLCR     (*(unsigned char volatile xdata *)0xfe83)
#define     I2CSLST     (*(unsigned char volatile xdata *)0xfe84)
#define     I2CSLADR    (*(unsigned char volatile xdata *)0xfe85)
#define     I2CTXD      (*(unsigned char volatile xdata *)0xfe86)
#define     I2CRXD      (*(unsigned char volatile xdata *)0xfe87)
#define     I2CMSAUX    (*(unsigned char volatile xdata *)0xfe88)

// 定时器预分频寄存器
#define     TM2PS       (*(unsigned char volatile xdata *)0xfea2)
#define     TM3PS       (*(unsigned char volatile xdata *)0xfea3)
#define     TM4PS       (*(unsigned char volatile xdata *)0xfea4)

/////////////////////////////////////////////////
//FD00H-FDFFH
/////////////////////////////////////////////////

// GPIO 端口中断使能寄存器
#define     P0INTE      (*(unsigned char volatile xdata *)0xfd00)
#define     P1INTE      (*(unsigned char volatile xdata *)0xfd01)
#define     P2INTE      (*(unsigned char volatile xdata *)0xfd02)
#define     P3INTE      (*(unsigned char volatile xdata *)0xfd03)
#define     P4INTE      (*(unsigned char volatile xdata *)0xfd04)
#define     P5INTE      (*(unsigned char volatile xdata *)0xfd05)

// GPIO 端口中断标志寄存器
#define     P0INTF      (*(unsigned char volatile xdata *)0xfd10)
#define     P1INTF      (*(unsigned char volatile xdata *)0xfd11)
#define     P2INTF      (*(unsigned char volatile xdata *)0xfd12)
#define     P3INTF      (*(unsigned char volatile xdata *)0xfd13)
#define     P4INTF      (*(unsigned char volatile xdata *)0xfd14)
#define     P5INTF      (*(unsigned char volatile xdata *)0xfd15)

// GPIO 端口中断模式0寄存器
#define     P0IM0       (*(unsigned char volatile xdata *)0xfd20)
#define     P1IM0       (*(unsigned char volatile xdata *)0xfd21)
#define     P2IM0       (*(unsigned char volatile xdata *)0xfd22)
#define     P3IM0       (*(unsigned char volatile xdata *)0xfd23)
#define     P4IM0       (*(unsigned char volatile xdata *)0xfd24)
#define     P5IM0       (*(unsigned char volatile xdata *)0xfd25)

// GPIO 端口中断模式1寄存器
#define     P0IM1       (*(unsigned char volatile xdata *)0xfd30)
#define     P1IM1       (*(unsigned char volatile xdata *)0xfd31)
#define     P2IM1       (*(unsigned char volatile xdata *)0xfd32)
#define     P3IM1       (*(unsigned char volatile xdata *)0xfd33)
#define     P4IM1       (*(unsigned char volatile xdata *)0xfd34)
#define     P5IM1       (*(unsigned char volatile xdata *)0xfd35)

/////////////////////////////////////////////////
//FC00H-FCFFH
/////////////////////////////////////////////////

// 多路复用数字输入端口寄存器
#define     MD3         (*(unsigned char volatile xdata *)0xfcf0)
#define     MD2         (*(unsigned char volatile xdata *)0xfcf1)
#define     MD1         (*(unsigned char volatile xdata *)0xfcf2)
#define     MD0         (*(unsigned char volatile xdata *)0xfcf3)
#define     MD5         (*(unsigned char volatile xdata *)0xfcf4)
#define     MD4         (*(unsigned char volatile xdata *)0xfcf5)

// 模拟复用器控制和ADC控制寄存器
#define     ARCON       (*(unsigned char volatile xdata *)0xfcf6)
#define     OPCON       (*(unsigned char volatile xdata *)0xfcf7)

/////////////////////////////////////////////////
//FB00H-FBFFH
/////////////////////////////////////////////////


/////////////////////////////////////////////////
//FA00H-FAFFH
/////////////////////////////////////////////////


/////////////////////////////////////////////////

#endif

