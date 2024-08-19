#include "STC8C.h"
#include "stdio.h"
#include "GPIO_init.h"

#define PORT_RESET (unsigned char)(0XFF)
#define IO_M0_RESET (unsigned char)(0x00)
#define IO_M1_RESET (unsigned char)(0x00)
#define IO_SPEED_RESET (unsigned char)(0XFF)
#define IO_TURBO_RESET (unsigned char)(0XFF)

#define DEBUG_INTO_STACK 1
#define SREEN_DEBUG 0
#define LED_DEBUG 1

#define CONFIG_DEBUG 0

static bit busy;  // 发送缓冲区是否忙
static char wptr; // 写指针
static char rptr; // 读指针
static char buffer[16];

// UART1中断服务程序, 中断等级4
static void UartIsr(void) interrupt 4
{
    if ( TI )
    {
        TI = 0;
        busy = 0;
    }
    if ( RI )
    {
        RI = 0;
        buffer[wptr++] = SBUF;
        wptr &= 0x0f;
    }
}

/// @brief 设置推挽输出
/// @param port 端口
/// @param pin 引脚
void GPIO_setPushPullupOutput(const unsigned char port, const unsigned char pin)
{
    unsigned char fr_M0 = IO_M0_RESET;
    unsigned char fr_M1 = IO_M1_RESET;
    if ( port > 5 || pin > 7 )
        return;

    switch ( port ) // 根据 port读取不同的寄存器
    {
        case 0:
            fr_M0 = P0M0;
            fr_M1 = P0M1;
            break;
        case 1:
            fr_M0 = P1M0;
            fr_M1 = P1M1;
            break;
        case 2:
            fr_M0 = P2M0;
            fr_M1 = P2M1;
            break;
        case 3:
            fr_M0 = P3M0;
            fr_M1 = P3M1;
            break;
        case 4:
            fr_M0 = P4M0;
            fr_M1 = P4M1;
            break;
        case 5:
            fr_M0 = P5M0;
            fr_M1 = P5M1;
            break;
        default:
            return;
    }
    BIT_SET_ONE(fr_M0, pin);
    BIT_SET_ZERO(fr_M1, pin);

    switch ( port ) // 根据 port读取不同的寄存器
    {
        case 0:
            P0M0 = fr_M0;
            P0M1 = fr_M1;
            break;
        case 1:
            P1M0 = fr_M0;
            P1M1 = fr_M1;
            break;
        case 2:
            P2M0 = fr_M0;
            P2M1 = fr_M1;
            break;
        case 3:
            P3M0 = fr_M0;
            P3M1 = fr_M1;
            break;
        case 4:
            P4M0 = fr_M0;
            P4M1 = fr_M1;
            break;
        case 5:
            P5M0 = fr_M0;
            P5M1 = fr_M1;
            break;
        default:
            return;
    }
    return;
}

/// @brief 设置开漏输出
/// @param port 端口
/// @param pin 引脚
void GPIO_setOpenDrainOutput(const unsigned char port, const unsigned char pin)
{
    unsigned char fr_M0 = IO_M0_RESET;
    unsigned char fr_M1 = IO_M1_RESET;
    if ( port > 5 || pin > 7 )
        return;

    switch ( port ) // 根据 port读取不同的寄存器
    {
        case 0:
            fr_M0 = P0M0;
            fr_M1 = P0M1;
            break;
        case 1:
            fr_M0 = P1M0;
            fr_M1 = P1M1;
            break;
        case 2:
            fr_M0 = P2M0;
            fr_M1 = P2M1;
            break;
        case 3:
            fr_M0 = P3M0;
            fr_M1 = P3M1;
            break;
        case 4:
            fr_M0 = P4M0;
            fr_M1 = P4M1;
            break;
        case 5:
            fr_M0 = P5M0;
            fr_M1 = P5M1;
            break;
        default:
            return;
    }
    BIT_SET_ONE(fr_M0, pin);
    BIT_SET_ONE(fr_M1, pin);

    switch ( port ) // 根据 port读取不同的寄存器
    {
        case 0:
            P0M0 = fr_M0;
            P0M1 = fr_M1;
            break;
        case 1:
            P1M0 = fr_M0;
            P1M1 = fr_M1;
            break;
        case 2:
            P2M0 = fr_M0;
            P2M1 = fr_M1;
            break;
        case 3:
            P3M0 = fr_M0;
            P3M1 = fr_M1;
            break;
        case 4:
            P4M0 = fr_M0;
            P4M1 = fr_M1;
            break;
        case 5:
            P5M0 = fr_M0;
            P5M1 = fr_M1;
            break;
        default:
            return;
    }
    return;
}

/// @brief 设置标准双向模式
/// @param port
/// @param pin
void GPIO_setStandardBidirectional(const unsigned char port, const unsigned char pin)
{
    unsigned char fr_M0 = IO_M0_RESET;
    unsigned char fr_M1 = IO_M1_RESET;
    if ( port > 5 || pin > 7 )
        return;

    switch ( port ) // 根据 port读取不同的寄存器
    {
        case 0:
            fr_M0 = P0M0;
            fr_M1 = P0M1;
            break;
        case 1:
            fr_M0 = P1M0;
            fr_M1 = P1M1;
            break;
        case 2:
            fr_M0 = P2M0;
            fr_M1 = P2M1;
            break;
        case 3:
            fr_M0 = P3M0;
            fr_M1 = P3M1;
            break;
        case 4:
            fr_M0 = P4M0;
            fr_M1 = P4M1;
            break;
        case 5:
            fr_M0 = P5M0;
            fr_M1 = P5M1;
            break;
        default:
            return;
    }
    BIT_SET_ZERO(fr_M0, pin);
    BIT_SET_ZERO(fr_M1, pin);

    switch ( port ) // 根据 port读取不同的寄存器
    {
        case 0:
            P0M0 = fr_M0;
            P0M1 = fr_M1;
            break;
        case 1:
            P1M0 = fr_M0;
            P1M1 = fr_M1;
            break;
        case 2:
            P2M0 = fr_M0;
            P2M1 = fr_M1;
            break;
        case 3:
            P3M0 = fr_M0;
            P3M1 = fr_M1;
            break;
        case 4:
            P4M0 = fr_M0;
            P4M1 = fr_M1;
            break;
        case 5:
            P5M0 = fr_M0;
            P5M1 = fr_M1;
            break;
        default:
            return;
    }
    return;
}

/// @brief 设置高阻输入
/// @param port
/// @param pin
void GPIO_setHighResistanceInput(const unsigned char port, const unsigned char pin)
{
    unsigned char fr_M0 = IO_M0_RESET;
    unsigned char fr_M1 = IO_M1_RESET;
    if ( port > 5 || pin > 7 )
        return;

    switch ( port ) // 根据 port读取不同的寄存器
    {
        case 0:
            fr_M0 = P0M0;
            fr_M1 = P0M1;
            break;
        case 1:
            fr_M0 = P1M0;
            fr_M1 = P1M1;
            break;
        case 2:
            fr_M0 = P2M0;
            fr_M1 = P2M1;
            break;
        case 3:
            fr_M0 = P3M0;
            fr_M1 = P3M1;
            break;
        case 4:
            fr_M0 = P4M0;
            fr_M1 = P4M1;
            break;
        case 5:
            fr_M0 = P5M0;
            fr_M1 = P5M1;
            break;
        default:
            return;
    }
    BIT_SET_ZERO(fr_M0, pin);
    BIT_SET_ONE(fr_M1, pin);

    switch ( port ) // 根据 port读取不同的寄存器
    {
        case 0:
            P0M0 = fr_M0;
            P0M1 = fr_M1;
            break;
        case 1:
            P1M0 = fr_M0;
            P1M1 = fr_M1;
            break;
        case 2:
            P2M0 = fr_M0;
            P2M1 = fr_M1;
            break;
        case 3:
            P3M0 = fr_M0;
            P3M1 = fr_M1;
            break;
        case 4:
            P4M0 = fr_M0;
            P4M1 = fr_M1;
            break;
        case 5:
            P5M0 = fr_M0;
            P5M1 = fr_M1;
            break;
        default:
            return;
    }
    return;
}

/// @brief 设置上拉电阻
/// @param port 端口
/// @param pin 引脚
/// @param en 0 关闭, 1 打开
void GPIO_setPullup(const unsigned char port,
                    const unsigned char pin,
                    const unsigned char en)
{
    unsigned char fr_status;
    switch ( en )
    {
        case 0:
            switch ( port )
            {
                case 0:
                    fr_status = P0PU;
                    BIT_SET_ZERO(fr_status, pin);
                    P0PU = fr_status;
                    break;
                case 1:
                    fr_status = P1PU;
                    BIT_SET_ZERO(fr_status, pin);
                    P1PU = fr_status;
                    break;
                case 2:
                    fr_status = P2PU;
                    BIT_SET_ZERO(fr_status, pin);
                    P2PU = fr_status;
                    break;
                case 3:
                    fr_status = P3PU;
                    BIT_SET_ZERO(fr_status, pin);
                    P3PU = fr_status;
                    break;
                case 4:
                    fr_status = P4PU;
                    BIT_SET_ZERO(fr_status, pin);
                    P4PU = fr_status;
                    break;
                case 5:
                    fr_status = P5PU;
                    BIT_SET_ZERO(fr_status, pin);
                    P5PU = fr_status;
                    break;
                default:
                    break;
            }
        default:
            switch ( port )
            {
                case 0:
                    fr_status = P0PU;
                    BIT_SET_ONE(fr_status, pin);
                    P0PU = fr_status;
                    break;
                case 1:
                    fr_status = P1PU;
                    BIT_SET_ONE(fr_status, pin);
                    P1PU = fr_status;
                    break;
                case 2:
                    fr_status = P2PU;
                    BIT_SET_ONE(fr_status, pin);
                    P2PU = fr_status;
                    break;
                case 3:
                    fr_status = P3PU;
                    BIT_SET_ONE(fr_status, pin);
                    P3PU = fr_status;
                    break;
                case 4:
                    fr_status = P4PU;
                    BIT_SET_ONE(fr_status, pin);
                    P4PU = fr_status;
                    break;
                case 5:
                    fr_status = P5PU;
                    BIT_SET_ONE(fr_status, pin);
                    P5PU = fr_status;
                    break;
                default:
                    break;
            }
    }
    return;
}

/// @brief 设置 Schmidt(施耐德)触发
/// @param port 端口
/// @param pin 引脚
/// @param en 0 打开, 1 关闭
/// @note 上电复位后默认使能
void GPIO_setSchmidtTrigger(const unsigned char port,
                            const unsigned char pin,
                            const unsigned char en)
{
    unsigned char fr_status;
    switch ( en )
    {
        case 0:
            switch ( port )
            {
                case 0:
                    fr_status = P0NCS;
                    BIT_SET_ZERO(fr_status, pin);
                    P0NCS = fr_status;
                    break;
                case 1:
                    fr_status = P1NCS;
                    BIT_SET_ZERO(fr_status, pin);
                    P1NCS = fr_status;
                    break;
                case 2:
                    fr_status = P2NCS;
                    BIT_SET_ZERO(fr_status, pin);
                    P2NCS = fr_status;
                    break;
                case 3:
                    fr_status = P3NCS;
                    BIT_SET_ZERO(fr_status, pin);
                    P3NCS = fr_status;
                    break;
                case 4:
                    fr_status = P4NCS;
                    BIT_SET_ZERO(fr_status, pin);
                    P4NCS = fr_status;
                    break;
                case 5:
                    fr_status = P5NCS;
                    BIT_SET_ZERO(fr_status, pin);
                    P5NCS = fr_status;
                    break;
                default:
                    break;
            }
        default:
            switch ( port )
            {
                case 0:
                    fr_status = P0NCS;
                    BIT_SET_ONE(fr_status, pin);
                    P0NCS = fr_status;
                    break;
                case 1:
                    fr_status = P1NCS;
                    BIT_SET_ONE(fr_status, pin);
                    P1NCS = fr_status;
                    break;
                case 2:
                    fr_status = P2NCS;
                    BIT_SET_ONE(fr_status, pin);
                    P2NCS = fr_status;
                    break;
                case 3:
                    fr_status = P3NCS;
                    BIT_SET_ONE(fr_status, pin);
                    P3NCS = fr_status;
                    break;
                case 4:
                    fr_status = P4NCS;
                    BIT_SET_ONE(fr_status, pin);
                    P4NCS = fr_status;
                    break;
                case 5:
                    fr_status = P5NCS;
                    BIT_SET_ONE(fr_status, pin);
                    P5NCS = fr_status;
                    break;
                default:
                    break;
            }
    }
    return;
}

/// @brief 设置电平转换速度控制
/// @param port 端口
/// @param pin 引脚
/// @param en 0 快, 1 慢
void GPIO_setLevelShiftingSpeed(const unsigned char port,
                                const unsigned char pin,
                                const unsigned char en)
{
    unsigned char fr_status;
    switch ( en )
    {
        case 0:
            switch ( port )
            {
                case 0:
                    fr_status = P0SR;
                    BIT_SET_ZERO(fr_status, pin);
                    P0SR = fr_status;
                    break;
                case 1:
                    fr_status = P1SR;
                    BIT_SET_ZERO(fr_status, pin);
                    P1SR = fr_status;
                    break;
                case 2:
                    fr_status = P2SR;
                    BIT_SET_ZERO(fr_status, pin);
                    P2SR = fr_status;
                    break;
                case 3:
                    fr_status = P3SR;
                    BIT_SET_ZERO(fr_status, pin);
                    P3SR = fr_status;
                    break;
                case 4:
                    fr_status = P4SR;
                    BIT_SET_ZERO(fr_status, pin);
                    P4SR = fr_status;
                    break;
                case 5:
                    fr_status = P5SR;
                    BIT_SET_ZERO(fr_status, pin);
                    P5SR = fr_status;
                    break;
                default:
                    break;
            }
        default:
            switch ( port )
            {
                case 0:
                    fr_status = P0SR;
                    BIT_SET_ONE(fr_status, pin);
                    P0SR = fr_status;
                    break;
                case 1:
                    fr_status = P1SR;
                    BIT_SET_ONE(fr_status, pin);
                    P1SR = fr_status;
                    break;
                case 2:
                    fr_status = P2SR;
                    BIT_SET_ONE(fr_status, pin);
                    P2SR = fr_status;
                    break;
                case 3:
                    fr_status = P3SR;
                    BIT_SET_ONE(fr_status, pin);
                    P3SR = fr_status;
                    break;
                case 4:
                    fr_status = P4SR;
                    BIT_SET_ONE(fr_status, pin);
                    P4SR = fr_status;
                    break;
                case 5:
                    fr_status = P5SR;
                    BIT_SET_ONE(fr_status, pin);
                    P5SR = fr_status;
                    break;
                default:
                    break;
            }
    }
    return;
}

/// @brief 设置驱动电流能力
/// @param port 端口
/// @param pin 引脚
/// @param en 0 一般, 1 增强
void GPIO_setDriveCurrent(const unsigned char port,
                          const unsigned char pin,
                          const unsigned char en)
{
    unsigned char fr_status;
    switch ( en )
    {
        case 0:
            switch ( port )
            {
                case 0:
                    fr_status = P0DR;
                    BIT_SET_ZERO(fr_status, pin);
                    P0DR = fr_status;
                    break;
                case 1:
                    fr_status = P1DR;
                    BIT_SET_ZERO(fr_status, pin);
                    P1DR = fr_status;
                    break;
                case 2:
                    fr_status = P2DR;
                    BIT_SET_ZERO(fr_status, pin);
                    P2DR = fr_status;
                    break;
                case 3:
                    fr_status = P3DR;
                    BIT_SET_ZERO(fr_status, pin);
                    P3DR = fr_status;
                    break;
                case 4:
                    fr_status = P4DR;
                    BIT_SET_ZERO(fr_status, pin);
                    P4DR = fr_status;
                    break;
                case 5:
                    fr_status = P5DR;
                    BIT_SET_ZERO(fr_status, pin);
                    P5DR = fr_status;
                    break;
                default:
                    break;
            }
        default:
            switch ( port )
            {
                case 0:
                    fr_status = P0DR;
                    BIT_SET_ONE(fr_status, pin);
                    P0DR = fr_status;
                    break;
                case 1:
                    fr_status = P1DR;
                    BIT_SET_ONE(fr_status, pin);
                    P1DR = fr_status;
                    break;
                case 2:
                    fr_status = P2DR;
                    BIT_SET_ONE(fr_status, pin);
                    P2DR = fr_status;
                    break;
                case 3:
                    fr_status = P3DR;
                    BIT_SET_ONE(fr_status, pin);
                    P3DR = fr_status;
                    break;
                case 4:
                    fr_status = P4DR;
                    BIT_SET_ONE(fr_status, pin);
                    P4DR = fr_status;
                    break;
                case 5:
                    fr_status = P5DR;
                    BIT_SET_ONE(fr_status, pin);
                    P5DR = fr_status;
                    break;
                default:
                    break;
            }
    }
    return;
}

/// @brief Uart1初始化
/// @param
/// @note 使用 timer1作为波特率生成器
void UartInit(void) // 115200bps@11.0592MHz
{
    /*
        串口四种模式:
        1. mode 0: 同步位移串行
        2. mode 1: 可变 baudrate 8bits
        3. mode 2: 固定 baudrate 9bits
        4. mode 3: 可变 baudrate 8bits
    */

#if CONFIG_DEBUG
    SCON = 0x50;  // 8 bits and variable baudrate
    AUXR |= 0x40; // imer clock is 1T mode
    AUXR &= 0xFE; // UART 1 use Timer1 as baudrate generator
    TMOD &= 0x0F; // Set timer work mode
#endif
#if !CONFIG_DEBUG
    unsigned char fr_status;
    GPIO_setStandardBidirectional(3, 0);
    GPIO_setStandardBidirectional(3, 1);

    fr_status = SCON;
    BIT_SET_ONE(fr_status, 4); // enable uart receive
    BIT_SET_ONE(fr_status, 6); // set mode1 of UART
    SCON = fr_status;

    fr_status = AUXR;
    BIT_SET_ONE(fr_status, 6);  // timer clock is 1T mode
    BIT_SET_ZERO(fr_status, 0); // UART 1 use Timer1 as baudrate generator
    AUXR = fr_status;

    fr_status = TMOD;
    BIT_SET_ZERO(fr_status, 5); // 8bit auto reload( timer work mode )
    TMOD = fr_status;
#endif
    TL1 = BRT;      // Initial timer value
    TH1 = BRT >> 8; // Initial timer value
    ET1 = 0;        // Disable Timer%d interrupt
    ES = 1;         // Enable UART1 interrupt
    EA = 1;         // Enable global interrupt
    TR1 = 1;        // Timer1 start run

    wptr = 0X00;
    rptr = 0X00;
    busy = 0;
}

/// @brief Uart发送一个字节
/// @param ch
void UartSendByte(unsigned char ch)
{
    while ( busy )
        ; // 等待发送缓冲区空闲, 否则一直等待
    busy = 1;
    SBUF = ch; // 给( 写缓冲区 )写 one byte
}

/// @brief Uart发送一个字符串
/// @param str
void UartSendString(const char* str)
{
    while ( *str )
    {
        UartSendByte(*str++);
    }
}


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

/// @brief GPIO初始化
/// @param
void GPIO_init(void)
{
    unsigned char output_turbo = IO_TURBO_RESET;
    unsigned char IO_speed = IO_SPEED_RESET;
    // P1 P0.0
    // P0 P3.3

    P0 = PORT_RESET;    // IO复位
    P0M0 = IO_M0_RESET; // 设为高阻输入
    P0M1 = ~IO_M1_RESET;

    P1 = PORT_RESET;
    P1M0 = IO_M0_RESET;
    P1M1 = ~IO_M1_RESET;

    P2 = PORT_RESET;
    P2M0 = IO_M0_RESET;
    P2M1 = ~IO_M1_RESET;

    P3 = PORT_RESET;
    P3M0 = IO_M0_RESET;
    P3M1 = ~IO_M1_RESET;

    output_turbo = ~IO_TURBO_RESET; // 增强驱动能力
    P1DR = output_turbo;
    P2DR = output_turbo;
    output_turbo = (unsigned char)~USE_P0;
    P0DR = output_turbo;
    output_turbo = (unsigned char)~USE_P3;
    P3DR = output_turbo;

    IO_speed = ~IO_SPEED_RESET; // 电平转换快
    P1SR = IO_speed;
    P2SR = IO_speed;
    IO_speed = (unsigned char)~USE_P0;
    P0SR = IO_speed;
    IO_speed = (unsigned char)~USE_P3;
    P3SR = IO_speed;

    GPIO_setOpenDrainOutput(3, 3);// P0
    GPIO_setOpenDrainOutput(0, 0);// P1
    GPIO_setOpenDrainOutput(0, 1);// P2
    GPIO_setOpenDrainOutput(0, 2);// P3
    GPIO_setOpenDrainOutput(0, 3);// P4
    GPIO_setOpenDrainOutput(3, 4);// P5
    GPIO_setOpenDrainOutput(3, 5);// P6
    GPIO_setOpenDrainOutput(3, 6);// P7
    GPIO_setOpenDrainOutput(3, 7);// P8
    GPIO_setOpenDrainOutput(2, 0);// P9
    GPIO_setOpenDrainOutput(2, 1);// P10
    GPIO_setOpenDrainOutput(2, 2);// P11
    GPIO_setOpenDrainOutput(2, 3);// P12
    GPIO_setOpenDrainOutput(1, 0);// P13
    GPIO_setOpenDrainOutput(1, 1);// P14
    GPIO_setOpenDrainOutput(1, 2);// P15
    GPIO_setOpenDrainOutput(1, 3);// P16
    GPIO_setOpenDrainOutput(1, 4);// P17
    GPIO_setOpenDrainOutput(1, 5);// P18
    GPIO_setOpenDrainOutput(1, 6);// P19
    GPIO_setOpenDrainOutput(1, 7);// P20
    GPIO_setOpenDrainOutput(2, 4);// P21
    GPIO_setOpenDrainOutput(2, 5);// P22
    GPIO_setOpenDrainOutput(2, 6);// P23
    GPIO_setOpenDrainOutput(2, 7);// P24

    return;
}

void LED_off(void)
{
    GPIO_setOpenDrainOutput(3, 3);// P0
    GPIO_setOpenDrainOutput(0, 0);// P1
    GPIO_setOpenDrainOutput(0, 1);// P2
    GPIO_setOpenDrainOutput(0, 2);// P3
    GPIO_setOpenDrainOutput(0, 3);// P4
    GPIO_setOpenDrainOutput(3, 4);// P5
    GPIO_setOpenDrainOutput(3, 5);// P6
    GPIO_setOpenDrainOutput(3, 6);// P7
    GPIO_setOpenDrainOutput(3, 7);// P8
    GPIO_setOpenDrainOutput(2, 0);// P9
    GPIO_setOpenDrainOutput(2, 1);// P10
    GPIO_setOpenDrainOutput(2, 2);// P11
    GPIO_setOpenDrainOutput(2, 3);// P12
    GPIO_setOpenDrainOutput(1, 0);// P13
    GPIO_setOpenDrainOutput(1, 1);// P14
    GPIO_setOpenDrainOutput(1, 2);// P15
    GPIO_setOpenDrainOutput(1, 3);// P16
    GPIO_setOpenDrainOutput(1, 4);// P17
    GPIO_setOpenDrainOutput(1, 5);// P18
    GPIO_setOpenDrainOutput(1, 6);// P19
    GPIO_setOpenDrainOutput(1, 7);// P20
    GPIO_setOpenDrainOutput(2, 4);// P21
    GPIO_setOpenDrainOutput(2, 5);// P22
    GPIO_setOpenDrainOutput(2, 6);// P23
    GPIO_setOpenDrainOutput(2, 7);// P24

    P0 = USE_P0;
    P1 = USE_P1;
    P2 = USE_P2;
    P3 = USE_P3;
}

#if LED_DEBUG
/// @brief LED 全开
/// @param LED_line
/// @note 当主循环中插入字符串输出时, 会有明显闪烁, 但亮度较高
void LED_allON(unsigned char LED_line)
{
    switch ( LED_line )
    {
        case 0:
            GPIO_setOpenDrainOutput(3, 3);//  P0
            GPIO_setPushPullupOutput(0, 0);// P1
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P27 = 0;
            P00 = 1;
            break;
        case 1:
            GPIO_setOpenDrainOutput(0, 0);//  P1
            GPIO_setPushPullupOutput(0, 1);// P2
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P00 = 0;
            P01 = 1;
            break;
        case 2:
            GPIO_setOpenDrainOutput(0, 1);//  P2
            GPIO_setPushPullupOutput(0, 2);// P3
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P01 = 0;
            P02 = 1;
            break;
        case 3:
            GPIO_setOpenDrainOutput(0, 2);//  P3
            GPIO_setPushPullupOutput(0, 3);// P4
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P02 = 0;
            P03 = 1;
            break;
        case 4:
            GPIO_setOpenDrainOutput(0, 3);//  P4
            GPIO_setPushPullupOutput(3, 4);// P5
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P03 = 0;
            P34 = 1;
            break;
        case 5:
            GPIO_setOpenDrainOutput(3, 4);//  P5
            GPIO_setPushPullupOutput(3, 5);// P6
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P34 = 0;
            P35 = 1;
            break;
        case 6:
            GPIO_setOpenDrainOutput(3, 5);//  P6
            GPIO_setPushPullupOutput(3, 6);// P7
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P35 = 0;
            P36 = 1;
            break;
        case 7:
            GPIO_setOpenDrainOutput(3, 6);//  P7
            GPIO_setPushPullupOutput(3, 7);// P8
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P36 = 0;
            P37 = 1;
            break;
        case 8:
            GPIO_setOpenDrainOutput(3, 7);//  P8
            GPIO_setPushPullupOutput(2, 0);// P9
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P37 = 0;
            P20 = 1;
            break;
        case 9:
            GPIO_setOpenDrainOutput(2, 0);//  P9
            GPIO_setPushPullupOutput(2, 1);//P10
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P20 = 0;
            P21 = 1;
            break;
        case 10:
            GPIO_setOpenDrainOutput(2, 1);// P10
            GPIO_setPushPullupOutput(2, 2);//P11
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P21 = 0;
            P22 = 1;
            break;
        case 11:
            GPIO_setOpenDrainOutput(2, 2);// P11
            GPIO_setPushPullupOutput(2, 3);//P12
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P22 = 0;
            P23 = 1;
            break;
        case 12:
            GPIO_setOpenDrainOutput(2, 3);// P12
            GPIO_setPushPullupOutput(1, 0);//P13
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P23 = 0;
            P10 = 1;
            break;
        case 13:
            GPIO_setOpenDrainOutput(1, 0);// P13
            GPIO_setPushPullupOutput(1, 1);//P14
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P10 = 0;
            P11 = 1;
            break;
        case 14:
            GPIO_setOpenDrainOutput(1, 1);// P14
            GPIO_setPushPullupOutput(1, 2);//P15
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P11 = 0;
            P12 = 1;
            break;
        case 15:
            GPIO_setOpenDrainOutput(1, 2);// P15
            GPIO_setPushPullupOutput(1, 3);//P16
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P12 = 0;
            P13 = 1;
            break;
        case 16:
            GPIO_setOpenDrainOutput(1, 3);// P16
            GPIO_setPushPullupOutput(1, 4);//P17
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P13 = 0;
            P14 = 1;
            break;
        case 17:
            GPIO_setOpenDrainOutput(1, 4);// P17
            GPIO_setPushPullupOutput(1, 5);//P18
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P14 = 0;
            P15 = 1;
            break;
        case 18:
            GPIO_setOpenDrainOutput(1, 5);// P18
            GPIO_setPushPullupOutput(1, 6);//P19
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P15 = 0;
            P16 = 1;
            break;
        case 19:
            GPIO_setOpenDrainOutput(1, 6);// P19
            GPIO_setPushPullupOutput(1, 7);//P20
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P16 = 0;
            P17 = 1;
            break;
        case 20:
            GPIO_setOpenDrainOutput(1, 7);// P20
            GPIO_setPushPullupOutput(2, 4);//P21
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P17 = 0;
            P24 = 1;
            break;
        case 21:
            GPIO_setOpenDrainOutput(2, 4);// P21
            GPIO_setPushPullupOutput(2, 5);//P22
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P24 = 0;
            P25 = 1;
            break;
        case 22:
            GPIO_setOpenDrainOutput(2, 5);// P22
            GPIO_setPushPullupOutput(2, 6);//P23
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P25 = 0;
            P26 = 1;
            break;
        case 23:
            GPIO_setOpenDrainOutput(2, 6);// P23
            GPIO_setPushPullupOutput(2, 7);//P24
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P26 = 0;
            P27 = 1;
            break;
        case 24:
            GPIO_setOpenDrainOutput(2, 7);// P24
            GPIO_setPushPullupOutput(3, 3);// P0
            P2 = ~USE_P2;
            P1 = ~USE_P1;
            P0 &= ~USE_P0;
            P3 &= ~USE_P3;
            P27 = 0;
            P33 = 1;
            break;

        default:
            return;
    }

}

/// @brief LED 全开测试
/// @param  
/// @note 当主循环中插入字符串输出时, 没有有明显闪烁, 但亮度较低
void LED_allON_test(void)
{
    GPIO_setOpenDrainOutput(3, 3);//  P0
    GPIO_setPushPullupOutput(0, 0);// P1
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P27 = 0;
    P00 = 1;

    GPIO_setOpenDrainOutput(0, 0);//  P1
    GPIO_setPushPullupOutput(0, 1);// P2
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P00 = 0;
    P01 = 1;

    GPIO_setOpenDrainOutput(0, 1);//  P2
    GPIO_setPushPullupOutput(0, 2);// P3
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P01 = 0;
    P02 = 1;

    GPIO_setOpenDrainOutput(0, 2);//  P3
    GPIO_setPushPullupOutput(0, 3);// P4
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P02 = 0;
    P03 = 1;

    GPIO_setOpenDrainOutput(0, 3);//  P4
    GPIO_setPushPullupOutput(3, 4);// P5
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P03 = 0;
    P34 = 1;

    GPIO_setOpenDrainOutput(3, 4);//  P5
    GPIO_setPushPullupOutput(3, 5);// P6
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P34 = 0;
    P35 = 1;

    GPIO_setOpenDrainOutput(3, 5);//  P6
    GPIO_setPushPullupOutput(3, 6);// P7
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P35 = 0;
    P36 = 1;

    GPIO_setOpenDrainOutput(3, 6);//  P7
    GPIO_setPushPullupOutput(3, 7);// P8
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P36 = 0;
    P37 = 1;

    GPIO_setOpenDrainOutput(3, 7);//  P8
    GPIO_setPushPullupOutput(2, 0);// P9
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P37 = 0;
    P20 = 1;

    GPIO_setOpenDrainOutput(2, 0);//  P9
    GPIO_setPushPullupOutput(2, 1);//P10
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P20 = 0;
    P21 = 1;

    GPIO_setOpenDrainOutput(2, 1);// P10
    GPIO_setPushPullupOutput(2, 2);//P11
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P21 = 0;
    P22 = 1;

    GPIO_setOpenDrainOutput(2, 2);// P11
    GPIO_setPushPullupOutput(2, 3);//P12
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P22 = 0;
    P23 = 1;

    GPIO_setOpenDrainOutput(2, 3);// P12
    GPIO_setPushPullupOutput(1, 0);//P13
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P23 = 0;
    P10 = 1;

    GPIO_setOpenDrainOutput(1, 0);// P13
    GPIO_setPushPullupOutput(1, 1);//P14
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P10 = 0;
    P11 = 1;

    GPIO_setOpenDrainOutput(1, 1);// P14
    GPIO_setPushPullupOutput(1, 2);//P15
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P11 = 0;
    P12 = 1;

    GPIO_setOpenDrainOutput(1, 2);// P15
    GPIO_setPushPullupOutput(1, 3);//P16
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P12 = 0;
    P13 = 1;

    GPIO_setOpenDrainOutput(1, 3);// P16
    GPIO_setPushPullupOutput(1, 4);//P17
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P13 = 0;
    P14 = 1;

    GPIO_setOpenDrainOutput(1, 4);// P17
    GPIO_setPushPullupOutput(1, 5);//P18
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P14 = 0;
    P15 = 1;

    GPIO_setOpenDrainOutput(1, 5);// P18
    GPIO_setPushPullupOutput(1, 6);//P19
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P15 = 0;
    P16 = 1;

    GPIO_setOpenDrainOutput(1, 6);// P19
    GPIO_setPushPullupOutput(1, 7);//P20
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P16 = 0;
    P17 = 1;

    GPIO_setOpenDrainOutput(1, 7);// P20
    GPIO_setPushPullupOutput(2, 4);//P21
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P17 = 0;
    P24 = 1;

    GPIO_setOpenDrainOutput(2, 4);// P21
    GPIO_setPushPullupOutput(2, 5);//P22
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P24 = 0;
    P25 = 1;

    GPIO_setOpenDrainOutput(2, 5);// P22
    GPIO_setPushPullupOutput(2, 6);//P23
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P25 = 0;
    P26 = 1;

    GPIO_setOpenDrainOutput(2, 6);// P23
    GPIO_setPushPullupOutput(2, 7);//P24
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P26 = 0;
    P27 = 1;

    GPIO_setOpenDrainOutput(2, 7);// P24
    GPIO_setPushPullupOutput(3, 3);// P0
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P27 = 0;
    P33 = 1;

    return;

}

void LED_leftON_test(void)
{
    GPIO_setOpenDrainOutput(3, 3);//  P0
    GPIO_setPushPullupOutput(0, 0);// P1
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P27 = 0;
    P00 = 1;

    GPIO_setOpenDrainOutput(0, 0);//  P1
    GPIO_setPushPullupOutput(0, 1);// P2
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P00 = 0;
    P01 = 1;

    GPIO_setOpenDrainOutput(0, 1);//  P2
    GPIO_setPushPullupOutput(0, 2);// P3
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P01 = 0;
    P02 = 1;

    GPIO_setOpenDrainOutput(0, 2);//  P3
    GPIO_setPushPullupOutput(0, 3);// P4
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P02 = 0;
    P03 = 1;

    GPIO_setOpenDrainOutput(0, 3);//  P4
    GPIO_setPushPullupOutput(3, 4);// P5
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P03 = 0;
    P34 = 1;

    GPIO_setOpenDrainOutput(3, 4);//  P5
    GPIO_setPushPullupOutput(3, 5);// P6
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P34 = 0;
    P35 = 1;

    GPIO_setOpenDrainOutput(3, 5);//  P6
    GPIO_setPushPullupOutput(3, 6);// P7
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P35 = 0;
    P36 = 1;

    GPIO_setOpenDrainOutput(3, 6);//  P7
    GPIO_setPushPullupOutput(3, 7);// P8
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P36 = 0;
    P37 = 1;

    GPIO_setOpenDrainOutput(3, 7);//  P8
    GPIO_setPushPullupOutput(2, 0);// P9
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P37 = 0;
    P20 = 1;

    GPIO_setOpenDrainOutput(2, 0);//  P9
    GPIO_setPushPullupOutput(2, 1);//P10
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P20 = 0;
    P21 = 1;

    GPIO_setOpenDrainOutput(2, 1);// P10
    GPIO_setPushPullupOutput(2, 2);//P11
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P21 = 0;
    P22 = 1;

    GPIO_setOpenDrainOutput(2, 2);// P11
    GPIO_setPushPullupOutput(2, 3);//P12
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P22 = 0;
    P23 = 1;

    return;
}

void LED_rightON_test(void)
{
    GPIO_setOpenDrainOutput(2, 3);// P12
    GPIO_setPushPullupOutput(1, 0);//P13
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P23 = 0;
    P10 = 1;

    GPIO_setOpenDrainOutput(1, 0);// P13
    GPIO_setPushPullupOutput(1, 1);//P14
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P10 = 0;
    P11 = 1;

    GPIO_setOpenDrainOutput(1, 1);// P14
    GPIO_setPushPullupOutput(1, 2);//P15
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P11 = 0;
    P12 = 1;

    GPIO_setOpenDrainOutput(1, 2);// P15
    GPIO_setPushPullupOutput(1, 3);//P16
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P12 = 0;
    P13 = 1;

    GPIO_setOpenDrainOutput(1, 3);// P16
    GPIO_setPushPullupOutput(1, 4);//P17
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P13 = 0;
    P14 = 1;

    GPIO_setOpenDrainOutput(1, 4);// P17
    GPIO_setPushPullupOutput(1, 5);//P18
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P14 = 0;
    P15 = 1;

    GPIO_setOpenDrainOutput(1, 5);// P18
    GPIO_setPushPullupOutput(1, 6);//P19
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P15 = 0;
    P16 = 1;

    GPIO_setOpenDrainOutput(1, 6);// P19
    GPIO_setPushPullupOutput(1, 7);//P20
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P16 = 0;
    P17 = 1;

    GPIO_setOpenDrainOutput(1, 7);// P20
    GPIO_setPushPullupOutput(2, 4);//P21
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P17 = 0;
    P24 = 1;

    GPIO_setOpenDrainOutput(2, 4);// P21
    GPIO_setPushPullupOutput(2, 5);//P22
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P24 = 0;
    P25 = 1;

    GPIO_setOpenDrainOutput(2, 5);// P22
    GPIO_setPushPullupOutput(2, 6);//P23
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P25 = 0;
    P26 = 1;

    GPIO_setOpenDrainOutput(2, 6);// P23
    GPIO_setPushPullupOutput(2, 7);//P24
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P26 = 0;
    P27 = 1;

    GPIO_setOpenDrainOutput(2, 7);// P24
    GPIO_setPushPullupOutput(3, 3);// P0
    P2 = ~USE_P2;
    P1 = ~USE_P1;
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P27 = 0;
    P33 = 1;

    return;
}

void LED_twoBytes_test(void)
{
    P0 &= ~USE_P0;                 // enable P0.0~P0.3
    P1 = USE_P1;                   // disable P1.0~P1.7
    P2 &= ~(USE_P2 >> 4);          // enable P2.0~P2.3, disable P2.4~P2.7

    GPIO_setOpenDrainOutput(3, 3);//  P0
    GPIO_setPushPullupOutput(0, 0);// P1
    P0 &= ~USE_P0;                 // enable P0.0~P0.3
    P2 &= ~(USE_P2 >> 4);           // enable P2.0~P2.3, disable P2.4~P2.7
    P3 &= ~USE_P3;                 // enable P3.3~P3.7
    P00 = 1;

    GPIO_setOpenDrainOutput(0, 0);//  P1
    GPIO_setPushPullupOutput(0, 1);// P2
    P0 &= ~USE_P0;
    P2 &= ~(USE_P2 >> 4);           // enable P2.0~P2.3, disable P2.4~P2.7
    P3 &= ~USE_P3;
    P01 = 1;

    GPIO_setOpenDrainOutput(0, 1);//  P2
    GPIO_setPushPullupOutput(0, 2);// P3
    P0 &= ~USE_P0;
    P2 &= ~(USE_P2 >> 4);           // enable P2.0~P2.3, disable P2.4~P2.7
    P3 &= ~USE_P3;
    P02 = 1;

    GPIO_setOpenDrainOutput(0, 2);//  P3
    GPIO_setPushPullupOutput(0, 3);// P4
    P0 &= ~USE_P0;
    P2 &= ~(USE_P2 >> 4);           // enable P2.0~P2.3, disable P2.4~P2.7
    P3 &= ~USE_P3;
    P03 = 1;

    GPIO_setOpenDrainOutput(0, 3);//  P4
    GPIO_setPushPullupOutput(3, 4);// P5
    P0 &= ~USE_P0;
    P2 &= ~(USE_P2 >> 4);           // enable P2.0~P2.3, disable P2.4~P2.7
    P3 &= ~USE_P3;
    P34 = 1;

    GPIO_setOpenDrainOutput(3, 4);//  P5
    GPIO_setPushPullupOutput(3, 5);// P6
    P0 &= ~USE_P0;
    P2 &= ~(USE_P2 >> 4);           // enable P2.0~P2.3, disable P2.4~P2.7
    P3 &= ~USE_P3;
    P35 = 1;

    GPIO_setOpenDrainOutput(3, 5);//  P6
    GPIO_setPushPullupOutput(3, 6);// P7
    P0 &= ~USE_P0;
    P2 &= ~(USE_P2 >> 4);           // enable P2.0~P2.3, disable P2.4~P2.7
    P3 &= ~USE_P3;
    P36 = 1;

    GPIO_setOpenDrainOutput(3, 6);//  P7
    GPIO_setPushPullupOutput(3, 7);// P8
    P0 &= ~USE_P0;
    P2 &= ~(USE_P2 >> 4);           // enable P2.0~P2.3, disable P2.4~P2.7
    P3 &= ~USE_P3;
    P37 = 1;

    GPIO_setOpenDrainOutput(3, 7);//  P8
    GPIO_setPushPullupOutput(2, 0);// P9
    P0 &= ~USE_P0;
    P2 &= ~(USE_P2 >> 4);           // enable P2.0~P2.3, disable P2.4~P2.7
    P3 &= ~USE_P3;
    P20 = 1;

    GPIO_setOpenDrainOutput(2, 0);//  P9
    GPIO_setPushPullupOutput(2, 1);//P10
    P2 &= ~(USE_P2 >> 4);           // enable P2.0~P2.3, disable P2.4~P2.7
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P21 = 1;

    GPIO_setOpenDrainOutput(2, 1);// P10
    GPIO_setPushPullupOutput(2, 2);//P11
    P2 &= ~(USE_P2 >> 4);           // enable P2.0~P2.3, disable P2.4~P2.7
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P22 = 1;

    GPIO_setOpenDrainOutput(2, 2);// P11
    GPIO_setPushPullupOutput(2, 3);//P12
    P2 &= ~(USE_P2 >> 4);           // enable P2.0~P2.3, disable P2.4~P2.7
    P0 &= ~USE_P0;
    P3 &= ~USE_P3;
    P23 = 1;

    GPIO_setOpenDrainOutput(2, 3);
    P23 = 0;
    return;
}

void LED_twoBytes_test1(void)
{
    GPIO_setHighResistanceInput(0, 0);
    GPIO_setOpenDrainOutput(3, 3);
    P2 = ~(USE_P2 << 4);// enable P2.4~P2.7
    P1 = ~USE_P1;       // enable P1.0~P1.7
    P33 = 1;

    GPIO_setOpenDrainOutput(3, 3);//  P0
    GPIO_setPushPullupOutput(0, 0);// P1
    P0 |= USE_P0;       // disable P0.0~P0.3
    P3 |= USE_P3;       // disable P3.3
    P00 = 1;

    GPIO_setOpenDrainOutput(0, 0);//  P1
    GPIO_setPushPullupOutput(0, 1);// P2
    P0 |= USE_P0;
    P3 |= USE_P3;
    P01 = 1;

    GPIO_setOpenDrainOutput(0, 1);//  P2
    GPIO_setPushPullupOutput(0, 2);// P3
    P0 |= USE_P0;
    P3 |= USE_P3;
    P02 = 1;

    GPIO_setOpenDrainOutput(0, 2);//  P3
    GPIO_setPushPullupOutput(0, 3);// P4
    P0 |= USE_P0;
    P3 |= USE_P3;
    P03 = 1;

    GPIO_setOpenDrainOutput(0, 3);//  P4
    GPIO_setPushPullupOutput(3, 4);// P5
    P0 |= USE_P0;
    P3 |= USE_P3;
    P34 = 1;


    GPIO_setOpenDrainOutput(3, 4);//  P5
    GPIO_setPushPullupOutput(3, 5);// P6
    P0 |= USE_P0;
    P3 |= USE_P3;
    P35 = 1;


    GPIO_setOpenDrainOutput(3, 5);//  P6
    GPIO_setPushPullupOutput(3, 6);// P7
    P0 |= USE_P0;
    P3 |= USE_P3;
    P36 = 1;


    GPIO_setOpenDrainOutput(3, 6);//  P7
    GPIO_setPushPullupOutput(3, 7);// P8
    P0 |= USE_P0;
    P3 |= USE_P3;
    P37 = 1;


    GPIO_setOpenDrainOutput(3, 7);//  P8
    GPIO_setPushPullupOutput(2, 0);// P9
    P0 |= USE_P0;
    P3 |= USE_P3;
    P20 = 1;


    GPIO_setOpenDrainOutput(2, 0);//  P9
    GPIO_setPushPullupOutput(2, 1);//P10
    P0 |= USE_P0;
    P3 |= USE_P3;
    P21 = 1;


    GPIO_setOpenDrainOutput(2, 1);// P10
    GPIO_setPushPullupOutput(2, 2);//P11
    P0 |= USE_P0;
    P3 |= USE_P3;
    P22 = 1;


    GPIO_setOpenDrainOutput(2, 2);// P11
    GPIO_setPushPullupOutput(2, 3);//P12
    P0 |= USE_P0;
    P3 |= (USE_P3 << 1);
    P33 = 1;
    P23 = 1;

    GPIO_setOpenDrainOutput(2, 3);
    P23 = 0;
    return;
}

void LED_twoBytes_test2(void)
{
    GPIO_setOpenDrainOutput(3, 3);
    GPIO_setOpenDrainOutput(2, 7);
    P33 = 1;
    P1 = USE_P1;         // disable P1.0~P1.7
    P3 &= ~(USE_P3 << 1);// enable P3.4~P3.7
    P2 = ~(USE_P2 >> 4); // enable P2.0~P2.3
    P0 &= ~USE_P0;       // disable P0.1~P0.3


    GPIO_setOpenDrainOutput(2, 3);// P12
    GPIO_setPushPullupOutput(1, 0);//P13
    P10 = 1;

    GPIO_setOpenDrainOutput(1, 0);// P13
    GPIO_setPushPullupOutput(1, 1);//P14
    P1 = USE_P1;        // P1.0~P1.7
    P11 = 1;

    GPIO_setOpenDrainOutput(1, 1);// P14
    GPIO_setPushPullupOutput(1, 2);//P15
    P1 = USE_P1;        // P1.0~P1.7
    P12 = 1;

    GPIO_setOpenDrainOutput(1, 2);// P15
    GPIO_setPushPullupOutput(1, 3);//P16
    P1 = USE_P1;        // P1.0~P1.7
    P13 = 1;

    GPIO_setOpenDrainOutput(1, 3);// P16
    GPIO_setPushPullupOutput(1, 4);//P17
    P1 = USE_P1;        // P1.0~P1.7
    P14 = 1;

    GPIO_setOpenDrainOutput(1, 4);// P17
    GPIO_setPushPullupOutput(1, 5);//P18
    P1 = USE_P1;        // P1.0~P1.7
    P15 = 1;

    GPIO_setOpenDrainOutput(1, 5);// P18
    GPIO_setPushPullupOutput(1, 6);//P19
    P1 = USE_P1;        // P1.0~P1.7
    P16 = 1;

    GPIO_setOpenDrainOutput(1, 6);// P19
    GPIO_setPushPullupOutput(1, 7);//P20
    P1 = USE_P1;        // P1.0~P1.7
    P17 = 1;

    GPIO_setOpenDrainOutput(1, 7);// P20
    GPIO_setPushPullupOutput(2, 4);//P21
    P1 = USE_P1;        // P1.0~P1.7
    P24 = 1;

    GPIO_setOpenDrainOutput(2, 4);// P21
    GPIO_setPushPullupOutput(2, 5);//P22
    P2 = ~(USE_P2 >> 4);// P2.0~P2.3
    P25 = 1;

    GPIO_setOpenDrainOutput(2, 5);// P22
    GPIO_setPushPullupOutput(2, 6);//P23
    P2 = ~(USE_P2 >> 4);// P2.0~P2.3
    P26 = 1;

    GPIO_setOpenDrainOutput(2, 6);// P23
    GPIO_setPushPullupOutput(2, 7);//P24
    P2 = ~(USE_P2 >> 4);// P2.0~P2.3
    P27 = 1;

    GPIO_setOpenDrainOutput(2, 7);// P24
    P2 = ~(USE_P2 >> 4);// P2.0~P2.3

    return;
}

void LED_twoBytes_test3(void)
{
    P0 &= USE_P0;     // disable P0.0~P0.3
    P33 = 1;
    P3 &= USE_P3 << 1;// enable P3.3, disable P3.4~P3.7


    GPIO_setOpenDrainOutput(3, 3);//  P0
    GPIO_setOpenDrainOutput(2, 3);// P12
    GPIO_setPushPullupOutput(1, 0);//P13
    P2 = USE_P2 >> 4; // enable P2.4~P2.7
    P1 = ~USE_P1;     // enable P1.0~P1.7
    P10 = 1;

    GPIO_setOpenDrainOutput(1, 0);// P13
    GPIO_setPushPullupOutput(1, 1);//P14
    P2 = USE_P2 >> 4; // enable P2.4~P2.7
    P1 = ~USE_P1;     // enable P1.0~P1.7
    P10 = 0;
    P11 = 1;

    GPIO_setOpenDrainOutput(1, 1);// P14
    GPIO_setPushPullupOutput(1, 2);//P15
    P2 = USE_P2 >> 4; // enable P2.4~P2.7
    P1 = ~USE_P1;     // enable P1.0~P1.7
    P11 = 0;
    P12 = 1;

    GPIO_setOpenDrainOutput(1, 2);// P15
    GPIO_setPushPullupOutput(1, 3);//P16
    P2 = USE_P2 >> 4; // enable P2.4~P2.7
    P1 = ~USE_P1;     // enable P1.0~P1.7
    P12 = 0;
    P13 = 1;

    GPIO_setOpenDrainOutput(1, 3);// P16
    GPIO_setPushPullupOutput(1, 4);//P17
    P2 = USE_P2 >> 4; // enable P2.4~P2.7
    P1 = ~USE_P1;     // enable P1.0~P1.7
    P13 = 0;
    P14 = 1;

    GPIO_setOpenDrainOutput(1, 4);// P17
    GPIO_setPushPullupOutput(1, 5);//P18
    P2 = USE_P2 >> 4; // enable P2.4~P2.7
    P1 = ~USE_P1;     // enable P1.0~P1.7
    P14 = 0;
    P15 = 1;

    GPIO_setOpenDrainOutput(1, 5);// P18
    GPIO_setPushPullupOutput(1, 6);//P19
    P2 = USE_P2 >> 4; // enable P2.4~P2.7
    P1 = ~USE_P1;     // enable P1.0~P1.7
    P15 = 0;
    P16 = 1;

    GPIO_setOpenDrainOutput(1, 6);// P19
    GPIO_setPushPullupOutput(1, 7);//P20
    P2 = USE_P2 >> 4; // enable P2.4~P2.7
    P1 = ~USE_P1;     // enable P1.0~P1.7
    P16 = 0;
    P17 = 1;

    GPIO_setOpenDrainOutput(1, 7);// P20
    GPIO_setPushPullupOutput(2, 4);//P21
    P2 = USE_P2 >> 4; // enable P2.4~P2.7
    P1 = ~USE_P1;     // enable P1.0~P1.7
    P17 = 0;
    P24 = 1;

    GPIO_setOpenDrainOutput(2, 4);// P21
    GPIO_setPushPullupOutput(2, 5);//P22
    P2 = USE_P2 >> 4; // enable P2.4~P2.7
    P1 = ~USE_P1;     // enable P1.0~P1.7
    P24 = 0;
    P25 = 1;

    GPIO_setOpenDrainOutput(2, 5);// P22
    GPIO_setPushPullupOutput(2, 6);//P23
    P2 = USE_P2 >> 4; // enable P2.4~P2.7
    P1 = ~USE_P1;     // enable P1.0~P1.7
    P25 = 0;
    P26 = 1;

    GPIO_setOpenDrainOutput(2, 6);// P23
    GPIO_setPushPullupOutput(2, 7);//P24
    P2 = USE_P2 >> 4; // enable P2.4~P2.7
    P1 = ~USE_P1;     // enable P1.0~P1.7
    P26 = 0;
    P27 = 1;

    GPIO_setOpenDrainOutput(2, 7);
    P27 = 0;
    return;
}
#endif // LED_DEBUG

/// @brief LED点阵0显示
/// @param strData 文字显示码([2][12])
/// @param bytes 该文字所占字节位
/// @param arrLen 显示码长度
void LED_screen0_test(const unsigned char(*strData)[12],
                      const unsigned char bytes,
                      const unsigned char arrLen)
{
    unsigned char p0_data = USE_P0,
        p1_data = USE_P1,
        p2_data = USE_P2,
        p3_data = USE_P3,
        pos = 0x00,
        bytePos = 0x00;
    P0 = p0_data;
    P1 = p1_data;                   // disable P1.0~P1.7
    P2 = p2_data;
    P3 = p3_data;

    if ( arrLen < MIN_ARR_LEN || strData == NULL)// 输入显示码长度不足或指针为空
        return;
#if 0
/***************************** P1 *************************************** */
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);

    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SECOND_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIRST_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], ZEROTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    GPIO_setOpenDrainOutput(3, 3);//  P0
    GPIO_setPushPullupOutput(0, 0);// P1
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P00 = 1;

/***************************** P2 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);

    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SECOND_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIRST_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], ZEROTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    GPIO_setOpenDrainOutput(0, 0);//  P1
    GPIO_setPushPullupOutput(0, 1);// P2
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P01 = 1;

/***************************** P3 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);

    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SECOND_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIRST_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], ZEROTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    GPIO_setOpenDrainOutput(0, 1);//  P2
    GPIO_setPushPullupOutput(0, 2);// P3
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P02 = 1;

/***************************** P4 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);

    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SECOND_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIRST_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], ZEROTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    GPIO_setOpenDrainOutput(0, 2);//  P3
    GPIO_setPushPullupOutput(0, 3);// P4
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P03 = 1;

/***************************** P5 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);

    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SECOND_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIRST_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], ZEROTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    GPIO_setOpenDrainOutput(0, 3);//  P4
    GPIO_setPushPullupOutput(3, 4);// P5
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P34 = 1;

/***************************** P6 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);

    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SECOND_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIRST_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], ZEROTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    GPIO_setOpenDrainOutput(3, 4);//  P5
    GPIO_setPushPullupOutput(3, 5);// P6
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P35 = 1;

/***************************** P7 *************************************** */
    pos += 2;
    if ( pos >= arrLen )
    {
        bytePos++;
        pos = 0;
    }
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);

    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SECOND_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIRST_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], ZEROTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    GPIO_setOpenDrainOutput(3, 5);//  P6
    GPIO_setPushPullupOutput(3, 6);// P7
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P36 = 1;

/***************************** P8 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);

    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SECOND_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIRST_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], ZEROTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    GPIO_setOpenDrainOutput(3, 6);//  P7
    GPIO_setPushPullupOutput(3, 7);// P8
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P37 = 1;

/***************************** P9 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);

    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SECOND_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIRST_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], ZEROTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    GPIO_setOpenDrainOutput(3, 7);//  P8
    GPIO_setPushPullupOutput(2, 0);// P9
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P20 = 1;

/***************************** P10 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);

    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SECOND_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIRST_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], ZEROTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    GPIO_setOpenDrainOutput(2, 0);//  P9
    GPIO_setPushPullupOutput(2, 1);//P10
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P21 = 1;

/***************************** P11 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);

    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SECOND_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIRST_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], ZEROTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    GPIO_setOpenDrainOutput(2, 1);// P10
    GPIO_setPushPullupOutput(2, 2);//P11
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P22 = 1;

/***************************** P12 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);

    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SECOND_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIRST_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], ZEROTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    GPIO_setOpenDrainOutput(2, 2);// P11
    GPIO_setPushPullupOutput(2, 3);//P12
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P23 = 1;
#else
/***************************** P1 *************************************** */
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(3, 3);//  P0
    GPIO_setPushPullupOutput(0, 0);// P1
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P00 = 1;

    /***************************** P2 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(0, 0);//  P1
    GPIO_setPushPullupOutput(0, 1);// P2
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P01 = 1;

    /***************************** P3 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(0, 1);//  P2
    GPIO_setPushPullupOutput(0, 2);// P3
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P02 = 1;

    /***************************** P4 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(0, 2);//  P3
    GPIO_setPushPullupOutput(0, 3);// P4
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P03 = 1;

    /***************************** P5 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(0, 3);//  P4
    GPIO_setPushPullupOutput(3, 4);// P5
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P34 = 1;

    /***************************** P6 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(3, 4);//  P5
    GPIO_setPushPullupOutput(3, 5);// P6
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P35 = 1;

    /***************************** P7 *************************************** */
    pos += 2;
    if ( bytes < MIN_BYTES )// 字符字节数不足, 结束该字显示
        return;
    if ( pos >= arrLen )
    {
        bytePos++;
        pos = 0;
    }
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(3, 5);//  P6
    GPIO_setPushPullupOutput(3, 6);// P7
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P36 = 1;

    /***************************** P8 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(3, 6);//  P7
    GPIO_setPushPullupOutput(3, 7);// P8
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P37 = 1;

    /***************************** P9 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], THIRD_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(3, 7);//  P8
    GPIO_setPushPullupOutput(2, 0);// P9
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P20 = 1;

    /***************************** P10 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(2, 0);//  P9
    GPIO_setPushPullupOutput(2, 1);//P10
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P21 = 1;

    /***************************** P11 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(2, 1);// P10
    GPIO_setPushPullupOutput(2, 2);//P11
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P22 = 1;

    /***************************** P12 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00) value
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01)
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    GPIO_setOpenDrainOutput(2, 2);// P11
    GPIO_setPushPullupOutput(2, 3);//P12
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P23 = 1;
#endif

    GPIO_setOpenDrainOutput(2, 3);
    P23 = 0;
    return;
}

/// @brief LED点阵1显示
/// @param strData 文字显示码([2][12])
/// @param bytes 该文字所占字节位
/// @param arrLen 显示码长度
void LED_screen1_test(const unsigned char(*strData)[12],
                      const unsigned char bytes,
                      const unsigned char arrLen)
{
    unsigned char p0_data = USE_P0,
        p1_data = USE_P1,
        p2_data = USE_P2,
        p3_data = USE_P3,
        pos = 0x00,
        bytePos = 0x00;
    P0 = p0_data;
    P1 = p1_data;                   // enable P1.0~P1.7
    P2 = p2_data;                   // enable P2.4~P2.7
    P3 = p3_data;
    if ( arrLen < MIN_ARR_LEN || strData == NULL )// 输入显示码长度不足或指针为空
        return;

/***************************** P1 *************************************** */
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(3, 3);//  P0
    GPIO_setPushPullupOutput(0, 0);// P1
    P1 = p1_data;
    P2 = p2_data;
    P00 = 1;

/***************************** P2 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(0, 0);//  P1
    GPIO_setPushPullupOutput(0, 1);// P2
    P1 = p1_data;
    P2 = p2_data;
    P01 = 1;

/***************************** P3 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(0, 1);//  P2
    GPIO_setPushPullupOutput(0, 2);// P3
    P1 = p1_data;
    P2 = p2_data;
    P02 = 1;

/***************************** P4 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(0, 2);//  P3
    GPIO_setPushPullupOutput(0, 3);// P4
    P1 = p1_data;
    P2 = p2_data;
    P03 = 1;

/***************************** P5 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(0, 3);//  P4
    GPIO_setPushPullupOutput(3, 4);// P5
    P1 = p1_data;
    P2 = p2_data;
    P34 = 1;

/***************************** P6 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(3, 4);//  P5
    GPIO_setPushPullupOutput(3, 5);// P6
    P1 = p1_data;
    P2 = p2_data;
    P35 = 1;

/***************************** P7 *************************************** */
    pos += 2;
    if ( bytes < MIN_BYTES )// 字符字节数不足, 结束该字显示
        return;
    if ( pos >= arrLen )
    {
        bytePos++;
        pos = 0;
    }
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(3, 5);//  P6
    GPIO_setPushPullupOutput(3, 6);// P7
    P1 = p1_data;
    P2 = p2_data;
    P36 = 1;

/***************************** P8 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(3, 6);//  P7
    GPIO_setPushPullupOutput(3, 7);// P8
    P1 = p1_data;
    P2 = p2_data;
    P37 = 1;

/***************************** P9 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(3, 7);//  P8
    GPIO_setPushPullupOutput(2, 0);// P9
    P1 = p1_data;
    P2 = p2_data;
    P20 = 1;

/***************************** P10 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(2, 0);//  P9
    GPIO_setPushPullupOutput(2, 1);//P10
    P1 = p1_data;
    P2 = p2_data;
    P21 = 1;

/***************************** P11 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(2, 1);// P10
    GPIO_setPushPullupOutput(2, 2);//P11
    P1 = p1_data;
    P2 = p2_data;
    P22 = 1;

/***************************** P12 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(2, 2);// P11
    GPIO_setPushPullupOutput(2, 3);//P12
    P1 = p1_data;
    P2 = p2_data;
    P23 = 1;

    GPIO_setOpenDrainOutput(2, 3);
    P23 = 0;
    return;
}

/// @brief LED点阵2显示
/// @param strData 文字显示码([2][12])
/// @param bytes 该文字所占字节位
/// @param arrLen 显示码长度
void LED_screen2_test(const unsigned char(*strData)[12],
                      const unsigned char bytes,
                      const unsigned char arrLen)
{
    unsigned char p0_data = USE_P0,
        p1_data = USE_P1,
        p2_data = USE_P2,
        p3_data = USE_P3,
        pos = 0x00,
        bytePos = 0x00;
    P0 = p0_data;
    P1 = p1_data;                   // disable P1.0~P1.7
    P2 = p2_data;                   // disable P2.4~P2.7
    P3 = p3_data;
    if ( arrLen < MIN_ARR_LEN || strData == NULL )// 输入显示码长度不足或指针为空
        return;

/***************************** P13 *************************************** */
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00)
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(2, 3);// P12
    GPIO_setPushPullupOutput(1, 0);//P13
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P10 = 1;

/***************************** P14 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00)
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(1, 0);// P13
    GPIO_setPushPullupOutput(1, 1);//P14
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P11 = 1;

/***************************** P15 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00)
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(1, 1);// P14
    GPIO_setPushPullupOutput(1, 2);//P15
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P12 = 1;

/***************************** P16 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00)
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(1, 2);// P15
    GPIO_setPushPullupOutput(1, 3);//P16
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P13 = 1;

/***************************** P17 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00)
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(1, 3);// P16
    GPIO_setPushPullupOutput(1, 4);//P17
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P14 = 1;

/***************************** P18 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00)
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(1, 4);// P17
    GPIO_setPushPullupOutput(1, 5);//P18
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P15 = 1;

/***************************** P19 *************************************** */
    pos += 2;
    if ( bytes < MIN_BYTES )// 字符字节数不足, 结束该字显示
        return;
    if ( pos >= arrLen )
    {
        bytePos++;
        pos = 0;
    }
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00)
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(1, 5);// P18
    GPIO_setPushPullupOutput(1, 6);//P19
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P16 = 1;

/***************************** P20 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00)
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(1, 6);// P19
    GPIO_setPushPullupOutput(1, 7);//P20
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P17 = 1;

/***************************** P21 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00)
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(1, 7);// P20
    GPIO_setPushPullupOutput(2, 4);//P21
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P24 = 1;

/***************************** P22 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00)
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(2, 4);// P21
    GPIO_setPushPullupOutput(2, 5);//P22
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P25 = 1;

/***************************** P23 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00)
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(2, 5);// P22
    GPIO_setPushPullupOutput(2, 6);//P23
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P26 = 1;

/***************************** P24 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P1(P00)
        BIT_SET_ZERO(p0_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P2(P01) value
        BIT_SET_ZERO(p0_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P3(P02)
        BIT_SET_ZERO(p0_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P4(P03)
        BIT_SET_ZERO(p0_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P5(P34)
        BIT_SET_ZERO(p3_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P6(P35)
        BIT_SET_ZERO(p3_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P7(P36)
        BIT_SET_ZERO(p3_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P8(P37)
        BIT_SET_ZERO(p3_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P9(P20)
        BIT_SET_ZERO(p2_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P10(P21)
        BIT_SET_ZERO(p2_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P11(P22)
        BIT_SET_ZERO(p2_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P12(P23)
        BIT_SET_ZERO(p2_data, 3);
    GPIO_setOpenDrainOutput(2, 6);// P23
    GPIO_setPushPullupOutput(2, 7);//P24
    P0 = p0_data;
    P2 = p2_data;
    P3 = p3_data;
    P27 = 1;

    GPIO_setOpenDrainOutput(2, 7);// P24
    P2 = ~(USE_P2 >> 4);// P2.0~P2.3

    return;
}

/// @brief LED点阵3显示
/// @param strData 文字显示码([2][12])
/// @param bytes 该文字所占字节位
/// @param arrLen 显示码长度
void LED_screen3_test(const unsigned char(*strData)[12],
                      const unsigned char bytes,
                      const unsigned char arrLen)
{
    unsigned char p0_data = USE_P0,
        p1_data = USE_P1,
        p2_data = USE_P2,
        p3_data = USE_P3,
        pos = 0x00,
        bytePos = 0x00;
    P0 = p0_data;
    P1 = p1_data;
    P2 = p2_data;
    P3 = p3_data;
    if ( arrLen < MIN_ARR_LEN || strData == NULL )// 输入显示码长度不足或指针为空
        return;

/***************************** P13 *************************************** */
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(2, 3);// P12
    GPIO_setPushPullupOutput(1, 0);//P13
    P3 = p3_data;
    P2 = p2_data;
    P1 = p1_data;
    P10 = 1;

/***************************** P14 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(1, 0);// P13
    GPIO_setPushPullupOutput(1, 1);//P14
    P3 = p3_data;
    P2 = p2_data;
    P1 = p1_data;
    P11 = 1;

/***************************** P15 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(1, 1);// P14
    GPIO_setPushPullupOutput(1, 2);//P15
    P3 = p3_data;
    P2 = p2_data;
    P1 = p1_data;
    P12 = 1;

/***************************** P16 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(1, 2);// P15
    GPIO_setPushPullupOutput(1, 3);//P16
    P3 = p3_data;
    P2 = p2_data;
    P1 = p1_data;
    P13 = 1;

/***************************** P17 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(1, 3);// P16
    GPIO_setPushPullupOutput(1, 4);//P17
    P3 = p3_data;
    P2 = p2_data;
    P1 = p1_data;
    P14 = 1;

/***************************** P18 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(1, 4);// P17
    GPIO_setPushPullupOutput(1, 5);//P18
    P3 = p3_data;
    P2 = p2_data;
    P1 = p1_data;
    P15 = 1;

/***************************** P19 *************************************** */
    pos += 2;
    if ( bytes < MIN_BYTES )// 字符字节数不足, 结束该字显示
        return;
    if ( pos >= arrLen )
    {
        bytePos++;
        pos = 0;
    }
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(1, 5);// P18
    GPIO_setPushPullupOutput(1, 6);//P19
    P3 = p3_data;
    P2 = p2_data;
    P1 = p1_data;
    P16 = 1;

/***************************** P20 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(1, 6);// P19
    GPIO_setPushPullupOutput(1, 7);//P20
    P3 = p3_data;
    P2 = p2_data;
    P1 = p1_data;
    P17 = 1;

/***************************** P21 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P20(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(1, 7);// P20
    GPIO_setPushPullupOutput(2, 4);//P21
    P3 = p3_data;
    P2 = p2_data;
    P1 = p1_data;
    P24 = 1;

/***************************** P22 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(2, 4);// P21
    GPIO_setPushPullupOutput(2, 5);//P22
    P3 = p3_data;
    P2 = p2_data;
    P1 = p1_data;
    P25 = 1;

/***************************** P23 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P24(P27)
        BIT_SET_ZERO(p2_data, 7);
    GPIO_setOpenDrainOutput(2, 5);// P22
    GPIO_setPushPullupOutput(2, 6);//P23
    P3 = p3_data;
    P2 = p2_data;
    P1 = p1_data;
    P26 = 1;

/***************************** P24 *************************************** */
    pos += 2;
    p0_data = USE_P0;
    p1_data = USE_P1;
    p2_data = USE_P2;
    p3_data = USE_P3;
    if ( (FIND_BIT(strData[bytePos][pos], SEVENTH_BIT)) )// check P13(P10)
        BIT_SET_ZERO(p1_data, 0);
    if ( (FIND_BIT(strData[bytePos][pos], SIXTH_BIT)) )// check P14(P11)
        BIT_SET_ZERO(p1_data, 1);
    if ( (FIND_BIT(strData[bytePos][pos], FIFTH_BIT)) )// check P15(P12)
        BIT_SET_ZERO(p1_data, 2);
    if ( (FIND_BIT(strData[bytePos][pos], FOURTH_BIT)) )// check P16(P13)
        BIT_SET_ZERO(p1_data, 3);
    if ( (FIND_BIT(strData[bytePos][pos], THIRD_BIT)) )// check P17(P14)
        BIT_SET_ZERO(p1_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos], SECOND_BIT)) )// check P18(P15)
        BIT_SET_ZERO(p1_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos], FIRST_BIT)) )// check P19(P16)
        BIT_SET_ZERO(p1_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos], ZEROTH_BIT)) )// check P20(P17)
        BIT_SET_ZERO(p1_data, 7);

    if ( (FIND_BIT(strData[bytePos][pos + 1], SEVENTH_BIT)) )// check P21(P24)
        BIT_SET_ZERO(p2_data, 4);
    if ( (FIND_BIT(strData[bytePos][pos + 1], SIXTH_BIT)) )// check P22(P25)
        BIT_SET_ZERO(p2_data, 5);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FIFTH_BIT)) )// check P23(P26)
        BIT_SET_ZERO(p2_data, 6);
    if ( (FIND_BIT(strData[bytePos][pos + 1], FOURTH_BIT)) )// check P0(P33)
        BIT_SET_ZERO(p3_data, 3);
    GPIO_setOpenDrainOutput(2, 6);// P23
    GPIO_setPushPullupOutput(2, 7);//P24
    P3 = p3_data;
    P2 = p2_data;
    P1 = p1_data;
    P27 = 1;

    GPIO_setOpenDrainOutput(2, 7);
    P27 = 0;
    return;
}
