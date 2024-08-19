#include "STC8C.h"
#include "stdio.h"
#include "GPIO_init.h"
#include "ShowStrings.h"
#define LED_DEBUG 0
#define DEBUG1 0
#define DEBUG2 0
#define OFF_TEST 0

static const char* TAG = "MAIN";
// "困难总比办法多"GB2312, ps: 若作为局部变量, 必须加'\0', 否则会两个数组黏在一起
static const char tempCN[] = {0xc0, 0xa7,
                 0xc4, 0xd1,
                 0xd7, 0xdc,
                 0xb1, 0xc8,
                 0xb0, 0xec,
                 0xb7, 0xa8,
                 0xb6, 0xe0, 0x00};
int main(void)
{
    /*
    * sizeof(char)-->1
    * sizeof(short) == sizeof(int)-->2
    * sizeof(long)-->4
    */
    char str[32] = "Hello World! ! !\r\n";
    unsigned int char_size = sizeof(char),
        short_size = sizeof(short),
        int_size = sizeof(int),
        long_size = sizeof(long),
        LedLine = 0;

 //printf("UART Initialization successful\r\n");
    GPIO_init();
    UartInit();

    UartSendString(str);
    while ( 1 )
    {
        //LED_allON(LedLine);
        /*LED_allON_test();*/
        /*LED_leftON_test();
        LED_rightON_test();*/

#if LED_DEBUG
#if DEBUG1
        LED_twoBytes_test1();
        LED_twoBytes_test3();
#else
        LED_twoBytes_test();
        LED_twoBytes_test2();
#endif
#if DEBUG2
        LED_twoBytes_test2();
        LED_twoBytes_test3();
#else
        LED_twoBytes_test();
        LED_twoBytes_test1();
#endif
#else
        LED_screen0_test(hello_cn_str(), MAX_BYTES, MAX_ARR_LEN);
#if OFF_TEST
        LED_off();
#endif
        LED_screen1_test(hello_cn_str(), MAX_BYTES, MAX_ARR_LEN);
#if OFF_TEST
        LED_off();
#endif
        LED_screen2_test(hello_cn_str(), MAX_BYTES, MAX_ARR_LEN);
#if OFF_TEST
        LED_off();
#endif
        LED_screen3_test(hello_cn_str(), MAX_BYTES, MAX_ARR_LEN);
#if OFF_TEST
        LED_off();
#endif

        //LED_twoBytes_test();
        //LED_twoBytes_test1();
        //LED_twoBytes_test2();
        //LED_twoBytes_test3();
#endif




        // 串口打印会非常耗费时间, 导致LED亮度下降
        //UartSendString(str);
       //printf("this is printf function...\r\n char %02x, short %02x, int %02x, long %02x\r\n",
       //       char_size, short_size, int_size, long_size);// printf()验证可用

       //printf("this function's name: %s\r\n", TAG);
        LedLine++;
        if ( LedLine > 24 )
            LedLine = 0;
    }
    return 0;
}