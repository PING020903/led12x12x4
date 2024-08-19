#ifndef _GPIO_INIT_H_
#define _GPIO_INIT_H_

#define BIT_SET_ZERO(src, cnt) src &= ~(1 << cnt) // 给 src 左移 cnt位写0
#define BIT_SET_ONE(src, cnt) src |= (1 << cnt) // 给 src 左移 cnt 位写1
#define FIND_BIT(src, cnt) (src & (1 << cnt)) // 找 src 中左移 cnt 位的 1

#define SEVENTH_BIT 7
#define SIXTH_BIT 6
#define FIFTH_BIT 5
#define FOURTH_BIT 4
#define THIRD_BIT 3
#define SECOND_BIT 2
#define FIRST_BIT 1
#define ZEROTH_BIT 0
#define NEXT_CH_BIT 8

#define LOW_FREQUENCY !1
#define TIMER_MAX_COUNT (65535U)
#define BAUD_RATE (115200U)
#if LOW_FREQUENCY
#define FOSC 11059200UL
#else
#define FOSC 33177200UL
#endif
#define BRT ((TIMER_MAX_COUNT + 1U) - (FOSC / BAUD_RATE / 4U))

#ifndef USE_P0
#define USE_P0 (unsigned char)(0x0f) // P0所使用的 pin
#endif // !USE_P0

#ifndef USER_P1
#define USE_P1 (unsigned char)(0xFF) // P1所使用的 pin
#endif // !USER_P1

#ifndef USE_P2
#define USE_P2 (unsigned char)(0xFF) // P2所使用的 pin
#endif // !USE_P2

#ifndef USE_P3
#define USE_P3 (unsigned char)(0xf8) // P3所使用的 pin
#endif // !USE_P3

#define MIN_BYTES (2)
#define MIN_ARR_LEN (12)



void GPIO_setPushPullupOutput(const unsigned char port, const unsigned char pin);

void GPIO_setOpenDrainOutput(const unsigned char port, const unsigned char pin);

void GPIO_setStandardBidirectional(const unsigned char port, const unsigned char pin);

void GPIO_setHighResistanceInput(const unsigned char port, const unsigned char pin);

void GPIO_setPullup(const unsigned char port,
                    const unsigned char pin,
                    const unsigned char en);

void GPIO_setSchmidtTrigger(const unsigned char port,
                            const unsigned char pin,
                            const unsigned char en);

void GPIO_setLevelShiftingSpeed(const unsigned char port,
                                const unsigned char pin,
                                const unsigned char en);

void GPIO_setDriveCurrent(const unsigned char port,
                          const unsigned char pin,
                          const unsigned char en);

void UartInit(void);

void UartSendByte(unsigned char ch);

void UartSendString(const char* str);

void GPIO_init(void);

void LED_off(void);

void LED_allON(unsigned char LED_line);

void LED_allON_test(void);

void LED_leftON_test(void);

void LED_rightON_test(void);

void LED_twoBytes_test(void);

void LED_twoBytes_test1(void);

void LED_twoBytes_test2(void);

void LED_twoBytes_test3(void);

void LED_screen0_test(const unsigned char(*strData)[12],
                      const unsigned char bytes,
                      const unsigned char arrLen);

void LED_screen1_test(const unsigned char(*strData)[12],
                     const unsigned char bytes,
                     const unsigned char arrLen);

void LED_screen2_test(const unsigned char(*strData)[12],
                      const unsigned char bytes,
                      const unsigned char arrLen);

void LED_screen3_test(const unsigned char(*strData)[12],
                      const unsigned char bytes,
                      const unsigned char arrLen);

#endif  // _GPIO_INIT_H_