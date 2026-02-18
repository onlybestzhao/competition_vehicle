#ifndef _SPORT_H
#define _SPORT_H
#include "sys.h"




uint32_t get_millis(void);
void SysTick_Init(void);
void SysTick_Handler(void);
void PWM_INIT(void);
void advance(void);
void recoil(void);
void turnleft(void);
void turnright(void);
void turnstop(void );
void turnout(void);
void left(void);
void right(void);
void allgo(void);
void adjustadvance(void);
void adjustrecoil(void);
void adjustadvanceb(void);
void adjustrecoilb(void);

void adjustright(void);
void adjustleft(void);
void judgment_process_place(void);
float P_calculate(void);
void do_turntable(void);
uint8_t do_turntableb(void);
void judgementput(void);
void judgementput1(void);
void judgementput2(void);
void judgementput3(void);
void change(void);
void turnlefttry(void);
void judgetakeprocess(void);
void judgementtake1(void);
void judgementtake2(void);
void judgementtake3(void);
void turntabletest (void);
void change2(void);
void judgementputb(void);
void do_turntable2(void);
#endif

