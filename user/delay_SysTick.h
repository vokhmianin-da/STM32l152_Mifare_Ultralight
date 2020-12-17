#include "stm32l1xx.h"

void delay_init_ms();//инициализация SysTick таймера на прерывание раз в 1 мс
void delay_init_100us();//инициализация SysTick таймера на прерывание раз в 0,1 мс
void delay_ms (uint16_t);//задержка в х мс
void delay_100us (uint16_t);//задержка в 0,1*x мс
extern uint16_t delay_count;
