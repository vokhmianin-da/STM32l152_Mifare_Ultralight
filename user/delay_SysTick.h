#include "stm32l1xx.h"

void delay_init_ms();//������������� SysTick ������� �� ���������� ��� � 1 ��
void delay_init_100us();//������������� SysTick ������� �� ���������� ��� � 0,1 ��
void delay_ms (uint16_t);//�������� � � ��
void delay_100us (uint16_t);//�������� � 0,1*x ��
extern uint16_t delay_count;
