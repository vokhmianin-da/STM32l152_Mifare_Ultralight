#include "delay_SysTick.h"


extern uint16_t delay_count=0;

void delay_init_ms()//������������� SysTick ������� �� ���������� ��� � 1 ��
{
  delay_count=0;
  //SysTick_Config(182000/1000);
  //SysTick_Config(SystemCoreClock/3800);
  SysTick_Config(SystemCoreClock/1000);
}

void delay_init_100us()//������������� SysTick ������� �� ���������� ��� � 0,1 ��
{
  delay_count=0;
  //SysTick_Config(182000/1000);
  //SysTick_Config(SystemCoreClock/3800);
  SysTick_Config(SystemCoreClock/10000);
}


void SysTick_Handler()
{
  if (delay_count>0) delay_count--;
}

void delay_100us (uint16_t delay_value)////�������� � 0,1*x ��
{
  delay_count=delay_value;
  while (delay_count);
}


void delay_ms (uint16_t delay_value)////�������� � x ��
{
  delay_count=delay_value*10;
  while (delay_count);
}
