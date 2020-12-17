/*
 * UART1.c
 *
 *  Created on: 14 авг. 2019 г.
 *      Author: Дмитрий
 */

#include "UART1.h"
   
extern uint8_t Received_Byte=0;//принятый из UART1 байт

void uart1_init()
{
	GPIO_InitTypeDef GPIO_Init_USART;
	USART_InitTypeDef USART_InitUser;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	
	GPIO_Init_USART.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_Init_USART.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init_USART.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init_USART.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_USART.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_Init(GPIOA, &GPIO_Init_USART);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	USART_InitUser.USART_BaudRate=9600;
	USART_InitUser.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitUser.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_InitUser.USART_Parity=USART_Parity_No;
	USART_InitUser.USART_StopBits=USART_StopBits_1;
	USART_InitUser.USART_WordLength=USART_WordLength_8b;
	
	USART_Init(USART1, &USART_InitUser);
	
	USART_Cmd(USART1, ENABLE);
        NVIC_EnableIRQ(USART1_IRQn);//глобальное разрешение прерываний от USART1
        USART_ITConfig (USART1, USART_IT_RXNE, ENABLE);//прерывание по приему вкл

}

void UART1_Send_Byte (uint16_t x)
{
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	USART_SendData(USART1, x);
}

void UART1_Send_String (uint8_t* str)
{
	uint8_t i=0;
	while (str[i])
	{
		UART1_Send_Byte (str[i++]);
	}
}

uint8_t UART1_Received_Byte ()
{  
  return USART_ReceiveData(USART1);
}

void USART1_IRQHandler()//обработчик прерывания по USART1
{
  if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)//Проверка флага приема
  {
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);//сброс флага приема
    Received_Byte=USART_ReceiveData(USART1);//прием байта по UART и запись в ReceiveByte
  }
}

/*Печать числа в десятичном виде*/
void print_number(uint8_t data)
{
  uint8_t dec=0;
  while(data>=10)
  {
    data-=10;
    dec++;
  }
  UART1_Send_Byte (dec+'0');
  UART1_Send_Byte (data+'0');
}

/*Печать числа в шестнадцатеричном виде*/
 void print_byte(uint8_t data)
 {
   uint8_t temp;
   temp=data>>4;
   if (temp>9) temp+=7;
   UART1_Send_Byte (temp+'0');
   temp=data&0x0F;
   if (temp>9) temp+=7;
   UART1_Send_Byte (temp+'0');
 }

/*Перевод курсора на следующую строку*/
void next_line()
{
  UART1_Send_Byte (0x0D);
  UART1_Send_Byte (0x0A);
}

/*Печать массива чисел в шестнадцатиричном виде*/
void _print_array (uint8_t* array, uint8_t number)
{
  for (uint8_t i=0; i<number; i++)
  {
    UART1_Send_Byte (' ');
    print_byte(array[i]);
  }
}