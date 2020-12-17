/*
 * UART1.h
 *
 *  Created on: 14 ���. 2019 �.
 *      Author: �������
 */

#include "stm32l1xx.h"
   
   
extern uint8_t Received_Byte;//�������� �� UART1 ����

void uart1_init();
void UART1_Send_Byte (uint16_t);
void UART1_Send_String (uint8_t*);
uint8_t UART1_Received_Byte();
void print_number(uint8_t);
void print_byte(uint8_t);
void next_line();
#define print_array(m) _print_array (m, sizeof(m)/sizeof(m[0]))
void _print_array (uint8_t*, uint8_t);

