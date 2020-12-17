#include "stm32l1xx.h"
#include "delay_SysTick.h"
#include "UART1.h"
#include "RC522.h"




int main()
{
  delay_init_100us(); //инициализаци€ библиотеки задержки, прерывание SysTick
  uart1_init(); //инициализаци€ UART1: 9600, 8 бит, без бита четности
  Received_Byte=0;
  init_RC522(); //инициализаци€ микросхемы MFRC522 и SPI под нее
  while (1)
  {
      uint8_t status;
      uint8_t data[18];
      
      /*ѕередача команд REQA и WUPA*/
       while (request_card (PICC_REQALL, data)==ERR){};
     //посылка команды REQA и ожидание ответа
      UART1_Send_String ("Request Answer (REQA, 0x26), Answer To Request (ATQA)=");
      _print_array (data, 2); //вывод 2 байт ответа от карты
      next_line();
      while (request_card(PICC_REQIDL, data)==ERR){};//посылка команды WUPA и ожидание ответа
      UART1_Send_String("Wake-Up command (WUPA, 0x52), Answer To Request (ATQA)=");
       _print_array (data, 2); //вывод 2 байт ответа от карты
       next_line();
       
       /*„тение 3 первых байт UID и выбор карты. ”ровень 1*/
       status=read_UID(PICC_ANTICOLL1, PICC_ARG_UID, data);
       if (status==OK)
       {
         UART1_Send_String("UID of cascade level 1 =");
         //UID of cascade level 1: 88h SN0 SN1 SN2 BCC1
          _print_array (data, 4);
       }
       else
         UART1_Send_String("Error read UID of cascade level 1");
       next_line();
        status=select_card(PICC_ANTICOLL1, PICC_ARG_SELECT, data);
        if (status==OK)
       {
         UART1_Send_String("Select ACKnowledge (SAK) of cascade level 1=");
         //Cascade level 1: SAK=0x04
          print_byte (data[0]);
       }
       else
         UART1_Send_String("Error Select ACKnowledge (SAK) of cascade level 1");
       next_line();
       
       /*„тение 4 байт UID и выбор карты. ”ровень 2*/
       status=read_UID(PICC_ANTICOLL2, PICC_ARG_UID, data);
       if (status==OK)
       {
         UART1_Send_String("UID of cascade level 2 =");
         //UID of cascade level 2: SN3 SN4 SN5 SN6 BCC2
          _print_array (data, 4);
       }
       else
         UART1_Send_String("Error read UID of cascade level 2");
       next_line();
       
       status=select_card(PICC_ANTICOLL2, PICC_ARG_SELECT, data);
        if (status==OK)
       {
         UART1_Send_String("Select ACKnowledge (SAK) of cascade level 2=");
         //Cascade level 2: SAK=0x00 (Mifare Ultralight)
          print_byte (data[0]);
       }
       else
         UART1_Send_String("Error Select ACKnowledge (SAK) of cascade level 2");
       next_line();
       
       /*„тение 16 страниц по 4 байта*/
       next_line();
       UART1_Send_String("*** PRINT PAGE DATA ***");
       next_line();
       for (uint8_t i=0; i<4; i++)
       {
         status=read_page(i*4, data);
         if (status==ERR)
         {
           UART1_Send_String("Error read page data");
           next_line();
           break;
         }
         for (uint8_t j=0; j<4; j++)
         {
           UART1_Send_String("Page ");
           print_number(i*4+j);
           UART1_Send_String("     ");
            _print_array (&data[j*4], 4);
            next_line();
         }
       }
       
       /*ѕередача карте команды останова (0x50 0x00)*/
       halt();
       next_line();
        UART1_Send_String("===============================================");
  }//while(1)
 
}//main()
