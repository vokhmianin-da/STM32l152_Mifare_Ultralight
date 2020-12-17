#include "stm32l1xx.h"
#include "RC522.h"

#define MAXRLEN 18 // максимальный размер массива для обмена данными

/*Инициализация SPI*/
void init_SPI_RC522()
{
  RCC_AHBPeriphClockCmd(RCC_RC522, ENABLE);
 
  GPIO_InitTypeDef RC522_SPI_OUTPUT;
  RC522_SPI_OUTPUT.GPIO_Pin=RC522_MOSI|RC522_SCK|RC522_SS|RST_RC522|SUPPLY_RC522;
  RC522_SPI_OUTPUT.GPIO_Mode=GPIO_Mode_OUT;
  RC522_SPI_OUTPUT.GPIO_Speed= GPIO_Speed_40MHz;
  RC522_SPI_OUTPUT.GPIO_OType=GPIO_OType_PP;
  
  GPIO_InitTypeDef RC522_SPI_INPUT;
  RC522_SPI_INPUT.GPIO_Pin=RC522_MISO;
  RC522_SPI_INPUT.GPIO_Mode=GPIO_Mode_IN;
  RC522_SPI_INPUT.GPIO_Speed= GPIO_Speed_40MHz;
  RC522_SPI_INPUT.GPIO_OType=GPIO_OType_PP;
  RC522_SPI_INPUT.GPIO_PuPd=GPIO_PuPd_UP;
  
  GPIO_Init(RC522_GPIO, &RC522_SPI_INPUT);
  GPIO_Init(RC522_GPIO, &RC522_SPI_OUTPUT);
  
  RC522_PIN_set(SUPPLY_RC522);//подача питания на считыватель
  RC522_PIN_set(RC522_SS);//запрещение обмена по SPI
  RC522_PIN_clr(RST_RC522);//сброс вывода RST
  RC522_PIN_clr(RC522_SCK);//сброс вывода SCK
}

/*
Обмен данными с микросхемой RC522 через SPI
DataIN - передаваемый байт данных
DataOUT - принимаемый байт данных
*/
uint8_t trans_SPI_RC522 (uint8_t DataIN)
{
  uint8_t DataOUT=0; //считанный байт
  for (uint8_t i=8; i>0; i--)
  {
    /*Задержка для медленного SPI*/
    //volatile uint8_t DelaySPI=5;
    //while(DelaySPI--);
    if (DataIN&0x80) //проверка старшего разряда
      RC522_PIN_set (RC522_MOSI);
    else
      RC522_PIN_clr (RC522_MOSI);
    DataIN<<=1;//сдвиг разрядов передаваемого байта влево
    RC522_PIN_set (RC522_SCK);//установка вывода SCK в 1
    DataOUT<<=1;//сдвиг разрядов принимаемого байта влево
    if (RC522_PIN_MISO) //проверка линии MISO
      DataOUT|=1; //если на MISO высокий уровень, то устанавливается 1
    RC522_PIN_clr (RC522_SCK); //сброс вывода SCK в 0
  }
  return DataOUT; //возврат из функции считанного байта
}

/*Запись в регистр RC522
Address - адрес регистра на запись
Data - байт данных
*/
void Write_Reg_RC522 (uint8_t Address, uint8_t Data)
{
  RC522_PIN_clr (RC522_SCK); //подготовка вывода синхронизации
  RC522_PIN_clr (RC522_SS); //разрешение работы модуля
  Address=(Address<<1)&(0x7E);//формируем адрес для записи
  trans_SPI_RC522(Address); //передаем адрес регистра на запись
  trans_SPI_RC522(Data); //передаем данные
  RC522_PIN_set (RC522_SS); //запрещение работы модуля
}

/*Чтение из регистра RC522
Address - адрес регистра на чтение
Data - значение прочитанного байта
*/
uint8_t Read_Reg_RC522 (uint8_t Address)
{
  uint8_t Data;
  RC522_PIN_clr (RC522_SCK); //подготовка вывода синхронизации
  RC522_PIN_clr (RC522_SS); //разрешение работы модуля
  Address=(Address<<1)|(1<<7); //формируем адрес для чтения
  trans_SPI_RC522(Address); //передаем адрес регистра для чтения
  Data=trans_SPI_RC522(0); //считываем данные в ответ на передачу 0
  RC522_PIN_set (RC522_SS); //запрещение работы модуля
  return Data; //возврат считанного байта данных
}

/*Установка разрядов в регистре по маске
RegisterAddress - адрес регистра
mask - биты для установки
*/
void set_bit_mask (uint8_t RegisterAddress, uint8_t mask)
{
  uint8_t RegisterData;
  RegisterData=Read_Reg_RC522(RegisterAddress); //считывание байта по адресу
  Write_Reg_RC522(RegisterAddress, RegisterData|mask); //установка бит в регистре
}

/*Сброс разрядов в регистре по маске
RegisterAddress - адрес регистра
mask - биты для сброса
*/
void clear_bit_mask (uint8_t RegisterAddress, uint8_t mask)
{
  uint8_t RegisterData;
  RegisterData=Read_Reg_RC522(RegisterAddress); //считывание байта по адресу
  Write_Reg_RC522(RegisterAddress, RegisterData&~mask); //сброс бит в регистре
}

/*Инициализация и сброс микросхемы MFRC522*/
void init_RC522()
{
  init_SPI_RC522();//инициализация SPI
  RC522_PIN_clr(RST_RC522); //сброс модуля низким сигналом на выводе RESET
  RC522_PIN_set(RST_RC522); //активация модуля высоким уровнем на выводе RESET
  Write_Reg_RC522(CommandReg, 0x0F); //перевод модуля в режим программного сброса
  Write_Reg_RC522(TxAutoReg, 0x40); //включение режима 100% ASK (карты типа А)
  Write_Reg_RC522(ModeReg, 0x3D); //начальное значение в регистразх CRC 0x6363
  set_bit_mask(TxControlReg, 0x03); //включение генерации сигнала на антенне
}

/*Подсчет контрольной суммы CRC16 средствами RC522*/
void calculate_CRC(uint8_t* Input_Data, //массив входных данных
                   uint8_t Length_Input, //количество входных байт данных
                   uint8_t* Out_Data) //2 байта CRC
{
  uint8_t i;
  uint8_t temp;
  clear_bit_mask(DivIrqReg, (1<<2)); //очистка бита прерывания готовности контрольной суммы
  Write_Reg_RC522(CommandReg, PCD_IDLE); //отмена всех текущих команд
  set_bit_mask(FIFOLevelReg, 0x80); //очистка указателя чтения и записи в буфере FIFO
  for (i=0; i<Length_Input; i++)
  {
    Write_Reg_RC522(FIFODataReg, Input_Data[i]); //заполняем буфер входными данными
  }
  Write_Reg_RC522(CommandReg, PCD_CALCCRC); //запуск расчета CRC
  i=255;
  do //защита от зацикливания
  {
    temp=Read_Reg_RC522(DivIrqReg);
    i--;
  }
  while ((i!=0) &&!(temp&0x04)); //проверка бита готовности расчета CRC
  Out_Data[0]=Read_Reg_RC522(CRCResultRegL); //считывание младшего байта CRC
  Out_Data[1]=Read_Reg_RC522(CRCResultRegM); //считывание старшего байта CRC
}

/*Отправка данных через RFID канал к карте
Входные параметры:
Input_Data - посылаемые в карту данные
Length_Byte_Input - длина посылаемых данных в байтах
Выходные параметры:
Out_Data - Принимаемые от карты данные
Length_Bit_Out - длина принятых данных в битах*/
uint8_t RC522_comm_light (uint8_t* Input_Data, uint8_t Length_Byte_Input,
                          uint8_t* Out_Data, uint8_t* Length_Bit_Out)
{
  uint8_t status=ERR;
  uint8_t lastBits=0;
  uint8_t temp;
  uint16_t i;
  
  clear_bit_mask(ComIrqReg, (1<<7)); //очистка всех битов прерываний
  Write_Reg_RC522(CommandReg, PCD_IDLE); //отмена всех текущих команд
  clear_bit_mask(FIFOLevelReg, (1<<7)); //сброс указателей чтения и записи из буфера
  for (i=0; i<Length_Byte_Input; i++)
  {
    Write_Reg_RC522(FIFODataReg, Input_Data[i]); //заполнение буфера из Input_Data
  }
   Write_Reg_RC522(CommandReg, PCD_TRANSCEIVE); //команда передачи в регистр команд
  set_bit_mask(BitFramingReg, (1<<7)); //установка бита запуска передачи
  i=1000; //защита от зацикливания
  do
  {
    temp=Read_Reg_RC522(ComIrqReg);
    //считываем регистр прерываний interrupt request bits
    i--;
  }
  while ((i!=0)&&!(temp&((1<<5)|(1<<4)))); //проверка отмены команды (IdleIRq),
  //принятия достоверных данных (RxIRq), защита от зацикливания
  clear_bit_mask(BitFramingReg, (1<<7)); //сброс бита StartSend (конец передачи)
  if(i!=0)
  {
    if (!(Read_Reg_RC522(ErrorReg)&0x1B)) //проверка флагов регистра ErrorReg
    {
      /*BufferOvfl - переполнение буфера, CollErr - коллизия,ParityErr -
      ошибка четности при приеме, ProtocolErr - ошибка протокола*/
      status=OK;
      temp=Read_Reg_RC522(FIFOLevelReg); //считываем указатель адреса в буфере
      lastBits=Read_Reg_RC522(ControlReg)&0x07;
      //младшие 3 бита ControlReg указывают число битов в последнем байте
      if (lastBits) *Length_Bit_Out=(temp-1)*8+lastBits; /*если не 0, то
      предыдущие байты умножаем на 8 и прибавляем последние биты*/
      else *Length_Bit_Out=temp*8; //иначе просто умножаем байты на 8
      if (temp==0) temp=1; //если указатель адреса равен 0, то записываем туда 1
      if (temp>MAXRLEN) temp=MAXRLEN; /*если указатель больше максимального, то
      ограничиваем его длину*/
      for (i=0; i<temp; i++)
      {
        Out_Data[i]=Read_Reg_RC522(FIFODataReg); //считываем буфер
      }
    }
      else 
      {
        status=ERR;
      }
    }
    Write_Reg_RC522(CommandReg, PCD_IDLE); //сбрасываем регистр команд
    return status;
  }

/*Перевод карты в режим останова*/
uint8_t halt()
{
  uint8_t status;
  uint8_t Buffer[4];
  uint8_t LengthBit;
  Buffer[0]=PICC_HALT; //отправка команды на переход в режим гибернации
  Buffer[1]=0x00;
  calculate_CRC(Buffer, 2, &Buffer[2]); //расчет CRC и добавление к коду команды
  status=RC522_comm_light(Buffer, 4, Buffer, &LengthBit);
  //отправка 4 байт команды к карте
  return status;
}

/*Запрос типа карты
Входные параметры:
ReqCode - передача кода запроса на поиск карт 
0x52 (PICC_REQALL) - команда для нахождения всех карт стандарта 14443A в поле
действия
0x26 (PICC_REQIDL) - команда для нахождения всех карт, кроме тех, что в 
гибернации*/
uint8_t request_card(uint8_t ReqCode, uint8_t *TypeCard)
{
  uint8_t status;
  uint8_t LengthBit; //длина считанных бит
  Write_Reg_RC522(BitFramingReg, 0x07); /*определяет, какие биты последнего
  байта будут переданы. Для команд Request (0x26) и Wake-up (0x52) должны быть
  переданы только 7 бит*/
  status=RC522_comm_light(&ReqCode, 1, TypeCard, &LengthBit);
  if ((status==OK)&&(LengthBit==2*8)) //проверка возврата 2 байт от карты
    status=OK;
  else
    status=ERR;
  return status;
}

/*Запроос на чтение UID
UID первого уровня: 88h SN0 SN1 SN2 BCC1
UID второго уровня: SN3 SN4 SN5 SN6 BCC2*/
uint8_t read_UID(uint8_t Anticoll_CMD, //команда антиколлизии
                 uint8_t Anticoll_ARG, //аргумент команды антиколлизии
                 uint8_t *Answer) //указатель на запись для 5 принятых байт
{
  uint8_t i;
  uint8_t xor=0;
  uint8_t status;
  uint8_t LengthBit;
  uint8_t Buffer[MAXRLEN]; //буфер принимаемых данных
  Write_Reg_RC522(BitFramingReg, 0x00); //количество бит в передаваемом
  //последнем байте
  Buffer[0]=Anticoll_CMD; //команда антиколлизии
  Buffer[1]=Anticoll_ARG; //аргумент команды антиколлизии для выбора карты
  //по UID
  status=RC522_comm_light(Buffer, 2, Buffer, &LengthBit); //передача команды
  //на карту
  if (status==OK)
  {
    for(i=0; i<5; i++) //расчет суммы XOR принятых данных
    {
      Answer[i]=Buffer[i];
      xor^=Buffer[i];
    }
    if (xor) status=ERR; //если XOR принятых данных не равна 0, то ошибка
  }
  return status;
}

/*Выбор карты по UID
Anticoll_CMD - команда антиколлизии
Anticoll_ARG - аргумент команды антиколлизии
SerialNum - указатель для передачи 5 байт (4 байта UID и 1 байт контрольной
суммы*/
uint8_t select_card(uint8_t Anticoll_CMD, uint8_t Anticoll_ARG,
                    uint8_t *SerialNum) //указатель на 4 байта серийного номера
{
  uint8_t i;
  uint8_t status;
  uint8_t LengthBit;
  uint8_t BufferRC522[MAXRLEN];
  BufferRC522[0]=Anticoll_CMD; //команда антиколлизии для выбора карты
  BufferRC522[1]=Anticoll_ARG; //аргумент команды антиколлизии для выбора карты
  BufferRC522[6]=0; //элемент массива для хранения XOR суммы передаваемых данных
  for (i=0; i<4; i++) //подсчет XOR суммы передаваемых данных
  {
    BufferRC522[i+2]=SerialNum[i]; //в элементах массива от 2 до 5 записан
    //серийный номер карты
    BufferRC522[6]^=SerialNum[i]; //в 6 элементе записана XOR сумма 4 байт UID
  }
  calculate_CRC(BufferRC522, 7, &BufferRC522[7]); //расчет контрольной
  //суммы и включение в конец массива посылки
  status=RC522_comm_light(BufferRC522, 9, &SerialNum[0], &LengthBit);
  //отправка 9 байт команды на выбор карты по UID
  if ((status==OK)&&(LengthBit==3*8)) //проверка на возврат 3 байт
    //(1 SAK + 2 CRC)
    status=OK;
  else
    status=ERR;
  return status;
  }
 
 
  /*Запись 4 байт на 1 страницу*/
uint8_t write_page (uint8_t AddrPage, uint8_t* array)
{
   uint8_t i;
   uint8_t status;
   uint8_t LengthBit;
   uint8_t BufferRC522[MAXRLEN];
  
   BufferRC522[0]=PICC_WRITE_4BYTE; //0xA2 для записи 4 байт одной страницы
   BufferRC522[1]=AddrPage; //адрес страницы для записи (0 ... 15)
   for (i=0; i<4; i++)
   {
      BufferRC522[i+2]=array[i];//заполнение буфера 4 байтами данных
   }
   calculate_CRC (BufferRC522, 6, &BufferRC522[6]);//расчет контрольной суммы
   status=RC522_comm_light (BufferRC522, 8, BufferRC522, &LengthBit);
   //отправка пакета
   if ((status!=OK)||(LengthBit!=4)||((BufferRC522[0]&0x0F)!=ACK))
   {
     status=ERR;
   }
   return status;
}

/*Чтение страницы из 4 байт*/
uint8_t read_page (uint8_t AddrPage, //адрес страницы для чтения (0..15)
                   uint8_t *Data) //указатель на массив для передачи 16 байт
{
  uint8_t i;
  uint8_t status;
  uint8_t LengthBit;
  uint8_t BufferRC522[MAXRLEN];
  BufferRC522[0]=PICC_READ_4BYTE; //команда на чтение 4-х страниц
  BufferRC522[1]=AddrPage; //адрес страницы для чтения
  calculate_CRC (BufferRC522, 2, &BufferRC522[2]); //расчет контрольной суммы
  status=RC522_comm_light(BufferRC522, 4, BufferRC522, &LengthBit);
  //отправка пакета
  if ((status==OK)&&(LengthBit==18*8)) //проверка правильности выполнения команд
    //и длины полученного ответа (16 байт данных + 2 байта CRC)
  {
    for (i=0; i<16; i++)
    {
      Data[i]=BufferRC522[i]; //если чтение прошло успешно, то копируем 16 байт
      //данных через указатель в массив
    }
  }
  else
    status=ERR;
  return status;
}