/*********************************
НАСТРОЙКА ВЫВОДОВ ДЛЯ РАБОТЫ SPI
*********************************/
#define RC522_GPIO GPIOB
#define RCC_RC522 RCC_AHBPeriph_GPIOB
#define RC522_MISO GPIO_Pin_3
#define RC522_SCK GPIO_Pin_4
#define RC522_SS GPIO_Pin_5
#define RC522_MOSI GPIO_Pin_6
#define RST_RC522 GPIO_Pin_7
#define SUPPLY_RC522 GPIO_Pin_8

#define RC522_PIN_set(pin) GPIO_SetBits(RC522_GPIO, pin)
#define RC522_PIN_clr(pin) GPIO_ResetBits(RC522_GPIO, pin)
#define RC522_PIN_MISO GPIO_ReadInputDataBit(RC522_GPIO, RC522_MISO)

/*********************************
ПРОТОТИПЫ ФУНКЦИЙ
*********************************/

uint8_t RC522_comm_light (uint8_t*, uint8_t, uint8_t*, uint8_t*);
uint8_t write_page (uint8_t, uint8_t*);
uint8_t read_page (uint8_t, uint8_t*);
uint8_t request_card (uint8_t, uint8_t*);
uint8_t select_card (uint8_t, uint8_t, uint8_t*);
uint8_t read_UID (uint8_t, uint8_t, uint8_t*);
uint8_t Read_Reg_RC522 (uint8_t);
uint8_t trans_SPI_RC522 (uint8_t);
uint8_t halt ();
void calculate_CRC (uint8_t*, uint8_t, uint8_t*);
void set_bit_mask (uint8_t, uint8_t);
void clear_bit_mask (uint8_t, uint8_t);
void Write_Reg_RC522 (uint8_t, uint8_t);
void init_RC522 ();
void init_SPI_RC522 ();

/*********************************
КОМАНДЫ RC522
*********************************/

#define PCD_IDLE 0x00 //отмена всех текущих команд
#define PCD_AUTHENT 0x0E //ключ аутентификации
#define PCD_RECEIVE 0x08 //прием данных
#define PCD_TRANSMIT 0x04 //отправка данных
#define PCD_TRANSCEIVE 0x0C //отправка и прием данных
#define PCD_RESETPHASE 0x0F //программный сброс модуля
#define PCD_CALCCRC 0x03 //запуск расчета CRC

/*********************************
КОМАНДЫ Mifare Ultralight
*********************************/

#define PICC_REQIDL 0x52 //поиск всех карт в пределах антенны
#define PICC_REQALL 0x26 //поиск карт, которые не находятся в режиме гибернации
#define PICC_ANTICOLL1 0x93 //команда антиколлизии, уровень 1
#define PICC_ANTICOLL2 0x95 //команда антиколлизии, уровень 2
#define PICC_ARG_SELECT 0x70 //аргумент команды антиколлизии для выбора карты по UID
#define PICC_ARG_UID 0x20 //аргумент команды антиколлизии для чтения UID
#define PICC_READ_4BYTE 0x30 //чтение страницы из 4 байт
#define PICC_WRITE_4BYTE 0xA2 //запись страницы из 4 байт
#define PICC_HALT 0x50 //режим останова

/*********************************
4-битные коды подтверждения/неподтверждения (ACK/NACK)
*********************************/

#define ACK 0x0A //код подтверждения ACK
/*
0x00 NACK for invalid argument
0x01 NACK for parity or CRC error
0x04 for counter overflow
0x05, 0x07 NACK for EEPROM write error
0x06, 0x09 other error
*/

/*********************************
АДРЕСА РЕГИСТРОВ MFRC522
*********************************/

// PAGE 0
#define     RFU00                 0x00
#define     CommandReg            0x01
#define     ComIEnReg             0x02
#define     DivlEnReg             0x03
#define     ComIrqReg             0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     RFU0F                 0x0F
// PAGE 1
#define     RFU10                 0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     RFU1A                 0x1A
#define     RFU1B                 0x1B
#define     MifareReg             0x1C
#define     RFU1D                 0x1D
#define     RFU1E                 0x1E
#define     SerialSpeedReg        0x1F
// PAGE 2
#define     RFU20                 0x20
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     RFU23                 0x23
#define     ModWidthReg           0x24
#define     RFU25                 0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsCfgReg            0x28
#define     ModGsCfgReg           0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
// PAGE 3
#define     RFU30                 0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39
#define     TestDAC2Reg           0x3A
#define     TestADCReg            0x3B
#define     RFU3C                 0x3C
#define     RFU3D                 0x3D
#define     RFU3E                 0x3E
#define     RFU3F		  0x3F

/*********************************
КОДЫ ЗАВЕРШЕНИЯ ФУНКЦИЙ
*********************************/

#define OK 1
#define ERR 0