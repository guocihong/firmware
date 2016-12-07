/*H**************************************************************************
* NAME:   eeprom_drv.h         
*----------------------------------------------------------------------------
* RELEASE:   2011.01.13   
* REVISION:  1.0
*----------------------------------------------------------------------------
* PURPOSE:
*   This file contains the EEPROM read/write driver definition.
*****************************************************************************/

#ifndef _EEPROM_DRV_H_
#define _EEPROM_DRV_H_


/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                 /* system configuration */


/*_____ D E F I N I T I O N ________________________________________________*/
/* Data EEPROM in MCU */
//For 12C5410AD
//#define EEPROM_SECTOR1    0x2800    //ÿ������512���ֽڣ���4������
//#define EEPROM_SECTOR2    0x2A00
//#define EEPROM_SECTOR3    0x2C00
//#define EEPROM_SECTOR4    0x2E00

//For 12C5608AD
/* 8��������������ʼ��ַ */
//#define EEPROM_SECTOR1    0x0000    //ÿ������512���ֽڣ���8������
//#define EEPROM_SECTOR2    0x0200
//#define EEPROM_SECTOR3    0x0400
//#define EEPROM_SECTOR4    0x0600
//#define EEPROM_SECTOR5    0x0800
//#define EEPROM_SECTOR6    0x0A00
//#define EEPROM_SECTOR7    0x0C00
//#define EEPROM_SECTOR8    0x0E00

//For 12C5A16
/* ��8K, 16��������������ʼ��ַ */
/*
#define EEPROM_SECTOR1     0x0000    //ÿ������512���ֽڣ��� 16 ������
#define EEPROM_SECTOR2     0x0200
#define EEPROM_SECTOR3     0x0400
#define EEPROM_SECTOR4     0x0600
#define EEPROM_SECTOR5     0x0800
#define EEPROM_SECTOR6     0x0A00
#define EEPROM_SECTOR7     0x0C00
#define EEPROM_SECTOR8     0x0E00
#define EEPROM_SECTOR9     0x1000
#define EEPROM_SECTOR10    0x1200
#define EEPROM_SECTOR11    0x1400
#define EEPROM_SECTOR12    0x1600
#define EEPROM_SECTOR13    0x1800
#define EEPROM_SECTOR14    0x1A00
#define EEPROM_SECTOR15    0x1C00
#define EEPROM_SECTOR16    0x1E00
*/

#define EEPROM_SECTOR_SIZE  0x200	// 512���ֽ�

//For 12C5A32
/* ��28K, 56��������������ʼ��ַ */
#define EEPROM_SECTOR1     0x0000    //ÿ������512���ֽڣ��� 56 ������
#define EEPROM_SECTOR2     0x0200
#define EEPROM_SECTOR3     0x0400
#define EEPROM_SECTOR4     0x0600
#define EEPROM_SECTOR5     0x0800
#define EEPROM_SECTOR6     0x0A00
#define EEPROM_SECTOR7     0x0C00
#define EEPROM_SECTOR8     0x0E00
#define EEPROM_SECTOR9     0x1000
#define EEPROM_SECTOR10    0x1200
#define EEPROM_SECTOR11    0x1400
#define EEPROM_SECTOR12    0x1600
#define EEPROM_SECTOR13    0x1800
#define EEPROM_SECTOR14    0x1A00
#define EEPROM_SECTOR15    0x1C00
#define EEPROM_SECTOR16    0x1E00


#define FLASH_CMD_STDBY   0x00      // ����ģʽ����ISP����
#define FLASH_CMD_READ    0x01      // ������FALSH���ֽڶ�
#define FLASH_CMD_WRITE   0x02      // ������FALSH���ֽ�д
#define FLASH_CMD_ERASE   0x03      // ������FALSH��������

#define WTIME_6MHz        0x04      // ������Ƶ��ֵ�м�ʱ��Ӧȡ��Ƶ��ֵ���Ա�֤���㹻����ʱ
#define WTIME_12MHz       0x03
#define WTIME_20MHz       0x02
#define WTIME_24MHz       0x01       
#define WTIME_30MHz       0x00      // ע��30MHz���Ͼ�ȡ��ֵ

#define FLASH_CMD_TRIG()  IAP_TRIG=0x5A;IAP_TRIG=0xA5   //�����


/*_____ D E C L A R A T I O N ______________________________________________*/
void flash_enable(void);                  // flash ���ʳ�ʼ�趨
void flash_disable(void);                 // ��ֹfalsh����
void flash_erase(Uint16 addr);            // ����ָ����ַ�������� 
Byte flash_read(Uint16 addr);             // ��ָ����ַ����һ���ֽ�
void flash_write(Byte val, Uint16 addr);  // ��ָ����ַ��д��һ���ֽ�ֵ

#endif /* _EEPROM_DRV_H_ */
