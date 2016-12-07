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
//#define EEPROM_SECTOR1    0x2800    //每个扇区512个字节，共4个扇区
//#define EEPROM_SECTOR2    0x2A00
//#define EEPROM_SECTOR3    0x2C00
//#define EEPROM_SECTOR4    0x2E00

//For 12C5608AD
/* 8个数据扇区的起始地址 */
//#define EEPROM_SECTOR1    0x0000    //每个扇区512个字节，共8个扇区
//#define EEPROM_SECTOR2    0x0200
//#define EEPROM_SECTOR3    0x0400
//#define EEPROM_SECTOR4    0x0600
//#define EEPROM_SECTOR5    0x0800
//#define EEPROM_SECTOR6    0x0A00
//#define EEPROM_SECTOR7    0x0C00
//#define EEPROM_SECTOR8    0x0E00

//For 12C5A16
/* 共8K, 16个数据扇区的起始地址 */
/*
#define EEPROM_SECTOR1     0x0000    //每个扇区512个字节，共 16 个扇区
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

#define EEPROM_SECTOR_SIZE  0x200	// 512个字节

//For 12C5A32
/* 共28K, 56个数据扇区的起始地址 */
#define EEPROM_SECTOR1     0x0000    //每个扇区512个字节，共 56 个扇区
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


#define FLASH_CMD_STDBY   0x00      // 待机模式，无ISP操作
#define FLASH_CMD_READ    0x01      // 对数据FALSH区字节读
#define FLASH_CMD_WRITE   0x02      // 对数据FALSH区字节写
#define FLASH_CMD_ERASE   0x03      // 对数据FALSH扇区擦除

#define WTIME_6MHz        0x04      // 介于两频率值中间时，应取大频率值，以保证有足够的延时
#define WTIME_12MHz       0x03
#define WTIME_20MHz       0x02
#define WTIME_24MHz       0x01       
#define WTIME_30MHz       0x00      // 注：30MHz以上均取此值

#define FLASH_CMD_TRIG()  IAP_TRIG=0x5A;IAP_TRIG=0xA5   //命令触发


/*_____ D E C L A R A T I O N ______________________________________________*/
void flash_enable(void);                  // flash 访问初始设定
void flash_disable(void);                 // 禁止falsh访问
void flash_erase(Uint16 addr);            // 擦除指定地址处的扇区 
Byte flash_read(Uint16 addr);             // 读指定地址处的一个字节
void flash_write(Byte val, Uint16 addr);  // 向指定地址处写入一个字节值

#endif /* _EEPROM_DRV_H_ */
