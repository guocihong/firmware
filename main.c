/*C**************************************************************************
* NAME:    main.c
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:      2013.09.04
* REVISION:     9.4     
*----------------------------------------------------------------------------
* PURPOSE:
* This is the main program.
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include <INTRINS.H>
#include "config.h"                      /* system macro definition */
#include "lib_mcu\c51_drv.h"             /* c51 driver definition */
#include "lib_mcu\wdt\wdt_drv.h"         /* WDT driver definition */
#include "lib_mcu\eeprom\eeprom_drv.h"   /* eeprom drive definition */
#include "scheduler.h"                   /* scheduler definition */
#include "tasks\comm\comm_task.h"        /* UART task definition */
#include "command.h"                     /* system command definition */
          

/*_____ D E C L A R A T I O N ______________________________________________*/
extern xdata  Byte    gl_reply_tick;     /* 设备返回延时*/

/* variables for beep */
extern xdata  Uint16  beep_during_temp;  //预设的最大蜂鸣持续时间, 单位tick

/* variables for 联动 */
extern xdata  Uint16  ld_during_temp;    //预设的一次联动最短输出时间, 单位tick  

/* AD sample */
extern idata  Uint16  ad_sensor_mask;     //sensor mask
extern xdata  Uint16  ad_sensor_mask_LR;  //按左右顺序重排序的sensor mask: 左8 ~ 1  右 8 ~ 1
extern bdata  bit     ad_sensor_extent;   //是否带扩展板: 1 - 带; 0 - 不带

extern xdata  Uint16  ad_still_dn;       //静态拉力值下限
extern xdata  Uint16  ad_still_up;       //静态拉力值上限
extern xdata  Byte    ad_still_Dup[14];  //报警阀值上限差

/* System */
extern idata  Byte   gl_comm_addr;     //本模块485通信地址（可能已改为规范值）		
extern xdata  Byte   gl_addr_origin;   //地址拨码开关原值（已取反）

extern bdata  bit    system_2or1;      //双/单防区标志: 0 - 双; 1 - 单												 

extern const  Byte   AREA_L_INDEX[7];  //左防区各道对应的板上(程序) index
extern const  Byte   AREA_R_INDEX[7];  //右防区各道对应的板上(程序) index

/* 传感器采样偏差 */
extern xdata  Uint16   sensor_sample_offset[14];    //传感器采样偏差：没有外力时，传感器采样值不为0，大约310左右，需要矫正。瞬间张力 = 采样值 - 采样偏差                 

static  void  get_sys_info(void);                                                                                                                                                   
static  void  main(void);
     

/*F**************************************************************************
* NAME: main
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: Main user routine 
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
void main(void)
{    
  Uint32 i;
 
  //set pins values
  P0 = 0xFF;       // 1111 1111b
  P0M1 = 0x00;	   // 0000 0000b 
  P0M0 = 0xC0; 	   // 1100 0000b
	
  P1    = 0xFF;    // 1111 1111b	  
  P1M1  = 0x01;    // 0000 0001b    
  P1M0  = 0x00;	   // 0000 0000b
  P1ASF = 0x01;    // P1.0 为AD模拟输入   
  
  P2 = 0xFF;       // 1111 1111b
  P2M1 = 0x00;	   // 准双向口
  P2M0 = 0x00;			  

  P3 = 0x33;       // 0011 0011b: Beep禁止, 报警1/2加电，联动不加电，RS485发送禁止（接收）
  P3M1 = 0x00;     // 0000 0000b
  P3M0 = 0xFC;	   // 1111 1100b
  
  P4 = 0xFF;       // 1111 1111b
  P4M1 = 0x00;     // 0000 0000b
  P4M0 = 0x00;	   // 0000 0000b   
  P4SW = 0x40;     // 0100 0000b

  P5 = 0xF0;       // 1111 0000b
	P5M1 = 0x00;     // 0000 0000b
	P5M0 = 0x0F;     // 0000 1111b

  AUXR = S1BRS_; //定时器0,1为8051兼容方式(12分频)
                 //BRT为12分频模式, UART1使用BRT作波特率发生器                      
  AUXR1 = 0x10;  //0001 0000b

  //system power-on hint
  //闪一下14个LED -- 全部点亮，用于检测所有LED是否完好
  P2 = 0x00;   //点亮8个LED 
  P0 = 0xC0;   //点亮6个LED (1100 0000b)
  i = 150000;
  while(i>0)  i--;
  P2 = 0xFF;	 //灭8个LED 
  P0 = 0xFF;	 //灭6个LED 
  //延时
  i = 150000;
  while(i>0)  i--;

  //从Flash中读取系统数据
  get_sys_info();

  //各模块任务初始化
  sch_scheduler_init();

  //使能全局中断
  Enable_interrupt();     //global interrupt enable

  //启动WDT
  #ifdef  Enable_WDT
    Wdt_enable();         //2s 溢出周期      
  #endif      

  sch_scheduler();        //endless scheduler execution
}        


/*F**************************************************************************
* NAME: get_sys_info
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: 读取系统预设数据
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
void get_sys_info(void)
{
    Uint16 i;
    Byte   j;
    Byte   temp;

    //0. 使能Flash访问
    flash_enable();

    //1. 双/单防区标志
    i = 0;
    system_2or1 = 0;  //缺省: 双防区
    do 
    {
    //延时
    j = 255;
    while(j > 0)  j--;	  	 
    if (system_2or1 == !bSel_2or1)
    { //读到相同的值
      i ++;
    }
    else
    { //读到不同的值
      i = 0;
      system_2or1 = !bSel_2or1;
    }
    } while (i < 8);

    //2. 读RS485通信地址 - 读拨码开关值
    i = 0;
    temp = 0xFF & b485_ADDR_MASK;
    do 
    {
        //延时
        j = 255;
        while (j>0)  j--;	  	 
        if (temp == (CommAddr_Port & b485_ADDR_MASK))
        { //读到相同的值
            i++;
        }
        else
        { //读到不同的值
            i = 0;
            temp = CommAddr_Port & b485_ADDR_MASK;
        }
    }while (i < 8);	
    gl_addr_origin = ~temp >> 1;      //地址拨码开关原值（已取反）
    gl_comm_addr = gl_addr_origin;    //本模块485通信地址（可能已改为规范值）		
    //设备地址合法性检查(总线设备地址要求从0x10开始,且<= 0xCF)
    //使用 7 位拨码开关，地址范围为 0x00 ~ 0x7F
    if ((gl_comm_addr < 0x10) || (gl_comm_addr > 0x7F))
    { //设备地址必须为 0x10 ~ 0x7F 之间(含)
    gl_comm_addr = CMD_ADDR_UNSOLV;
    }	

    //3. 读张力传感器mask
    //3.a. 允许拨码开关输入
    bP2_Gate = 0;	  //ZZX：此时mask位会驱动点亮对应的LED
    bP0_Gate = 0;

    //3.b 读开关值
    i = 0;
    ad_sensor_mask = 0x3FFF;
    do 
    {
      //延时
      j = 255;
      while (j > 0)  j--;	  	 
      if (ad_sensor_mask == ((((Uint16)(P0 & 0x3F)) << 8) + P2))
      {	//读到相同的值
        i ++;
      }
      else
      {	//读到不同的值
          i = 0;
          ad_sensor_mask = ((((Uint16)(P0 & 0x3F)) << 8) + P2);
      }
    } while (i < 15000);
    ad_sensor_mask = ~ad_sensor_mask & 0x3FFF;
    //计算 ad_sensor_mask_LR
    ad_sensor_mask_LR = change_to_LR(ad_sensor_mask); 	
    //判断是否带扩展板
    if ((ad_sensor_mask & 0x3C00) == 0x0)
      ad_sensor_extent = 0;   //是否带扩展板: 0 - 不带 
    else
        ad_sensor_extent = 1;   //是否带扩展板: 1 - 带

    //3.c. 禁止拨码开关输入，允许报警LED
    bP2_Gate = 1;		//ZZX: 禁止拨码开关输入，灭所有mask灯
    bP0_Gate = 1;
    
    //4. 读静态张力值范围
    temp = flash_read(EEPROM_SECTOR3);
    if (temp == 0x5A) 
    { //有有效设置
    //下限
    temp = flash_read(EEPROM_SECTOR3 + 1);
      ad_still_dn = (Uint16)temp << 8;
    temp = flash_read(EEPROM_SECTOR3 + 2);    
      ad_still_dn += temp;
      //上限
    temp = flash_read(EEPROM_SECTOR3 + 3);
      ad_still_up = (Uint16)temp << 8;
    temp = flash_read(EEPROM_SECTOR3 + 4);    
      ad_still_up += temp;
    //有效否? 		
      if ((ad_still_dn < STD_STILL_DN) || 
        (ad_still_up > STD_STILL_UP) || 
        (ad_still_dn >= ad_still_up))
      { //无合法数据，取缺省值
        ad_still_dn = STD_STILL_DN;
      ad_still_up = STD_STILL_UP;
      }	    
    }
    else
    {	//无有效设置，取缺省值
    ad_still_dn = STD_STILL_DN;
    ad_still_up = STD_STILL_UP;
    }
    
    //5.读报警阀值参数 
    temp = flash_read(EEPROM_SECTOR4);
    if (temp == 0x5A) 
    { //有有效设置
        //左防区各道
      for (j=0; j<7; j++)
      { 
            ad_still_Dup[AREA_L_INDEX[j]] = flash_read(EEPROM_SECTOR4 + 2 + j);
        }
        //右防区各道
      for (j=0; j<7; j++)
      { 
            ad_still_Dup[AREA_R_INDEX[j]] = flash_read(EEPROM_SECTOR4 + 10 + j);
        }    		
        //是否有效？ 	
    for (j=0; j<14; j++)
      { 
        if ((ad_still_Dup[j] < STD_ALARM_MIN) || (ad_still_Dup[j] > STD_ALARM_MAX))
        ad_still_Dup[j] = STD_ALARM_DEF;    //无合法数据，取缺省值
        }				
    }
    else
    {	//无有效设置，取缺省值
      for (j=0; j<14; j++)
      ad_still_Dup[j] = STD_ALARM_DEF;
    }  

    //6. 读声光报警时间设置  
    temp = flash_read(EEPROM_SECTOR5);
    if (temp == 0x5A) 
    { //有有效设置
      temp = flash_read(EEPROM_SECTOR5 + 1);
      beep_during_temp = (Uint16)(((Uint32)temp * 1000) / SCHEDULER_TICK);	
    }
    else
    {	//取缺省值
    beep_during_temp = 0;   //单位： tick
    }

    //7. 读联动输出时间设置  
    temp = flash_read(EEPROM_SECTOR6);
    if (temp == 0x5A) 
    { //有有效设置
      temp = flash_read(EEPROM_SECTOR6 + 1);
      ld_during_temp = (Uint16)(((Uint32)temp * 1000) / SCHEDULER_TICK);	
    }
    else
    {	//取缺省值: 5秒
    ld_during_temp = (Uint16)(((Uint32)5 * 1000) / SCHEDULER_TICK);   //单位： tick
    }

    //8. 读传感器采样偏差
    temp = flash_read(EEPROM_SECTOR7);
    if (temp == 0x5A) { //有有效设置
        for (j = 0; j < 14; j++) {
            temp = flash_read(EEPROM_SECTOR7 + 1 + (j << 1));
            sensor_sample_offset[j] = ((Uint16)temp << 8);
            temp = flash_read(EEPROM_SECTOR7 + 2 + (j << 1));
            sensor_sample_offset[j] += temp;
        }

        //杆自身
        sensor_sample_offset[9] = 0;
    } else {	//无有效设置
        for (j = 0; j < 14; j++) {
            sensor_sample_offset[j] = 0;
        }
    }
    
    //9. 读取延时时间
    temp = flash_read(EEPROM_SECTOR8);
    if (temp == 0x5A) { //有有效设置
        gl_reply_tick = flash_read(EEPROM_SECTOR8 + 1);
    } else {	//无有效设置
        gl_reply_tick = 0;
    }
    
    //10. 禁止Flash访问
    flash_disable();                                            
}
