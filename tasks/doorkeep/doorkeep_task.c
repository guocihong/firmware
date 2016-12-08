/*C**************************************************************************
* NAME:   doorkeep_task.c
*----------------------------------------------------------------------------
* Copyright (c) 2008.
*----------------------------------------------------------------------------
* RELEASE:      2008.11.15
* REVISION:     1.0     
*----------------------------------------------------------------------------
* PURPOSE:          
* This file contains the doorkeep scanning task and attached routines
*
* NOTES:
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                  /* system configuration */
#include "lib_mcu\c51_drv.h"         /* c51 driver definition */
#include "doorkeep_task.h"           /* doorkeep task definition */


/*_____ D E F I N I T I O N ________________________________________________*/
#define DK_CHECK_TEMP    (1000/SCHEDULER_TICK)	  /* 门磁开关检查周期 */
#define DK_CONFORM_TEMP  ( 100/SCHEDULER_TICK)    /* 门磁开关变化确认延时 */


/*_____ D E C L A R A T I O N ______________________________________________*/
extern bdata  bit   gl_dk_status;    //门磁开关状态（每1s动态检测）: 1 - 闭合; 0 - 打开(需要报警)                    
extern  data  Byte  gl_dk_tick;  	 //门磁检测计时tick
extern idata  Byte  system_status;   //系统状态	
static  data  Byte  dk_read_state;   //task state

extern void save_alarm_detail_info(void);

/*F**************************************************************************
* NAME: doorkeep_task_init
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE:  doorkeep scan task initialization
*****************************************************************************/
void doorkeep_task_init(void)
{ 
  gl_dk_status = 1;      //上电时，缺省为闭合  
  dk_read_state = DK_READ_START;
}


/*F**************************************************************************
* NAME: doorkeep_task
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: doorkeep scanning process
*****************************************************************************/
void doorkeep_task(void)
{ 
  switch (dk_read_state)
  {
    case DK_READ_START: //开始任务
                    if (system_status > SYS_SAMP_BASE)
                    { //系统进入实时检测阶段才开始检查门磁
                      gl_dk_tick = 0;
                      dk_read_state = DK_READ_IDLE;
                    }
                    break;  
    
    case DK_READ_IDLE://检测门磁有无变化
                    if (gl_dk_tick > DK_CHECK_TEMP)
                    { //每秒检测一次
                      gl_dk_tick = 0;
                      if (gl_dk_status == 1)
                      { //原来为门磁闭合
                        if (bDoorKeeper == 0)
                        { //门磁可能被打开, 进入延时确认阶段   					     	                      
                          dk_read_state = DK_READ_DELAY;
                        }
                      }
                      else
                      { //原来为门磁打开   
                        if (bDoorKeeper == 1)
                        { //门磁可能被闭合, 进入延时确认阶段   		                      
                          dk_read_state = DK_READ_DELAY;
                        }
                      }
                    }
                    break;     

    case DK_READ_DELAY://延时确认
                    if (gl_dk_tick > DK_CONFORM_TEMP)
                    { //延时时间到, 判断是否是稳定的变化    
                      if ((gl_dk_status == 1) && (bDoorKeeper == 0))
                      { //门磁已经打开
                        gl_dk_status = 0;
                          
                        //2016-12-07新增
                        //保存报警详细信息
                        save_alarm_detail_info();
                      }						  
                      else if ((gl_dk_status == 0) && (bDoorKeeper == 1))
                      { //门磁已经闭合
                        gl_dk_status = 1;
                      }				  
                      gl_dk_tick = 0;
                      dk_read_state = DK_READ_IDLE;                                    
                    }
	                  break;
  }//end switch (dk_read_state)
}//end FUNC()
