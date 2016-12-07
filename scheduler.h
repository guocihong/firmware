/*H**************************************************************************
* NAME:  scheduler.h         
*----------------------------------------------------------------------------
* Copyright (c) 2008.
*----------------------------------------------------------------------------
* RELEASE:      
* REVISION:     1.0    
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the scheduler definition and the task function to be
* executed by the scheduler
*
* NOTE:
* SCHEDULER_TICK is defined in config.h
*
*****************************************************************************/

#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_


/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                          // configuration header
#include "tasks\comm\comm_task.h"            // UART通信处理
#include "tasks\ad\ad_task.h"                // 采样处理
#include "tasks\doorkeep\doorkeep_task.h"    // 门磁处理
#include "tasks\status\status_task.h"        // 门磁和张力状态处理
#include "tasks\alarm\alarm_task.h"          // 报警处理


/*----- Scheduler Types -----*/
#define SCH_TIMED       0    
#define SCH_TASK        1    
#define SCH_FREE        2    

/*----- Scheduler Timer -----*/
#define SCH_TIMER0        0
#define SCH_TIMER1        1
#define SCH_TIMER_NONE  255


/*----- Scheduler Configuration -----*/
#define SCH_TYPE        SCH_FREE      /* SCH_TIMED, SCH_TASK, SCH_FREE */
#define SCH_TIMER       SCH_TIMER0    /* SCH_TIMER0, SCH_TIMER1, SCH_TIMER_NONE */

/*----- Scheduler Timer Configuration -----*/
#define TIM_ADJUST      ((Byte)4)     /* to be modified for timing adjustment */

//ZZX: 下面的计算式使用 Fosc / 12 作为Counter 的时钟.
#define TIM_LOW         LOW( 65536 - (Uint16)(((Uint32)SCHEDULER_TICK * FOSC_Hz) / 12000) + TIM_ADJUST)
#define TIM_HIGH        HIGH(65536 - (Uint16)(((Uint32)SCHEDULER_TICK * FOSC_Hz) / 12000))


/*----- Task Definitions -----*/
#define Task_1_init()     comm_task_init()          //UART通信处理初始化
#define Task_2_init()     ad_task_init()            //AD采样初始化
#define Task_3_init()     doorkeep_task_init()      //门磁处理初始化   
#define Task_4_init()	    status_task_init()        //门磁和张力状态处理初始化 
#define Task_5_init()     alarm_task_init()         //报警处理初始化
#define Task_6_init()     
#define Task_7_init()   
#define Task_8_init()
#define Task_9_init()
#define Task_10_init()

#define Task_1_fct()       comm_task()		      //UART通信处理
#define Task_2_fct()       ad_task()       		  //AD采样
#define Task_3_fct()       doorkeep_task()      //门磁处理  
#define Task_4_fct()       status_task()        //门磁和张力状态处理  
#define Task_5_fct()       alarm_task()         //报警处理
#define Task_6_fct()       
#define Task_7_fct()
#define Task_8_fct()
#define Task_9_fct()
#define Task_10_fct()


/*_____ D E F I N I T I O N ________________________________________________*/


/*_____ D E C L A R A T I O N ______________________________________________*/
void sch_scheduler_init(void);
void sch_scheduler(void);


#endif /* _SCHEDULER_H_ */
