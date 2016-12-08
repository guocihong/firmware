/*C**************************************************************************
* NAME:  scheduler.c
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:     2013.         
* REVISION:     9.4     
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the scheduler routines
*
* NOTES:
* Configuration:
*   - SCH_TYPE in scheduler.h header file
*   - SCH_TIMER in scheduler.h header file
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                         /* system definition */
#include "lib_mcu\wdt\wdt_drv.h"            /* WDT driver definition */
#include "lib_mcu\timer\timer_drv.h"        /* timer definition */
#include "lib_mcu\uart\uart_drv.h"          /* uart definition */
#include "scheduler.h"                      /* scheduler definition */


/*_____ D E F I N I T I O N ________________________________________________*/
/* 系统计时 */
extern idata  Uint16  gl_ack_tick;	  /* 应答延时计时 tick */
extern  data  Uint16  gl_delay_tick;  /* 通用延时用tick */ 

/* for UART1/2 : 超时处理 */
extern idata  Byte  recv_state;      // uart receive state
extern idata  Byte  recv_timer;      // uart receive time-out
extern idata  Byte  recv2_state;     // UART2 receive state
extern idata  Byte  recv2_timer;     // UART2 receive time-out	 

/* for beep */
extern bdata  bit     beep_flag;     // 蜂鸣标志: 0 - 禁鸣; 1 - 正在蜂鸣
extern  data  Uint16  beep_timer;    // 计时器，蜂鸣剩余时间, 单位:tick

/* for alarm output (继电器及LED) */
extern bdata  Byte    alarm_out_flag;  //报警输出标志
extern bdata  bit     alarm3_flag;	 
extern bdata  bit     alarm2_flag;	 
extern bdata  bit     alarm1_flag;
extern bdata  bit     alarm1C_flag;
extern  data  Uint16  alarm1_timer;    // 计时器，报警器1已报警时间,单位tick 
extern  data  Uint16  alarm2_timer;    // 计时器，报警器2已报警时间,单位tick
extern  data  Uint16  alarm3_timer;    // 计时器，攀爬报警已报警时间,单位tick
extern  data  Uint16  ld_timer;        // 计时器，联动剩余输出时间, 单位tick

/* for AD */
extern idata  Byte        ad_index;     //正在采样的通道号, 取值范围0~13
extern  data  sAD_Sample  ad_sample;    //保存当前采样值
extern bdata  bit  ad_sensor_extent;    //是否带扩展板: 1 - 带; 0 - 不带

/* Doorkeep */   
extern  data  Byte   gl_dk_tick;        //门磁检测计时tick  
			  
/* 传感器采样偏差 */
extern xdata  Uint16   sensor_sample_offset[14];    //传感器采样偏差：没有外力时，传感器采样值不为0，大约310左右，需要矫正。瞬间张力 = 采样值 - 采样偏差                 

/*_____ D E C L A R A T I O N ______________________________________________*/
static  void sch_time_init(void);



/*F**************************************************************************
* NAME:     sch_scheduler_init
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: Scheduler initialization
*----------------------------------------------------------------------------
* NOTE:	Task_x_init() and Task_x_fct() are defined in scheduler.h
*****************************************************************************/
void sch_scheduler_init(void)
{ 
  Task_1_init();   
  Task_2_init();     
  Task_3_init(); 
  Task_4_init();  
  Task_5_init();  
  Task_6_init(); 
  Task_7_init();  
  Task_8_init();  
  Task_9_init(); 
  Task_10_init(); 

  sch_time_init();      
}



/*F**************************************************************************
* NAME:     sch_scheduler
*----------------------------------------------------------------------------
* PARAMS: 
* return:
*----------------------------------------------------------------------------
* PURPOSE:  Task execution scheduler 
*****************************************************************************/
void sch_scheduler(void)
{
  while (1)
  {
    Task_1_fct();
    Task_2_fct();
    Task_3_fct();            
    Task_4_fct(); 
    Task_5_fct();    
    Task_6_fct(); 
    Task_7_fct();         
    Task_8_fct();          
    Task_9_fct();          
    Task_10_fct();         

	  #ifdef  Enable_WDT
      Wdt_refresh();         // 2s 溢出周期      
    #endif     
  }
}


#if (SCH_TIMER == SCH_TIMER0)
/*F**************************************************************************
* NAME:     sch_time_init
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: Scheduler time base (timer 0) initialization
*----------------------------------------------------------------------------
* NOTE:
*   mode 16-bit Timer, Time counter
*   T0_PRIO to be defined in config.h
*   TIM_LOW & TIM_HIGH defined in scheduler.h
*****************************************************************************/
void sch_time_init(void)
{ 
  T0_stop(); 
  T0_init(T0_NOT_GATED, T0_TIMER, T0_MODE_1);
  TL0 = TIM_LOW;
  TH0 = TIM_HIGH;
  t0_set_prio(T0_PRIO);       /* set-up priority */
  T0_enable_int();            /* enable interrupt */
  T0_start();                 /* start time base */
}


/*F**************************************************************************
* NAME:     sch_timer_int
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: Timer 0 interrupt function
******************************************************************************/
void sch_timer_int(void) interrupt TF0_VECTOR using 1
{  
    T0_stop();                /* stop timer */
    TL0 = TIM_LOW;            /* reload timer */ 
    TH0 = TIM_HIGH;
    T0_start();               /* restart timer */

    /* 读AD值 */ 
    if (ad_sample.valid == FALSE)
    { 
        //原数据已经被处理, 可以写入新数据
        ad_sample.val = ADC_RES;             //读高8位
        ad_sample.val = ad_sample.val << 2;         
        ad_sample.val += (ADC_RESL & 0x03);  //得到10bit采样值
        ad_sample.index = ad_index;
        ad_sample.valid = TRUE;
        if ((ad_sensor_extent == 0) && (ad_index >= 10)){   //不带扩展板
            ad_sample.val = 0;
        }

        //采样值减去采样偏差
        if (ad_sample.val > sensor_sample_offset[ad_index]) {
            ad_sample.val -= sensor_sample_offset[ad_index];
        } else {
            ad_sample.val = 0;
        }
        
        //启动下一通道采样     
        if (ad_index >= 13){
            ad_index = 0;
        }else{   
            ad_index ++;   
        }

        if ((ad_sensor_extent == 0) && (ad_index >= 10)){   //不带扩展板且欲采样扩展通道
            P5 = 9;          //扩展通道只采样通道9
        }else{
            P5 = ad_index;	 //选择模拟输入	
        }

        ADC_CONTR = 0x80;          // 1000 0000b
        ADC_CONTR |= ADC_START_;   // 启动转换  
    }  
	
    /* increment task tick counters */
    gl_delay_tick ++;      /* 通用延时用tick */
    gl_dk_tick ++;         /* 门磁检测计时tick */
    if (gl_ack_tick > 0)
    gl_ack_tick --;      /* 应答延时计时 */

    /* beep & alarm out */
    if (beep_flag)
    { 
        //正在beep
        if (beep_timer > 0){
            beep_timer--;
        }
        
        if (beep_timer == 0)
        { 
            //蜂鸣时间到，静音
            bBeep_Ctrl = 0;
            beep_flag = 0;
        }
    }
    
    if (alarm1C_flag)
    {	
        //正在联动输出
        if (ld_timer > 0){
            ld_timer --;
        }
        
        if (ld_timer == 0)
        { 
            //联动输出时间到，停止输出
            bRelay_L1 = 0;
            alarm1C_flag = 0;	  
        }
    }
    
    if (alarm1_flag)  alarm1_timer ++;
    if (alarm2_flag)  alarm2_timer ++;
    if (alarm3_flag)  alarm3_timer ++;

    /* UART字节之间接收超时 */
    if (recv_state != FSA_INIT) 
    {
        //非初始状态，需要检测是否超时
        if (recv_timer > 0){
            recv_timer --;
        }
        if (recv_timer == 0)  {
            recv_state = FSA_INIT;   //接收超时, 恢复至初始状态      
        }
    }  

    /* UART2字节之间接收超时 */
    if (recv2_state != FSA_INIT) 
    { 
        //非初始状态，需要检测是否超时
        if (recv2_timer > 0){
            recv2_timer --;
        }
        
        if (recv2_timer == 0){
            recv2_state = FSA_INIT;   //接收超时, 恢复至初始状态      
        }
    }
}

#endif
