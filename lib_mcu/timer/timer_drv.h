/*H**************************************************************************
* NAME:   timer_drv.h         
*----------------------------------------------------------------------------
* Copyright (c) 2011.
*----------------------------------------------------------------------------
* RELEASE:      2011.01.11
* REVISION:     1.0     
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the timer0, timer1 driver definition
*****************************************************************************/

#ifndef _TIMER_DRV_H_
#define _TIMER_DRV_H_


/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                   /* configuration header */


/*_____ M A C R O S ________________________________________________________*/


/*_____ D E C L A R A T I O N ______________________________________________*/
void   t0_set_prio(Byte priority);    //设置T0的中断优先级
void   t1_set_prio(Byte priority);    //设置T1的中断优先级
//void  timer_init(void);             //分拆到各个功能模块中初始化


/*_____ D E F I N I T I O N ________________________________________________*/
/* Timer clock source */
#define T0_MODE_0     0x00      //13bit counter
#define T0_MODE_1     0x01      //16bit counter
#define T0_MODE_2     0x02      //8bit counter with reload
#define T0_MODE_3     0x03      //two 8bit counters, T1 stop 
#define T0_COUNTER    0x04		//从T0/P3.4输入计数脉冲
#define T0_TIMER      0x00		//Timer0 为定时器
#define T0_GATED      0x08		//仅在INT0脚为高及TR0置1时使能计数
#define T0_NOT_GATED  0x00		//TR0置1时即使能计数

#define T1_MODE_0     0x00
#define T1_MODE_1     0x10
#define T1_MODE_2     0x20
#define T1_MODE_3     0x30		 //T1 invalid
#define T1_COUNTER    0x40		 //从T1/P3.5输入计数脉冲
#define T1_TIMER      0x00		 //Timer1 为定时器
#define T1_GATED      0x80		 //仅在INT1脚为高及TR1置1时使能计数
#define T1_NOT_GATED  0x00		 //TR1置1时即使能计数


#define T0_init(g,c,m)          (TMOD &= 0xF0); (TMOD |= (g | c | m))
#define T0_start()              (TR0= 1)
#define T0_stop()               (TR0= 0)
#define T0_enable_int()         (ET0= 1)
#define T0_disable_int()        (ET0= 0)
#define T0_set_low(l)           (TL0 = l)
#define T0_set_high(h)          (TH0 = h)

#define T1_init(g,c,m)          (TMOD &= 0x0F); (TMOD |= (g | c | m))
#define T1_start()              (TR1= 1)
#define T1_stop()               (TR1= 0)
#define T1_enable_int()         (ET1= 1)
#define T1_disable_int()        (ET1= 0)
#define T1_set_low(l)           (TL1 = l)
#define T1_set_high(h)          (TH1 = h)


/*----- TimerX Period Computation -----*/
// Tperiod unit is ms(毫秒), FTx_IN unit is KHz.
// FTx_IN为机器周期频率,12 clocks情况下    FTx_IN = Fosc / 12.
//                      若使用1T clock, 则 FTx_IN = Fosc.

/* 16 bit Timer/counter */
//#define TIM_LOW         LOW( 65536 - (Tperiod * FTx_IN))
//#define TIM_HIGH        HIGH(65536 - (Tperiod * FTx_IN))

/* 13 bit Timer/counter */
//#define TIM_LOW         LOW( 8192 - (Tperiod * FTx_IN))
//#define TIM_HIGH        HIGH(8192 - (Tperiod * FTx_IN))

/* 8 bit Timer/counter */
//#define TIM_LOW         LOW( 256 - (Tperiod * FTx_IN))
//#define TIM_HIGH        HIGH(256 - (Tperiod * FTx_IN))


#endif  /* _TIMER_DRV_H_ */
