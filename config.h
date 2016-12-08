/*H**************************************************************************
* NAME:  config.h         
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:      2013.09.03
* REVISION:     9.4    
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the system configuration definition
*****************************************************************************/

#ifndef _CONFIG_H_
#define _CONFIG_H_


/*_____ I N C L U D E S ____________________________________________________*/
#include "lib_mcu\compiler.h"           /* compiler definitions */
#include "lib_mcu\regs52.h"             /* SFR declaration */
#include "stress.h"

/* Clock */
#define FOSC_Hz   ((Uint32)22118400)    /* oscillator frequency (Hz)  */

/* Interrupt Priorities : 取值范围为 0 ~ 3 */
#define EX0_PRIO           ((Byte)0)
#define EX1_PRIO           ((Byte)0)
#define T0_PRIO            ((Byte)2)    //调度器
#define T1_PRIO            ((Byte)0)    
#define UART_PRIO          ((Byte)2)    //UART1数据收发中断
#define ADC_PRIO           ((Byte)0)	  //ADC中断(没有使用)
#define LVD_PRIO           ((Byte)0)
#define PCA_PRIO           ((Byte)0)
#define UART2_PRIO         ((Byte)2)    //UART2数据收发中断
#define SPI_PRIO           ((Byte)0)

/* Timer adjust */
#define TIMER_ADJ_T0       0     // compensate for TIMER0 enter/exit interrupt                                 
#define TIMER_ADJ_T1       0     // compensate for TIMER1 enter/exit interrupt

/* Scheduler Tick */
#define SCHEDULER_TICK     5     // unit is ms

/* UART */
#define	FRAME_STX        0x16	   // Frame header
#define	MAX_RecvFrame      45    // 接收缓存区大小
#define	MAX_TransFrame     45    // 发送缓存区大小
#define RECV_TIMEOUT        4    // 字节间的最大时间间隔, 单位为tick
                                 // 最小值可以为1, 如果为0则表示不进行超时判定
                          
/* Uart Queue */
typedef struct strUART_Q
{
  Byte  flag;         //状态： 0 - 空闲； 1 - 等待发送； 2 - 正在发送; 3 - 已发送，等待应答
//Byte 	external;     //目的地： 0 - 外485总线; 1 - 其它
  Byte  tdata[MAX_TransFrame];  //数据包(最后一个校验字节可以不提前计算，而在发送时边发送边计算)
  Byte  len;					      //数据包有效长度(含校验字节)
//Byte  need_wait_ack;      //是否需要等待目的机应答： 0 - 否； 1 - 是
//Byte  wait_ack_time;			//发送后等待目的机应答所剩余的时间，单位： tick 
                            //  当flag=3,且等待应答剩余时间减为0时，重发       
  Byte  package_type;       //0-来自下位机的数据包;1-设备自身的数据包
}sUART_Q;

//#define UART_WAIT_ACK  (2000/SCHEDULER_TICK)	  //发送后等待目的机应答所允许的最大时间, 单位：tick
#define UART_QUEUE_NUM   4   //UART1 队列数, 用于上位机
#define UART2_QUEUE_NUM	 4   //UART2 队列数, 用于下位机 

/* Port & Pin definition */
// P0, P2 - Mask输入/LED驱动 
#define bP0_Gate        P0_7     // 推挽输出, mask值输入门控： 1 - 禁止; 0 - 允许输入拨码开关值
#define bP2_Gate        P0_6     // 推挽输出, mask值输入门控： 1 - 禁止; 0 - 允许输入拨码开关值
#define bLED_JL_MASK    0x3C     // 0011 1100b

// P1 - AD0,485通信地址
#define CommAddr_Port   P1       // 准双向口(输入), RS485通信地址(7位), 地址值与端口脚值相反 
#define b485_ADDR_MASK  0xFE     // 1111 1110b
#define bADC_IN         P1_0     // AD 输入, 高阻

//P3 - 
#define bBeep_Ctrl   P3_7        // 推挽输出，Beep控制:  1-蜂鸣; 0-禁鸣
#define bRS485_DE2   P3_6        // 推挽输出，RS485发送使能: 1-允许发送; 0 - 禁止发送(接收)
#define bRelay_A2    P3_5		     // 推挽输出，(右侧)报警输出2：1-继电器加电吸合(上电缺省)； 0-不加电
#define bRelay_A1    P3_4		     // 推挽输出，(左侧)报警输出1：1-继电器加电吸合(上电缺省)； 0-不加电
#define bRelay_L1    P3_3		     // 推挽输出，联动输出 ：1-继电器加电吸合； 0-不加电(上电缺省)
#define bRS485_DE    P3_2        // 推挽输出，RS485发送使能: 1-允许发送; 0 - 禁止发送(接收)
#define bTxD         P3_1 		   //	准双向口，UART发送
#define bRxD         P3_0        //	准双向口，UART接收   

//P4 - 
#define bDoorKeeper  P4_6        // 准双向口-输入，门磁检测: 1-门磁关闭; 0-门磁打开（应报警）
#define bTxD2        P4_3        // 准双向口, UART2发送
#define bRxD2        P4_2        // 准双向口, UART2接收
#define bSel_2or1    P4_0	       // 准双向口, 双/单防区选择输入: 1 - 双防区; 0 - 单防区

//P5 - AD采样通道选择
#define bSel_ADC_MASK  0x0F      // 0000 1111b, 推挽输出                                

/* AD */
typedef struct strAD_Sample
{ //每点采样值
  Uint16   val;     //当前采样值
  Uint8    index;   //通道号，范围0 ~ 13
  Byte     valid;   //采样数据处理标志: 0 - 已处理，可以写入新值; 1 - 新值，等待处理                                    
}sAD_Sample;

typedef struct strAD_Sum
{ //采样值累加和
  Uint16   sum;     //累计和 (最多达64点,不会溢出)
  Uint8    point;   //已采样点数
}sAD_Sum;

typedef struct strAD_BASE
{ //系统运行时静态基准值对应的采样值
  Uint16   base;       //静态基准值
  Uint16   base_down;  //基准值下限(含)
  Uint16   base_up;    //基准值上限(含)
}sAD_BASE;

/* System status */
#define SYS_PowerON     0      // 0 - 初始上电
#define SYS_B5S         1      // 1 - 基准值采样前延时(约5秒)
#define SYS_SAMP_BASE   2      // 2 - 基准值采样(10秒左右)
#define SYS_CHECK       3      // 3 - 实时监测

/* WDT */
#define Enable_WDT             //使能WDT


/* 485_DE Enable/disable delay */
#define NOP_NUM_WAIT  100


//when UART_MODE_1 + UART_SM2_0 + SMOD=0
//     BRT = 256 - ((SYSclk / 12 / 32) / 波特率)
#define BRT_BAUD_9600  (256 -  6)
#define BRT_BAUD_4800  (256 - 12)
#define BRT_BAUD_2400  (256 - 24)
#define BRT_BAUD_1200  (256 - 48)

#define BRT_BAUD_RATE   BRT_BAUD_9600

//2016-12-07新增
typedef struct strAlarmDetailInfo
{
    Union16  InstantSampleValue[14];//瞬间张力
    sAD_BASE StaticBaseValue[14];   //静态基准
    Uint16  ExternalAlarm     ;    //外力报警            
    Uint16  StaticAlarm;	       //静态报警                                					
    Byte    DoorKeepAlarm;         //门磁报警
}sAlarmDetailInfo;
#endif    /* _CONFIG_H_ */
