/*H**************************************************************************
* NAME:  uart_drv.h         
*----------------------------------------------------------------------------
* RELEASE:      2011.01.13   
* REVISION:     1.0
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the uart driver definition.
*****************************************************************************/

#ifndef _UART_DRV_H_
#define _UART_DRV_H_


/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"               /* configuration header */


/*_____ M A C R O S ________________________________________________________*/
#define UART_MODE_0     0x00      //UART mode 0 : 8bit shift-register
                                  //  当 UART_M0x6 = 0 时,波特率是 SYSclk / 12
								                  //  当 UART_M0x6 = 1 时,波特率是 SYSclk /  2
#define UART_MODE_1     0x40      //UART mode 1 : 10bit UART, variable-baud rate
                                  //  波特率 = ((2幂SMOD) / 32) x (定时器1或BRT的溢出率)
																	//  也就是说，若 SMOD=0，波特率 = 定时器1或BRT的溢出率 / 32
																	//            若 SMOD=1，波特率 = 定时器1或BRT的溢出率 / 16
#define UART_MODE_2     0x80      //UART mode 2 : 11bit UART
                                  //  波特率 = ((2幂SMOD) / 64) x (SYSclk 系统工作时钟频率)
#define UART_MODE_3     0xC0      //UART mode 3 : 11bit UART, variable-baud rate
                                  //  波特率 = ((2幂SMOD) / 32) x (定时器1或BRT的溢出率)

//注: 当 T1x12  = 0 时,定时器1的溢出率 = SYSclk / 12 / (256 - TH1)
//注: 当 T1x12  = 1 时,定时器1的溢出率 = SYSclk / (256 - TH1)
//注: 当 BRTx12 = 0 时,BRT的溢出率 = SYSclk / 12 / (256 - BRT)
//注: 当 BRTx12 = 1 时,BRT的溢出率 = SYSclk / (256 - BRT)

#define UART_SM2_0      0x00      //此位用于控制多处理机通信, 仅当UART 模式为2或3时
#define UART_SM2_1      0x20
//注: 当 UART 工作于模式0或1时, 应设置 SM2 为 0. 


/* UART enable */
#define UART_RECEIVE_ENABLE()    (REN = 1)
#define UART_RECEIVE_DISABLE()   (REN = 0)

/* interrupt enable */
#define uart_enable_int()        (ES = 1)
#define uart_disable_int()       (ES = 0)

/* baud rate */
#define uart_baud_x2             (PCON |= SMOD_)    //仅当 UART 为方式1、2、3 时
#define uart_baud_x1             (PCON &= ~SMOD_)
//ZZX: 上面宏只针对串口1, 串口2的波特率加倍由 AUXR.S2SMOD 位值决定.


//ZZX: 以下适用于 UART2 								  
/* UART2 enable */
#define UART2_RECEIVE_ENABLE()    (S2CON |= S2REN_)
#define UART2_RECEIVE_DISABLE()   (S2CON &= ~S2REN_)

/* interrupt enable */
#define uart2_enable_int()        (IE2 |= ES2_)
#define uart2_disable_int()       (IE2 &= ~ES2_)

/* baud rate */
#define uart2_baud_x2             (AUXR |= S2SMOD_)    //仅当 UART 为方式1、2、3 时
#define uart2_baud_x1             (AUXR &= ~S2SMOD_)


/* state constant(仅用于接收) */
#define FSA_INIT      0      //等待帧头
#define FSA_ADDR_D    1      //等待目的地址
#define FSA_ADDR_S    2      //等待源地址
#define FSA_LENGTH    3      //等待长度字节
#define FSA_DATA      4      //等待命令串(包括 命令ID 及 参数)
#define FSA_CHKSUM    5      //等待校验和

/*_____ D E C L A R A T I O N ______________________________________________*/
//for UART1
void   uart_set_prio(Byte priority);     //设置uart1的中断优先级
void   uart_start_trans(void);           //开始串口1发送

//for UART2
void   uart2_set_prio(Byte priority);    //设置uart2的中断优先级
void   uart2_start_trans(void);          //开始串口2发送

//for UART1 & UART2
void   uart_init(void);                  //串口初始化, 含UART1 及 UART2 

#endif   /* _UART_DRV_H_ */
