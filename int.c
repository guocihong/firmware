/*C**************************************************************************
* NAME:   int.c
*----------------------------------------------------------------------------
* Copyright (c) 2009.
*----------------------------------------------------------------------------
* RELEASE:   2009.04.07   
* REVISION:     1.0     
*----------------------------------------------------------------------------
* PURPOSE:
*   This file contains the ex0/1 routines.
* NOTES:
*****************************************************************************/


/*_____ I N C L U D E S ____________________________________________________*/
#include "INTRINS.H"
#include "config.h"                  /* configuration header */
#include "lib_mcu\c51_drv.h"         /* c51 driver definition */
#include "int.h"                     /* configuration header */


/*_____ D E F I N I T I O N ________________________________________________*/


/*_____ D E C L A R A T I O N ______________________________________________*/
/* UART */
extern xdata  Byte   msg_buf[MAX_RecvFrame];     // have received message buffer 
extern bdata  bit    msg_buf_valid;	             // whether received valid message flag
extern xdata  Byte   recv_buf[MAX_RecvFrame];    // is receiving message buffer               
extern idata  Byte   recv_state;                 // receive state
extern idata  Byte   recv_timer;                 // receive time-out 
extern xdata  Byte   recv_chksum;                // have received message checksum
extern xdata  Byte   recv_ctr;                   // reveive pointer         

extern xdata  Byte   trans_buf[MAX_TransFrame];  // uart transfer message buffer
extern xdata  Byte   trans_ctr;                  // transfer pointer
extern xdata  Byte   trans_size;                 // transfer bytes number  
extern xdata  Byte   trans_chksum;               // computed check-sum of already transfered message 
extern bdata  bit    trans_occupy;               // 发送器被占用标志，1-被占用, 0-空闲  

/* UART Queue */
extern  data  Byte     uart_q_index;             // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
extern xdata  sUART_Q  uart_q[UART_QUEUE_NUM];	 // 串口队列


/*F***************************************************************************
* NAME:  ex0_set_prio
*-----------------------------------------------------------------------------
* PARAMS: priority ： 优先级值
* return:
*----------------------------------------------------------------------------
* PURPOSE: Set the external-interrupt0 priority 
*----------------------------------------------------------------------------
* EXAMPLE:
*----------------------------------------------------------------------------
* REQUIREMENTS:
******************************************************************************/
/*
void ex0_set_prio(Byte priority)
{ 
  PX0 =  0;       //复位至优先值0
  IPH &= ~PX0H_;

  if ((priority == 1) || (priority == 3))     //set LOW priority bit
  {
    PX0 = 1;
  }
  if ((priority == 2) || (priority == 3))     //set HIGH priority bit
  {
    IPH |= PX0H_;
  }
}
*/


/*F***************************************************************************
* NAME:   ex1_set_prio
*-----------------------------------------------------------------------------
* PARAMS: priority ： 优先级值
* return:
*----------------------------------------------------------------------------
* PURPOSE: Set the external-interrupt1 priority 
*----------------------------------------------------------------------------
* EXAMPLE:
*----------------------------------------------------------------------------
* REQUIREMENTS:
******************************************************************************/
void ex1_set_prio(Byte priority)
{ 
  PX1 =  0;       //复位至优先值0
  IPH &= ~PX1H_;

  if ((priority == 1) || (priority == 3))     //set LOW priority bit
  {
    PX1 = 1;
  }
  if ((priority == 2) || (priority == 3))     //set HIGH priority bit
  {
    IPH |= PX1H_;
  }
}


/*F**************************************************************************
* NAME: ISR_EX1
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: 
*   EX1 interrupt service routine 
*----------------------------------------------------------------------------
* EXAMPLE:
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
void ISR_EX1(void) interrupt IE1_VECTOR using 3
{
  if (uart_q_index < UART_QUEUE_NUM)
  { //有进入发送流程的队列项
    if (uart_q[uart_q_index].flag == 3) 	   
	{ //已收到应答, 释放该项
	  uart_q[uart_q_index].flag = 0;
	  uart_q_index = 0xFF;
	}
  }
}


/*F**************************************************************************
* NAME: ex1_init
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: EX1 initialize
*----------------------------------------------------------------------------
* EXAMPLE:
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
void ex1_init(void)
{
  IE1 = 0;   //清标志
  IT1 = 1;   //下降沿有效   

  ex1_set_prio(EX1_PRIO);
  EX1_enable_int();
}

