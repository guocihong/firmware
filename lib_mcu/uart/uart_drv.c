/*C**************************************************************************
* NAME:   uart_drv.c
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:      2013.09.04
* REVISION:     9.4     
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the UART1 & UART2 driver routines.
*
* NOTES:
*  串口通信
*	数据帧格式: 
*		发送采用中断方式: (有缓存区)
*		接收采用中断方式: (有缓存区)
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include <INTRINS.H>
#include <string.h>
#include "config.h"                     /* configuration header */
#include "lib_mcu\timer\timer_drv.h"    /* timer definition */
#include "uart_drv.h"                   /* uart driver definition */


/*_____ D E F I N I T I O N ________________________________________________*/

/*_____ D E C L A R A T I O N ______________________________________________*/
/* UART1 */									  
extern xdata  Byte  msg_buf[MAX_RecvFrame];     // received message, used for proceding
extern bdata  bit   msg_buf_valid;	            // received valid flag
extern xdata  Byte  recv_buf[MAX_RecvFrame];    // receiving buffer               
extern idata  Byte  recv_state;                 // receive state
extern idata  Byte  recv_timer;                 // receive time-out 
extern idata  Byte  recv_chksum;                // computed checksum
extern idata  Byte  recv_ctr;                   // reveiving pointer         

extern xdata  Byte  trans_buf[MAX_TransFrame];  // uart transfer buffer
extern idata  Byte  trans_size;                 // transfer bytes number  
extern idata  Byte  trans_ctr;                  // transfer pointer
extern idata  Byte  trans_chksum;               // check-sum 
extern bdata  bit   trans_occupy;               // 发送器被占用标志，1-被占用, 0-空闲  

extern  data  Byte     uart_q_index;            // 正在发送某队列项的序号：若为0xFF, 表示没有任何项进入发送流程
extern xdata  sUART_Q  uart_q[UART_QUEUE_NUM];	// 串口队列

/* for UART2 */
extern xdata  Byte  msg2_buf[MAX_RecvFrame];    // received message, used for proceding
extern bdata  bit   msg2_buf_valid;	            // received valid flag
extern xdata  Byte  recv2_buf[MAX_RecvFrame];   // receiving buffer               
extern idata  Byte  recv2_state;                // receive state
extern idata  Byte  recv2_timer;                // receive time-out, 用于字节间超时判定
extern idata  Byte  recv2_chksum;               // computed checksum
extern idata  Byte  recv2_ctr;                  // reveiving pointer 

extern xdata  Byte  trans2_buf[MAX_TransFrame];    // uart transfer buffer（发送前需要填充所有字节，除校验字节外）
extern  data  Byte  trans2_ctr;                    // transfering pointer
extern  data  Byte  trans2_size;                   // transfered bytes number（含所有字节，即含帧头和校验）
extern  data  Byte  trans2_chksum;                 // check-sum （边发送边计算）
extern bdata  bit   trans2_occupy;                 // 发送器2被占用标志，1-被占用, 0-空闲                      

extern  data  Byte     uart2_q_index;              // 正在发送某队列项的序号：若为0xFF, 表示没有任何项在发送
extern xdata  sUART_Q  uart2_q[UART2_QUEUE_NUM];   // 串口2队列


/*F***************************************************************************
* NAME:  uart_set_prio
*-----------------------------------------------------------------------------
* PARAMS: priority ： 优先级值, 范围 0 ~ 3
* return:
*----------------------------------------------------------------------------
* PURPOSE: Set uart1 interrupt priority 
*----------------------------------------------------------------------------
* REQUIREMENTS:
******************************************************************************/
void uart_set_prio(Byte priority)
{
  PS =  0;         //复位至优先值0
  IPH &= ~PSH_;

  if ((priority == 1) || (priority == 3))     // set LOW priority bit
  {
    PS = 1;
  }
  if ((priority == 2) || (priority == 3))     // set HIGH priority bit
  {
    IPH |= PSH_;
  }
}


/*F***************************************************************************
* NAME:  uart2_set_prio
*-----------------------------------------------------------------------------
* PARAMS: priority ： 优先级值
* return:
*----------------------------------------------------------------------------
* PURPOSE: Set 串口2 interrupt priority 
*----------------------------------------------------------------------------
* REQUIREMENTS:
******************************************************************************/
void uart2_set_prio(Byte priority)
{
  IP2  &= ~PS2_;	//复位至优先值0
  IP2H &= ~PS2H_;

  if ((priority == 1) || (priority == 3))     /* set LOW priority bit */
  {
    IP2 |= PS2_;
  }
  if ((priority == 2) || (priority == 3))     /* set HIGH priority bit */
  {
    IP2H |= PS2H_;
  }
}


/*F**************************************************************************
* NAME: uart_init
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: uart initial routine, 含串口1 及 串口2  
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
void uart_init(void)
{	
  // 复位变量: for UART1
  recv_state = FSA_INIT;
  recv_timer = 0;
  recv_ctr = 0;	
  //trans_buf[0] = FRAME_STX;             
  trans_size = 0;
  trans_ctr = 0;
  trans_occupy = 0;   //空闲

  // 复位变量: for UART2
  recv2_state = FSA_INIT;
  recv2_timer = 0;
  recv2_ctr = 0;
	//trans2_buf[0] = FRAME_STX;             
  trans2_size = 0;
  trans2_ctr = 0;
	trans2_occupy = 0;   //空闲

  // BRT初始化: baud rate generator
  BRT = BRT_BAUD_RATE;	  //?bps, Sysclk=22.1184MHz, SMOD=0    
  PCON &= ~SMOD_;         //SMOD=0, 波特率不加倍
	AUXR &= ~(BRTx12_ | S2SMOD_);   //BRT 使用12分频; S2SMOD=0, UART2的波特率不加倍
	AUXR |= (BRTR_ | S1BRS_);       //UART1 使用 BRT 为波特率发生器，立即运行 
	                                //UART2 只能使用 BRT 为波特率发生器
    
	// UART1 初始化   
	SCON = UART_MODE_1 + UART_SM2_0;	  //1bit start, 8bit data, 1 stop, no odd-pair      
																		  //  and clear interrupt flag   
	uart_set_prio(UART_PRIO);
	uart_enable_int(); 
	UART_RECEIVE_ENABLE();              //enable UART1 receive   
    
	// UART2 初始化   
	S2CON = UART_MODE_1 + UART_SM2_0;	  //1bit start, 8bit data, 1 stop, no odd-pair      
																		  //  and clear interrupt flag   
	uart2_set_prio(UART2_PRIO);
	uart2_enable_int(); 
	UART2_RECEIVE_ENABLE();             //enable UART2 receive 

	// RS485接口初始化
	bRS485_DE  = 0;	 //半双工，接收	
	bRS485_DE2 = 0;	 //半双工，接收 	 
	_nop_();
	_nop_();
	_nop_();
	_nop_();	
}                    


//*********************************************************************************
//	UART1中断服务程序
//*********************************************************************************
void UART_IntHandle(void) interrupt SIO_VECTOR using 2
{   
	  volatile Byte nop_num;
    Byte c;

    if (_testbit_(TI))
    { //发送中断	  
      trans_ctr ++;   //取下一个待传送index
      if (trans_ctr < trans_size)
      { //未传送完成
        if (trans_ctr == (trans_size - 1))
        { //已经指向校验字节     
          SBUF = trans_chksum;    //发送校验字节  
        }
        else
        { //非校验字节, 需要传送并计算checksum
          SBUF = trans_buf[trans_ctr];
					if (trans_ctr > 0) 
					{ //计算check_sum        
						trans_chksum += trans_buf[trans_ctr];   //更新chksum
					}
		    }
      }
      else 
			{ //已经全部传送完成(含校验字节)，可以置发送器空闲    
				//目前设计：均不需等待应答, 可以释放该队列项
				if (uart_q_index < UART_QUEUE_NUM)
				  uart_q[uart_q_index].flag = 0;   //该队列项空闲
				uart_q_index = 0xFF;	//无队列项在发送  	   	
				trans_occupy = 0;		//发送器空闲
				nop_num = NOP_NUM_WAIT;
				while (nop_num--);				
				bRS485_DE = 0;	        //禁止发送, 转为接收
				UART_RECEIVE_ENABLE();  //UART1允许接收
			}	  
      TI = 0;   //must clear by user software 
    }//end if (_testbit_(TI))
		
    if (_testbit_(RI))
    { //接收中断
      c = SBUF;      
      switch (recv_state)
      {
        case FSA_INIT://是否为帧头
                      if (c == FRAME_STX)
                      { //为帧头, 开始新的一帧                        
												recv_ctr = 0;
												recv_chksum = 0;
												recv_timer = RECV_TIMEOUT;
												recv_state = FSA_ADDR_D;
                      }
                      break;				 

        case FSA_ADDR_D://为目的地址, 开始保存并计算效验和
											recv_buf[recv_ctr++] = c;
											recv_chksum += c;                      
											recv_timer = RECV_TIMEOUT;
											recv_state = FSA_ADDR_S;
                      break;

        case FSA_ADDR_S://为源地址
											recv_buf[recv_ctr++] = c;
											recv_chksum += c;                      
											recv_timer = RECV_TIMEOUT;
											recv_state = FSA_LENGTH;					  					 
                      break;

        case FSA_LENGTH://为长度字节
                      if ((c > 0) && (c < (MAX_RecvFrame - 3))) 
                      { //有效串   
												recv_buf[recv_ctr++] = c;    //第三个字节保存长度                        
											  recv_chksum += c;						
											  recv_timer = RECV_TIMEOUT; 
											  recv_state = FSA_DATA;				    
                      }
											else
											{	//非有效串
												recv_state = FSA_INIT;
											}
                      break;

        case FSA_DATA://读取命令串
					            recv_buf[recv_ctr] = c;     
                      recv_chksum += c;   //更新校验和                                       
                      if (recv_ctr == (recv_buf[2] + 2))             
                      { //已经收到指定长度的命令数据
                        recv_state = FSA_CHKSUM;
                      }
											else
											{	//还未结束
												recv_ctr ++;
											}
                      recv_timer = RECV_TIMEOUT;
                      break;

        case FSA_CHKSUM://检查校验字节
                      if ((recv_chksum == c) && (msg_buf_valid == FALSE))
                      { //已经收到完整一帧并且以前收到的消息已经被处理了
                        memcpy(msg_buf, recv_buf, recv_buf[2] + 3);
                        msg_buf_valid = TRUE;
                      }
        default:      //复位
                      recv_state = FSA_INIT;
                      break;
      }         
      RI = 0;     //must clear by user software
    }//end if (_testbit_(RI))
}


//*********************************************************************************
//	UART2中断服务程序
//*********************************************************************************
void UART2_IntHandle(void) interrupt SIO2_VECTOR  using 3
{   
	  volatile Byte nop_num;
    Byte c;

    if ((S2CON & S2TI_) == S2TI_)
    { //UART2发送中断	  
      trans2_ctr ++;   //取下一个待传送index
      if (trans2_ctr < trans2_size)
      { //未传送完成
        if (trans2_ctr == (trans2_size - 1))
        { //已经指向校验字节     
          S2BUF = trans2_chksum;    //发送校验字节  
        }
        else
        { //非校验字节, 需要传送并计算checksum
          S2BUF = trans2_buf[trans2_ctr];
					if (trans2_ctr > 0) 
					{ //计算check_sum        
						trans2_chksum += trans2_buf[trans2_ctr];   //更新chksum
					}
		    }
      }
      else 
			{ //已经全部传送完成(含校验字节)，可以置发送器空闲 
				//目前设计：均不需等待应答, 可以释放该队列项
				if (uart2_q_index < UART2_QUEUE_NUM)
				  uart2_q[uart2_q_index].flag = 0;   //该队列项空闲
				uart2_q_index = 0xFF;	   //无队列项在发送
				trans2_occupy = 0;	   	 //发送器空闲  
				nop_num = NOP_NUM_WAIT;
				while (nop_num--);	
				bRS485_DE2 = 0;	         //禁止发送, 转为接收 		
				UART2_RECEIVE_ENABLE();  //UART2允许接收
			}	  
      S2CON &= ~S2TI_;   //must clear by user software 
    }//end 发送中断

    if ((S2CON & S2RI_) == S2RI_)
    { //接收中断 	
      c = S2BUF; 
      switch (recv2_state)
      {
        case FSA_INIT://是否为帧头
                      if (c == FRAME_STX)
                      { //为帧头, 开始新的一帧
												recv2_ctr = 0;
												recv2_chksum = 0;
												recv2_timer = RECV_TIMEOUT;
												recv2_state = FSA_ADDR_D;					   			
                      }
                      break;

        case FSA_ADDR_D://为目的地址, 开始保存并计算效验和
											recv2_buf[recv2_ctr++] = c;
											recv2_chksum += c;                      
											recv2_timer = RECV_TIMEOUT;
											recv2_state = FSA_ADDR_S;
                      break;

        case FSA_ADDR_S://为源地址
											recv2_buf[recv2_ctr++] = c;
											recv2_chksum += c;                      
											recv2_timer = RECV_TIMEOUT;
											recv2_state = FSA_LENGTH;					  					 
                      break;

        case FSA_LENGTH://为长度字节
                      if ((c > 0) && (c < (MAX_RecvFrame - 3)))
                      { //有效串   
												recv2_buf[recv2_ctr++] = c;    //第三个字节保存长度                        
												recv2_chksum += c;	
												recv2_timer = RECV_TIMEOUT; 
												recv2_state = FSA_DATA;				    
                      }
											else
											{	//非有效串
											  recv2_state = FSA_INIT;
											}
                      break;

        case FSA_DATA://读取命令串
					            recv2_buf[recv2_ctr] = c;     
                      recv2_chksum += c;   //更新校验和                                       
                      if (recv2_ctr == (recv2_buf[2] + 2))             
                      { //已经收到指定长度的命令数据
                        recv2_state = FSA_CHKSUM;
                      }
											else
											{	//还未结束
												recv2_ctr ++;
											}
                      recv2_timer = RECV_TIMEOUT;
                      break;

        case FSA_CHKSUM://检查校验字节
                      if ((recv2_chksum == c) && (msg2_buf_valid == FALSE))
                      { //已经收到完整一帧并且以前收到的消息已经被处理了
                        memcpy(msg2_buf, recv2_buf, recv2_buf[2] + 3);
                        msg2_buf_valid = TRUE;
                      }
        default:      //复位
                      recv2_state = FSA_INIT; 
                      break;
      }         
      S2CON &= ~S2RI_;     //must clear by user software
    }//end 接收中断
}


/*F**************************************************************************
* NAME: uart_start_trans
*----------------------------------------------------------------------------
* PARAMS: 
* return:
*----------------------------------------------------------------------------
* PURPOSE: 开始串口1发送
*----------------------------------------------------------------------------
* REQUIREMENTS:	 使用顺序如下
*                a. while (trans_occupy)   //等待前一个发送完成
*                b. 填trans_buf[] 
*                c. 置trans_size
*                d. 调用本函数
* ZZX： 若使用UART发送队列，则所有发送有 comm_task() 处理。
*****************************************************************************/
void uart_start_trans(void)
{ 
	volatile Byte nop_num;
	
  UART_RECEIVE_DISABLE();
  _nop_();
	_nop_();
  bRS485_DE = 1;    //允许发送
	nop_num = NOP_NUM_WAIT;
	while (nop_num--);
  trans_occupy = 1;       
  trans_chksum = 0;  
  trans_ctr = 0;               
  SBUF = trans_buf[trans_ctr]; 
}


/*F**************************************************************************
* NAME: uart2_start_trans
*----------------------------------------------------------------------------
* PARAMS: 
* return:
*----------------------------------------------------------------------------
* PURPOSE: 开始串口2发送
*----------------------------------------------------------------------------
* REQUIREMENTS:	 使用顺序如下
*                a. while (trans2_occupy)   //等待前一个发送完成
*                b. 填trans2_buf[] 
*                c. 置trans2_size
*                d. 调用本函数
* ZZX： 若使用UART发送队列，则所有发送处理有 comm_task() 处理。
*****************************************************************************/
void uart2_start_trans(void)
{ 
	volatile Byte nop_num;
	
  UART2_RECEIVE_DISABLE();
  _nop_();
  _nop_();
  bRS485_DE2 = 1;    //允许发送
	nop_num = NOP_NUM_WAIT;
	while (nop_num--);  
  trans2_occupy = 1;      
  trans2_chksum = 0;  
  trans2_ctr = 0;               
  S2BUF = trans2_buf[trans2_ctr];  
}
