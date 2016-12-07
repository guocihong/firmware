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
*  ����ͨ��
*	����֡��ʽ: 
*		���Ͳ����жϷ�ʽ: (�л�����)
*		���ղ����жϷ�ʽ: (�л�����)
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
extern bdata  bit   trans_occupy;               // ��������ռ�ñ�־��1-��ռ��, 0-����  

extern  data  Byte     uart_q_index;            // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ�����뷢������
extern xdata  sUART_Q  uart_q[UART_QUEUE_NUM];	// ���ڶ���

/* for UART2 */
extern xdata  Byte  msg2_buf[MAX_RecvFrame];    // received message, used for proceding
extern bdata  bit   msg2_buf_valid;	            // received valid flag
extern xdata  Byte  recv2_buf[MAX_RecvFrame];   // receiving buffer               
extern idata  Byte  recv2_state;                // receive state
extern idata  Byte  recv2_timer;                // receive time-out, �����ֽڼ䳬ʱ�ж�
extern idata  Byte  recv2_chksum;               // computed checksum
extern idata  Byte  recv2_ctr;                  // reveiving pointer 

extern xdata  Byte  trans2_buf[MAX_TransFrame];    // uart transfer buffer������ǰ��Ҫ��������ֽڣ���У���ֽ��⣩
extern  data  Byte  trans2_ctr;                    // transfering pointer
extern  data  Byte  trans2_size;                   // transfered bytes number���������ֽڣ�����֡ͷ��У�飩
extern  data  Byte  trans2_chksum;                 // check-sum ���߷��ͱ߼��㣩
extern bdata  bit   trans2_occupy;                 // ������2��ռ�ñ�־��1-��ռ��, 0-����                      

extern  data  Byte     uart2_q_index;              // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ����ڷ���
extern xdata  sUART_Q  uart2_q[UART2_QUEUE_NUM];   // ����2����


/*F***************************************************************************
* NAME:  uart_set_prio
*-----------------------------------------------------------------------------
* PARAMS: priority �� ���ȼ�ֵ, ��Χ 0 ~ 3
* return:
*----------------------------------------------------------------------------
* PURPOSE: Set uart1 interrupt priority 
*----------------------------------------------------------------------------
* REQUIREMENTS:
******************************************************************************/
void uart_set_prio(Byte priority)
{
  PS =  0;         //��λ������ֵ0
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
* PARAMS: priority �� ���ȼ�ֵ
* return:
*----------------------------------------------------------------------------
* PURPOSE: Set ����2 interrupt priority 
*----------------------------------------------------------------------------
* REQUIREMENTS:
******************************************************************************/
void uart2_set_prio(Byte priority)
{
  IP2  &= ~PS2_;	//��λ������ֵ0
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
* PURPOSE: uart initial routine, ������1 �� ����2  
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
void uart_init(void)
{	
  // ��λ����: for UART1
  recv_state = FSA_INIT;
  recv_timer = 0;
  recv_ctr = 0;	
  //trans_buf[0] = FRAME_STX;             
  trans_size = 0;
  trans_ctr = 0;
  trans_occupy = 0;   //����

  // ��λ����: for UART2
  recv2_state = FSA_INIT;
  recv2_timer = 0;
  recv2_ctr = 0;
	//trans2_buf[0] = FRAME_STX;             
  trans2_size = 0;
  trans2_ctr = 0;
	trans2_occupy = 0;   //����

  // BRT��ʼ��: baud rate generator
  BRT = BRT_BAUD_RATE;	  //?bps, Sysclk=22.1184MHz, SMOD=0    
  PCON &= ~SMOD_;         //SMOD=0, �����ʲ��ӱ�
	AUXR &= ~(BRTx12_ | S2SMOD_);   //BRT ʹ��12��Ƶ; S2SMOD=0, UART2�Ĳ����ʲ��ӱ�
	AUXR |= (BRTR_ | S1BRS_);       //UART1 ʹ�� BRT Ϊ�����ʷ��������������� 
	                                //UART2 ֻ��ʹ�� BRT Ϊ�����ʷ�����
    
	// UART1 ��ʼ��   
	SCON = UART_MODE_1 + UART_SM2_0;	  //1bit start, 8bit data, 1 stop, no odd-pair      
																		  //  and clear interrupt flag   
	uart_set_prio(UART_PRIO);
	uart_enable_int(); 
	UART_RECEIVE_ENABLE();              //enable UART1 receive   
    
	// UART2 ��ʼ��   
	S2CON = UART_MODE_1 + UART_SM2_0;	  //1bit start, 8bit data, 1 stop, no odd-pair      
																		  //  and clear interrupt flag   
	uart2_set_prio(UART2_PRIO);
	uart2_enable_int(); 
	UART2_RECEIVE_ENABLE();             //enable UART2 receive 

	// RS485�ӿڳ�ʼ��
	bRS485_DE  = 0;	 //��˫��������	
	bRS485_DE2 = 0;	 //��˫�������� 	 
	_nop_();
	_nop_();
	_nop_();
	_nop_();	
}                    


//*********************************************************************************
//	UART1�жϷ������
//*********************************************************************************
void UART_IntHandle(void) interrupt SIO_VECTOR using 2
{   
	  volatile Byte nop_num;
    Byte c;

    if (_testbit_(TI))
    { //�����ж�	  
      trans_ctr ++;   //ȡ��һ��������index
      if (trans_ctr < trans_size)
      { //δ�������
        if (trans_ctr == (trans_size - 1))
        { //�Ѿ�ָ��У���ֽ�     
          SBUF = trans_chksum;    //����У���ֽ�  
        }
        else
        { //��У���ֽ�, ��Ҫ���Ͳ�����checksum
          SBUF = trans_buf[trans_ctr];
					if (trans_ctr > 0) 
					{ //����check_sum        
						trans_chksum += trans_buf[trans_ctr];   //����chksum
					}
		    }
      }
      else 
			{ //�Ѿ�ȫ���������(��У���ֽ�)�������÷���������    
				//Ŀǰ��ƣ�������ȴ�Ӧ��, �����ͷŸö�����
				if (uart_q_index < UART_QUEUE_NUM)
				  uart_q[uart_q_index].flag = 0;   //�ö��������
				uart_q_index = 0xFF;	//�޶������ڷ���  	   	
				trans_occupy = 0;		//����������
				nop_num = NOP_NUM_WAIT;
				while (nop_num--);				
				bRS485_DE = 0;	        //��ֹ����, תΪ����
				UART_RECEIVE_ENABLE();  //UART1�������
			}	  
      TI = 0;   //must clear by user software 
    }//end if (_testbit_(TI))
		
    if (_testbit_(RI))
    { //�����ж�
      c = SBUF;      
      switch (recv_state)
      {
        case FSA_INIT://�Ƿ�Ϊ֡ͷ
                      if (c == FRAME_STX)
                      { //Ϊ֡ͷ, ��ʼ�µ�һ֡                        
												recv_ctr = 0;
												recv_chksum = 0;
												recv_timer = RECV_TIMEOUT;
												recv_state = FSA_ADDR_D;
                      }
                      break;				 

        case FSA_ADDR_D://ΪĿ�ĵ�ַ, ��ʼ���沢����Ч���
											recv_buf[recv_ctr++] = c;
											recv_chksum += c;                      
											recv_timer = RECV_TIMEOUT;
											recv_state = FSA_ADDR_S;
                      break;

        case FSA_ADDR_S://ΪԴ��ַ
											recv_buf[recv_ctr++] = c;
											recv_chksum += c;                      
											recv_timer = RECV_TIMEOUT;
											recv_state = FSA_LENGTH;					  					 
                      break;

        case FSA_LENGTH://Ϊ�����ֽ�
                      if ((c > 0) && (c < (MAX_RecvFrame - 3))) 
                      { //��Ч��   
												recv_buf[recv_ctr++] = c;    //�������ֽڱ��泤��                        
											  recv_chksum += c;						
											  recv_timer = RECV_TIMEOUT; 
											  recv_state = FSA_DATA;				    
                      }
											else
											{	//����Ч��
												recv_state = FSA_INIT;
											}
                      break;

        case FSA_DATA://��ȡ���
					            recv_buf[recv_ctr] = c;     
                      recv_chksum += c;   //����У���                                       
                      if (recv_ctr == (recv_buf[2] + 2))             
                      { //�Ѿ��յ�ָ�����ȵ���������
                        recv_state = FSA_CHKSUM;
                      }
											else
											{	//��δ����
												recv_ctr ++;
											}
                      recv_timer = RECV_TIMEOUT;
                      break;

        case FSA_CHKSUM://���У���ֽ�
                      if ((recv_chksum == c) && (msg_buf_valid == FALSE))
                      { //�Ѿ��յ�����һ֡������ǰ�յ�����Ϣ�Ѿ���������
                        memcpy(msg_buf, recv_buf, recv_buf[2] + 3);
                        msg_buf_valid = TRUE;
                      }
        default:      //��λ
                      recv_state = FSA_INIT;
                      break;
      }         
      RI = 0;     //must clear by user software
    }//end if (_testbit_(RI))
}


//*********************************************************************************
//	UART2�жϷ������
//*********************************************************************************
void UART2_IntHandle(void) interrupt SIO2_VECTOR  using 3
{   
	  volatile Byte nop_num;
    Byte c;

    if ((S2CON & S2TI_) == S2TI_)
    { //UART2�����ж�	  
      trans2_ctr ++;   //ȡ��һ��������index
      if (trans2_ctr < trans2_size)
      { //δ�������
        if (trans2_ctr == (trans2_size - 1))
        { //�Ѿ�ָ��У���ֽ�     
          S2BUF = trans2_chksum;    //����У���ֽ�  
        }
        else
        { //��У���ֽ�, ��Ҫ���Ͳ�����checksum
          S2BUF = trans2_buf[trans2_ctr];
					if (trans2_ctr > 0) 
					{ //����check_sum        
						trans2_chksum += trans2_buf[trans2_ctr];   //����chksum
					}
		    }
      }
      else 
			{ //�Ѿ�ȫ���������(��У���ֽ�)�������÷��������� 
				//Ŀǰ��ƣ�������ȴ�Ӧ��, �����ͷŸö�����
				if (uart2_q_index < UART2_QUEUE_NUM)
				  uart2_q[uart2_q_index].flag = 0;   //�ö��������
				uart2_q_index = 0xFF;	   //�޶������ڷ���
				trans2_occupy = 0;	   	 //����������  
				nop_num = NOP_NUM_WAIT;
				while (nop_num--);	
				bRS485_DE2 = 0;	         //��ֹ����, תΪ���� 		
				UART2_RECEIVE_ENABLE();  //UART2�������
			}	  
      S2CON &= ~S2TI_;   //must clear by user software 
    }//end �����ж�

    if ((S2CON & S2RI_) == S2RI_)
    { //�����ж� 	
      c = S2BUF; 
      switch (recv2_state)
      {
        case FSA_INIT://�Ƿ�Ϊ֡ͷ
                      if (c == FRAME_STX)
                      { //Ϊ֡ͷ, ��ʼ�µ�һ֡
												recv2_ctr = 0;
												recv2_chksum = 0;
												recv2_timer = RECV_TIMEOUT;
												recv2_state = FSA_ADDR_D;					   			
                      }
                      break;

        case FSA_ADDR_D://ΪĿ�ĵ�ַ, ��ʼ���沢����Ч���
											recv2_buf[recv2_ctr++] = c;
											recv2_chksum += c;                      
											recv2_timer = RECV_TIMEOUT;
											recv2_state = FSA_ADDR_S;
                      break;

        case FSA_ADDR_S://ΪԴ��ַ
											recv2_buf[recv2_ctr++] = c;
											recv2_chksum += c;                      
											recv2_timer = RECV_TIMEOUT;
											recv2_state = FSA_LENGTH;					  					 
                      break;

        case FSA_LENGTH://Ϊ�����ֽ�
                      if ((c > 0) && (c < (MAX_RecvFrame - 3)))
                      { //��Ч��   
												recv2_buf[recv2_ctr++] = c;    //�������ֽڱ��泤��                        
												recv2_chksum += c;	
												recv2_timer = RECV_TIMEOUT; 
												recv2_state = FSA_DATA;				    
                      }
											else
											{	//����Ч��
											  recv2_state = FSA_INIT;
											}
                      break;

        case FSA_DATA://��ȡ���
					            recv2_buf[recv2_ctr] = c;     
                      recv2_chksum += c;   //����У���                                       
                      if (recv2_ctr == (recv2_buf[2] + 2))             
                      { //�Ѿ��յ�ָ�����ȵ���������
                        recv2_state = FSA_CHKSUM;
                      }
											else
											{	//��δ����
												recv2_ctr ++;
											}
                      recv2_timer = RECV_TIMEOUT;
                      break;

        case FSA_CHKSUM://���У���ֽ�
                      if ((recv2_chksum == c) && (msg2_buf_valid == FALSE))
                      { //�Ѿ��յ�����һ֡������ǰ�յ�����Ϣ�Ѿ���������
                        memcpy(msg2_buf, recv2_buf, recv2_buf[2] + 3);
                        msg2_buf_valid = TRUE;
                      }
        default:      //��λ
                      recv2_state = FSA_INIT; 
                      break;
      }         
      S2CON &= ~S2RI_;     //must clear by user software
    }//end �����ж�
}


/*F**************************************************************************
* NAME: uart_start_trans
*----------------------------------------------------------------------------
* PARAMS: 
* return:
*----------------------------------------------------------------------------
* PURPOSE: ��ʼ����1����
*----------------------------------------------------------------------------
* REQUIREMENTS:	 ʹ��˳������
*                a. while (trans_occupy)   //�ȴ�ǰһ���������
*                b. ��trans_buf[] 
*                c. ��trans_size
*                d. ���ñ�����
* ZZX�� ��ʹ��UART���Ͷ��У������з����� comm_task() ����
*****************************************************************************/
void uart_start_trans(void)
{ 
	volatile Byte nop_num;
	
  UART_RECEIVE_DISABLE();
  _nop_();
	_nop_();
  bRS485_DE = 1;    //������
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
* PURPOSE: ��ʼ����2����
*----------------------------------------------------------------------------
* REQUIREMENTS:	 ʹ��˳������
*                a. while (trans2_occupy)   //�ȴ�ǰһ���������
*                b. ��trans2_buf[] 
*                c. ��trans2_size
*                d. ���ñ�����
* ZZX�� ��ʹ��UART���Ͷ��У������з��ʹ����� comm_task() ����
*****************************************************************************/
void uart2_start_trans(void)
{ 
	volatile Byte nop_num;
	
  UART2_RECEIVE_DISABLE();
  _nop_();
  _nop_();
  bRS485_DE2 = 1;    //������
	nop_num = NOP_NUM_WAIT;
	while (nop_num--);  
  trans2_occupy = 1;      
  trans2_chksum = 0;  
  trans2_ctr = 0;               
  S2BUF = trans2_buf[trans2_ctr];  
}
