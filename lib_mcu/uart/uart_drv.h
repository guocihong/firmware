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
                                  //  �� UART_M0x6 = 0 ʱ,�������� SYSclk / 12
								                  //  �� UART_M0x6 = 1 ʱ,�������� SYSclk /  2
#define UART_MODE_1     0x40      //UART mode 1 : 10bit UART, variable-baud rate
                                  //  ������ = ((2��SMOD) / 32) x (��ʱ��1��BRT�������)
																	//  Ҳ����˵���� SMOD=0�������� = ��ʱ��1��BRT������� / 32
																	//            �� SMOD=1�������� = ��ʱ��1��BRT������� / 16
#define UART_MODE_2     0x80      //UART mode 2 : 11bit UART
                                  //  ������ = ((2��SMOD) / 64) x (SYSclk ϵͳ����ʱ��Ƶ��)
#define UART_MODE_3     0xC0      //UART mode 3 : 11bit UART, variable-baud rate
                                  //  ������ = ((2��SMOD) / 32) x (��ʱ��1��BRT�������)

//ע: �� T1x12  = 0 ʱ,��ʱ��1������� = SYSclk / 12 / (256 - TH1)
//ע: �� T1x12  = 1 ʱ,��ʱ��1������� = SYSclk / (256 - TH1)
//ע: �� BRTx12 = 0 ʱ,BRT������� = SYSclk / 12 / (256 - BRT)
//ע: �� BRTx12 = 1 ʱ,BRT������� = SYSclk / (256 - BRT)

#define UART_SM2_0      0x00      //��λ���ڿ��ƶദ���ͨ��, ����UART ģʽΪ2��3ʱ
#define UART_SM2_1      0x20
//ע: �� UART ������ģʽ0��1ʱ, Ӧ���� SM2 Ϊ 0. 


/* UART enable */
#define UART_RECEIVE_ENABLE()    (REN = 1)
#define UART_RECEIVE_DISABLE()   (REN = 0)

/* interrupt enable */
#define uart_enable_int()        (ES = 1)
#define uart_disable_int()       (ES = 0)

/* baud rate */
#define uart_baud_x2             (PCON |= SMOD_)    //���� UART Ϊ��ʽ1��2��3 ʱ
#define uart_baud_x1             (PCON &= ~SMOD_)
//ZZX: �����ֻ��Դ���1, ����2�Ĳ����ʼӱ��� AUXR.S2SMOD λֵ����.


//ZZX: ���������� UART2 								  
/* UART2 enable */
#define UART2_RECEIVE_ENABLE()    (S2CON |= S2REN_)
#define UART2_RECEIVE_DISABLE()   (S2CON &= ~S2REN_)

/* interrupt enable */
#define uart2_enable_int()        (IE2 |= ES2_)
#define uart2_disable_int()       (IE2 &= ~ES2_)

/* baud rate */
#define uart2_baud_x2             (AUXR |= S2SMOD_)    //���� UART Ϊ��ʽ1��2��3 ʱ
#define uart2_baud_x1             (AUXR &= ~S2SMOD_)


/* state constant(�����ڽ���) */
#define FSA_INIT      0      //�ȴ�֡ͷ
#define FSA_ADDR_D    1      //�ȴ�Ŀ�ĵ�ַ
#define FSA_ADDR_S    2      //�ȴ�Դ��ַ
#define FSA_LENGTH    3      //�ȴ������ֽ�
#define FSA_DATA      4      //�ȴ����(���� ����ID �� ����)
#define FSA_CHKSUM    5      //�ȴ�У���

/*_____ D E C L A R A T I O N ______________________________________________*/
//for UART1
void   uart_set_prio(Byte priority);     //����uart1���ж����ȼ�
void   uart_start_trans(void);           //��ʼ����1����

//for UART2
void   uart2_set_prio(Byte priority);    //����uart2���ж����ȼ�
void   uart2_start_trans(void);          //��ʼ����2����

//for UART1 & UART2
void   uart_init(void);                  //���ڳ�ʼ��, ��UART1 �� UART2 

#endif   /* _UART_DRV_H_ */
