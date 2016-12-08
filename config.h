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

/* Interrupt Priorities : ȡֵ��ΧΪ 0 ~ 3 */
#define EX0_PRIO           ((Byte)0)
#define EX1_PRIO           ((Byte)0)
#define T0_PRIO            ((Byte)2)    //������
#define T1_PRIO            ((Byte)0)    
#define UART_PRIO          ((Byte)2)    //UART1�����շ��ж�
#define ADC_PRIO           ((Byte)0)	  //ADC�ж�(û��ʹ��)
#define LVD_PRIO           ((Byte)0)
#define PCA_PRIO           ((Byte)0)
#define UART2_PRIO         ((Byte)2)    //UART2�����շ��ж�
#define SPI_PRIO           ((Byte)0)

/* Timer adjust */
#define TIMER_ADJ_T0       0     // compensate for TIMER0 enter/exit interrupt                                 
#define TIMER_ADJ_T1       0     // compensate for TIMER1 enter/exit interrupt

/* Scheduler Tick */
#define SCHEDULER_TICK     5     // unit is ms

/* UART */
#define	FRAME_STX        0x16	   // Frame header
#define	MAX_RecvFrame      45    // ���ջ�������С
#define	MAX_TransFrame     45    // ���ͻ�������С
#define RECV_TIMEOUT        4    // �ֽڼ�����ʱ����, ��λΪtick
                                 // ��Сֵ����Ϊ1, ���Ϊ0���ʾ�����г�ʱ�ж�
                          
/* Uart Queue */
typedef struct strUART_Q
{
  Byte  flag;         //״̬�� 0 - ���У� 1 - �ȴ����ͣ� 2 - ���ڷ���; 3 - �ѷ��ͣ��ȴ�Ӧ��
//Byte 	external;     //Ŀ�ĵأ� 0 - ��485����; 1 - ����
  Byte  tdata[MAX_TransFrame];  //���ݰ�(���һ��У���ֽڿ��Բ���ǰ���㣬���ڷ���ʱ�߷��ͱ߼���)
  Byte  len;					      //���ݰ���Ч����(��У���ֽ�)
//Byte  need_wait_ack;      //�Ƿ���Ҫ�ȴ�Ŀ�Ļ�Ӧ�� 0 - �� 1 - ��
//Byte  wait_ack_time;			//���ͺ�ȴ�Ŀ�Ļ�Ӧ����ʣ���ʱ�䣬��λ�� tick 
                            //  ��flag=3,�ҵȴ�Ӧ��ʣ��ʱ���Ϊ0ʱ���ط�       
  Byte  package_type;       //0-������λ�������ݰ�;1-�豸��������ݰ�
}sUART_Q;

//#define UART_WAIT_ACK  (2000/SCHEDULER_TICK)	  //���ͺ�ȴ�Ŀ�Ļ�Ӧ������������ʱ��, ��λ��tick
#define UART_QUEUE_NUM   4   //UART1 ������, ������λ��
#define UART2_QUEUE_NUM	 4   //UART2 ������, ������λ�� 

/* Port & Pin definition */
// P0, P2 - Mask����/LED���� 
#define bP0_Gate        P0_7     // �������, maskֵ�����ſأ� 1 - ��ֹ; 0 - �������벦�뿪��ֵ
#define bP2_Gate        P0_6     // �������, maskֵ�����ſأ� 1 - ��ֹ; 0 - �������벦�뿪��ֵ
#define bLED_JL_MASK    0x3C     // 0011 1100b

// P1 - AD0,485ͨ�ŵ�ַ
#define CommAddr_Port   P1       // ׼˫���(����), RS485ͨ�ŵ�ַ(7λ), ��ֵַ��˿ڽ�ֵ�෴ 
#define b485_ADDR_MASK  0xFE     // 1111 1110b
#define bADC_IN         P1_0     // AD ����, ����

//P3 - 
#define bBeep_Ctrl   P3_7        // ���������Beep����:  1-����; 0-����
#define bRS485_DE2   P3_6        // ���������RS485����ʹ��: 1-������; 0 - ��ֹ����(����)
#define bRelay_A2    P3_5		     // ���������(�Ҳ�)�������2��1-�̵����ӵ�����(�ϵ�ȱʡ)�� 0-���ӵ�
#define bRelay_A1    P3_4		     // ���������(���)�������1��1-�̵����ӵ�����(�ϵ�ȱʡ)�� 0-���ӵ�
#define bRelay_L1    P3_3		     // ���������������� ��1-�̵����ӵ����ϣ� 0-���ӵ�(�ϵ�ȱʡ)
#define bRS485_DE    P3_2        // ���������RS485����ʹ��: 1-������; 0 - ��ֹ����(����)
#define bTxD         P3_1 		   //	׼˫��ڣ�UART����
#define bRxD         P3_0        //	׼˫��ڣ�UART����   

//P4 - 
#define bDoorKeeper  P4_6        // ׼˫���-���룬�Ŵż��: 1-�ŴŹر�; 0-�ŴŴ򿪣�Ӧ������
#define bTxD2        P4_3        // ׼˫���, UART2����
#define bRxD2        P4_2        // ׼˫���, UART2����
#define bSel_2or1    P4_0	       // ׼˫���, ˫/������ѡ������: 1 - ˫����; 0 - ������

//P5 - AD����ͨ��ѡ��
#define bSel_ADC_MASK  0x0F      // 0000 1111b, �������                                

/* AD */
typedef struct strAD_Sample
{ //ÿ�����ֵ
  Uint16   val;     //��ǰ����ֵ
  Uint8    index;   //ͨ���ţ���Χ0 ~ 13
  Byte     valid;   //�������ݴ����־: 0 - �Ѵ�������д����ֵ; 1 - ��ֵ���ȴ�����                                    
}sAD_Sample;

typedef struct strAD_Sum
{ //����ֵ�ۼӺ�
  Uint16   sum;     //�ۼƺ� (����64��,�������)
  Uint8    point;   //�Ѳ�������
}sAD_Sum;

typedef struct strAD_BASE
{ //ϵͳ����ʱ��̬��׼ֵ��Ӧ�Ĳ���ֵ
  Uint16   base;       //��̬��׼ֵ
  Uint16   base_down;  //��׼ֵ����(��)
  Uint16   base_up;    //��׼ֵ����(��)
}sAD_BASE;

/* System status */
#define SYS_PowerON     0      // 0 - ��ʼ�ϵ�
#define SYS_B5S         1      // 1 - ��׼ֵ����ǰ��ʱ(Լ5��)
#define SYS_SAMP_BASE   2      // 2 - ��׼ֵ����(10������)
#define SYS_CHECK       3      // 3 - ʵʱ���

/* WDT */
#define Enable_WDT             //ʹ��WDT


/* 485_DE Enable/disable delay */
#define NOP_NUM_WAIT  100


//when UART_MODE_1 + UART_SM2_0 + SMOD=0
//     BRT = 256 - ((SYSclk / 12 / 32) / ������)
#define BRT_BAUD_9600  (256 -  6)
#define BRT_BAUD_4800  (256 - 12)
#define BRT_BAUD_2400  (256 - 24)
#define BRT_BAUD_1200  (256 - 48)

#define BRT_BAUD_RATE   BRT_BAUD_9600

//2016-12-07����
typedef struct strAlarmDetailInfo
{
    Union16  InstantSampleValue[14];//˲������
    sAD_BASE StaticBaseValue[14];   //��̬��׼
    Uint16  ExternalAlarm     ;    //��������            
    Uint16  StaticAlarm;	       //��̬����                                					
    Byte    DoorKeepAlarm;         //�Ŵű���
}sAlarmDetailInfo;
#endif    /* _CONFIG_H_ */
