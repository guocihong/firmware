/*C**************************************************************************
* NAME:  comm_task.c
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:      2013.09.05
* REVISION:     9.4     
*----------------------------------------------------------------------------
* PURPOSE:          
* This file contains the communication task and attached routines
*
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include <INTRINS.H>
#include <string.h>
#include "config.h"                         /* system configuration */
#include "lib_mcu\wdt\wdt_drv.h"            /* WDT driver definition */
#include "lib_mcu\timer\timer_drv.h"        /* timer definition */
#include "lib_mcu\uart\uart_drv.h"          /* uart drive definition */
#include "lib_mcu\eeprom\eeprom_drv.h"      /* eeprom drive definition */
#include "comm_task.h"                      /* comm task definition */
#include "command.h"                        /* system command definition */


/*_____ D E F I N I T I O N ________________________________________________*/
#define REPLY_DLY   (100/SCHEDULER_TICK)    //�յ�PC������Ӧ����ʱ

/*_____ D E C L A R A T I O N ______________________________________________*/
extern idata  Uint16 gl_ack_tick;	            /* Ӧ����ʱ��ʱ tick */
extern xdata  Byte   gl_reply_tick;              /* �豸������ʱ*/

/* UART1 */	
extern xdata  Byte  msg_buf[MAX_RecvFrame];     // received message, used for proceding
extern bdata  bit   msg_buf_valid;	            // received valid flag
extern xdata  Byte  recv_buf[MAX_RecvFrame];    // receiving buffer               
extern idata  Byte  recv_state;                 // receive state
extern idata  Byte  recv_timer;                 // receive time-out, �����ֽڼ䳬ʱ�ж�
extern idata  Byte  recv_chksum;                // computed checksum
extern idata  Byte  recv_ctr;                   // reveiving pointer         

extern xdata  Byte  trans_buf[MAX_TransFrame];  // uart transfer message buffer
extern idata  Byte  trans_ctr;                  // transfer pointer
extern idata  Byte  trans_size;                 // transfer bytes number  
extern idata  Byte  trans_chksum;               // computed check-sum of already transfered message 
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

/* variables for beep */
extern xdata  Uint16  beep_during_temp;   //Ԥ��ķ�������ʱ��, ��λtick

/* variables for ���� */
extern xdata  Uint16  ld_during_temp;   //Ԥ���һ��������С���ʱ��, ��λtick  

/* for alarm */
extern bdata  Byte  alarm_out_flag;     //���������־��λֵ0 - �ޱ������ϵ磩;  λֵ1 - ����(�޵�)                                      
extern bdata  bit   alarm2_flag;
extern bdata  bit   alarm1_flag;
extern bdata  bit   alarm1C_flag;	

extern bdata  bit   climb_alarm_flag; //����������־��0-�ޱ�����1-����
extern bdata  bit   adl_alarm_flag;     //���������ϱ�����־: 0 - �ޱ���; 1 - ����ֵ������
extern bdata  bit   adr_alarm_flag;     //�Ҳ�������ϱ�����־: 0 - �ޱ���; 1 - ����ֵ������
extern  data  Uint16  alarm_led_flag;	  //LED����ָʾ: 0 - �ޱ������𣩣�1 - ����������

/* for AD & Sensor */
extern xdata  Uint16  ad_still_dn;        //��̬����ֵ����
extern xdata  Uint16  ad_still_up;        //��̬����ֵ����
extern xdata  Byte    ad_still_Dup[14];   //������ֵ���޲�

extern idata  Uint16  ad_still_dn_s;      //��̬����ֵ����(��λ��10bit����ֵ)
extern idata  Uint16  ad_still_up_s;      //��̬����ֵ����(��λ��10bit����ֵ)
extern idata  Byte    ad_still_Dup_s[14]; //������ֵ���޲�(��λ��10bit����ֵ)

extern idata  Uint16     ad_sensor_mask;  //�Ѱ�װ����Ҫ�ж���sensor mask
				                                  // ��Ӧλ 0 - û�а�װ, ���ж�
						  	                          //        1 - �Ѿ���װ����Ҫ�ж�
extern xdata  Uint16  ad_sensor_mask_LR;  //������˳���������sensor mask: ��8 ~ 1  �� 8 ~ 1																					
extern xdata  Union16   ad_chn_sample[14];  //����һ�ֲ���ֵ���Ѿ���ȥ������ÿͨ��һ���㣬ѭ�����棩
extern xdata  sAD_BASE  ad_chn_base[14];    //��ͨ����̬��׼ֵ/�����޷�ֵ
extern idata  Uint16    ad_alarm_exts;      //����������־����mask���� λֵ 0 - �ޣ� 1 - ����ֵ
extern idata  Uint16    ad_alarm_base;	    //��̬����������־����mask���� λֵ 0 - ����Χ�ڣ� 1 - ������Χ

/* Doorkeep(�Ŵ�) */
extern bdata  bit    gl_dk_status;    //�Ŵſ���״̬��ÿ1s��̬��⣩: 1 - �պ�; 0 - ��(��Ҫ����)                    

/* for system */
extern idata  Byte   gl_comm_addr;     //��ģ��485ͨ�ŵ�ַ�������Ѹ�Ϊ�淶ֵ��																 
extern xdata  Byte   gl_addr_origin;   //��ַ���뿪��ԭֵ����ȡ����
extern idata  Byte   system_status;    //ϵͳ״̬	
extern bdata  bit    system_2or1;      //˫/��������־: 0 - ˫��ȱʡ��; 1 - ��												 
extern bdata  bit    uart_send_samp;   //��׼ֵ/����ֵ��UART�ͳ���־: 0 - ������; 1 - ����
									                     //  �� ad_task_init()	�г�ʼ��      
extern const  Byte   AREA_L_INDEX[7];  //�����������Ӧ�İ���(����) index
extern const  Byte   AREA_R_INDEX[7];  //�ҷ���������Ӧ�İ���(����) index
                 
/* ����������ƫ�� */
extern xdata  Uint16   sensor_sample_offset[14];    //����������ƫ�û������ʱ������������ֵ��Ϊ0����Լ310���ң���Ҫ������˲������ = ����ֵ - ����ƫ��                 

//2016-12-07����
extern xdata  sAlarmDetailInfo  AlarmDetailInfo;//�������һ�α�����ϸ��Ϣ

extern void check_still_stress(index);

/*F**************************************************************************
* NAME: comm_task_init
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: Communication task initialization
*****************************************************************************/
void comm_task_init(void)
{ 
  Byte i;

  //UART1��ʼ��
  msg_buf_valid = 0;  
  for (i=0; i<UART_QUEUE_NUM; i++)
  {
    uart_q[i].flag = 0;            //������
    //uart_q[i].external = 0;	     //485����
    //uart_q[i].tdata[0] = FRAME_STX;
    //uart_q[i].need_wait_ack = 0; //����Ӧ�� 
  }
  uart_q_index = 0xFF;   //�޶�������뷢������

  //UART2 ��ʼ��
  msg2_buf_valid = 0;
  for (i=0; i<UART2_QUEUE_NUM; i++)
  {
    uart2_q[i].flag = 0;                //������
		//uart2_q[i].external = 0;	        //
		//uart2_q[i].tdata[0] = FRAME_STX;	//֡ͷ
		//uart2_q[i].need_wait_ack = 0;     //����Ӧ�� 
  }
  uart2_q_index = 0xFF;   //�޶�������뷢������  

  //UARTӲ����ʼ��
  uart_init();            //֮���Ѿ�׼���ô����շ���ֻ�ǻ�δʹ��ȫ���ж�
}
						

/*F**************************************************************************
* NAME: comm_task
*----------------------------------------------------------------------------
* PARAMS:
* return:
*----------------------------------------------------------------------------
* PURPOSE: communication task
*****************************************************************************/
void comm_task(void)
{ 
  Byte   i, j;
  Uint16 temp16;

  //1. ���ղ����� UART1��������λ���������
  if (msg_buf_valid)
  {	//��������λ���������
		//a.1 �Ƿ���Ҫת��������
		if ((msg_buf[0] == CMD_ADDR_BC) || (msg_buf[0] != gl_comm_addr))
		{ //�㲥��ַ��Ǳ��豸, ��Ҫת���� UART2      
			//��UART2 �������ҿ���Buffer
			i = uart2_get_buffer();
			if (i < UART2_QUEUE_NUM)				 
			{ //�ҵ��˿���buffer, д��data
				uart2_q[i].tdata[0] = FRAME_STX;
				memcpy(&uart2_q[i].tdata[1], msg_buf, msg_buf[2] + 3);                    
				uart2_q[i].len = msg_buf[2] + 5;
			}
			else
			{ //�޿���buffer
				//���: ���ж��������ڷ���, �ȴ����
			  while (uart2_q_index != 0xFF);	//������,������ WDT ��λ				                
			}
		}

		//a.2 �Ƿ���Ҫִ�б�����
		if ((msg_buf[0] == CMD_ADDR_BC) || (msg_buf[0] == gl_comm_addr))
		{ //�㲥��ַ��ָ�����豸, ��Ҫִ�� 
			switch (msg_buf[3])
			{						
				case CMD_DADDR_qSTAT://ѯ�ʷ���״̬ - �������λ��
						 //��UART1�������ҿ���Buffer
						 i = uart_get_buffer();
						 if (i < UART_QUEUE_NUM)
						 { //�ҵ��˿���buffer, ׼��Ӧ��
							 uart_q[i].tdata[0] = FRAME_STX;	   //֡ͷ
							 uart_q[i].tdata[1] = msg_buf[1];	   //Ŀ�ĵ�ַ
							 uart_q[i].tdata[2] = gl_comm_addr;  //Դ��ַ
							 if (gl_comm_addr == CMD_ADDR_UNSOLV)
							 { //���豸����Ч��ַ
								 //ֻ�ز���Ӧ��
								 uart_q[i].tdata[3] = 1;
								 uart_q[i].tdata[4] = CMD_DADDR_aPARA;
								 uart_q[i].len = 6;	
							 }
							 else
							 { //����Ч��ַ,�ط���״̬                                 
								 if ((alarm_out_flag & 0x38) == 0x00)
								 { //2���������ޱ���
									 uart_q[i].tdata[3] = 1;
									 uart_q[i].tdata[4] = CMD_ACK_OK;
									 uart_q[i].len = 6;													
								 }
								 else
								 { //�б���
									 uart_q[i].tdata[3] = 2;
									 uart_q[i].tdata[4] = CMD_DADDR_aSTAT;
									 uart_q[i].tdata[5] = (alarm_out_flag & 0x38) >> 3;
									 uart_q[i].len = 7;
								 }
							 }							
						 }
						 else
						 { //�޿���buffer, ����������
							 //���: ���ж��������ڷ���, �ȴ������
							 while (uart_q_index != 0xFF);	//������,������ WDT ��λ				           
						 }
						 break;													 

				case CMD_DADDR_qPARA://ѯ�ʲ���
						 //��UART1�������ҿ���Buffer
						 i = uart_get_buffer();
						 if (i < UART_QUEUE_NUM)
						 { //�ҵ��˿���buffer, ׼��Ӧ��
							 uart_q[i].tdata[0] = FRAME_STX;	//֡ͷ
							 uart_q[i].tdata[1] = msg_buf[1];	//Ŀ�ĵ�ַ
							 uart_q[i].tdata[2] = gl_comm_addr;	    //Դ��ַ																 
							 uart_q[i].tdata[3] = 1;
							 uart_q[i].tdata[4] = CMD_DADDR_aPARA;
							 uart_q[i].len = 6;														
						 }
						 else
						 { //�޿���buffer, ����������
							 //���: ���ж��������ڷ���, �ȴ������
							 while (uart_q_index != 0xFF);	//������,������ WDT ��λ				           
						 }
						 break;	
                         
                case 0xE3://������ʱʱ��
                        //1. д��flash
                        flash_enable();                              
                        flash_erase(EEPROM_SECTOR8);                                        
                        flash_write(msg_buf[4], EEPROM_SECTOR8 + 1);  
                        flash_write(0x5a, EEPROM_SECTOR8);                                                              
                        flash_disable();
                    
                        //2. ���±���
                        gl_reply_tick = msg_buf[4];
                    
                        break;
                
                case 0xE4://��ȡ��ʱʱ��
						 //��UART1�������ҿ���Buffer
						 i = uart_get_buffer();
						 if (i < UART_QUEUE_NUM)
						 { //�ҵ��˿���buffer, ׼��Ӧ��
							 uart_q[i].tdata[0] = FRAME_STX;	//֡ͷ
							 uart_q[i].tdata[1] = msg_buf[1];	//Ŀ�ĵ�ַ
							 uart_q[i].tdata[2] = gl_comm_addr;	    //Դ��ַ																 
							 uart_q[i].tdata[3] = 2;
							 uart_q[i].tdata[4] = 0xF4;
                             uart_q[i].tdata[5] = gl_reply_tick;
							 uart_q[i].len = 7;														
						 }
						 else
						 { //�޿���buffer, ����������
							 //���: ���ж��������ڷ���, �ȴ������
							 while (uart_q_index != 0xFF);	//������,������ WDT ��λ				           
						 }
						 break;	

	      case CMD_ZL_PRE://����/����ר�������־
						 switch (msg_buf[5]) 
						 {					 																							
							 case 0x10: //�����ò���
                                //��UART�������ҿ���Buffer
                                i = uart_get_buffer();
                                if (i < UART_QUEUE_NUM)
                                { //�ҵ��˿���buffer, д��data
                                    uart_q[i].tdata[0] = FRAME_STX;
                                  uart_q[i].tdata[1] = msg_buf[1];	    //Ŀ�ĵ�ַ
                                  uart_q[i].tdata[2] = gl_comm_addr;	  //Դ��ַ																 
                                  uart_q[i].tdata[3] = 0x1E;
                                  uart_q[i].tdata[4] = CMD_ZL_PRE;															
                                    uart_q[i].tdata[5] = 0x1C;
                                    uart_q[i].tdata[6] = 0x08;
                                    uart_q[i].tdata[7] = HIGH(ad_sensor_mask_LR);    
                                    uart_q[i].tdata[8] = LOW(ad_sensor_mask_LR);
                                    uart_q[i].tdata[9]  = HIGH(ad_still_dn);
                                    uart_q[i].tdata[10] = LOW(ad_still_dn);
                                    uart_q[i].tdata[11] = HIGH(ad_still_up);
                                    uart_q[i].tdata[12] = LOW(ad_still_up);
                                    uart_q[i].tdata[13]  = 66;   //ad_still_Ddn;
                                    for (j=0; j<7; j++)
                                      uart_q[i].tdata[14 + j] = ad_still_Dup[AREA_L_INDEX[j]]; 
                                    uart_q[i].tdata[21] = 0;
                                    for (j=0; j<7; j++)
                                      uart_q[i].tdata[22 + j] = ad_still_Dup[AREA_R_INDEX[j]]; 
                                    uart_q[i].tdata[29] = 0;																											
                                    uart_q[i].tdata[30] = system_2or1;
                                    uart_q[i].tdata[31] = gl_addr_origin;
                                    uart_q[i].tdata[32] = (Byte)(((Uint32)beep_during_temp * SCHEDULER_TICK) / 1000);	//���ⱨ�����ʱ��
                                    uart_q[i].tdata[33] = (Byte)(((Uint32)ld_during_temp * SCHEDULER_TICK) / 1000);	  //�������ʱ��              																					
                                    uart_q[i].len = 35;
                                }
                                else
                                { //�޿���buffer, ����������
                                    //���: ���ж��������ڷ���, �ȴ������
                                    while (uart_q_index != 0xFF);	//������,������ WDT ��λ				           
                                }                 
                                break;
												
							 case 0x11: //�����������״̬
                                //��UART�������ҿ���Buffer
                                i = uart_get_buffer();
                                if (i < UART_QUEUE_NUM)
                                { //�ҵ��˿���buffer, д��data
                                    uart_q[i].tdata[0] = FRAME_STX;
                                  uart_q[i].tdata[1] = msg_buf[1];	    //Ŀ�ĵ�ַ
                                  uart_q[i].tdata[2] = gl_comm_addr;	  //Դ��ַ																 
                                  uart_q[i].tdata[3] = 0x05;
                                  uart_q[i].tdata[4] = CMD_ZL_PRE;
                                  uart_q[i].tdata[5] = 0x03;
                                  uart_q[i].tdata[6] = 0x19;
                                  uart_q[i].tdata[7] = system_2or1;
                                  uart_q[i].tdata[8] = (alarm_out_flag & 0x18) >> 3;														
                                    uart_q[i].len = 10;
                                } 
                                else
                                { //�޿���buffer, ����������
                                    //���: ���ж��������ڷ���, �ȴ������
                                    while (uart_q_index != 0xFF);	//������,������ WDT ��λ				           
                                }                
                                break;
													
							 case 0x12: //������ʵʱ��Ϣ
                                //��UART�������ҿ���Buffer
                                i = uart_get_buffer();
                                if (i < UART_QUEUE_NUM)
                                { //�ҵ��˿���buffer, д��data
                                    uart_q[i].tdata[0] = FRAME_STX;
                                    uart_q[i].tdata[1] = msg_buf[1];	    //Ŀ�ĵ�ַ
                                    uart_q[i].tdata[2] = gl_comm_addr;	  //Դ��ַ																 
                                    uart_q[i].tdata[3] = 0x0A;
                                    uart_q[i].tdata[4] = CMD_ZL_PRE;
                                    uart_q[i].tdata[5] = 0x08;
                                    uart_q[i].tdata[6] = 0x1A;													
                                    uart_q[i].tdata[7] = HIGH(ad_sensor_mask_LR);    
                                    uart_q[i].tdata[8] = LOW(ad_sensor_mask_LR);														
                                    temp16 = change_to_LR(ad_alarm_exts);																						
                                    uart_q[i].tdata[9] = HIGH(temp16);
                                    uart_q[i].tdata[10] = LOW(temp16);
                                    temp16 = change_to_LR(ad_alarm_base);
                                    uart_q[i].tdata[11] = HIGH(temp16);
                                    uart_q[i].tdata[12] = LOW(temp16);														
                                    uart_q[i].tdata[13] = (Byte)(!gl_dk_status);                     
                                    uart_q[i].len = 15;												
                                } 
                                else
                                { //�޿���buffer, ����������
                                    //���: ���ж��������ڷ���, �ȴ������
                                    while (uart_q_index != 0xFF);	//������,������ WDT ��λ				           
                                }                
                                break;

							 case 0x14: //��˲̬����
                                //��UART�������ҿ���Buffer
                                i = uart_get_buffer();
                                if (i < UART_QUEUE_NUM)
                                { //�ҵ��˿���buffer, д��data
                                    uart_q[i].tdata[0] = FRAME_STX;
                                    uart_q[i].tdata[1] = msg_buf[1];	    //Ŀ�ĵ�ַ
                                    uart_q[i].tdata[2] = gl_comm_addr;	  //Դ��ַ																 
                                    uart_q[i].tdata[3] = 0x23;
                                    uart_q[i].tdata[4] = CMD_ZL_PRE;
                                    uart_q[i].tdata[5] = 0x21;
                                    uart_q[i].tdata[6] = 0x1C;																																																				
                                    for (j=0; j<7; j++)
                                    { //��7~1
                                        temp16 = ad_chn_sample[AREA_L_INDEX[j]].w;
                                        uart_q[i].tdata[7+(j<<1)] = HIGH(temp16);
                                        uart_q[i].tdata[8+(j<<1)] = LOW(temp16);
                                    }
                                    uart_q[i].tdata[21] = 0;
                                    uart_q[i].tdata[22] = 0;														
                                  for (j=0; j<7; j++)
                                    { //��7~1
                                        temp16 = ad_chn_sample[AREA_R_INDEX[j]].w;
                                        uart_q[i].tdata[23+(j<<1)] = HIGH(temp16);
                                        uart_q[i].tdata[24+(j<<1)] = LOW(temp16);
                                    }
                                    uart_q[i].tdata[37] = 0;
                                    uart_q[i].tdata[38] = 0;																																																						
                                    uart_q[i].len = 40;
                                }
                                else
                                { //�޿���buffer, ����������
                                    //���: ���ж��������ڷ���, �ȴ������
                                    while (uart_q_index != 0xFF);	//������,������ WDT ��λ				           
                                }                
                                break;
											
							 case 0x15: //����̬������׼
                                //��UART�������ҿ���Buffer
                                i = uart_get_buffer();
                                if (i < UART_QUEUE_NUM)
                                { //�ҵ��˿���buffer, д��data
                                    uart_q[i].tdata[0] = FRAME_STX;
                                    uart_q[i].tdata[1] = msg_buf[1];	    //Ŀ�ĵ�ַ
                                    uart_q[i].tdata[2] = gl_comm_addr;	  //Դ��ַ																 
                                    uart_q[i].tdata[3] = 0x23;
                                    uart_q[i].tdata[4] = CMD_ZL_PRE;
                                    uart_q[i].tdata[5] = 0x21;
                                    uart_q[i].tdata[6] = 0x1D;																																																				
                                    for (j=0; j<7; j++)
                                    { //��7~1
                                        temp16 = ad_chn_base[AREA_L_INDEX[j]].base;
                                        uart_q[i].tdata[7+(j<<1)] = HIGH(temp16);
                                        uart_q[i].tdata[8+(j<<1)] = LOW(temp16);
                                    }
                                    uart_q[i].tdata[21] = 0;
                                    uart_q[i].tdata[22] = 0;														
                                  for (j=0; j<7; j++)
                                    { //��7~1
                                        temp16 = ad_chn_base[AREA_R_INDEX[j]].base;
                                        uart_q[i].tdata[23+(j<<1)] = HIGH(temp16);
                                        uart_q[i].tdata[24+(j<<1)] = LOW(temp16);
                                    }
                                    uart_q[i].tdata[37] = 0;
                                    uart_q[i].tdata[38] = 0;																																																						
                                    uart_q[i].len = 40;														
                                }  
                                else
                                { //�޿���buffer, ����������
                                    //���: ���ж��������ڷ���, �ȴ������
                                    while (uart_q_index != 0xFF);	//������,������ WDT ��λ				           
                                }                
                                break;

							 case 0x17: //ͨ��Ӧ��
                                //��UART�������ҿ���Buffer
                                i = uart_get_buffer();
                                if (i < UART_QUEUE_NUM)
                                { //�ҵ��˿���buffer, д��data
                                    uart_q[i].tdata[0] = FRAME_STX;
                                    uart_q[i].tdata[1] = msg_buf[1];	    //Ŀ�ĵ�ַ
                                    uart_q[i].tdata[2] = gl_comm_addr;	  //Դ��ַ																 
                                    uart_q[i].tdata[3] = 0x04;
                                    uart_q[i].tdata[4] = CMD_ZL_PRE;
                                    uart_q[i].tdata[5] = 0x02;
                                    uart_q[i].tdata[6] = 0x1F;																																									
                                    uart_q[i].tdata[7] = gl_addr_origin;													
                                    uart_q[i].len = 9;
                                } 
                                else
                                { //�޿���buffer, ����������
                                    //���: ���ж��������ڷ���, �ȴ������
                                    while (uart_q_index != 0xFF);	//������,������ WDT ��λ				           
                                }                
                                break;                             
												
							 case 0x55: //AD����ֵ�������
                                if (msg_buf[6] == 0x00)		                
                                    uart_send_samp = 0;   //ֹͣ����				   
                                else                
                                    uart_send_samp = 1;	  //��ʼ����                   
                                break;
												
							 case 0x40: //���þ�̬����ֵ��Χ
                                //1. д��flash
                                flash_enable();                              
                                flash_erase(EEPROM_SECTOR3);                                        
                                flash_write(msg_buf[6], EEPROM_SECTOR3 + 1);  
                                flash_write(msg_buf[7], EEPROM_SECTOR3 + 2);
                                flash_write(msg_buf[8], EEPROM_SECTOR3 + 3);  
                                flash_write(msg_buf[9], EEPROM_SECTOR3 + 4);
                                flash_write(0x5a, EEPROM_SECTOR3);                                                              
                                flash_disable();
                                //2. ���±���
                                ad_still_dn = ((Uint16)msg_buf[6] << 8) + msg_buf[7];	 //����
                                ad_still_up = ((Uint16)msg_buf[8] << 8) + msg_buf[9];	 //����
                                //3. ͬ�����¶�Ӧ����ֵ
                                //ad_still_dn_s = (Uint16)(((Uint32)ad_still_dn  * SENSOR_RATIO_SAMP) >> SENSOR_RATIO_FS);   //��̬����ֵ����
                                //ad_still_up_s = (Uint16)(((Uint32)ad_still_up  * SENSOR_RATIO_SAMP) >> SENSOR_RATIO_FS);	 //��̬����ֵ����									  
                                ad_still_dn_s = ad_still_dn;    //ZZX: ֱ��ʹ�ò���ֵ
                                ad_still_up_s = ad_still_up;
                                //4. ��鵱ǰ��̬����ֵ
                                if (system_status == SYS_CHECK)
                                {	//�ѿ�ʼ���м��
                                    for (i=0; i<14; i++)
                                    {
                                        check_still_stress(i);
                                    }
                                }
                                break;
																									
							 case 0x50: //���ñ�����ֵ (������)
                                //1. д��flash�����±���
                                flash_enable();                              
                                flash_erase(EEPROM_SECTOR4);                                        
                                flash_write(msg_buf[6], EEPROM_SECTOR4 + 1); 	 //���޸���ֵ������
                                //ZZX: ���� Flash �����ޱ���, Ŀǰû�б���ȡʹ��						
                                for (j=0; j<7; j++)
                                { //��1 ~7
                                    ad_still_Dup[AREA_L_INDEX[j]] = msg_buf[7 + j];	 
                                    flash_write(msg_buf[7 + j], EEPROM_SECTOR4 + 2 + j);
                                }							 
                                for (j=0; j<7; j++)
                                { //��1 ~7
                                    ad_still_Dup[AREA_R_INDEX[j]] = msg_buf[15 + j];	 
                                    flash_write(msg_buf[15 + j], EEPROM_SECTOR4 + 10 + j);
                                }				
                                flash_write(0x5a, EEPROM_SECTOR4);                                                               
                                flash_disable();													
                                //2.ͬ�����²�����ֵ
                                for (j=0; j<14; j++)
                                {
                                    ad_still_Dup_s[j] = ad_still_Dup[j];
                                }													
                                //���޹̶�ȡ��׼ֵ�� 1/3
                                //3. ���»�����������������(����ֵ)													
                                if (system_status == SYS_CHECK)
                                {	//�ѿ�ʼ���м��
                                    for (i=0; i<14; i++)
                                    {					       
                                        if ((1023 - ad_chn_base[i].base) > ad_still_Dup_s[i])
                                            ad_chn_base[i].base_up = ad_chn_base[i].base + ad_still_Dup_s[i];
                                        else
                                            ad_chn_base[i].base_up = 1023; 					  
                                    }
                                }													
                                break;	
															
							 case 0x60: //�������ⱨ��ʱ��	              
                                //1. д��flash				  
                                flash_enable();                              
                                flash_erase(EEPROM_SECTOR5);                                        
                                flash_write(msg_buf[6], EEPROM_SECTOR5 + 1);  
                                flash_write(0x5a, EEPROM_SECTOR5);                                                              
                                flash_disable();
                                //2. ���±���
                                beep_during_temp = (Uint16)(((Uint32)msg_buf[6] * 1000) / SCHEDULER_TICK);				  
                                break;

							 case 0x70: //���������������ʱ��	              	              
                                //1. д��flash				  
                                flash_enable();                              
                                flash_erase(EEPROM_SECTOR6);                                        
                                flash_write(msg_buf[6], EEPROM_SECTOR6 + 1);  
                                flash_write(0x5a, EEPROM_SECTOR6);                                                              
                                flash_disable();
                                //2. ���±���
                                ld_during_temp = (Uint16)(((Uint32)msg_buf[6] * 1000) / SCHEDULER_TICK);				  				  
                                break;	

                            case 0xF1: //���ô���������ƫ��---->������·�ϵ����                  
                                //1. д��flash�����±���
                                flash_enable();
                                flash_erase(EEPROM_SECTOR7);

                                for (j = 0; j < 5; j++) {
                                    //��1 ~5
                                    sensor_sample_offset[j] =
                                        ((Uint16)msg_buf[6 + (j << 1)] << 8) + msg_buf[7 + (j << 1)];
                                    flash_write(msg_buf[6 + (j << 1)], EEPROM_SECTOR7 + 1 + (j << 1));
                                    flash_write(msg_buf[7 + (j << 1)], EEPROM_SECTOR7 + 2 + (j << 1));
                                }

                                for (j = 0; j < 5; j++) {
                                    //��1 ~5
                                    sensor_sample_offset[5 + j] =
                                        ((Uint16)msg_buf[22 + (j << 1)] << 8) + msg_buf[23 + (j << 1)];
                                    flash_write(msg_buf[22 + (j << 1)], EEPROM_SECTOR7 + 11 + (j << 1));
                                    flash_write(msg_buf[23 + (j << 1)], EEPROM_SECTOR7 + 12 + (j << 1));
                                }

                                for (j = 0; j < 2; j++) {
                                    //��6 ~7
                                    sensor_sample_offset[10 + j] =
                                        ((Uint16)msg_buf[32 + (j << 1)] << 8) + msg_buf[33 + (j << 1)];
                                    flash_write(msg_buf[32 + (j << 1)], EEPROM_SECTOR7 + 21 + (j << 1));
                                    flash_write(msg_buf[33 + (j << 1)], EEPROM_SECTOR7 + 22 + (j << 1));
                                }
                                
                                for (j = 0; j < 2; j++) {
                                    //��6 ~7
                                    sensor_sample_offset[12 + j] =
                                        ((Uint16)msg_buf[16 + (j << 1)] << 8) + msg_buf[17 + (j << 1)];
                                    flash_write(msg_buf[16 + (j << 1)], EEPROM_SECTOR7 + 25 + (j << 1));
                                    flash_write(msg_buf[17 + (j << 1)], EEPROM_SECTOR7 + 26 + (j << 1));
                                }
                                
                                //������
                                sensor_sample_offset[9] = 0;
                                
                                flash_write(0x5a, EEPROM_SECTOR7);
                                flash_disable(); 

                                break;     

                            //2016-12-07����
                            case 0xF8://��ȡ������ϸ��Ϣ                                
                                get_alarm_detail_info();
                                break;
                }//end switch
				break;													
			}//end switch	��������
            
            //a.3 ����Ӧ����ʱ
            Disable_interrupt();	
            gl_ack_tick = REPLY_DLY + (gl_comm_addr - 16) * gl_reply_tick / SCHEDULER_TICK;
            Enable_interrupt(); 
		}//end if �㲥��ַ��ָ�����豸, ��Ҫִ��
		
    //b. ��λ��־
    msg_buf_valid = FALSE;	   
  }//end if (msg_buf_valid) 						 

  //2. ���ղ�����UART2��������λ���������
  if (msg2_buf_valid)
  {	//��������λ���������
    //a. ת��������
		//��UART1�������ҿ���Buffer
		i = uart_get_buffer();
		if (i < UART_QUEUE_NUM)				 
		{ //�ҵ��˿���buffer, д��data
			uart_q[i].tdata[0] = FRAME_STX;
		  memcpy(&uart_q[i].tdata[1], msg2_buf, msg2_buf[2] + 3);                    
			uart_q[i].len = msg2_buf[2] + 5;
		}
		else
		{ //�޿���buffer
			//���: ���ж��������ڷ���, �ȴ����
			while (uart_q_index != 0xFF);	//������,������ WDT ��λ				                
		}
    //b. ��λ��־
    msg2_buf_valid = FALSE;	   
  }//end if (msg2_buf_valid)	  
//
  //3. UART1 ���з���
  if ((uart_q_index == 0xFF) && (recv_state == FSA_INIT) && (gl_ack_tick == 0))
  {	//UART1�޽��뷢�����̵Ķ�����, ���Ƿ��еȴ����͵���
    for (i=0; i<UART_QUEUE_NUM; i++)
    {
			if (uart_q[i].flag == 1)
			{	//�еȴ����͵�����Ŵ����
				uart_q[i].flag = 2;
				uart_q_index = i;
				memcpy(trans_buf, uart_q[i].tdata, uart_q[i].len - 1);
				trans_size = uart_q[i].len;
				uart_start_trans();
				break;
			}
    }
  }
  
  //4. UART2 ���з���
  if ((uart2_q_index == 0xFF) && (recv2_state == FSA_INIT))  
  {	//UART2�޽��뷢�����̵Ķ�����, ���Ƿ��еȴ����͵���
    for (i=0; i<UART2_QUEUE_NUM; i++)
    {
			if (uart2_q[i].flag == 1)
			{	//�еȴ����͵�����Ŵ����
				uart2_q[i].flag = 2;
				uart2_q_index = i;
				memcpy(trans2_buf, uart2_q[i].tdata, uart2_q[i].len - 1);
				trans2_size = uart2_q[i].len;
				uart2_start_trans();
				break;
			}
    }
  }  
}//end FUNC()


/*F**************************************************************************
* NAME: uart_get_buffer
*----------------------------------------------------------------------------
* PARAMS:          
* return: Byte
*         ������ֵ >= UART_QUEUE_NUM, ���ʾû�����뵽����buffer
*----------------------------------------------------------------------------
* PURPOSE: �ڴ��ڶ�����Ѱ�ҿ��ж�������ҵ������ض��������(0 ~ (UART_QUEUE_NUM-1))
*----------------------------------------------------------------------------
* REQUIREMENTS:
*****************************************************************************/
Byte uart_get_buffer(void)
{
  Byte i, flag;

  for (i=0; i<UART_QUEUE_NUM; i++)
  { 
    Disable_interrupt();
		flag = uart_q[i].flag;
		Enable_interrupt();
		if (flag == 0)
		{ //���ҵ�����Buffer
			uart_q[i].flag = 1;
			break;
		}
  }
  return i;
}


/*F**************************************************************************
* NAME: uart2_get_buffer
*----------------------------------------------------------------------------
* PARAMS:          
* return: Byte
*         ������ֵ >= UART2_QUEUE_NUM, ���ʾû�����뵽����buffer
*----------------------------------------------------------------------------
* PURPOSE: �ڴ���2������Ѱ�ҿ��ж�������ҵ������ض��������(0 ~ (UART2_QUEUE_NUM-1))
*****************************************************************************/
Byte uart2_get_buffer(void)
{
  Byte i, flag;

  for (i=0; i<UART2_QUEUE_NUM; i++)
  { 
    Disable_interrupt();
		flag = uart2_q[i].flag;
		Enable_interrupt();
		if (flag == 0)
		{ //���ҵ�����Buffer
			uart2_q[i].flag = 1;
			break;
		}
  }
  return i;
}


/*F**************************************************************************
* NAME: change_to_LR
*----------------------------------------------------------------------------
* PARAMS:   
* return: 
*----------------------------------------------------------------------------
* PURPOSE: ���ڽ� ad_sensor_mask, ad_alarm_exts, ad_alarm_base ����8~1,��8~1
*          ��˳�򷵻ء� 
*****************************************************************************/
Uint16 change_to_LR(Uint16 val)
{
  Byte   j;
  Uint16 temp16;
	
	temp16 = 0;
	for (j=0; j<7; j++)
	{ //��7 ~ 1
		if (val & ((Uint16)0x0001 << AREA_L_INDEX[j]))
			temp16 |= 0x01 << j;
	}
	temp16 = temp16 << 8;  //�����λ�ƶ������ֽ�
	for (j=0; j<7; j++)
	{ //��7 ~ 1
		if (val & ((Uint16)0x0001 << AREA_R_INDEX[j]))
			temp16 |= 0x01 << j;
	} 
	
	return temp16;	
}

//2016-12-07����
void get_alarm_detail_info(void)
{
    Byte i,j;
    Uint16 temp16;
    
    //1������������Ϣ   
    //��UART�������ҿ���Buffer
    i = uart_get_buffer();
    if (i < UART_QUEUE_NUM)
    { 
        //�ҵ��˿���buffer, д��data
        uart_q[i].tdata[0] = FRAME_STX;
        uart_q[i].tdata[1] = 0x01;     	      //Ŀ�ĵ�ַ
        uart_q[i].tdata[2] = gl_comm_addr;	  //Դ��ַ																 
        uart_q[i].tdata[3] = 0x1E;
        uart_q[i].tdata[4] = CMD_ZL_PRE;															
        uart_q[i].tdata[5] = 0x1C;
        uart_q[i].tdata[6] = 0x08;
        uart_q[i].tdata[7] = HIGH(ad_sensor_mask_LR);    
        uart_q[i].tdata[8] = LOW(ad_sensor_mask_LR);
        uart_q[i].tdata[9]  = HIGH(ad_still_dn);
        uart_q[i].tdata[10] = LOW(ad_still_dn);
        uart_q[i].tdata[11] = HIGH(ad_still_up);
        uart_q[i].tdata[12] = LOW(ad_still_up);
        uart_q[i].tdata[13]  = 66;   //ad_still_Ddn;
        for (j=0; j<7; j++)
          uart_q[i].tdata[14 + j] = ad_still_Dup[AREA_L_INDEX[j]]; 
        uart_q[i].tdata[21] = 0;
        for (j=0; j<7; j++)
          uart_q[i].tdata[22 + j] = ad_still_Dup[AREA_R_INDEX[j]]; 
        uart_q[i].tdata[29] = 0;																											
        uart_q[i].tdata[30] = system_2or1;
        uart_q[i].tdata[31] = gl_addr_origin;
        uart_q[i].tdata[32] = (Byte)(((Uint32)beep_during_temp * SCHEDULER_TICK) / 1000);	//���ⱨ�����ʱ��
        uart_q[i].tdata[33] = (Byte)(((Uint32)ld_during_temp * SCHEDULER_TICK) / 1000);	  //�������ʱ��              																					
        uart_q[i].len = 35;
    }
          
    //2����ȡ������Ϣ
    //��UART�������ҿ���Buffer
    i = uart_get_buffer();
    if (i < UART_QUEUE_NUM)
    { 
        //�ҵ��˿���buffer, д��data
        uart_q[i].tdata[0] = FRAME_STX;
        uart_q[i].tdata[1] = 0x01;	          //Ŀ�ĵ�ַ
        uart_q[i].tdata[2] = gl_comm_addr;	  //Դ��ַ																 
        uart_q[i].tdata[3] = 0x0A;
        uart_q[i].tdata[4] = CMD_ZL_PRE;
        uart_q[i].tdata[5] = 0x08;
        uart_q[i].tdata[6] = 0x1A;													
        uart_q[i].tdata[7] = HIGH(ad_sensor_mask_LR);    
        uart_q[i].tdata[8] = LOW(ad_sensor_mask_LR);														
        temp16 = change_to_LR(AlarmDetailInfo.ExternalAlarm);																						
        uart_q[i].tdata[9] = HIGH(temp16);
        uart_q[i].tdata[10] = LOW(temp16);
        temp16 = change_to_LR(AlarmDetailInfo.StaticAlarm);
        uart_q[i].tdata[11] = HIGH(temp16);
        uart_q[i].tdata[12] = LOW(temp16);														
        uart_q[i].tdata[13] = !AlarmDetailInfo.DoorKeepAlarm;                     
        uart_q[i].len = 15;												
    } 
    
    //3����˲̬����
    //��UART�������ҿ���Buffer
    i = uart_get_buffer();
    if (i < UART_QUEUE_NUM)
    { //�ҵ��˿���buffer, д��data
        uart_q[i].tdata[0] = FRAME_STX;
        uart_q[i].tdata[1] = 0x01;	          //Ŀ�ĵ�ַ
        uart_q[i].tdata[2] = gl_comm_addr;	  //Դ��ַ																 
        uart_q[i].tdata[3] = 0x23;
        uart_q[i].tdata[4] = CMD_ZL_PRE;
        uart_q[i].tdata[5] = 0x21;
        uart_q[i].tdata[6] = 0x1C;																																																				
        for (j=0; j<7; j++)
        { //��7~1
            temp16 = AlarmDetailInfo.InstantSampleValue[AREA_L_INDEX[j]].w;
            uart_q[i].tdata[7+(j<<1)] = HIGH(temp16);
            uart_q[i].tdata[8+(j<<1)] = LOW(temp16);
        }
        uart_q[i].tdata[21] = 0;
        uart_q[i].tdata[22] = 0;														
        for (j=0; j<7; j++)
        { //��7~1
            temp16 = AlarmDetailInfo.InstantSampleValue[AREA_R_INDEX[j]].w;
            uart_q[i].tdata[23+(j<<1)] = HIGH(temp16);
            uart_q[i].tdata[24+(j<<1)] = LOW(temp16);
        }
        uart_q[i].tdata[37] = 0;
        uart_q[i].tdata[38] = 0;																																																						
        uart_q[i].len = 40;
    }
    
    //4������̬������׼
    //��UART�������ҿ���Buffer
    i = uart_get_buffer();
    if (i < UART_QUEUE_NUM)
    { //�ҵ��˿���buffer, д��data
        uart_q[i].tdata[0] = FRAME_STX;
        uart_q[i].tdata[1] = 0x01;	          //Ŀ�ĵ�ַ
        uart_q[i].tdata[2] = gl_comm_addr;	  //Դ��ַ																 
        uart_q[i].tdata[3] = 0x23;
        uart_q[i].tdata[4] = CMD_ZL_PRE;
        uart_q[i].tdata[5] = 0x21;
        uart_q[i].tdata[6] = 0x1D;																																																				
        for (j=0; j<7; j++)
        { //��7~1
            temp16 = AlarmDetailInfo.StaticBaseValue[AREA_L_INDEX[j]].base;
            uart_q[i].tdata[7+(j<<1)] = HIGH(temp16);
            uart_q[i].tdata[8+(j<<1)] = LOW(temp16);
        }
        uart_q[i].tdata[21] = 0;
        uart_q[i].tdata[22] = 0;														
        for (j=0; j<7; j++)
        { //��7~1
            temp16 = AlarmDetailInfo.StaticBaseValue[AREA_R_INDEX[j]].base;
            uart_q[i].tdata[23+(j<<1)] = HIGH(temp16);
            uart_q[i].tdata[24+(j<<1)] = LOW(temp16);
        }
        uart_q[i].tdata[37] = 0;
        uart_q[i].tdata[38] = 0;																																																						
        uart_q[i].len = 40;														
    }  
}