/*C**************************************************************************
* NAME:   variable.c
*----------------------------------------------------------------------------
* Copyright (c) 2013.
*----------------------------------------------------------------------------
* RELEASE:      2013.09.04
* REVISION:     9.4 
*----------------------------------------------------------------------------
* PURPOSE:
* This file contains the definition of the global variables
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include "config.h"                      /* system config definition */


/*_____ D E F I N I T I O N ________________________________________________*/
                   
        				
/*_____ D E C L A R A T I O N ______________________________________________*/
/* ϵͳ��ʱ */
idata  Uint16  gl_ack_tick = 0;	         /* ��λ��485��Ӧ����ʱ��ʱ tick */	
 data  Uint16  gl_delay_tick;            /* ͨ����ʱ��tick */  
xdata  Byte    gl_reply_tick;            /* �豸������ʱ*/

/* variables for UART */
xdata  Byte  msg_buf[MAX_RecvFrame];     // received message, used for process
bdata  bit   msg_buf_valid;	             // received valid flag
xdata  Byte  recv_buf[MAX_RecvFrame];    // receiving buffer               
idata  Byte  recv_state;                 // receive state
idata  Byte  recv_timer;                 // receive time-out, �����ֽڼ䳬ʱ�ж�
idata  Byte  recv_chksum;                // computed checksum
idata  Byte  recv_ctr;                   // reveiving pointer         

xdata  Byte  trans_buf[MAX_TransFrame];  // uart transfer buffer������ǰ��Ҫ��������ֽڣ���У���ֽ��⣩
idata  Byte  trans_ctr;                  // transfering pointer
idata  Byte  trans_size;                 // transfered bytes number���������ֽڣ�����֡ͷ��У�飩
idata  Byte  trans_chksum;               // check-sum ���߷��ͱ߼��㣩
bdata  bit   trans_occupy;               // ��������ռ�ñ�־��1-��ռ��, 0-����                      

 data  Byte     uart_q_index;            // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ����ڷ���
xdata  sUART_Q 	uart_q[UART_QUEUE_NUM];	 // ���ڶ��� 

/* for UART2 */
xdata  Byte  msg2_buf[MAX_RecvFrame];    // received message, used for proceding
bdata  bit   msg2_buf_valid;	           // received valid flag
xdata  Byte  recv2_buf[MAX_RecvFrame];   // receiving buffer               
idata  Byte  recv2_state;                // receive state
idata  Byte  recv2_timer;                // receive time-out, �����ֽڼ䳬ʱ�ж�
idata  Byte  recv2_chksum;               // computed checksum
idata  Byte  recv2_ctr;                  // reveiving pointer 

xdata  Byte  trans2_buf[MAX_TransFrame];  // uart transfer buffer������ǰ��Ҫ��������ֽڣ���У���ֽ��⣩
 data  Byte  trans2_ctr;                  // transfering pointer
 data  Byte  trans2_size;                 // transfered bytes number���������ֽڣ�����֡ͷ��У�飩
 data  Byte  trans2_chksum;               // check-sum ���߷��ͱ߼��㣩
bdata  bit   trans2_occupy;               // ������2��ռ�ñ�־��1-��ռ��, 0-����                      

 data  Byte     uart2_q_index;            // ���ڷ���ĳ���������ţ���Ϊ0xFF, ��ʾû���κ����ڷ���
xdata  sUART_Q 	uart2_q[UART2_QUEUE_NUM]; // ����2����

/* variables for beep */
bdata  bit     beep_flag;           // ������־: 0 - ����; 1 - ���ڷ���
 data  Uint16  beep_timer;          // ��ʱ����ʣ�����ʱ��, ��λ:tick
xdata  Uint16  beep_during_temp;    // Ԥ���һ�η�������ʱ��, ��λ:tick 

/* variables for alarm output (�̵�����LED) */
bdata  Byte    alarm_out_flag;  //���������־��λֵ0 - �ޱ������̵����ϵ����ϣ�; λֵ1 - ����(�ϵ�)
                                //���������־��λֵ0 - ��������ϵ�,ʹ�ó��գ�;  λֵ1 - ���������(�̵����ϵ����ϣ���·) 
                                //λַ76543210  ��Ӧ  X X �������� ����2 ����1 ������� X X
								                //ZZX: �������λֵ��ʵ��Ӳ�����ƽŵ�ƽ�෴; 	�������λֵ��ʵ��Ӳ�����ƽŵ�ƽ��ͬ							
       sbit    alarm3_flag  = alarm_out_flag^5;                                                
       sbit    alarm2_flag  = alarm_out_flag^4;
       sbit    alarm1_flag  = alarm_out_flag^3;
       sbit    alarm1C_flag = alarm_out_flag^2;
 data  Uint16  alarm1_timer;    // ��ʱ����������1�ѱ���ʱ��,��λ:tick 
 data  Uint16  alarm2_timer;    // ��ʱ����������2�ѱ���ʱ��,��λ:tick 
 data  Uint16  alarm3_timer;    // ��ʱ�������������ѱ���ʱ��,��λtick
 data  Uint16  ld_timer;        // ��ʱ�����������ʣ��ʱ��, ��λ:tick
xdata  Uint16  ld_during_temp;  // Ԥ���һ��������С���ʱ��, ��λ:tick

/* variables for alarm flag */
bdata  bit     climb_alarm_flag; //����������־��0-�ޱ�����1-����
bdata  bit     adl_alarm_flag;  //���ذ����������ϱ�����־: 0 - �ޱ���; 1 - ����
bdata  bit     adr_alarm_flag;  //���ذ��Ҳ�������ϱ�����־: 0 - �ޱ���; 1 - ����
																//  ����ԭ�����Ϊ����������,������̬��׼ֵ����Χ����
																//  ZZX: �Ѿ��� mask ����������չģ�飬������������
 data  Uint16  alarm_led_flag;	//LED����ָʾ: 0 - �ޱ������𣩣�1 - ����������
								                //  ZZX: ��λֵ��ʵ��Ӳ�����ƽŵ�ƽ�෴								 						  								   

/* AD sample */
//(�û����õĻ�׼)��̬��׼ֵ��/���ޡ�������ֵ��/���� (��λ��Ŀǰ�Ѹ�Ϊʹ�ò���ֵ)
xdata  Uint16  ad_still_dn;         //��̬����ֵ����
xdata  Uint16  ad_still_up;         //��̬����ֵ����
xdata  Byte    ad_still_Dup[14];    //������ֵ����

//(�û����õĻ�׼)��̬��׼ֵ��/���ޡ�������ֵ��/���ޣ�ת��Ϊ����ֵ
idata  Uint16  ad_still_dn_s;       //��̬����ֵ����(��λ��10bit����ֵ)
idata  Uint16  ad_still_up_s;       //��̬����ֵ����(��λ��10bit����ֵ)
idata  Byte    ad_still_Dup_s[14];  //������ֵ����(��λ��10bit����ֵ)

idata  Byte        ad_index;        //���ڲ�����ͨ����, ȡֵ��Χ0~13
 data  sAD_Sample  ad_sample;       //��ǰ����ֵ
idata  Uint16      ad_sensor_mask;  //�Ѱ�װ����Ҫ�ж���sensor mask
				                            // ��Ӧλ 0 - û�а�װ, ���ж�
						  	                    //        1 - �Ѿ���װ����Ҫ�ж�
                                    // λ    13   12   11   10    9 8 7 6 5   4 3 2 1 0 
																		// ��Ӧ  ��7  ��6  ��7  ��6   ��5 ~ ��1   ��5 ~ ��1                                    
                                    // ZZX: ��λֵ��ʵ�ʶ����Ĳ��뿪�ؽ�ֵ�෴
xdata  Uint16   ad_sensor_mask_LR;  //������˳���������sensor mask: ��8 ~ 1  �� 8 ~ 1
bdata  bit      ad_sensor_extent;   //�Ƿ����չ��: 1 - ��; 0 - ����

idata  Uint16   ad_samp_pnum;       //��������
idata  sAD_Sum  ad_samp_equ[14];    //ԭʼ����ֵ��ͣ����ھ���ȥ����, �˺�õ���������
idata  sAD_Sum  ad_samp_sum[14];    //����ֵ�׶���ͣ���Ծ���ȥ�������������ͣ�
xdata  Union16  ad_chn_sample[14];  //ѭ�����������һ�ֲ���ֵ���Ѿ���ȥ������ÿͨ��һ���㣬ѭ�����棩
								                    //  ZZX: ���ڷ��͸�PC
xdata  sAD_BASE ad_chn_base[14];    //��ͨ������ʱ��̬��׼ֵ/�����޷�ֵ����λ������ֵ��
 data  Byte     ad_chn_over[14];    //��ͨ������������(�����)�ķ�ֵ�ж��� 0 - ��Χ�ڣ� 1 - ����ֵ
                                    //ÿͨ��һ���ֽڣ� CH0~13 ��Ӧ ad_chn_over[0~13]
								                    //ĳ�ֽ��е�ÿ��λ��Ӧ˳�������ķ�ֵ�ж�									
idata  Uint16   ad_alarm_exts;      //����������־��δ��mask����λֵ 0 - �ޣ� 1 - ����ֵ                    
idata  Uint16   ad_alarm_base;	    //��̬����������־��δ��mask����λֵ 0 - ����Χ�ڣ� 1 - ������Χ                                   					

/* Doorkeep(�Ŵ�) */
bdata  bit   gl_dk_status;     //�Ŵſ���״̬��ÿ1s��̬��⣩: 1 - �պ�; 0 - ��(��Ҫ����)                    
 data  Byte  gl_dk_tick;  	   //�Ŵż���ʱtick

/* for system */
idata  Byte   system_status;   //ϵͳ״̬
                               // 0 - ��ʼ�ϵ�
                               // 1 - ��׼ֵ����ǰ��ʱ(Լ5��)
                               // 2 - ��׼ֵ����(10������)
                               // 3 - ʵʱ���
															 
idata  Byte   gl_comm_addr;    //��ģ��485ͨ�ŵ�ַ�������Ѹ�Ϊ�淶ֵ��																 
xdata  Byte   gl_addr_origin;  //��ַ���뿪��ԭֵ����ȡ����

bdata  bit    system_2or1;     //˫/��������־: 0 - ˫��ȱʡ��; 1 - ��												 

bdata  bit    uart_send_samp;  //ʵʱ����ֵ��UART�ͳ���־: 0 - �����ͣ�ȱʡ��; 1 - ����

const  Byte   AREA_L_INDEX[7] = {0, 1, 2, 3, 4, 12, 13};   //�����������Ӧ�İ���(����) index
const  Byte   AREA_R_INDEX[7] = {5, 6, 7, 8, 9, 10, 11};   //�ҷ���������Ӧ�İ���(����) index
	
/* ����������ƫ�� */
xdata  Uint16   sensor_sample_offset[14];    //����������ƫ�û������ʱ������������ֵ��Ϊ0����Լ400���ң���Ҫ������˲������ = ����ֵ - ����ƫ��
                                             //0-13�ֱ������1~��5����1~��5����6~��7����6~��7


//2016-12-07����
xdata sAlarmDetailInfo  AlarmDetailInfo;//�������һ�α�����ϸ��Ϣ